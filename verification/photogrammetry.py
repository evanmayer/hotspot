# Great Value photogrammetry package with OpenCV.

import concurrent
import cv2
import glob
import logging
import matplotlib.pyplot as plt
from matplotlib import image
from matplotlib.patches import Circle, Rectangle
import multiprocessing as mp
import numba
import numpy as np
import os
import pickle
import re
import seaborn as sns
from scipy import spatial
from scipy import signal
from scipy import ndimage

logging.basicConfig(level='INFO')
logger = logging.getLogger(__name__)

METERS_PER_INCH = 0.0254
# These globals may change if you have a chessboard printout of different
# dimensions/pattern.
BOARD_VERT_SHAPE = (3,6)
BOARD_SQUARE_SIZE = 0.027890 # m
CORNER_TERM_CRIT = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# Locations of chessboard corner coords in the plane of the chessboard
BOARD_CORNER_LOCS = np.zeros((1, BOARD_VERT_SHAPE[0] * BOARD_VERT_SHAPE[1], 3), np.float32)
BOARD_CORNER_LOCS[0,:,:2] = np.mgrid[0:BOARD_VERT_SHAPE[0], 0:BOARD_VERT_SHAPE[1]].T.reshape(-1, 2) * BOARD_SQUARE_SIZE
# This may change if your printed targets area different size
TARGET_W_AS_PRINTED = 0.03987 # m
# Number of 90deg rotations to apply to images before corner finding. Use 0 if
# your images show chessboards of 6 cols by 9 rows, otherwise -1 or 1.
IMG_ROT_NUM = 0


def gen_target(w: float, h: float, dpi: int, loc: str, plot=False):
    '''
    Generate an image of a photogrammetry target.

    Parameters
    ----------
    w
        total image width, m
    h
        total image height, m
    dpi
        output image dpi, converts between matrix coordinates and real width
    loc
        [SW, NW, SE, NE, raft] generates a different pattern for each loc
    '''
    w_in = w / METERS_PER_INCH
    h_in = h / METERS_PER_INCH
    w_px = np.round(w_in * dpi).astype(int)
    h_px = np.round(h_in * dpi).astype(int)

    fig, ax = plt.subplots()
    fig.set_size_inches(w_in, h_in)
    fig.tight_layout()
    fig.subplots_adjust(left=0., right=1., top=1., bottom=0.)

    pattern_size = 8
    pattern = np.random.choice([0.05,0.95], size=(pattern_size,pattern_size))
    pattern[0][0] = 0
    pattern[pattern_size - 1][pattern_size - 1] = 1
    ax.imshow(ndimage.zoom(pattern, h_px / pattern_size, order=0), cmap='Greys')

    # center cross
    c_size = dpi * 0.01 / METERS_PER_INCH # 10 cm
    c_thick = dpi * 0.001 / METERS_PER_INCH # 1 mm

    ax.add_patch(Rectangle((w_px / 2 - c_size / 2, h_px / 2 - c_thick / 2), c_size, c_thick, color='silver')) # horz
    ax.add_patch(Rectangle((w_px / 2 - c_thick / 2, h_px / 2 - c_size / 2), c_thick, c_size, color='silver')) # vert
    ax.add_patch(Rectangle((w_px / 2 - c_size / 2, h_px / 2 - c_thick / 4), c_size, c_thick / 2, color='k')) # horz
    ax.add_patch(Rectangle((w_px / 2 - c_thick / 4, h_px / 2 - c_size / 2), c_thick / 2, c_size, color='k')) # vert

    ax.set_aspect('equal')
    ax.set_xlim([0, w_px])
    ax.set_ylim([0, h_px])
    ax.set_xticks([])
    ax.set_yticks([])
    fig.savefig(f'{loc}_target.png')
    if not plot:
        plt.close()
    return


def find_corners(file: str):
    '''
    Thin wrapper for OpenCV findChessboardCorners
    '''
    im = cv2.imread(file)
    im = np.rot90(im, k=IMG_ROT_NUM) # may not need this, depending on source of images.
    gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(
        gray,
        BOARD_VERT_SHAPE,
        cv2.CALIB_CB_ADAPTIVE_THRESH + \
        cv2.CALIB_CB_FAST_CHECK + \
        cv2.CALIB_CB_NORMALIZE_IMAGE
    )
    logger.info(f'Camera calibration corner finding in {file}: {ret}')
    return ret, corners, im, gray


def calibrate_camera(image_dir: str, plot=False):
    '''
    The general idea is to take a set of test images with a chessboard pattern
    (6x9, i.e. 5x8 intersections) to calculate the distortion parameters of the
    camera/lens combo being used. This will be used to undo distortion effects
    before we can undo projection effects.

    For more information, see

    https://docs.opencv.org/3.4/dc/dbb/tutorial_py_calibration.html

    and

    https://learnopencv.com/camera-calibration-using-opencv/

    Parameters
    ----------
    image_dir
        path to directory containing ~30-60 images of a 6x9 square chessboard
    
    Returns
    -------
    mtx
        Upper-triangular OpenCV camera matrix describing intrinsic camera
        parameters: focal length, optical centers
    dist
        Matrix of lens distortion coefficients
    optimal_camera_matrix
        Refined camera matrix
    roi
        Region of interest, may be used for cropping out blank pixels after
        distortion removal
    '''
    extensions = ['.jpeg', '.jpg', '.JPG']
    files = []
    for ext in extensions:
        files += sorted(glob.glob(os.path.join(image_dir, '**' + ext)))
    assert files, 'No image files found when searching for images for camera calibration.'
    objpoints_q = mp.Queue() # the chessboard vertex locations in the plane of the board
    imgpoints_q = mp.Queue() # the chessboard vertex locations in the image space
    # do camera calibration from chessboard images
    with concurrent.futures.ThreadPoolExecutor() as pool:
        future_to_file = {pool.submit(find_corners, file) : file for file in files}
        for future in concurrent.futures.as_completed(future_to_file):
            file = future_to_file[future]
            try:
                ret, corners, im, gray = future.result()
            except Exception as e:
                logger.warning(f'file {file} generated an exception: {e}')
            else:
                if ret:
                    objpoints_q.put(BOARD_CORNER_LOCS)
                    corners2 = cv2.cornerSubPix(gray, corners, (5,5),(-1,-1), CORNER_TERM_CRIT)
                    imgpoints_q.put(corners2)
                    im = cv2.drawChessboardCorners(gray, BOARD_VERT_SHAPE, corners2, ret)
                if plot:
                    plt.figure()
                    plt.get_current_fig_manager().window.setGeometry(600,400,1000,800)
                    plt.imshow(im)
                    ax = plt.gca()
                    ax.set_aspect('equal')

    # Do the actual camera calibration with all the data we gathered.
    # OpenCV likes lists.
    objpoints = []
    while not objpoints_q.empty():
        objpoints.append(objpoints_q.get(timeout=0.1))
    imgpoints = []
    while not imgpoints_q.empty():
        imgpoints.append(imgpoints_q.get(timeout=0.1))

    h,w = cv2.imread(files[0]).shape[:2]
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    optimal_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        mtx,
        dist, 
        (w,h), 
        1, 
        (w,h)
    )

    return mtx, dist, optimal_camera_matrix, roi


@numba.jit(nopython=True)
def xcorr_prep(im: np.ndarray):
    '''
    Helper function to apply pre-processing that makes cross-correlation work
    '''
    im_corr = np.sum(im.astype(np.float64), axis=2) # ensure single channel
    im_corr /= im_corr.max() # normalize
    im_corr -= np.mean(im_corr) # detrend
    return im_corr


def find_template(template_filename: str, im_warped: np.ndarray, avg_px_per_m: float, stride=2, ax=None):
    '''
    Returns the pixel coordinates of the best-cross-corr match of the template image in im_warped,
    which ought to be a lens-distortion-free, camera-angle-projection-free image.
    avg_px_per_m is needed to scale the target's known size in m to the image's pixel scale.
    
    Parameters
    ----------
    template_filename
        Path to an image file to search for in `im_warped`
    im_warped
        ndarray of distortion/perspective-free image to look for
        photogrammetry targets in
    avg_px_per_m
        Scalar to scale the template image to the correct expected number of
        pixels, given the known template size
    stride : optional
        Downsample `im_warped` and the template by taking every `stride`-th
        element. Makes function faster at the expense of accuracy.
    ax : optional
        Matplotlib axes object. If specified, will plot the template's found
        location on the axes.
    
    Returns
    -------
    xfound, yfound
        Pixel coordinates of the center of the template, as found in `im_warped`
    '''
    im_tmp = np.rot90(cv2.imread(template_filename), k=IMG_ROT_NUM)
    # number of pixels in a shape with some real size in meters:
    target_px = TARGET_W_AS_PRINTED * avg_px_per_m
    tmp_px = im_tmp.shape[0]
    # zoom the template to the right size in pixels
    tmp = ndimage.zoom(im_tmp, target_px / tmp_px)
    a = xcorr_prep(im_warped[::stride,::stride])
    b = xcorr_prep(tmp[::stride,::stride])
    xcorr = signal.fftconvolve(
        a,
        b[::-1, ::-1],
        mode='valid'
    )
    y,x = np.unravel_index(np.argmax(xcorr), xcorr.shape)
    xfound = x + b.shape[0] / 2
    yfound = y + b.shape[1] / 2

    if ax:
        ax.imshow(a, cmap='Greys_r')
        ax.scatter(xfound, yfound, s=80, color='w')
        ax.scatter(xfound, yfound, label=template_filename)

    return xfound * stride, yfound * stride


def find_targets(image_dir, target_dir, mtx, dist, optimal_camera_matrix, stride=2, plot=False):
    '''
    Uses an OpenCV camera matrix to undo distortion in all test images, then
    finds a homographic transform of the undistorted chessboard corners to the
    ideal, rectified plane of the chessboard (the "bird's-eye" view of it).
    Uses that and information of how far apart the chessboard intersections
    really are to calculate a px-to-m conversion.
    Finally, does a cross-correlation to find the location of all target
    images, in pixels.

    Parameters
    ----------
    image_dir
        Path to a directory full of test images. These images contain the
        calibration chessboard and all photogrammetry targets.
    target_dir
        Path to a directory full of template images. These images contain the
        patterns of photogrammetry targets to search for.
    mtx, dist, optimal_camera_matrix
        See `calibrate_camera` docstring.
    stride: optional
        Downsample images before finding targets in them, to do it faster but with less fidelity.
    plot : optional
        If True, make plots of the found locations of all targets in the image

    Returns
    -------
    image_data : dict
        Dictionary full of key, value pairs describing the results of target
        finding and px-to-meter calculation.
    '''
    def find_target(file, mtx, dist, optimal_camera_matrix, stride=2, plot=False):
        '''
        See docstring of `find_targets`.
        '''
        # use camera cal matrix to de-distort all images
        im = cv2.imread(file)
        im = np.rot90(im, k=IMG_ROT_NUM)
        undistorted_image = cv2.undistort(
            im,
            mtx,
            dist,
            None, 
            optimal_camera_matrix
        )

        # re-find chessboard corners in undistorted image
        gray = cv2.cvtColor(undistorted_image, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(
            gray,
            BOARD_VERT_SHAPE,
            cv2.CALIB_CB_ADAPTIVE_THRESH + \
            cv2.CALIB_CB_FAST_CHECK + \
            cv2.CALIB_CB_NORMALIZE_IMAGE
        )
        logger.info(f'Corner detection after de-distortion in {file}: {ret}')
        if ret == True:
            corners2 = cv2.cornerSubPix(
                gray,
                corners,
                (5,5),
                (-1,-1),
                CORNER_TERM_CRIT
            )
            undistorted_image_corners = cv2.drawChessboardCorners(
                gray,
                BOARD_VERT_SHAPE,
                corners2,
                ret
            )
        else:
            return

        # find the homographic transform that undistorts the found chessboard
        # corners into the rectified, "bird's eye" view of the chessboard.
        this_img = corners2
        # magic number: homographic transform for de-projection sometimes makes
        # resulting image really tiny for some reason. if we didn't scale it 
        # back up, we'd have only a few pixels across each chessboard
        # intersection, degrading the quality of the subpixel refinement and 
        # eventually the template location xcorr too.
        sf = 2700.
        h, status = cv2.findHomography(
            this_img[:,0,:],
            BOARD_CORNER_LOCS[0,:,:2] * sf + corners2[0][0]
        )
        # deproject chessboard points
        corners_warped = cv2.perspectiveTransform(
            corners2,
            h
        )
        im_warped = cv2.warpPerspective(
            undistorted_image,
            h,
            (undistorted_image.shape[1], undistorted_image.shape[0])
        )

        # calc pixel scale: get pairs of euclidean distances along each axis.
        # this leaves some information on the table (e.g. expected distance
        # info of non-adjacent pairs), but is simple and there are enough
        # pairs that the average should be good to submm precision across ~1 m
        found_corners = corners_warped[:,0,:].reshape((BOARD_VERT_SHAPE[1], BOARD_VERT_SHAPE[0], 2))
        dists0 = np.array([])
        for i in range(found_corners.shape[0]):
            d = np.diff(found_corners[i,:,:], axis=0)
            dists0 = np.concatenate([dists0, np.sqrt((d ** 2).sum(axis=1))])
        dists1 = np.array([])
        for j in range(found_corners.shape[1]):
            d = np.diff(found_corners[:,j,:], axis=0)
            dists1 = np.concatenate([dists1, np.sqrt((d ** 2).sum(axis=1))])
        dists = np.concatenate([dists0, dists1])
        # print('dists0', dists0.mean(), 'dists1', dists1.mean(), 'discrepancy', dists0.mean() - dists1.mean())
        avg_spacing = np.mean(np.abs(dists))
        px_per_m = avg_spacing / BOARD_SQUARE_SIZE

        if plot:
            plt.figure()
            # plt.get_current_fig_manager().window.setGeometry(600,400,1000,800)
            ax = plt.axes()
        else:
            ax = None

        target_locs = {}
        targets = ['raft_target', 'SW_target', 'SE_target', 'NW_target', 'NE_target']
        for target in targets:
            x,y = find_template(
                os.path.join(target_dir, target + '.png'),
                im_warped,
                px_per_m,
                stride=stride,
                ax=ax)
            target_locs[target] = (x,y)
        if plot:
            ax.legend(loc='best')
            # plt.savefig(f'found_{os.path.basename(file)}.png')
        return target_locs, px_per_m


    extensions = ['.jpeg', '.jpg', '.JPG']
    files = []
    for ext in extensions:
        files += sorted(glob.glob(os.path.join(image_dir, '**' + ext)))
    assert files, 'No image files found when searching for images for target measurement.'
    # files = files[:10]
    image_data = {}
    px_per_ms = []
    # dicts are kinda thread-safe
    with concurrent.futures.ThreadPoolExecutor() as pool:
        future_to_file = {pool.submit(find_target, *(file, mtx, dist, optimal_camera_matrix), **{'stride' : stride, 'plot' : plot}) : file for file in files}
        for future in concurrent.futures.as_completed(future_to_file):
            file = future_to_file[future]
            try:
                target_locs, px_per_m = future.result()
            except Exception as e:
                logger.warning(f'file {file} generated an exception: {e}')
            else:
                px_per_ms.append(px_per_m)
                image_data[file] = {}
                image_data[file]['px_per_m'] = px_per_m
                image_data[file]['target_locs'] = target_locs
    # for file in files:
    #     target_locs, px_per_m = find_target(file, mtx, dist, optimal_camera_matrix)
    #     px_per_ms.append(px_per_m)
    #     image_data[file] = {}
    #     image_data[file]['px_per_m'] = px_per_m
    #     image_data[file]['target_locs'] = target_locs
    avg_px_per_m = np.mean(px_per_ms)
    image_data['source dir'] = os.path.abspath(image_dir)
    image_data['avg_px_per_m'] = avg_px_per_m

    return image_data


def post_process_many_to_one(image_data: dict):
    '''
    Assume the target images are all of the raft in the same location.
    Average the results to get some decent precision on the
    localization of that raft.
    '''
    # Ignore target loc solutions with distances from SW target significantly 
    # different from the median
    outlier_thresh = 0.01

    # independently meas dist from SW corner to each target
    actuals = {
        'raft_target': 0.61198125
    }

    for target in ['raft_target', 'SE_target', 'NW_target', 'NE_target']:
        query_meas = []
        for img in image_data.keys():
            if not isinstance(image_data[img], dict):
                continue
            query_raft = np.linalg.norm(
                np.array(image_data[img]['target_locs'][target]) - 
                np.array(image_data[img]['target_locs']['SW_target'])
            ) / image_data['avg_px_per_m']
            query_meas.append(query_raft)

        # clean data
        query_meas = [item for item in query_meas if np.median(query_meas) - item < outlier_thresh]
        
        query_meas = np.array(query_meas)
        query_raft_mean = query_meas.mean()
        mean_inch = query_raft_mean / METERS_PER_INCH
        mean_inch_frac = (mean_inch - np.floor(mean_inch)) / (1./32.)
        query_raft_std = query_meas.std()
        
        logger.info(
            f'dist of {target} from SW\n' + \
            f'\tmean {query_raft_mean}\n' + \
            f'\tstd {query_raft_std}\n' + \
            f'\tstd (mm) {query_raft_std * 1000}\n' + \
            f'\t% err {100. * query_raft_std / query_raft_mean} %'
        )
        logger.info(f'{np.floor(mean_inch):.0f} + {mean_inch_frac:.2f}/32\"')
        
        plt.figure()
        ax = plt.axes()
        ax.set_title(f'{target} distance from SW target (L2 norm) (m)\nN={len(query_meas)}')
        sns.kdeplot(query_meas, shade=True, color='k', label='Kernel Density Estimate')
        sns.rugplot(query_meas, color='k')
        ax.axvline(query_raft_mean, color='k', label=f'Mean ~{query_raft_mean:.4f} m')
        ax.axvline(query_raft_mean - query_raft_std, color='k', linestyle='--')
        ax.axvline(query_raft_mean + query_raft_std, color='k', linestyle='--', label=f'1 std. dev. ~{query_raft_std * 1000:.1f} mm')
        if target in actuals.keys():
            ax.axvline(actuals[target], color='r', label=f'Actual (1/16" graduated tape measure): {actuals[target]:.4f} m')
        ax.grid(True)
        ax.legend(loc='best')
        ax.set_xlabel('Distance (m)')
        ax.set_ylabel('Probability Density')
    return


def post_process_many_to_many(image_data: dict, command_file: str, ref=None):
    '''
    Assume the target images are each of a different point in a raster scan.
    Compare to the commanded profile.
    '''
    # Reshape command profile assuming a square box is scanned.
    commanded_pts_flat = np.genfromtxt(command_file, delimiter=',', skip_header=1, usecols=[-2,-1])
    assert np.round(np.sqrt(commanded_pts_flat.shape[0])) == np.sqrt(commanded_pts_flat.shape[0]), 'Command profile shape not square. Rewrite alg to handle non-square shapes.'
    commanded_pts = commanded_pts_flat.reshape(
        (
            int(np.sqrt(len(commanded_pts_flat))),
            int(np.sqrt(len(commanded_pts_flat))),
            2
        )
    )
    xs = np.unique([pt[0] for pt in commanded_pts_flat])
    ys = np.unique([pt[1] for pt in commanded_pts_flat])
    
    plt.figure()
    plt.suptitle('Commanded vs. Measured Positions', fontsize=18)
    ax = plt.axes()
    ax.set_xticks(xs)
    plt.xticks(rotation=45)
    ax.set_yticks(ys)
    ax.set_title(command_file + '\n' + image_data['source dir'])
    ax.set_xlabel('x-distance from SW corner (m)')
    ax.set_ylabel('y-distance from SW corner (m)')
    ax.grid(True)
    ax.set_aspect('equal')
    ax.set_facecolor('gainsboro')

    colors = ['k', 'g', 'b', 'm', 'r']
    targets = ['raft_target']#, 'SW_target', 'SE_target', 'NW_target', 'NE_target']
    queries = np.zeros_like(commanded_pts)
    residuals = np.zeros_like(commanded_pts)
    for i, target in enumerate(targets):
        # Get the per-image data out of the image_data dict
        img_keys = [img for img in sorted(list(image_data.keys())) if os.path.exists(img)]
        imgs = np.array(img_keys).reshape(commanded_pts.shape[:2])
        for j in range(imgs.shape[0]):
            for k in range(imgs.shape[1]):
                if not isinstance(image_data[imgs[j][k]], dict):
                    continue
                # image space to mapper space transform
                if ref == 'SW':
                    query = (
                        np.array(image_data[imgs[j][k]]['target_locs'][target]) -
                        np.array(image_data[imgs[j][k]]['target_locs']['SW_target'])
                    )
                else: # reference measurements to first point
                    query = (
                        np.array(image_data[imgs[j][k]]['target_locs'][target]) -
                        np.array(image_data[imgs[0][0]]['target_locs'][target])
                    )
                query[1] *= -1
                query /= image_data['avg_px_per_m']
                if ref != 'SW': # assume first position is perfect and calc errors relative to that
                    query += commanded_pts[0,0,:]
                queries[j][k][0] = query[0]
                queries[j][k][1] = query[1]

        ax.scatter(queries[:,:,0], queries[:,:,1], facecolor=colors[i], label=f'{target} Measured Pos.')
        if target == 'raft_target':
            ax.scatter(
                commanded_pts[:,:,0],
                commanded_pts[:,:,1],
                facecolor='none',
                color='k',
                label=f'{target} Commanded Pos.'
            )
            residuals = queries - commanded_pts
            ax.quiver(
                queries[:,:,0],
                queries[:,:,1],
                residuals[:,:,0],
                residuals[:,:,1],
                np.linalg.norm(residuals, axis=-1),
                pivot='tip',
                angles='xy',
                scale_units='xy',
                scale=1,
                headwidth=2.5,
                headlength=4,
                edgecolors='k',
                linewidth=.5,
                width=4e-3
            )
        
        ax.legend(bbox_to_anchor=(1.05, 1.0), loc='upper left')
        plt.tight_layout()

        plt.figure()
        ax = plt.axes()
        ax.set_title('Residuals: Commanded - Measured')
        residuals_mm = np.array(residuals) * 1000.
        sns.kdeplot(x=residuals_mm[:,:,0].flatten(), y=residuals_mm[:,:,1].flatten(), shade=True, ax=ax)
        ax.scatter(residuals_mm[:,:,0], residuals_mm[:,:,1], color='k', alpha=0.5)
        ax.set_xlabel('X-dir Residuals (mm)')
        ax.set_ylabel('Y-dir Residuals (mm)')
        ax.grid(True)
        ax.set_xlim(-8,8)
        ax.set_ylim(-8,8)

        plt.figure()
        ax = plt.axes()
        ax.set_title('Residual Magnitude: Commanded - Measured')
        residuals_mag_mm = np.linalg.norm(residuals, axis=-1) * 1000.
        levels = np.arange(np.floor(residuals_mag_mm.min()), np.ceil(residuals_mag_mm.max()) + .5, .5)
        X,Y = np.meshgrid(xs, ys)
        im = ax.contourf(X, Y, residuals_mag_mm, levels=levels)
        cbar = plt.colorbar(im, label='Magnitude (mm)')
        ax.set_xticks(xs)
        plt.xticks(rotation=45)
        ax.set_yticks(ys)
        ax.set_xlabel('X-dir Position (m)')
        ax.set_ylabel('Y-dir Position (m)')
        ax.grid(True)
        plt.tight_layout()
    return


if __name__ == '__main__':
    plt.ion()
    # check for pickled camera matrices to avoid expensive recalibration
    if not (
        os.path.exists('camera_cal_mtx.pickle') and
        os.path.exists('camera_cal_dist.pickle') and
        os.path.exists('camera_cal_optimal_camera_matrix.pickle')
    ):
        logger.info('No pickled camera calibration found. Recalibrating...')
        # calibrate the camera for distortion
        mtx, dist, optimal_camera_matrix, roi = calibrate_camera(
            os.path.join('input', 'camera_cal'),
            plot=False
        )
        with open('camera_cal_mtx.pickle', 'wb') as f:
            pickle.dump(mtx, f, protocol=pickle.HIGHEST_PROTOCOL)
        with open('camera_cal_dist.pickle', 'wb') as f:
            pickle.dump(dist, f, protocol=pickle.HIGHEST_PROTOCOL)
        with open('camera_cal_optimal_camera_matrix.pickle', 'wb') as f:
            pickle.dump(optimal_camera_matrix, f, protocol=pickle.HIGHEST_PROTOCOL)
    else:
        logger.info('Found pickled camera calibration. Skipping camera calibration.')
        with open('camera_cal_mtx.pickle', 'rb') as f:
            mtx = pickle.load(f)
        with open('camera_cal_dist.pickle', 'rb') as f:
            dist = pickle.load(f)
        with open('camera_cal_optimal_camera_matrix.pickle', 'rb') as f:
            optimal_camera_matrix = pickle.load(f)
    logger.info('Finding photogrammetry targets in images.')
    image_data = find_targets(
        os.path.join('input', 'meas', 'try1'),
        os.path.join('input', 'targets'),
        mtx,
        dist,
        optimal_camera_matrix,
        plot=False
    )
    logger.info('Post-processing found targets.')
    command_file = os.path.join('..', 'data', 'input', 'profiles', '24in_breadboard_raster_9x9_.2x.2.csv')
    post_process_many_to_many(image_data, command_file)
    plt.show(block=True)