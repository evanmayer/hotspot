# Great Value photogrammetry package with OpenCV.

import concurrent
import cv2
import logging
import matplotlib.pyplot as plt
import multiprocessing as mp
import numpy as np
import os
import re
from scipy.spatial.transform import Rotation as R
import seaborn as sns
import time


logging.basicConfig(level='INFO')
logger = logging.getLogger(__name__)

# natsort, from https://stackoverflow.com/a/16090640
nsort = lambda s: [int(t) if t.isdigit() else t.lower() for t in re.split('(\d+)', s)]

METERS_PER_INCH = 0.0254
# These globals may change if you have a chessboard printout of different
# dimensions/pattern.
BOARD_VERT_SHAPE = (14,20) # this was the shape of the board that is in Dan's lab
# BOARD_VERT_SHAPE = (18,11) # this was the shape of the finer board printed on sticker paper
BOARD_SQUARE_SIZE = 0.020245 / 2 # m, this was the size of the board that is in Dan's lab
# BOARD_SQUARE_SIZE = 15e-3 #0.021004444 # m, this was the size of the board used to calibrate GSI Nikon D810 in MIL
# BOARD_SQUARE_SIZE = 13.091e-3 # m, this was the size of the finer board printed on sticker paper
BOARD_ARUCO_SIZE = 0.015 / 2 # m
# DEFAULT_TARGET_SIZE = 0.02506 # m
DEFAULT_TARGET_SIZE = 0.0249 # m
DEFAULT_ARUCO_DICT = cv2.aruco.DICT_4X4_1000

# Locations of chessboard corner coords in the plane of the chessboard
BOARD_CORNER_LOCS = np.zeros((1, BOARD_VERT_SHAPE[0] * BOARD_VERT_SHAPE[1], 3), np.float32)
BOARD_CORNER_LOCS[0,:,:2] = BOARD_SQUARE_SIZE + np.mgrid[0:BOARD_VERT_SHAPE[0], 0:BOARD_VERT_SHAPE[1]].T.reshape(-1, 2) * BOARD_SQUARE_SIZE
# Number of 90deg rotations to apply to images when loading.
IMG_ROT_NUM = 0

CAMERA_FOCAL_LENGTH_GUESS = 2.7e-3 # m, iPhone 13 mini ultrawide
CAMERA_PIXEL_SIZE_GUESS = 1.7e-6 # m, iPhone 13 mini ultrawide
CAMERA_PIXEL_SHAPE = (4032, 3024)


def load_to_gray(file, camera_matrix=None, camera_dist=None):
    '''
    Load the image `file` and convert to gray, applying de-distortion if
    camera parameters provided
    '''
    im = cv2.imread(file, flags=(cv2.IMREAD_IGNORE_ORIENTATION + cv2.IMREAD_COLOR))
    im = np.rot90(im, k=IMG_ROT_NUM) # may not need this, depending on source of images.
    gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    if (camera_matrix is not None) and (camera_dist is not None):
        gray = cv2.undistort(
            gray,
            camera_matrix,
            camera_dist,
        )
    return gray


def find_corners_aruco(file: str, aruco_dict=DEFAULT_ARUCO_DICT, refine_subpixel=False, valid_ids=[], camera_matrix=None, camera_dist=None):
    '''
    Thin wrapper for OpenCV detectMarkers
    '''
    gray = load_to_gray(file, camera_matrix=camera_matrix, camera_dist=camera_dist)

    dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict)
    parameters = cv2.aruco.DetectorParameters()
    if refine_subpixel:
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        parameters.cornerRefinementMaxIterations = 40 # default 30
        parameters.cornerRefinementMinAccuracy = 0.01 # default 0.1
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)
    corners, ids, rejectedImgPoints = detector.detectMarkers(gray)

    if (ids is not None):
        found = True
        mask = range(len(ids))
    else:
        found = False
        mask = []

    if valid_ids:
        mask = []
        for valid_id in valid_ids:
            if valid_id in ids:
                mask.append(list(ids).index(valid_id))

    logger.info(f'ArUco finding in {file}: {found}')
    logger.info(f'ID\'d {len([corners[i] for i in mask])} targets.')
    return found, np.array([corners[i] for i in mask]), np.array([ids[i][0] for i in mask]), gray


def estimate_pose_aruco(file, corners, aruco_size, camera_matrix, camera_dist, fig=None, ax=None, plot=False):
    rvec, tvec, obj_points = cv2.aruco.estimatePoseSingleMarkers(corners, aruco_size, camera_matrix, camera_dist)
    if plot:
        if (not fig or not ax):
            fig, ax = plt.subplots()
        gray = load_to_gray(file, camera_matrix=camera_matrix, camera_dist=camera_dist)
        im_with_axes = cv2.drawFrameAxes(gray, camera_matrix, camera_dist, rvec, tvec, 0.075)
        ax.imshow(im_with_axes, cmap='bone')
    return rvec, tvec, obj_points


def generate_charuco_board(board_vert_shape, square_size, aruco_size, aruco_dict=DEFAULT_ARUCO_DICT, gen_png=False):
    dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict)
    board = cv2.aruco.CharucoBoard(
        (board_vert_shape[0]+1,
        board_vert_shape[1]+1),
        square_size,
        aruco_size,
        dictionary
    )
    if gen_png:
        # change these parameters for different printers or sheets of paper.
        dpi = 300
        img = board.generateImage((int(8.5 * dpi), int(11 * dpi)), marginSize=int(dpi * square_size / METERS_PER_INCH))
        cv2.imwrite(f'{board_vert_shape[1]+1}x{board_vert_shape[0]+1}_square_{square_size}m_aruco_{aruco_size}m_charuco_dict_{aruco_dict}.png', img)

    return board, dictionary


def estimate_pose_charuco(file, camera_matrix, camera_dist, board_vert_shape=BOARD_VERT_SHAPE, square_size=BOARD_SQUARE_SIZE, aruco_size=BOARD_ARUCO_SIZE, aruco_dict=DEFAULT_ARUCO_DICT, fig=None, ax=None, plot=False):
    '''
    Thin wrapper for OpenCV aruco detectBoard
    '''
    board, _ = generate_charuco_board(board_vert_shape, square_size, aruco_size, aruco_dict=aruco_dict)
    gray = load_to_gray(file, camera_matrix=camera_matrix, camera_dist=camera_dist)

    detector = cv2.aruco.CharucoDetector(board)
    params = detector.getDetectorParameters()
    params.minMarkerPerimeterRate = 1e-2
    detector.setDetectorParameters(params)
    aruco_corners, aruco_ids, marker_corners, marker_ids = detector.detectBoard(gray)
    charuco_ret, aruco_corners, aruco_ids = cv2.aruco.interpolateCornersCharuco(marker_corners, marker_ids, gray, board)
    ret, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(aruco_corners, aruco_ids, board, camera_matrix, camera_dist, None, None)

    if not ret:
        logger.critical('failed to find charuco pose')
    # Board z-axis points away from the camera, but we want it pointing
    # toward the camera to align the board axes with the mapper coordinate
    # frame
    board_rvec_row = rvec.T[0]
    Rb = R.from_rotvec(board_rvec_row)
    Rb *= R.from_euler('zyx', [-np.pi / 2., 0.0, np.pi])
    rvec = np.array([Rb.as_rotvec()]).T # Repackage like it came out of pose estimator

    if plot:
        if (not fig or not ax):
            fig, ax = plt.subplots()
        im_with_charuco_board = cv2.drawFrameAxes(gray, camera_matrix, camera_dist, rvec, tvec, 0.1)
        ax.imshow(im_with_charuco_board, cmap='bone')

    return rvec, tvec


def calibrate_camera(image_files: list, board_vert_shape=BOARD_VERT_SHAPE, square_size=BOARD_SQUARE_SIZE, aruco_size=BOARD_ARUCO_SIZE, guess_intrinsics=False, plot=False):
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
    image_files
        list of images of a chessboard or charuco board
    board_vert_shape : tuple of ints (optional)
        shape of chessboard vertices
    square_size : float (optional)
        size of chessboard square, m
    aruco_size : float (optional)
        size of ArUco square, m
    plot (optional)
        Draw a figure for each image analyzed.
    
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
    num_found_thresh = 20

    objpoints_q = mp.Queue() # the chessboard vertex locations in the plane of the board
    imgpoints_q = mp.Queue() # the chessboard vertex locations in the image space
    ids_q = mp.Queue()
    rets_q = mp.Queue()
    board, _ = generate_charuco_board(board_vert_shape, square_size, aruco_size)

    # do camera calibration from chessboard images
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        future_to_file = {pool.submit(find_corners_aruco, file, DEFAULT_ARUCO_DICT) : file for file in image_files}

        for future in concurrent.futures.as_completed(future_to_file):
            file = future_to_file[future]
            try:
                ret, corners, ids, gray = future.result()
            except Exception as e:
                logger.warning(f'file {file} generated an exception: {e}')
            else:
                rets_q.put(ret)
                if ret:
                    objpoints_q.put(BOARD_CORNER_LOCS)
                    res2 = cv2.aruco.interpolateCornersCharuco(
                        corners,
                        ids,
                        gray,
                        board
                    )
                    corners2 = res2[1]
                    # only consider images with substantial portions of board in view
                    if (res2[2] is not None) and len(res2[2]) >= num_found_thresh:
                        ids_q.put(res2[2])
                    # only consider images with substantial number of corners found in view
                    if (corners2 is not None) and len(corners2) >= num_found_thresh:
                        imgpoints_q.put(corners2)
                    if plot:
                        im = cv2.aruco.drawDetectedMarkers(
                            gray,
                            corners,
                            ids
                        )
                        im_with_corners = cv2.drawChessboardCorners(gray, BOARD_VERT_SHAPE, corners2, ret)
                if plot:
                    plt.figure(figsize=(14,24))
                    plt.imshow(im_with_corners)
                    ax = plt.gca()
                    ax.set_aspect('equal')

    # Do the actual camera calibration with all the data we gathered.
    # OpenCV likes lists.
    rets = []
    while not rets_q.empty():
        rets.append(rets_q.get(timeout=0.1))
    if not any(rets):
        raise ValueError('No images had aruco targets for charuco calibration.')

    objpoints = []
    while not objpoints_q.empty():
        objpoints.append(objpoints_q.get(timeout=0.1))
    imgpoints = []
    while not imgpoints_q.empty():
        imgpoints.append(imgpoints_q.get(timeout=0.1))

    # assumption: all images taken with same camera
    h,w = cv2.imread(image_files[0], flags=(cv2.IMREAD_IGNORE_ORIENTATION + cv2.IMREAD_COLOR)).shape[:2]

    calib_ids = []
    while not ids_q.empty():
        calib_ids.append(ids_q.get(timeout=0.1))

    flags = (
        cv2.CALIB_RATIONAL_MODEL + 
        # cv2.CALIB_FIX_ASPECT_RATIO +
        cv2.CALIB_FIX_PRINCIPAL_POINT +
        0
    )
    if guess_intrinsics:
        flags += cv2.CALIB_USE_INTRINSIC_GUESS
        guess_mtx = np.array([
            [CAMERA_FOCAL_LENGTH_GUESS / CAMERA_PIXEL_SIZE_GUESS, 0, CAMERA_PIXEL_SHAPE[0] / 2],
            [0, CAMERA_FOCAL_LENGTH_GUESS / CAMERA_PIXEL_SIZE_GUESS, CAMERA_PIXEL_SHAPE[1] / 2],
            [0, 0, 1],
        ])
    else:
        guess_mtx = None
    ret, mtx, dist, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
        imgpoints,
        calib_ids,
        board,
        gray.shape[::-1],
        guess_mtx,
        None,
        flags=flags
    )

    logger.info(f'RMS reprojection error {ret} px')

    optimal_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        mtx,
        dist, 
        (w,h), 
        1, 
        (w,h)
    )

    if plot:
        for i in range(len(objpoints)):
            fig, ax = plt.subplots(figsize=(12,7))
            
            reprojected_obj_points, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
            reprojected_obj_points = reprojected_obj_points.reshape(-1,2)
            these_imgpoints = imgpoints[i].reshape(-1, 2)

            ax.scatter((reprojected_obj_points)[:,0], (reprojected_obj_points)[:,1], color='k')
            ax.scatter(these_imgpoints[:,0], these_imgpoints[:,1], color='r')
            ax.set_ylim(0, 3024)
            ax.set_xlim(0, 4032)
            ax.set_aspect('equal')
            fig.show()

    return mtx, dist, optimal_camera_matrix, roi


def measure_images(files, camera_matrix, camera_dist, aruco_ids=[], aruco_size=DEFAULT_TARGET_SIZE, plot=False):
    '''
    Determine the pose of specified aruco ids relative to the camera.
    '''
    img_data = {}
    for file in files:
        found, corners, ids, gray = find_corners_aruco(
            file,
            camera_matrix=camera_matrix,
            camera_dist=camera_dist,
            refine_subpixel=True,
            valid_ids=aruco_ids
        )
        if not found or not any(ids):
            continue
        logger.info(f'Found specified ArUco targets {ids}.')

        if plot:
            fig, ax = plt.subplots(subplot_kw={'projection':'3d'})

        img_data[file] = {}

        board_rvec, board_tvec = estimate_pose_charuco(
            file,
            camera_matrix,
            camera_dist,
            plot=plot
        )
        board_rvec_row = board_rvec.T[0]
        board_tvec_row = board_tvec.T[0]

        # the rotation of the board relative to the camera
        Rb = R.from_rotvec(board_rvec_row)

        logger.info(f'Board euler angles: {Rb.as_euler("zyx", degrees=True)} deg')

        for i in range(len(ids)):
            target_rvec, target_tvec, obj_points = estimate_pose_aruco(
                file,
                corners[i],
                aruco_size,
                camera_matrix,
                camera_dist,
                plot=plot
            )
            target_rvec_row = target_rvec[0][0]
            target_tvec_row = target_tvec[0][0]

            # the rotation of the aruco target relative to the camera
            Rt = R.from_rotvec(target_rvec_row)
            # the rotation of the target frame relative to the board frame
            Rtb = Rb.inv() * Rt
            # the offset of the target frame origin from the board origin
            # in camera coordinates
            tb = target_tvec_row - board_tvec_row
            # that offset in the frame of the board
            tb_b = Rb.inv().apply(tb)

            target_rvec_final = Rtb.as_rotvec()
            target_tvec_final = tb_b

            # we are now in a frame where the rotation matrix is the identity
            # and the translation is the origin
            board_rvec_final = (Rb.inv() * Rb).as_rotvec()
            board_tvec_final = board_tvec_row - board_tvec_row

            img_data[file][ids[i]] = {}
            img_data[file][ids[i]]['rvec'] = target_rvec_final
            img_data[file][ids[i]]['tvec'] = target_tvec_final

            img_data[file]['board'] = {}
            img_data[file]['board']['rvec'] = board_rvec_final
            img_data[file]['board']['tvec'] = board_tvec_final

            if plot:
                if i == 0:
                    label_prefix = ''
                else:
                    label_prefix = '_'
                ax.scatter(
                    target_tvec_final[0],
                    target_tvec_final[1],
                    target_tvec_final[2],
                    label=f'Target {ids[i]}'
                )
                ax.scatter(
                    board_tvec_final[0],
                    board_tvec_final[1],
                    board_tvec_final[2],
                    color='k',
                    label=label_prefix + 'ArUco Board Origin'
                )
        if plot:
            ax.legend(loc='best')
    return img_data


def estimate_angular_offset(img_data, file, entity0, entity1):
    '''
    Calculate the minimum angle needed to rotate one coordinate system into the other
    from the axis-angle representations.
    https://math.stackexchange.com/questions/2113634/comparing-two-rotation-matrices
    https://math.stackexchange.com/questions/3999557/how-to-compare-two-rotations-represented-by-axis-angle-rotation-vectors
    '''
    rvec0 = img_data[file][entity0]['rvec']
    rvec1 = img_data[file][entity1]['rvec']
    rmat0, _ = cv2.Rodrigues(rvec0)
    rmat1, _ = cv2.Rodrigues(rvec1)
    AB = rmat0 @ rmat1.T
    theta = np.arccos((np.trace(AB) - 1) / 2)
    return theta


def post_process_scan(img_data: dict, out_dir: str , command_file: str, raft_id, origin_id=None, savefig=True):
    '''
    Assume the target images are each of a different point in a raster scan.
    Compare to the commanded profile.

    Parameters
    ----------
    img_data: dict
        Return dict of `measure_images`.
    out_dir: str
        Location to save plots
    command_file: str
        .csv file that was used to run the scan. Should be from the
        hotspot/data/input/profiles dir.
    raft_id: int
        ArUco ID of a target mounted on the center of the end effector.
    origin_id: int (optional)
        ArUco ID of a target mounted at a known location relative to the origin
        of the mapper coordinate frame.
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
    # add a degenerate z-dir coord for comparison with our data
    commanded_pts = np.concatenate([commanded_pts, np.zeros_like(commanded_pts[:,:,:1])], axis=-1)

    xs = np.unique([pt[0] for pt in commanded_pts_flat])
    ys = np.unique([pt[1] for pt in commanded_pts_flat])
    
    fig, ax = plt.subplots(figsize=(7,5))#subplot_kw={'projection':'3d'})
    fig.suptitle('Commanded vs. Measured Positions', fontsize=18)

    queries = np.zeros_like(commanded_pts)
    residuals = np.zeros_like(commanded_pts)

    # Get the per-image data out of the img_data dict
    # natural sort image data to align with command sequence
    # Assumes images are numbered something like 1.jpg, 2.jpg, 3.jpg...
    img_keys = [file for file in sorted(list(img_data.keys()), key=nsort) if os.path.exists(file)]

    img_keys = [file for file in list(img_data.keys()) if os.path.exists(file)]

    imgs = np.array(img_keys).reshape(commanded_pts.shape[:2])
    for j in range(imgs.shape[0]):
        for k in range(imgs.shape[1]):
            if not isinstance(img_data[imgs[j][k]], dict):
                continue
            if not (raft_id in img_data[imgs[j][k]].keys()):
                continue
            # Measured positions are 3D poses relative to aruco board
            query_tvec = img_data[imgs[j][k]][raft_id]['tvec']
            query_rvec = img_data[imgs[j][k]][raft_id]['rvec']
            board_tvec = img_data[imgs[j][k]]['board']['tvec']
            queries[j][k][0] = query_tvec[0]
            queries[j][k][1] = query_tvec[1]
            queries[j][k][2] = query_tvec[2]

            if origin_id:
                if not (origin_id in img_data[imgs[j][k]].keys()):
                    continue
                origin_tvec = img_data[imgs[j][k]][origin_id]['tvec']
                origin_rvec = img_data[imgs[j][k]][origin_id]['rvec']
                queries[j][k][0] -= origin_tvec[0]
                queries[j][k][1] -= origin_tvec[1]
                queries[j][k][2] -= origin_tvec[2]
            else:
                # No reference given, take the first location as perfectly correct.
                if (j == 0 and k == 0):
                    fudge_offset = queries[j][k] - commanded_pts[j][k]
                queries[j][k] -= fudge_offset

    residuals = queries - commanded_pts

    # error check: ignore ridiculous residuals
    rmse = None
    # if residuals.size > 16: # need good stats
    #     rmse = np.sqrt(np.mean(residuals ** 2.))
    #     print('RMSE:', rmse)
    #     sig_thresh = 3. * rmse
    #     bad_x = np.where(np.abs(residuals[:,:,0]) > sig_thresh)
    #     bad_y = np.where(np.abs(residuals[:,:,1]) > sig_thresh)
    #     queries[bad_x] = commanded_pts[bad_x]
    #     queries[bad_y] = commanded_pts[bad_y]
    #     print(f'{len(bad_x[0])} x-measurements with >3x RMSE ignored (highlighted red).')
    #     print(f'{len(bad_y[0])} y-measurements with >3x RMSE ignored (highlighted red).')
    #     residuals = queries - commanded_pts
    #     rmse = np.sqrt(np.mean(residuals ** 2.))
    # else:
    #     bad_x = None
    #     bad_y = None
    bad_x = []
    bad_y = []

    ax.scatter(
        commanded_pts[:,:,0],
        commanded_pts[:,:,1],
        facecolor='none',
        color='k',
        label=f'Raft Commanded Pos.',
        zorder=1
    )
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
        width=4e-3,
        zorder=2
    )
    ax.scatter(queries[:,:,0], queries[:,:,1], facecolor='k', label='Raft Measured Pos.', zorder=1)
    if any(bad_x):
        ax.scatter(commanded_pts[bad_x][:,0], commanded_pts[bad_x][:,1], facecolor='r', label='Ignored\n(Error >3RMSEs)', zorder=1)
    if any(bad_y):
        ax.scatter(commanded_pts[bad_y][:,0], commanded_pts[bad_y][:,1], facecolor='r', label='Ignored\n(Error >3RMSEs)', zorder=1)
    
    plt.xticks(rotation=45)
    if rmse:
        ax.set_title('Profile:\n' + os.path.basename(command_file) + '\n' +f'RMSE: {rmse * 1000.:.2f} mm')
    else:
        ax.set_title('Profile:\n' + os.path.basename(command_file))
    ax.set_xlabel('x-distance from SW corner (m)')
    ax.set_ylabel('y-distance from SW corner (m)')
    ax.grid(True)
    ax.set_aspect('equal')
    ax.set_facecolor('gainsboro')
    ax.legend(bbox_to_anchor=(1.05, 1.0), loc='upper left')
    plt.subplots_adjust(top=0.9)
    plt.tight_layout()
    t_str = str(time.time())
    if savefig:
        plt.savefig(os.path.join(out_dir, f'quiver_{t_str}.png'), facecolor='white', transparent=False)

    plt.figure()
    ax = plt.axes()
    ax.set_title('Residual Magnitude: Commanded - Measured')
    residuals_mag_mm = np.linalg.norm(residuals[:,:,:2], axis=-1) * 1000.
    levels = np.arange(np.floor(residuals_mag_mm.min()), np.ceil(residuals_mag_mm.max()) + .5, .5)
    X,Y = np.meshgrid(xs, ys)
    im = ax.contourf(X, Y, residuals_mag_mm, levels=levels, vmin=0, vmax=10)
    cbar = plt.colorbar(plt.cm.ScalarMappable(norm=im.norm, cmap=im.cmap), label='Magnitude (mm)')
    ax.set_xticks(xs)
    plt.xticks(rotation=45)
    ax.set_yticks(ys)
    ax.set_xlabel('X-dir Position (m)')
    ax.set_ylabel('Y-dir Position (m)')
    ax.grid(True)
    plt.tight_layout()
    if savefig:
        plt.savefig(os.path.join(out_dir, f'magnitudes_{str(time.time())}.png'), facecolor='white', transparent=False)

    plt.figure()
    ax = plt.axes()
    ax.set_title('Position z-component')
    sns.kdeplot(1e3 * queries[:,:,2].flatten(), fill=True, ax=ax)
    sns.rugplot(1e3 * queries[:,:,2].flatten(), ax=ax)
    ax.set_xlabel('Z-coordinate relative to Charuco board (mm)')
    ax.set_ylabel('Density')
    ax.grid(True)
    if savefig:
        plt.savefig(os.path.join(out_dir, f'zdir_{t_str}.png'), facecolor='white', transparent=False)

    plt.figure()
    ax = plt.axes()
    ax.set_title('Residuals: Commanded - Measured')
    residuals_mm = np.array(residuals) * 1000.
    sns.kdeplot(x=residuals_mm[:,:,0].flatten(), y=residuals_mm[:,:,1].flatten(), fill=True, ax=ax)
    ax.scatter(residuals_mm[:,:,0], residuals_mm[:,:,1], color='k', alpha=0.3)
    ax.set_xlim(-7,7)
    ax.set_ylim(-7,7)
    ax.set_xlabel('X-dir Residuals (mm)')
    ax.set_ylabel('Y-dir Residuals (mm)')
    ax.grid(True)
    ax.set_aspect('equal')
    if savefig:
        plt.savefig(os.path.join(out_dir, f'residuals_{t_str}.png'), facecolor='white', transparent=False)

    plt.figure(figsize=(12,7))
    ax = plt.axes()
    ax.set_title('Residuals vs. Position Num.')
    residuals_ordered_mm = np.ravel(residuals_mag_mm)
    ax.scatter(range(len(residuals_ordered_mm)), residuals_ordered_mm)
    ax.set_ylim(0, 10)
    ax.set_xlabel('Scan Order')
    ax.set_ylabel('Position Error Magnitude (mm)')
    ax.grid(True)
    plt.tight_layout()
    plt.subplots_adjust(top=0.85)
    if savefig:
        plt.savefig(os.path.join(out_dir, f'error_vs_time_{t_str}.png'), facecolor='white', transparent=False)


    plt.figure()
    ax = plt.axes()
    ax.set_title('Residuals vs. z-dir residual')
    residuals_ordered_mm = np.ravel(residuals_mag_mm)
    ax.scatter(residuals_mm[:,:,2].ravel(), residuals_ordered_mm)
    # ax.set_ylim(0, 10)
    ax.set_xlabel('Z-dir residual')
    ax.set_ylabel('Position Error Magnitude (mm)')
    ax.grid(True)
    plt.tight_layout()
    plt.subplots_adjust(top=0.85)

    # Publication-ready figures
    plt.style.use('seaborn-v0_8-paper')

    # from matplotlib
    # plt.figure()
    # ax = plt.axes()
    # ax.set_title('Residuals: Commanded - Measured')
    # residuals_mm = np.array(residuals) * 1000.
    # sns.kdeplot(x=residuals_mm[:,:,0].flatten(), y=residuals_mm[:,:,1].flatten(), fill=True, ax=ax)
    # ax.scatter(residuals_mm[:,:,0], residuals_mm[:,:,1], color='k', alpha=0.3)
    # ax.
    # ax.set_xlim(-7,7)
    # ax.set_ylim(-7,7)
    # ax.set_xlabel('X-dir Residuals (mm)')
    # ax.set_ylabel('Y-dir Residuals (mm)')
    # ax.grid(True)
    # ax.set_aspect('equal')
    # plt.savefig(os.path.join(out_dir, f'pub_residuals_{t_str}.png'), facecolor='white', transparent=False)

    plt.figure()
    ax = plt.axes()
    ax.set_title('Error vs. Position Number')
    residuals_ordered_mm = np.ravel(residuals_mag_mm)
    ax.scatter(range(len(residuals_ordered_mm)), residuals_ordered_mm)
    ax.set_ylim(0, 10)
    ax.set_xlabel('Scan Order')
    ax.set_ylabel('Position Error Magnitude (mm)')
    plt.tight_layout()
    plt.subplots_adjust(top=0.85)
    if savefig:
        plt.savefig(os.path.join(out_dir, f'pub_error_vs_time_{t_str}.png'), facecolor='white', transparent=False)

    return commanded_pts, queries, residuals
