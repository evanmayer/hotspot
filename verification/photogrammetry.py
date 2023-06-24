# Great Value photogrammetry package with OpenCV.

import concurrent
import cv2
import logging
import matplotlib.pyplot as plt
from matplotlib import colors
import multiprocessing as mp
import numpy as np
import os
import pandas as pd
import pickle
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
BOARD_VERT_SHAPE = (14,19)
BOARD_SQUARE_SIZE = 0.01200 # m
BOARD_ARUCO_SIZE = 0.00882 # m
DEFAULT_TARGET_SIZE = 0.02493 # m, this WILL vary from printer to printer and from target to target on the page!
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
    im = cv2.imread(
        file,
        flags=(
            cv2.IMREAD_IGNORE_ORIENTATION +
            cv2.IMREAD_COLOR
        )
    )
    im = np.rot90(im, k=IMG_ROT_NUM) # may not need this, depending on source of images.
    gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    if (camera_matrix is not None) and (camera_dist is not None):
        gray = cv2.undistort(
            gray,
            camera_matrix,
            camera_dist,
        )
    return gray


def find_corners_aruco(file: str, gray=None, aruco_dict=DEFAULT_ARUCO_DICT, refine_subpixel=False, valid_ids=[], camera_matrix=None, camera_dist=None):
    '''
    Thin wrapper for OpenCV detectMarkers
    '''
    gray = load_to_gray(file, camera_matrix=camera_matrix, camera_dist=camera_dist)

    dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict)
    parameters = cv2.aruco.DetectorParameters()
    if refine_subpixel:
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
        # parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        parameters.cornerRefinementWinSize = 4 # default 5 
        parameters.cornerRefinementMaxIterations = 40 # default 30
        parameters.cornerRefinementMinAccuracy = 0.01 # default 0.1
        parameters.minMarkerPerimeterRate = 5e-3
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
            fig, ax = plt.subplots(figsize=(20,15))
        gray = load_to_gray(file, camera_matrix=camera_matrix, camera_dist=camera_dist)
        im_with_axes = cv2.drawFrameAxes(gray, camera_matrix, camera_dist, rvec, tvec, 0.07)
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


def estimate_pose_charuco(file, camera_matrix, camera_dist, gray=None, board_vert_shape=BOARD_VERT_SHAPE, square_size=BOARD_SQUARE_SIZE, aruco_size=BOARD_ARUCO_SIZE, aruco_dict=DEFAULT_ARUCO_DICT, prev_rvec=None, prev_tvec=None, fig=None, ax=None, plot=False):
    '''
    Thin wrapper for OpenCV aruco detectBoard
    '''
    board, _ = generate_charuco_board(board_vert_shape, square_size, aruco_size, aruco_dict=aruco_dict)
    if gray is None:
        gray = load_to_gray(file, camera_matrix=camera_matrix, camera_dist=camera_dist)

    detector = cv2.aruco.CharucoDetector(board)
    params = detector.getDetectorParameters()
    # This allows detection of smaller targets. But...not too small.
    params.minMarkerPerimeterRate = 7e-3 # default 0.03
    # Added this because of a diversity of marker sizes in the
    # scene. It erroneously detected 17 inside the black space of 998 when 17
    # was obscured, throwing off the interpolation of an entire board.
    params.maxErroneousBitsInBorderRate = 0.2 # default 0.35
    # Added this because some FOD in the scene was being detected as markers.
    # especially 17
    params.polygonalApproxAccuracyRate = 0.008 # default 0.03
    detector.setDetectorParameters(params)
    aruco_corners, aruco_ids, marker_corners, marker_ids = detector.detectBoard(gray)
    charuco_ret, aruco_corners, aruco_ids = cv2.aruco.interpolateCornersCharuco(marker_corners, marker_ids, gray, board)

    logger.info(f'Found {len(aruco_ids)} aruco markers on board.')

    if prev_rvec is not None and prev_tvec is not None:
        ret, raw_rvec, raw_tvec = cv2.aruco.estimatePoseCharucoBoard(
            aruco_corners,
            aruco_ids,
            board,
            camera_matrix,
            camera_dist,
            prev_rvec,
            prev_tvec,
            useExtrinsicGuess=True
        )
    else:
        ret, raw_rvec, raw_tvec = cv2.aruco.estimatePoseCharucoBoard(
            aruco_corners,
            aruco_ids,
            board,
            camera_matrix,
            camera_dist,
            None,
            None
        )

    if not ret:
        logger.critical('failed to find charuco pose')
    # Board z-axis points away from the camera, but we want it pointing
    # toward the camera to align the board axes with the mapper coordinate
    # frame
    board_rvec_row = raw_rvec.T[0]
    Rb = R.from_rotvec(board_rvec_row)
    Rb *= R.from_euler('zyx', [-np.pi / 2., 0.0, np.pi])
    rvec = np.array([Rb.as_rotvec()]).T # Repackage like it came out of pose estimator
    tvec = raw_tvec

    if plot:
        if (not fig or not ax):
            fig, ax = plt.subplots(figsize=(20,15))
        im_with_charuco_board = cv2.drawFrameAxes(gray, camera_matrix, camera_dist, rvec, tvec, 0.3)
        ax.imshow(im_with_charuco_board, cmap='bone')
        marker_sqz = np.squeeze(marker_corners)
        for i, id in enumerate(np.squeeze(marker_ids)):
            ax.scatter(marker_sqz[i][:,0], marker_sqz[i][:,1], s=5, color='m')
            ax.text(marker_sqz[i][0,0], marker_sqz[i][0,1], str(id), color='m')
        ax.scatter(np.squeeze(aruco_corners)[:,0], np.squeeze(aruco_corners)[:,1], s=5, color='r')

    return rvec, tvec, raw_rvec, raw_tvec


def calibrate_camera(cal_path: str, image_files: list, board_vert_shape=BOARD_VERT_SHAPE, square_size=BOARD_SQUARE_SIZE, aruco_size=BOARD_ARUCO_SIZE, guess_intrinsics=False, plot=False, savefig=False):
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
    cal_path
        path to calibration data (for saving figures)
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
    theoretical_num_verts = np.multiply(*board_vert_shape)
    num_found_thresh = 0.95 * theoretical_num_verts#0.98 * theoretical_num_verts

    objpoints_q = mp.Queue() # the chessboard vertex locations in the plane of the board
    imgpoints_q = mp.Queue() # the chessboard vertex locations in the image space
    ids_q = mp.Queue()
    rets_q = mp.Queue()
    board, _ = generate_charuco_board(board_vert_shape, square_size, aruco_size)

    # do camera calibration from chessboard images
    with concurrent.futures.ProcessPoolExecutor(max_workers=4) as pool:
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
                    res2 = cv2.aruco.interpolateCornersCharuco(
                        corners,
                        ids,
                        gray,
                        board
                    )
                    corners2 = res2[1]
                    # only consider images with substantial number of corners found in view
                    if corners2 is None:
                        logger.warning(f'file {file} had no corners found.')
                    elif (corners2 is not None) and len(corners2) >= num_found_thresh:
                        ids_q.put(res2[2])
                        imgpoints_q.put(corners2)
                        objpoints_q.put(BOARD_CORNER_LOCS)
                    else:
                        logger.warning(f'file {file} failed threshold check for number of corners found: {len(corners2)} < {num_found_thresh}')
                #     if plot:
                #         im = cv2.aruco.drawDetectedMarkers(
                #             gray,
                #             corners,
                #             ids
                #         )
                #         im_with_corners = cv2.drawChessboardCorners(gray, BOARD_VERT_SHAPE, corners2, ret)
                # if plot:
                #     plt.figure(figsize=(14,24))
                #     plt.imshow(im_with_corners)
                #     ax = plt.gca()
                #     ax.set_aspect('equal')

    # Do the actual camera calibration with all the data we gathered.
    # OpenCV likes lists.
    timeout = 5
    time.sleep(timeout)
    rets = []
    while not rets_q.empty():
        rets.append(rets_q.get(timeout=timeout))
    if not any(rets):
        raise ValueError('No images had aruco targets for charuco calibration.')
    objpoints = []
    while not objpoints_q.empty():
        objpoints.append(objpoints_q.get(timeout=timeout))
    imgpoints = []
    counter = 0
    sz = imgpoints_q.qsize()
    while not imgpoints_q.empty() and counter < sz:
        imgpoints.append(imgpoints_q.get(timeout=timeout))
        counter += 1

    # assumption: all images taken with same camera
    h,w = cv2.imread(image_files[0], flags=(cv2.IMREAD_IGNORE_ORIENTATION + cv2.IMREAD_COLOR)).shape[:2]

    calib_ids = []
    while not ids_q.empty():
        calib_ids.append(ids_q.get(timeout=timeout))

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

    print(len(imgpoints), len(calib_ids))
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
        residuals_x = []
        residuals_y = []
        fig_overall, ax_overall = plt.subplots(figsize=(12,7))
        for i in range(len(objpoints)):
            reprojected_obj_points, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
            reprojected_obj_points = reprojected_obj_points.reshape(-1,2)
            these_imgpoints = imgpoints[i].reshape(-1, 2)

            # wayyy too hard to deal with ragged imgpoints from charuco
            if reprojected_obj_points.shape[0] == these_imgpoints.shape[0]:
                residuals_x += list(reprojected_obj_points[:these_imgpoints.shape[0],0] - these_imgpoints[:,0])
                residuals_y += list(reprojected_obj_points[:these_imgpoints.shape[0],1] - these_imgpoints[:,1])

                # fig, ax = plt.subplots(figsize=(12,7))
                # ax.scatter(
                #    reprojected_obj_points[:these_imgpoints.shape[0],0] - these_imgpoints[:,0],
                #     reprojected_obj_points[:these_imgpoints.shape[0],1] - these_imgpoints[:,1],
                #     s=10, color='r', alpha=0.2
                # )
                # ax.scatter((reprojected_obj_points)[:,0], (reprojected_obj_points)[:,1], s=5, color='k')
                # ax.scatter(these_imgpoints[:,0], these_imgpoints[:,1], s=5, color='r')
                # ax.set_ylim(0, 3024)
                # ax.set_xlim(0, 4032)
                # ax.set_aspect('equal')
                # fig.show()

            ax_overall.scatter(these_imgpoints[:,0], these_imgpoints[:,1], s=5, color='b', alpha=0.2)

        ax_overall.set_ylim(0, h)
        ax_overall.set_xlim(0, w)
        ax_overall.set_ylabel('Pixels')
        ax_overall.set_xlabel('Pixels')
        ax_overall.set_aspect('equal')
        ax_overall.set_title('Overall sensor coverage')
        fig_overall.show()

        fig_residuals, ax_residuals = plt.subplots(figsize=(12,7))
        sns.kdeplot(x=residuals_x, y=residuals_y, fill=True, ax=ax_residuals)
        ax_residuals.scatter(residuals_x, residuals_y, s=1, color='k', alpha=0.2)
        ax_residuals.set_title(f'Reprojection Error Residuals (N={len(residuals_x)})')
        ax_residuals.set_xlabel('X Error (px)')
        ax_residuals.set_ylabel('Y Error (px)')
        ax_residuals.set_aspect('equal')

        if savefig:
            timestamp = time.time()
            fig_overall.savefig(os.path.join(cal_path, f'sensor_coverage_{timestamp}.png'), facecolor='white', transparent=False)
            fig_residuals.savefig(os.path.join(cal_path, f'reprojection_error_residuals_{timestamp}.png'), facecolor='white', transparent=False)

    return mtx, dist, optimal_camera_matrix, roi


def measure_images(files, camera_matrix, camera_dist, aruco_ids=[], aruco_size=DEFAULT_TARGET_SIZE, use_extrinsic_priors=True, plot=False):
    '''
    Determine the pose of specified aruco ids relative to the camera.
    '''
    img_data = {}
    prev_rvec = None
    prev_tvec = None
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

        board_rvec, board_tvec, raw_board_rvec, raw_board_tvec = estimate_pose_charuco(
            file,
            camera_matrix,
            camera_dist,
            gray=gray,
            prev_rvec=prev_rvec,
            prev_tvec=prev_tvec,
            plot=plot
        )
        if use_extrinsic_priors:
            # Use previous board pose as a guess at the next identified pose.
            # This is valid because we never expect the board to move relative
            # to the camera in this situation.
            prev_rvec = raw_board_rvec
            prev_tvec = raw_board_tvec
            # Refer to the first estimate.
            use_extrinsic_priors = False

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
            img_data[file][ids[i]]['tvec_rel_camera'] = target_tvec_row

            img_data[file]['board'] = {}
            img_data[file]['board']['euler_zyx_deg'] = Rb.as_euler("zyx", degrees=True)
            img_data[file]['board']['tvec_rel_camera'] = board_tvec_row
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


def post_process_scan(img_data: dict, out_dir: str , command_file: str, raft_id: int, origin_id=None, timestamp=''):
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
    if not timestamp:
        timestamp = str(time.time())
    with open(os.path.join(out_dir, f'{timestamp}_img_data.pickle'), 'wb') as f:
        pickle.dump(img_data, f, protocol=pickle.HIGHEST_PROTOCOL)

    # Reshape command profile assuming a square box is scanned.
    commanded_pts_flat = np.genfromtxt(command_file, delimiter=',', skip_header=1, usecols=[-2,-1])
    assert int(np.sqrt(commanded_pts_flat.shape[0])) == np.round(np.sqrt(commanded_pts_flat.shape[0])), f'Command profile shape {commanded_pts_flat.shape[0]} not square. Rewrite alg to handle non-square shapes.'
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

    measured_pts = np.zeros_like(commanded_pts)
    camera_dist = np.zeros_like(commanded_pts[:,:,0])

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
            camera_dist[j][k] = np.linalg.norm(img_data[imgs[j][k]][raft_id]['tvec_rel_camera'])
            measured_pts[j][k][0] = query_tvec[0]
            measured_pts[j][k][1] = query_tvec[1]
            measured_pts[j][k][2] = query_tvec[2]

            if origin_id:
                if not (origin_id in img_data[imgs[j][k]].keys()):
                    continue
                origin_tvec = img_data[imgs[j][k]][origin_id]['tvec']
                origin_rvec = img_data[imgs[j][k]][origin_id]['rvec']
                measured_pts[j][k][0] -= origin_tvec[0]
                measured_pts[j][k][1] -= origin_tvec[1]
                measured_pts[j][k][2] -= origin_tvec[2]
            else:
                # No reference given, take the first location as perfectly correct.
                if (j == 0 and k == 0):
                    fudge_offset = measured_pts[j][k] - commanded_pts[j][k]
                measured_pts[j][k] -= fudge_offset
    return xs, ys, commanded_pts, measured_pts, camera_dist


def calc_rmse(commanded_pts, measured_pts):
    residuals = measured_pts - commanded_pts
    mags = np.linalg.norm(residuals[:,:,:2], axis=-1)
    mags_flat = np.ravel(mags)
    rmse = np.linalg.norm(mags_flat) / np.sqrt(len(mags_flat))
    return rmse


def filter_by_rmse(commanded_pts, measured_pts, n_sigma):
    residuals = measured_pts - commanded_pts
    # error check: ignore ridiculous residuals
    rmse = None
    if residuals.size >= 16: # need good stats
        rmse = calc_rmse(commanded_pts, measured_pts)
        logger.info(f'RMSE: {rmse}')
        sig_thresh = n_sigma * rmse
        bad_x = np.where(np.abs(residuals[:,:,0]) > sig_thresh)
        bad_y = np.where(np.abs(residuals[:,:,1]) > sig_thresh)
        measured_pts[bad_x] = commanded_pts[bad_x]
        measured_pts[bad_y] = commanded_pts[bad_y]
        logger.info(f'{len(bad_x[0])} x-measurements with >3x RMSE ignored.')
        logger.info(f'{len(bad_y[0])} y-measurements with >3x RMSE ignored.')
    else:
        bad_x = []
        bad_y = []
    return measured_pts, bad_x, bad_y


def make_plots(out_dir, xs, ys, commanded_pts, measured_pts, camera_dist, filter_results=False, savefig=False, timestamp=''):

    requirement = 11.2 * 1e-3
    goal = requirement / 2

    if not timestamp:
        timestamp = str(time.time())
    if filter_results:
        n_sigma = 6
        measured_pts, bad_x, bad_y = filter_by_rmse(commanded_pts, measured_pts, n_sigma)
    else:
        bad_x = []
        bad_y = []

    rmse = calc_rmse(commanded_pts, measured_pts)
    residuals = measured_pts - commanded_pts
    residuals_mm = residuals * 1e3
    residuals_mag = np.linalg.norm(residuals[:,:,:2], axis=-1)
    residuals_mag_mm = residuals_mag * 1e3

    cm = plt.cm.ScalarMappable(
        norm=colors.Normalize(
            vmin=0,
            vmax=np.max(residuals_mag),
        ),
        cmap='viridis'
    )
    cs = cm.to_rgba(np.sort(residuals_mag.ravel()))
    viridis_lower = cs[len(cs) // 4]
    viridis_mid = cs[len(cs) // 2]
    viridis_upper = cs[3 * len(cs) // 4]

    # -------------------------------------------------------------------------
    # Quiver plot
    # -------------------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(7,5))
    # fig.suptitle('Commanded vs. Measured Positions', fontsize=18)
    ax.scatter(
        commanded_pts[:,:,0],
        commanded_pts[:,:,1],
        facecolor='none',
        color='k',
        label=f'Commanded Position',
        zorder=1
    )
    ax.scatter(
        measured_pts[:,:,0] + residuals[:,:,0],
        measured_pts[:,:,1] + residuals[:,:,1],
        facecolor='k',
        label='Measured Position',
        zorder=1
    )
    exag = 10
    ax.quiver(
        measured_pts[:,:,0] + residuals[:,:,0] * exag,
        measured_pts[:,:,1] + residuals[:,:,1] * exag,
        residuals[:,:,0] * exag,
        residuals[:,:,1] * exag,
        residuals_mag,
        pivot='tip',
        angles='xy',
        scale_units='xy',
        scale=1,
        headwidth=2.5,
        headlength=4,
        color=viridis_lower,
        edgecolors='k',
        linewidth=.5,
        width=4e-3,
        zorder=2,
        label='10x Errors'
    )
    if any(bad_x):
        ax.scatter(commanded_pts[bad_x][:,0], commanded_pts[bad_x][:,1], facecolor='lightcoral', label=f'Outlier ignored\n(Error >{n_sigma} RMSEs)', zorder=1)
    if any(bad_y):
        if any(bad_x):
            label = '_'
        else:
            label = f'Ignored\n(Error >{n_sigma} RMSEs)'
        ax.scatter(commanded_pts[bad_y][:,0], commanded_pts[bad_y][:,1], facecolor='lightcoral', label=label, zorder=1)
    plt.xticks(rotation=45, fontsize=14)
    plt.yticks(rotation=45, fontsize=14)
    # if rmse:
    #     ax.set_title('Profile:\n' + os.path.basename(command_file) + '\n' +f'RMSE: {rmse * 1000.:.2f} mm')
    # else:
    #     ax.set_title('Profile:\n' + os.path.basename(command_file))
    ax.set_xlim(0, 24 * 0.0254)
    ax.set_ylim(0, 24 * 0.0254)
    ax.set_xlabel('X Position (m)', fontsize=14)
    ax.set_ylabel('Y Position (m)', fontsize=14)
    # ax.grid(True)
    ax.set_aspect('equal')
    # ax.set_facecolor('gainsboro')
    ax.legend(bbox_to_anchor=(1.05, 1.0), loc='upper left')
    plt.subplots_adjust(top=0.9)
    plt.tight_layout()
    if savefig:
        plt.savefig(os.path.join(out_dir, f'{timestamp}_quiver.png'), facecolor='white', transparent=False)


    # -------------------------------------------------------------------------
    # Contour plot
    # -------------------------------------------------------------------------
    plt.figure()
    ax = plt.axes()
    # ax.set_title('Residual Magnitude: Commanded - Measured')
    levels = np.arange(np.floor(residuals_mag_mm.min()), np.ceil(residuals_mag_mm.max()) + .1, .1)
    X,Y = np.meshgrid(xs, ys)
    im = ax.contourf(X, Y, residuals_mag_mm, levels=levels, cmap='viridis')#, vmin=0, vmax=10)
    contour_line_levels = [goal * 1e3, requirement * 1e3]
    contour_line_colors = ['k', 'k']
    contour_line_styles = ['-', '-']
    plt.contour(im, levels=contour_line_levels, colors=contour_line_colors, linestyles=contour_line_styles, linewidths=(1,))
    cbar = plt.colorbar(plt.cm.ScalarMappable(norm=im.norm, cmap=im.cmap), label='X-Y Error Magnitude (mm)')
    [cbar.ax.axhline(level, color='k', linestyle='-') for level in contour_line_levels]
    ax.set_xlim(0, 24 * 0.0254)
    ax.set_ylim(0, 24 * 0.0254)
    # ax.set_xticks(xs)
    # ax.set_yticks(ys)
    plt.xticks(rotation=45, fontsize=11)
    plt.yticks(rotation=45, fontsize=11)
    ax.set_xlabel('X position (m)', fontsize=14)
    ax.set_ylabel('Y position (m)', fontsize=14)
    ax.set_aspect('equal')
    # ax.grid(True)
    plt.tight_layout()
    if savefig:
        plt.savefig(os.path.join(out_dir, f'{timestamp}_magnitudes.png'), facecolor='white', transparent=False)

    # -------------------------------------------------------------------------
    # Corner plot
    # -------------------------------------------------------------------------
    residuals_df = pd.DataFrame({
        'X (mm)': residuals[:,:,0].ravel() * 1e3,
        'Y (mm)': residuals[:,:,1].ravel() * 1e3,
        'Z (mm)': residuals[:,:,2].ravel() * 1e3,
    })
    sns.pairplot(
        residuals_df,
        plot_kws={'color':viridis_lower, 'alpha':0.6},
        diag_kws={'color':viridis_lower, 'alpha':0.6},
        kind='scatter',
        diag_kind='kde',
        corner=True,
    )
    ax = plt.gca()
    fig = plt.gcf()
    fig.suptitle('Residuals')
    plt.tight_layout()
    if savefig:
        plt.savefig(os.path.join(out_dir, f'{timestamp}_corner.png'), facecolor='white', transparent=False)

    # plt.figure()
    # ax = plt.axes()
    # ax.set_title('Position z-component')
    # sns.kdeplot(1e3 * measured_pts[:,:,2].flatten(), fill=True, ax=ax)
    # sns.rugplot(1e3 * measured_pts[:,:,2].flatten(), ax=ax)
    # ax.set_xlabel('Z-coordinate relative to Charuco board (mm)')
    # ax.set_ylabel('Density')
    # ax.grid(True)
    # if savefig:
    #     plt.savefig(os.path.join(out_dir, f'{timestamp}_zdir.png'), facecolor='white', transparent=False)


    # -------------------------------------------------------------------------
    # X-Y KDE plot
    # -------------------------------------------------------------------------
    # plt.figure()
    # ax = plt.axes()
    # ax.set_title('Residuals: Commanded - Measured')
    # sns.kdeplot(x=residuals_mm[:,:,0].flatten(), y=residuals_mm[:,:,1].flatten(), fill=True, ax=ax)
    # ax.scatter(residuals_mm[:,:,0], residuals_mm[:,:,1], color='k', alpha=0.3)
    # ax.set_xlim(-7,7)
    # ax.set_ylim(-7,7)
    # ax.set_xlabel('X-dir Residuals (mm)')
    # ax.set_ylabel('Y-dir Residuals (mm)')
    # ax.grid(True)
    # ax.set_aspect('equal')
    # if savefig:
    #     plt.savefig(os.path.join(out_dir, f'{timestamp}_residuals.png'), facecolor='white', transparent=False)


    # -------------------------------------------------------------------------
    # Error over time plot
    # -------------------------------------------------------------------------
    residuals_ordered_mm = np.ravel(residuals_mag_mm)

    plt.figure(figsize=(7,5))
    ax = plt.axes()
    # ax.set_title('Residuals vs. Position Num.')
    ax.scatter(range(len(residuals_ordered_mm)), residuals_ordered_mm, color=viridis_lower)
    ax.axhline(requirement * 1e3, color=viridis_lower, linestyle='-', label='Requirement')
    ax.axhline(goal * 1e3, color=viridis_mid, linestyle='--', label='Goal')
    ax.legend(loc='best', fontsize=14)
    # ax.set_ylim(0, 10)
    ax.set_xlabel('Scan Order', fontsize=14)
    ax.set_ylabel('X-Y Plane Error Magnitude (mm)', fontsize=14)
    plt.xticks(fontsize=12)
    plt.yticks(fontsize=12)
    # ax.grid(True)
    plt.tight_layout()
    plt.subplots_adjust(top=0.85)
    if savefig:
        plt.savefig(os.path.join(out_dir, f'{timestamp}_error_vs_time.png'), facecolor='white', transparent=False)


    # -------------------------------------------------------------------------
    # error vs. camera distance plot
    # -------------------------------------------------------------------------
    tvecs_mag_mm = camera_dist * 1000.
    tvecs_ordered_mm = np.ravel(tvecs_mag_mm)

    plt.figure(figsize=(12,7))
    ax = plt.axes()
    # ax.set_title('Residuals vs. Distance from Camera')
    ax.scatter(tvecs_ordered_mm, residuals_ordered_mm, color=viridis_lower)
    ax.set_ylim(0, 10)
    ax.set_xlabel('Distance from Camera Center (mm)', fontsize=14)
    ax.set_ylabel('X-Y Plane Error Magnitude (mm)', fontsize=14)
    plt.xticks(fontsize=12)
    plt.yticks(fontsize=12)
    # ax.set_aspect('equal')
    plt.tight_layout()
    plt.subplots_adjust(top=0.85)
    if savefig:
        plt.savefig(os.path.join(out_dir, f'{timestamp}_error_vs_cam_dist.png'), facecolor='white', transparent=False)

    return


def post_process_repeatability(img_data: dict, out_dir: str , command_file: str, raft_id, origin_id=None, savefig=False, timestamp=''):
    '''
    Assume the target images are part of a repeatability measurement, moving
    between two points over and over.

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
    if not timestamp:
        timestamp = str(time.time())
    if savefig:
        with open(os.path.join(out_dir, f'{timestamp}_repeatability_img_data.pickle'), 'wb') as f:
            pickle.dump(img_data, f, protocol=pickle.HIGHEST_PROTOCOL)

    commanded_pts_flat = np.genfromtxt(command_file, delimiter=',', skip_header=1, usecols=[-2,-1])[2::2]
    # add a degenerate z-dir coord for comparison with our data
    commanded_pts = np.concatenate([commanded_pts_flat, np.zeros_like(commanded_pts_flat[:,:1])], axis=-1)

    measured_pts = np.zeros_like(commanded_pts)
    residuals = np.zeros_like(commanded_pts)

    img_keys = [file for file in list(img_data.keys()) if os.path.exists(file)]
    try:
        imgs = np.array(img_keys).reshape(commanded_pts.shape[:1])
    except ValueError:
        logger.warning(
            (f'Number of images provided ({len(img_keys)}) doesn\'t match ' +
             f'commanded profile ({commanded_pts.shape[:1][0]}). Don\'t trust results.')
        )
        imgs = np.array(img_keys).reshape(commanded_pts[:len(img_keys)].shape[:1])
    for j in range(imgs.shape[0]):
        if not isinstance(img_data[imgs[j]], dict):
            continue
        if not (raft_id in img_data[imgs[j]].keys()):
            continue
        # Measured positions are 3D poses relative to aruco board
        query_tvec = img_data[imgs[j]][raft_id]['tvec']
        query_rvec = img_data[imgs[j]][raft_id]['rvec']
        board_tvec = img_data[imgs[j]]['board']['tvec']
        measured_pts[j][0] = query_tvec[0]
        measured_pts[j][1] = query_tvec[1]
        measured_pts[j][2] = query_tvec[2]

        if origin_id:
            if not (origin_id in img_data[imgs[j]].keys()):
                logger.warning(f'No data for id {origin_id} found in img {imgs[j]}. Skipping...')
                continue
            origin_tvec = img_data[imgs[j]][origin_id]['tvec']
            origin_rvec = img_data[imgs[j]][origin_id]['rvec']
            measured_pts[j][0] -= origin_tvec[0]
            measured_pts[j][1] -= origin_tvec[1]
            measured_pts[j][2] -= origin_tvec[2]
        else:
            # No reference given, take the first location as perfectly correct.
            if j == 0:
                fudge_offset = measured_pts[j] - commanded_pts[j]
            measured_pts[j] -= fudge_offset
    return commanded_pts, measured_pts


def make_repeatability_plots(out_dir, commanded_pts, measured_pts, savefig=False, timestamp=''):
    if not timestamp:
        timestamp = str(time.time())

    residuals = measured_pts - commanded_pts
    residuals_mm = residuals * 1e3
    residuals_mag = np.linalg.norm(residuals[:,:2], axis=-1)
    residuals_mag_mm = residuals_mag * 1e3

    cm = plt.cm.ScalarMappable(
        norm=colors.Normalize(
            vmin=0,
            vmax=np.max(residuals_mag),
        ),
        cmap='viridis'
    )

    cs = cm.to_rgba(np.sort(residuals_mag.ravel()))
    viridis_lower = cs[len(cs) // 4]
    viridis_mid = cs[len(cs) // 2]
    viridis_upper = cs[3 * len(cs) // 4]

    # -------------------------------------------------------------------------
    # Quiver plot
    # -------------------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(7,5))
    # fig.suptitle('Commanded vs. Measured Positions', fontsize=18)
    ax.scatter(
        commanded_pts[:,0],
        commanded_pts[:,1],
        facecolor='none',
        color='k',
        label=f'Commanded Position',
        zorder=1
    )
    ax.scatter(
        measured_pts[:,0] + residuals[:,0],
        measured_pts[:,1] + residuals[:,1],
        facecolor=viridis_lower,
        label='Measured Position',
        zorder=1
    )
    exag = 1
    # ax.quiver(
    #     measured_pts[:,0] + residuals[:,0] * exag,
    #     measured_pts[:,1] + residuals[:,1] * exag,
    #     residuals[:,0] * exag,
    #     residuals[:,1] * exag,
    #     residuals_mag,
    #     pivot='tip',
    #     angles='xy',
    #     scale_units='xy',
    #     scale=1,
    #     headwidth=2.5,
    #     headlength=4,
    #     color=viridis_lower,
    #     edgecolors='k',
    #     linewidth=.5,
    #     width=4e-3,
    #     zorder=2,
    #     label=f'{int(exag)}x Errors'
    # )
    plt.xticks(rotation=45, fontsize=14)
    plt.yticks(rotation=45, fontsize=14)
    # ax.set_title('Profile:\n' + os.path.basename(command_file))
    ax.set_xlabel('X Position (m)', fontsize=14)
    ax.set_ylabel('Y Position (m)', fontsize=14)
    # ax.grid(True)
    ax.set_aspect('equal')
    # ax.set_facecolor('gainsboro')
    ax.legend(bbox_to_anchor=(1.05, 1.0), loc='upper left')
    plt.subplots_adjust(top=0.9)
    plt.tight_layout()
    if savefig:
        plt.savefig(os.path.join(out_dir, f'{timestamp}_repeatability_quiver.png'), facecolor='white', transparent=False)


    # -------------------------------------------------------------------------
    # Corner plot
    # -------------------------------------------------------------------------
    residuals_df = pd.DataFrame({
        'X (mm)': residuals[:,0].ravel() * 1e3,
        'Y (mm)': residuals[:,1].ravel() * 1e3,
        'Z (mm)': residuals[:,2].ravel() * 1e3,
    })
    sns.pairplot(
        residuals_df,
        plot_kws={'color':viridis_lower, 'alpha':0.6},
        diag_kws={'color':viridis_lower, 'alpha':0.6},
        kind='scatter',
        diag_kind='kde',
        corner=True,
    )
    ax = plt.gca()
    fig = plt.gcf()
    fig.suptitle('Residuals')
    plt.tight_layout()
    if savefig:
        plt.savefig(os.path.join(out_dir, f'{timestamp}_repeatability_corner.png'), facecolor='white', transparent=False)


    # -------------------------------------------------------------------------
    # X-Y KDE plot
    # -------------------------------------------------------------------------
    # plt.figure()
    # ax = plt.axes()
    # ax.set_title('Residuals: Commanded - Measured')
    # sns.kdeplot(x=residuals_mm[:,0].flatten(), y=residuals_mm[:,1].flatten(), fill=True, ax=ax)
    # ax.scatter(residuals_mm[:,0], residuals_mm[:,1], color='k', alpha=0.3)
    # ax.set_xlim(-4,4)
    # ax.set_ylim(-4,4)
    # ax.set_xlabel('X-dir Residuals (mm)')
    # ax.set_ylabel('Y-dir Residuals (mm)')
    # ax.grid(True)
    # ax.set_aspect('equal')
    # if savefig:
    #     plt.savefig(os.path.join(out_dir, f'{timestamp}_repeatability_residuals.png'), facecolor='white', transparent=False)


    # -------------------------------------------------------------------------
    # Error over time plot
    # -------------------------------------------------------------------------
    residuals_ordered_mm = np.ravel(residuals_mag_mm)

    plt.figure(figsize=(12,7))
    ax = plt.axes()
    # ax.set_title('Residuals vs. Position Num.')
    ax.scatter(range(len(residuals_ordered_mm)), residuals_ordered_mm, color=viridis_lower)
    # ax.set_ylim(0, 10)
    ax.set_xlabel('Scan Order', fontsize=14)
    ax.set_ylabel('Position Error Magnitude (mm)', fontsize=14)
    # ax.grid(True)
    plt.tight_layout()
    plt.subplots_adjust(top=0.85)
    if savefig:
        plt.savefig(os.path.join(out_dir, f'{timestamp}_repeatability_error_vs_time.png'), facecolor='white', transparent=False)

    return
