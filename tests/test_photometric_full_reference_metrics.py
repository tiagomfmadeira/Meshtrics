import argparse
import copy
import os
import shutil

import cv2
import numpy as np
import open3d as o3d
import pye57
import trimesh

from Meshtrics.photometric_metrics import sim_view_point, solve_pnp, compare_images

color_map = [(0, 180, 255),
             (237, 166, 0),
             (29, 81, 246),
             (0, 184, 127),
             (84, 44, 13)]


def click_event(event, x, y, flags, param):
    global img
    global cache
    if event == cv2.EVENT_LBUTTONDOWN:
        # store image in cache before drawing
        cache.append(copy.deepcopy(img))
        print("[OpenCV INFO] Picked point (" + str(x) + ", " + str(y) + ") to add in queue.")
        photo_selected_points.append([x, y])
        font = cv2.FONT_HERSHEY_SIMPLEX
        # strXY = "(" + str(x) + ", " + str(y) + ")"
        strXY = " (" + str(len(photo_selected_points)) + ")"
        curr_color = (len(photo_selected_points) - 1) % len(color_map)
        cv2.circle(img, (x, y), 20, color_map[curr_color], -1)
        cv2.putText(img, strXY, (x, y), font, 2.3, color_map[curr_color], 5)
        cv2.imshow("photo", img)
        curr_color = (curr_color + 1) % len(color_map)

    if event == cv2.EVENT_RBUTTONDOWN:
        if len(cache) > 0:
            img = cache.pop()
            cv2.imshow("photo", img)
            point = photo_selected_points.pop()
            print("[OpenCV INFO] Removed picked point (" + str(point[0]) + ", " + str(point[1]) + ") from pick queue.")


def pick_points(pcd):
    print("\n1) Please pick at least 3 points using [shift + left click].")
    print("   Press [shift + right click] to undo point picking.")
    print("2) After picking points, press q to close the window.\n")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()
    return vis.get_picked_points()


def add_arguments(ap):
    ap.add_argument("-photos", "--input_photos_directory",
                    help="Path to a directory containing the photos to be used as ground-truth", required=True)
    ap.add_argument("-pext", "--photo_extention", help="Extension of the photo files to be read from the directory",
                    required=True)
    ap.add_argument("-e57", "--input_e57", help="Path to .e57 input file")
    ap.add_argument("-K", "--intrinsics", help="Path to intrinsics input file")
    ap.add_argument("-mesh", "--input_mesh", help="Path to mesh input file. Can be .ply, .obj, etc.", required=True)
    ap.add_argument("-o", "--output_path", help="Path to save output files. Defaults to . ", default=".")
    ap.add_argument("-show", "--show_visual_feedback", help="Show visual feedback of the operations",
                    action='store_true', default=False)
    return ap


if __name__ == "__main__":

    ap = argparse.ArgumentParser()
    add_arguments(ap)
    args = vars(ap.parse_args())
    print("\n" + str(args))

    if args['output_path']:
        if os.path.exists(args['output_path'] + '/fullref_output'):
            shutil.rmtree(args['output_path'] + '/fullref_output')
        os.makedirs(args['output_path'] + '/fullref_output')

    print("\nLoading input mesh...")
    mesh = trimesh.load(args['input_mesh'])

    o3d_mesh = o3d.io.read_triangle_mesh(args['input_mesh'])
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d_mesh.vertices
    pcd.colors = o3d_mesh.vertex_colors
    pcd.normals = o3d_mesh.vertex_normals

    if args['input_e57']:
        # get intrinsic matrix from e57 (can be replaced with camera calibration)
        e57 = pye57.E57(args['input_e57'])
        imf = e57.image_file
        root = imf.root()
        image2D = root['images2D'][0]
        pinhole = image2D['pinholeRepresentation']
        focal_length = pinhole['focalLength'].value()
        pixel_height = pinhole['pixelHeight'].value()
        pixel_width = pinhole['pixelWidth'].value()
        principal_point_x = pinhole['principalPointX'].value()
        principal_point_y = pinhole['principalPointY'].value()
        K = np.zeros((3, 3))
        K[2, 2] = 1
        K[0, 0] = focal_length / pixel_width
        K[1, 1] = focal_length / pixel_height
        K[0, 2] = principal_point_x
        K[1, 2] = principal_point_y
        print(K)
    elif args['intrinsics']:
        with open(args['intrinsics'], 'r') as f:
            K = np.array([[float(num) for num in line.split(' ')] for line in f])[:3, :3]
    else:
        print('Please provide camera intrinsics! Exiting...')

    print("\nLoading photos...")
    for file_idx, file in enumerate(os.listdir(args['input_photos_directory'])):
        if file.endswith(args['photo_extention']):
            img_path = os.path.join(args['input_photos_directory'], file)
            while True:
                img = cv2.imread(img_path)
                original_photo = copy.deepcopy(img)
                cache = []
                image_height, image_width, channels = img.shape

                photo_selected_points = []

                cv2.namedWindow("photo", cv2.WINDOW_NORMAL)
                cv2.imshow("photo", img)
                cv2.setMouseCallback("photo", click_event)

                model_selected_points_idx = pick_points(pcd)
                model_selected_points = [pcd.points[idx].tolist() for idx in model_selected_points_idx]

                cv2.destroyAllWindows()

                if not (model_selected_points_idx or photo_selected_points):
                    print("\nNo points selected! Exiting...")
                    exit()
                elif len(model_selected_points_idx) != len(photo_selected_points):
                    print("\nMismatch between 2D and 3D picked points... Please reselect!")
                    continue
                elif len(model_selected_points_idx) < 4:
                    print("\nPlease select at least 4 matching point pairs!")
                    continue
                else:
                    model_selected_points = np.array(model_selected_points).reshape(len(model_selected_points), -1, 3)
                    photo_selected_points = np.array(photo_selected_points).reshape(len(photo_selected_points), -1, 2)

                    fg_cam_matrix = solve_pnp(model_selected_points.astype(np.float32),
                                              photo_selected_points.astype(np.float32), K)
                    # print("\nFirst-guess camera matrix= ")
                    # print(fg_cam_matrix)

                    if fg_cam_matrix is None:
                        print("\nCould not estimate transformation. "
                              "Please select more points, preferably in different planes!")
                        continue

                    print("\nGenerating viewpoint...")

                    est_color, est_depth = sim_view_point(mesh, fg_cam_matrix, K, image_width, image_height)

                    print("\nFinding matching features between photo and generated viewpoint...")
                    # Find matches between images and take those points from 2D to 3D. Then SolvePnP once more.
                    # Query image is what you want to find in train image
                    query_img = cv2.cvtColor(original_photo[:, :, :3], cv2.COLOR_BGR2GRAY)
                    train_img = cv2.cvtColor(est_color[:, :, :3], cv2.COLOR_BGR2GRAY)

                    # detector = cv2.ORB_create(edgeThreshold=31, patchSize=31, fastThreshold=1000)
                    # detector = cv2.xfeatures2d.SURF_create(hessianThreshold=100)
                    detector = cv2.SIFT_create(nOctaveLayers=6, contrastThreshold=0.01, edgeThreshold=30, sigma=0.8)
                    query_keypoints, query_descriptors = detector.detectAndCompute(query_img, None)
                    train_keypoints, train_descriptors = detector.detectAndCompute(train_img, None)

                    # Matching descriptor vectors with a FLANN based matcher
                    matcher = cv2.DescriptorMatcher_create(cv2.DescriptorMatcher_FLANNBASED)
                    knn_matches = matcher.knnMatch(query_descriptors, train_descriptors, 2)

                    # Filter matches using the Lowe's ratio test
                    ratio_thresh = 0.6
                    good_matches = []
                    for m, n in knn_matches:
                        if m.distance < ratio_thresh * n.distance:
                            good_matches.append(m)

                    # Draw matches
                    if args['show_visual_feedback']:
                        img_matches = np.empty(
                            (max(query_img.shape[0], train_img.shape[0]), query_img.shape[1] + train_img.shape[1], 3),
                            dtype=np.uint8)
                        cv2.drawMatches(query_img, query_keypoints, cv2.cvtColor(train_img, cv2.COLOR_RGB2BGR),
                                        train_keypoints, good_matches, img_matches,
                                        flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
                        cv2.namedWindow('Good Matches', cv2.WINDOW_NORMAL)
                        cv2.setWindowProperty('Good Matches', cv2.WINDOW_FULLSCREEN, 1)
                        cv2.imshow('Good Matches', img_matches)
                        cv2.waitKey()
                        cv2.destroyAllWindows()

                    model_points = []
                    photo_pixels = []
                    # Take the pixels and use the depth image to get 3D coordinates
                    for match in good_matches:
                        query_pixel = (
                            round(query_keypoints[match.queryIdx].pt[1]), round(query_keypoints[match.queryIdx].pt[0]))
                        train_pixel = (
                            round(train_keypoints[match.trainIdx].pt[1]), round(train_keypoints[match.trainIdx].pt[0]))

                        mask = np.zeros(est_depth.shape)
                        mask[train_pixel] = est_depth[train_pixel]

                        e_d = o3d.geometry.Image((mask).astype(np.float32))

                        o3d_intrinsics = o3d.camera.PinholeCameraIntrinsic(image_width, image_height, K[0, 0], K[1, 1],
                                                                           K[0, 2],
                                                                           K[1, 2])
                        est_pcd = o3d.geometry.PointCloud.create_from_depth_image(e_d, intrinsic=o3d_intrinsics)
                        est_pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
                        est_pcd.transform(fg_cam_matrix)

                        # o3d.io.write_point_cloud(str(query_pixel)+".ply", est_pcd)

                        if est_pcd.has_points():
                            model_points.append(est_pcd.points[0].tolist())
                            # To use OpenCV's SolvePnP the col and row must be switched
                            photo_pixels.append((query_pixel[1], query_pixel[0]))

                    model_points = np.array(model_points).reshape(len(model_points), -1, 3)
                    photo_pixels = np.array(photo_pixels).reshape(len(photo_pixels), -1, 2)
                    est_cam_matrix = solve_pnp(model_points.astype(np.float32), photo_pixels.astype(np.float32), K)

                    if est_cam_matrix is None:
                        print("\nCould not estimate transformation. "
                              "Please select more points, preferably in different planes!")
                        continue
                    else:
                        break

            print("\nEstimated camera matrix = ")
            print(est_cam_matrix)

            f_color, f_depth = sim_view_point(mesh, est_cam_matrix, K, image_width, image_height)

            if args['show_visual_feedback']:
                est_view = np.hstack((original_photo[:, :, :3], cv2.cvtColor(f_color, cv2.COLOR_RGBA2BGRA)[:, :, :3]))
                cv2.namedWindow('Original photo and estimated viewpoint', cv2.WINDOW_NORMAL)
                cv2.setWindowProperty('Original photo and estimated viewpoint', cv2.WINDOW_FULLSCREEN, 1)
                cv2.imshow('Original photo and estimated viewpoint', est_view)
                cv2.waitKey()
                cv2.destroyAllWindows()

            if args['output_path']:
                cv2.imwrite(args['output_path'] + '/fullref_output/' + str(file_idx) + '_original_photo' + '.png',
                            original_photo)
                cv2.imwrite(args['output_path'] + '/fullref_output/' + str(file_idx) + '_first_guess.png',
                            cv2.cvtColor(est_color, cv2.COLOR_RGBA2BGRA))
                cv2.imwrite(args['output_path'] + '/fullref_output/' + str(file_idx) + '_simulated_photo' + '.png',
                            cv2.cvtColor(f_color, cv2.COLOR_RGBA2BGRA))

            # Elements are False where image is transparent
            mask = np.ma.masked_not_equal(f_color[:, :, 3], 0).mask
            # cv2.imwrite('mask.png', (mask*255).astype(np.uint8))
            if not mask.any():
                mask = None

            compare_images(cv2.cvtColor(original_photo, cv2.COLOR_BGRA2RGBA), f_color, str(file_idx), mask,
                           args['output_path'])
