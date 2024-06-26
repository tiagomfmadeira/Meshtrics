import argparse
import os
import shutil

import cv2
import numpy as np
import open3d as o3d
import trimesh
from skimage.morphology import disk

from Meshtrics.photometric_metrics import sim_view_point, get_entropy


def draw_roi(event, x, y, flags, param):
    global pts

    img2 = img.copy()

    if event == cv2.EVENT_LBUTTONDOWN:  # Left click, select point
        pts.append((x, y))
        print("[OpenCV INFO] Picked point (" + str(x) + ", " + str(y) + ") to add in queue.")

    if event == cv2.EVENT_RBUTTONDOWN:  # Right click to cancel the last selected point
        if len(pts) > 0:
            x, y = pts.pop()
            print("[OpenCV INFO] Removed picked point (" + str(x) + ", " + str(y) + ") from pick queue.")

    if len(pts) > 0:
        cv2.circle(img2, pts[-1], 3, (0, 0, 255), -1)

    if len(pts) > 1:
        for i in range(len(pts) - 1):
            cv2.circle(img2, pts[i], 5, (0, 0, 255), -1)
            cv2.line(img=img2, pt1=pts[i], pt2=pts[i + 1], color=(255, 0, 0), thickness=2)

    cv2.imshow('image', img2)


def add_arguments(ap):
    ap.add_argument("-mesh1", "--input_mesh1", help="Path to mesh 1 input file. Can be .ply, .obj, etc.")
    ap.add_argument("-mesh2", "--input_mesh2", help="Path to mesh 2 input file. Can be .ply, .obj, etc.")
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
        if os.path.exists(args['output_path'] + '/noref_output'):
            shutil.rmtree(args['output_path'] + '/noref_output')
        os.makedirs(args['output_path'] + '/noref_output')

    print("\nLoading input mesh...")
    pcd = o3d.io.read_triangle_mesh(args['input_mesh1'], enable_post_processing=True, print_progress=True)
    mesh = trimesh.load(args['input_mesh1'])
    mesh2 = trimesh.load(args['input_mesh2'])

    i = 0
    while True:
        i += 1

        vis = o3d.visualization.Visualizer()
        vis.create_window()
        vis.add_geometry(pcd)
        vis.run()
        vis.destroy_window()
        control = vis.get_view_control()
        pinhole = control.convert_to_pinhole_camera_parameters()
        cam_matrix = np.copy(pinhole.extrinsic)
        cam_matrix[1, :] = -cam_matrix[1, :]
        cam_matrix[2, :] = -cam_matrix[2, :]
        cam_matrix = np.linalg.inv(cam_matrix)

        # Mesh 1
        f_color, f_depth = sim_view_point(mesh, cam_matrix, pinhole.intrinsic.intrinsic_matrix, pinhole.intrinsic.width,
                                          pinhole.intrinsic.height)
        img = cv2.cvtColor(f_color, cv2.COLOR_RGBA2BGRA)

        # Mesh 2
        f_color, f_depth = sim_view_point(mesh2, cam_matrix, pinhole.intrinsic.intrinsic_matrix,
                                          pinhole.intrinsic.width,
                                          pinhole.intrinsic.height)
        img_2 = cv2.cvtColor(f_color, cv2.COLOR_RGBA2BGRA)

        cv2.namedWindow('image', cv2.WINDOW_NORMAL)
        cv2.setWindowProperty('image', cv2.WINDOW_FULLSCREEN, 1)
        cv2.setMouseCallback('image', draw_roi)
        print("\n1) Please pick at least 3 points using [left click].")
        print("   Press [right click] to undo point picking.")
        print("2) Press ‘q’ to determine the selection area and save it.")
        print("Press ESC to quit program.\n")

        pts = []
        while True:
            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                print("\nExiting...")
                exit()
            if key == ord("q"):
                saved_data = {
                    "ROI": pts
                }
                break
        cv2.destroyAllWindows()

        if len(pts) < 3:
            print("\nNo region was selected! Please select at least 3 points!")
            continue

        points = np.array(pts, np.int32)
        points = points.reshape((-1, 1, 2))

        if args['show_visual_feedback']:
            # Display ROI
            show_mask = np.zeros(img.shape, np.uint8)
            show_mask = cv2.fillPoly(show_mask, [points], (0, 255, 0))
            show_image = cv2.addWeighted(src1=img, alpha=0.8, src2=show_mask, beta=0.2, gamma=0)
            cv2.namedWindow('show_img', cv2.WINDOW_NORMAL)
            cv2.setWindowProperty('show_img', cv2.WINDOW_FULLSCREEN, 1)
            cv2.imshow("show_img", show_image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        mask = np.zeros(img.shape[0:2], np.uint8)
        mask = cv2.fillPoly(mask, [points], 1)
        rows, cols = np.where(mask == 1)

        cropped_img = np.zeros((rows.max() - rows.min(), cols.max() - cols.min(), 4), np.uint8)
        cropped_img = img[rows.min():rows.max(), cols.min():cols.max(), :]
        cropped_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGRA2GRAY)

        cropped_mask = np.zeros((rows.max() - rows.min(), cols.max() - cols.min()), np.uint8)
        cropped_mask = mask[rows.min():rows.max(), cols.min():cols.max()]

        get_entropy(cropped_img, neighborhood=disk(5), region_name=str(i) + "_mesh1", mask=cropped_mask,
                    output_path=args['output_path'])

        cropped_img = np.zeros((rows.max() - rows.min(), cols.max() - cols.min(), 4), np.uint8)
        cropped_img = img_2[rows.min():rows.max(), cols.min():cols.max(), :]
        cropped_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGRA2GRAY)

        get_entropy(cropped_img, neighborhood=disk(5), region_name=str(i) + "_mesh2", mask=cropped_mask,
                    output_path=args['output_path'])
