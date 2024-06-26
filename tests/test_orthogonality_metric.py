import argparse

import numpy as np
import open3d as o3d

from Meshtrics.geometric_metrics import get_orthogonality, find_plane


def pick_points(pcd):
    print("")
    print("1) Please pick at least 3 points using [shift + left click]")
    print("   Press [shift + right click] to undo point picking")
    print("2) After picking points, press q to close the window\n")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()
    print("")
    return vis.get_picked_points()


def choose_planes(pcd):
    planes = []

    while True:
        picked_points = pick_points(pcd)
        picked_points = np.asarray(pcd.select_by_index(picked_points).points)

        if len(picked_points) >= 3:
            plane = find_plane(pcd, picked_points, 100)

            while True:
                val = input("\nDo you want to get save this plane? [Y/n] ")
                if val == '' or val.casefold() == 'y':
                    planes.append(plane)
                    print("Plane saved!")
                    break
                elif val.casefold() == 'n':
                    print('Plane discarted!')
                    break
                else:
                    print('Unknown input!')
        else:
            print('Error! At least 3 points are needed to estimate plane!')

        while True:
            val = input("\nDo you want to get another plane? [Y/n] ")
            if val == '' or val.casefold() == 'y':
                break
            elif val.casefold() == 'n':
                print('All planes saved!')
                return planes
            else:
                print('Unknown input!')


def add_arguments(ap):
    ap.add_argument("-mesh", "--input_mesh", help="Path to mesh input file. Can be .ply, .obj, etc.")
    ap.add_argument("-show", "--show_visual_feedback", help="Show visual feedback of the operations",
                    action='store_true', default=False)
    return ap


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    add_arguments(ap)
    args = vars(ap.parse_args())
    print("\n" + str(args))

    print("\nLoading input mesh...")
    pcd = o3d.io.read_point_cloud(args['input_mesh'])

    planes = choose_planes(pcd)

    get_orthogonality(planes)
