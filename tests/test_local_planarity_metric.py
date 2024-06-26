import argparse

import numpy as np
import open3d as o3d
import trimesh

from Meshtrics.geometric_metrics import get_planarity, find_plane


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
    scene = trimesh.load(args['input_mesh'])

    if isinstance(scene, trimesh.Trimesh):
        tri_mesh = scene
    elif isinstance(scene, trimesh.Scene):
        tri_mesh = trimesh.util.concatenate([value for key, value in scene.geometry.items()])
    else:
        print("\nERROR! Cannot compute planarity with given mesh object.")
        print("Accepted is trimesh.Trimesh or trimesh.Scene")
        print("Called with:")
        print(type(scene).__module__ + '.' + type(scene).__qualname__)
        exit()

    geometry_type = o3d.io.read_file_geometry_type(args['input_mesh'])
    if geometry_type & o3d.io.CONTAINS_TRIANGLES:
        model = o3d.io.read_triangle_model(args['input_mesh'], print_progress=True)
        vertices = []
        for mesh in model.meshes:
            vertices.extend(mesh.mesh.vertices)
        pcd = o3d.geometry.PointCloud(o3d.cpu.pybind.utility.Vector3dVector(vertices))
    else:
        pcd = o3d.io.read_point_cloud(args['input_mesh'])
    pcd.paint_uniform_color([0, 1, 0])

    while True:
        picked_points = pick_points(pcd)
        picked_points = np.asarray(pcd.select_by_index(picked_points).points)

        if len(picked_points) >= 3:
            plane = find_plane(pcd, picked_points, 100)
            break
        else:
            print("Please pick at least 3 points!")
            continue

    [a, b, c, d] = plane['equation']
    print(f"\nPlane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

    inlier_cloud = plane['inliers']
    inlier_cloud.paint_uniform_color([1.0, 0, 0])

    o3d.visualization.draw_geometries([mesh.mesh for mesh in model.meshes] + [inlier_cloud])

    # ProximityQuery().vertex() returns (distance, idx)
    subsample = trimesh.proximity.ProximityQuery(tri_mesh).vertex(inlier_cloud.points)[1]
    vertex_mask = np.zeros(len(tri_mesh.vertices), dtype=np.int8)
    vertex_mask[subsample] = 1

    face_mask = vertex_mask[tri_mesh.faces].all(axis=1)

    # Create submesh with chosen faces
    submesh = tri_mesh.submesh(np.where(face_mask), append=True)

    # In alternative we could update current mesh
    # tri_mesh.update_faces(face_mask)
    # if args['show_visual_feedback']:
    #     tri_mesh.show()

    if args['show_visual_feedback']:
        submesh.show()

    get_planarity(submesh, plane['equation'])
