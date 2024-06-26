import argparse
import math

import cv2
import numpy as np
import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import trimesh


def setup_camera(height, width):
    fovy = math.radians(60)
    f = height / (2 * math.tan(fovy / 2))
    cx = width / 2
    cy = height / 2
    o3d_intrinsics = o3d.camera.PinholeCameraIntrinsic(width, height, f, f, cx, cy)
    o3d_extrinsics = np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]])
    widget3d.setup_camera(o3d_intrinsics, o3d_extrinsics, bbox)


def pairs(lst):
    i = iter(lst)
    first = prev = item = next(i)
    for item in i:
        yield prev, item
        prev = item
    yield item, first


def unproject_point(point2d, depth, intrinsics, extrinsics):
    x_d = point2d[0]
    y_d = point2d[1]
    fx_d, fy_d, cx_d, cy_d = intrinsics[0, 0], intrinsics[1, 1], intrinsics[0, 2], intrinsics[1, 2]

    x = (x_d - cx_d) * depth / fx_d
    y = (y_d - cy_d) * depth / fy_d
    z = depth

    x, y, z, pad = np.dot(extrinsics, [x, y, z, 1])

    return np.asarray([x, y, z])


def on_key_widget3d(event):
    if event.key == gui.KeyName.SPACE:
        if event.type == gui.KeyEvent.UP:

            def mask_callback(depth_image):
                global selecting_polygon
                global polygon_vertices
                global polygon_vertices_2D

                if selecting_polygon:
                    selecting_polygon = False
                    print("\nSaving polygon selection...")
                    if len(polygon_vertices) >= 3:

                        depth = np.asarray(depth_image)

                        points = np.array(polygon_vertices_2D, np.int32)
                        points = points.reshape((-1, 1, 2))

                        mask = np.zeros((widget3d.frame.height, widget3d.frame.width), np.uint8)
                        mask = cv2.fillPoly(mask, [points], 1)

                        masked_depth = cv2.bitwise_and(depth, depth, mask=mask)

                        cv2.imshow("depth", masked_depth)
                        cv2.waitKey(0)

                        #########

                        A = polygon_vertices[0]
                        B = polygon_vertices[1]
                        C = polygon_vertices[2]

                        poly_normal = np.cross(A - B, C - B)

                        cropped_mesh = mesh
                        # Cut around the polygon
                        for pair in pairs(polygon_vertices):
                            plane_normal = np.cross(poly_normal, pair[0] - pair[1])
                            cropped_mesh = trimesh.intersections.slice_mesh_plane(cropped_mesh, plane_normal, pair[0])
                        # Cut in depth - 10cm?
                        height = 0.1
                        unit_vec = poly_normal / np.linalg.norm(poly_normal)
                        point_above = A + unit_vec * height
                        point_below = A - unit_vec * height
                        cropped_mesh = trimesh.intersections.slice_mesh_plane(cropped_mesh, -poly_normal, point_above)
                        cropped_mesh = trimesh.intersections.slice_mesh_plane(cropped_mesh, poly_normal, point_below)

                        cropped_mesh.show()

                        # widget3d.scene.clear_geometry()
                        # widget3d.scene.add_geometry(args['input_mesh'], cropped_mesh.as_open3d, mat)

                    else:
                        print("Not enough vertices have been selected!")

                else:
                    selecting_polygon = True
                    polygon_vertices = []
                    polygon_vertices_2D = []
                    for label in labels:
                        widget3d.remove_3d_label(label)
                    widget3d.scene.clear_geometry()
                    for mi in pcd.meshes:
                        m = pcd.materials[mi.material_idx]
                        m.shader = "defaultUnlit"
                        m.base_color = [1.0, 1.0, 1.0, 1.0]
                        widget3d.scene.add_geometry(mi.mesh_name, mi.mesh, m)
                    print("\nPlease select the polygon vertices")

            widget3d.scene.scene.render_to_depth_image(mask_callback)
            return gui.Widget.EventCallbackResult.HANDLED

    return gui.Widget.EventCallbackResult.IGNORED


def on_mouse_widget3d(event):
    if event.type == gui.MouseEvent.Type.BUTTON_DOWN and event.is_modifier_down(gui.KeyModifier.CTRL):

        def depth_callback(depth_image):
            global selecting_polygon
            global polygon_vertices
            global polygon_vertices_2D

            x = event.x - widget3d.frame.x
            y = event.y - widget3d.frame.y
            depth = np.asarray(depth_image)[y, x]

            if depth != 1.0:  # clicked on nothing (i.e. the far plane)

                matrix = np.array(widget3d.scene.camera.get_view_matrix())
                matrix[1, :] = -matrix[1, :]
                matrix[2, :] = -matrix[2, :]
                matrix = np.linalg.inv(matrix)

                z_near = widget3d.scene.camera.get_near()
                z_far = widget3d.scene.camera.get_far()
                depth = 2.0 * z_near * z_far / (z_far + z_near - (2.0 * depth - 1.0) * (z_far - z_near))

                intrinsics = np.zeros((3, 3))
                fovy = math.radians(widget3d.scene.camera.get_field_of_view())
                intrinsics[0, 0] = widget3d.frame.height / (2 * math.tan(fovy / 2))
                intrinsics[1, 1] = widget3d.frame.height / (2 * math.tan(fovy / 2))
                intrinsics[0, 2] = widget3d.frame.width / 2
                intrinsics[1, 2] = widget3d.frame.height / 2
                depth = unproject_point([x, y], depth, intrinsics, matrix)

                distance, vertex_id = mesh.nearest.vertex([depth])
                point_pos = mesh.vertices[vertex_id][0]

                tmp_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.001 * bbox.get_max_extent())
                tmp_sphere.paint_uniform_color([1, 0, 0])
                tmp_sphere.translate(point_pos)
                widget3d.scene.add_geometry(str(point_pos), tmp_sphere, mat)

                if selecting_polygon:
                    text = "({:.3f}, {:.3f}, {:.3f})".format(point_pos[0], point_pos[1], point_pos[2])
                    print("[Open3D INFO] Added point " + text + " to polygon")
                    polygon_vertices.append(point_pos)
                    polygon_vertices_2D.append((x, y))
                    idx = len(polygon_vertices) - 1
                    labels.append(widget3d.add_3d_label(point_pos, str(idx)))
                    if len(polygon_vertices) > 1:
                        line_set = o3d.geometry.LineSet(
                            points=o3d.utility.Vector3dVector([point_pos, polygon_vertices[idx - 1]]),
                            lines=o3d.utility.Vector2iVector([[0, 1]]))
                        line_set.colors = o3d.utility.Vector3dVector([[1, 0, 0]])
                        widget3d.scene.add_geometry("line_" + str(idx), line_set, mat)

        widget3d.scene.scene.render_to_depth_image(depth_callback)

        return gui.Widget.EventCallbackResult.HANDLED
    return gui.Widget.EventCallbackResult.IGNORED


def add_arguments(ap):
    """ Adds to an argument parser struct the list of command line arguments
    :param ap:
    :return:
    """
    ap.add_argument("-mesh", "--input_mesh", help="Path to mesh input file. Can be .ply, .obj, etc.")
    ap.add_argument("-o", "--output_path", help="Path to save output files. Defaults to . ", default=".")
    return ap


if __name__ == "__main__":

    ap = argparse.ArgumentParser()
    add_arguments(ap)
    args = vars(ap.parse_args())
    print("\n" + str(args))

    polygon_vertices = []
    polygon_vertices_2D = []
    labels = []
    selecting_polygon = False

    print("\nLoading input mesh...")
    mesh = trimesh.load(args['input_mesh'], force='mesh')
    pcd = o3d.io.read_triangle_model(args['input_mesh'], print_progress=True)

    # Get structure going
    mesh.nearest.vertex([[0, 0, 0]])

    #####################################

    app = gui.Application.instance
    app.initialize()

    w = app.create_window("Meshtrics")

    mat = rendering.Material()
    mat.shader = "defaultUnlit"
    mat.point_size = 5 * w.scaling

    widget3d = gui.SceneWidget()
    widget3d.enable_scene_caching(True)

    widget3d.scene = rendering.Open3DScene(w.renderer)

    for idx, mi in enumerate(pcd.meshes):
        m = pcd.materials[mi.material_idx]
        m.shader = "defaultUnlit"
        m.base_color = [1.0, 1.0, 1.0, 1.0]
        widget3d.scene.add_geometry(mi.mesh_name, mi.mesh, m)

    bbox = widget3d.scene.bounding_box

    setup_camera(w.size.height, w.size.width)

    w.add_child(widget3d)

    widget3d.set_on_mouse(on_mouse_widget3d)
    widget3d.set_on_key(on_key_widget3d)

    app.run()
