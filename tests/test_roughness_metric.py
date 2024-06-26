import argparse

import trimesh

from Meshtrics.geometric_metrics import get_roughness


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

    get_roughness(tri_mesh)
