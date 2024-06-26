import argparse
import os
import shutil

import trimesh

from Meshtrics.topological_metrics import get_topological_metrics


def add_arguments(ap):
    ap.add_argument("-mesh", "--input_mesh", help="Path to mesh input file. Can be .ply, .obj, etc.")
    ap.add_argument("-o", "--output_path", help="Path to save output files. Defaults to . ", default=".")
    return ap


if __name__ == "__main__":

    ap = argparse.ArgumentParser()
    add_arguments(ap)
    args = vars(ap.parse_args())
    print("\n" + str(args))

    if args['output_path']:
        if os.path.exists(args['output_path'] + '/topological_output'):
            shutil.rmtree(args['output_path'] + '/topological_output')
        os.makedirs(args['output_path'] + '/topological_output')

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

    get_topological_metrics(tri_mesh, os.path.basename(args['input_mesh']))
