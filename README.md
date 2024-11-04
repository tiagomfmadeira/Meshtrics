<!-- PROJECT SHIELDS -->
[![Python 3.6](https://img.shields.io/badge/python-3.6-blue.svg)](https://www.python.org/downloads/release/python-360/)
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![stability-experimental](https://img.shields.io/badge/stability-experimental-orange.svg)](https://github.com/emersion/stability-badges#experimental)

<!-- PROJECT LOGO -->
<p align="center">
<img src="https://github.com/tiagomfmadeira/Meshtrics/blob/main/meshtrics_logo.png">
Objective Quality Assessment of Textured 3D Meshes for 3D Reconstruction</p>

# Test scripts

## [test_photometric_full_reference_metrics.py](https://github.com/tiagomfmadeira/Meshtrics/blob/main/tests/test_photometric_full_reference_metrics.py)

This script uses image-based metrics to evaluate the visual quality of a 3D model by leveraging renders and ground-truth photos as reference.

### Description

The script performs the following tasks:
1. Loads a 3D mesh and optionally a .e57 file or a .txt file containing camera intrinsics.
2. Prompts the user to select corresponding points in the photos and the 3D model.
3. Calculates a first guess for the camera position and orientation using the selected points.
4. Simulates viewpoints from the estimated camera positions.
5. Matches features between the ground-truth photos and the simulated viewpoints.
6. Refines the camera position
7. Generates model render and applies image-based metrics.
8. Exports the results.

### Usage 

#### Command-line Arguments

- `-photos`, `--input_photos_directory` (required): Path to a directory containing the photos to be used as ground-truth.
- `-pext`, `--photo_extension` (required): Extension of the photo files to be read from the directory.
- `-e57`, `--input_e57`: Path to the .e57 input file (optional).
- `-K`, `--intrinsics`: Path to the camera intrinsics input file (optional if `-e57` is provided).
- `-mesh`, `--input_mesh` (required): Path to the 3D mesh input file (e.g., .ply, .obj).
- `-o`, `--output_path`: Path to save output files. Defaults to the current directory.
- `-show`, `--show_visual_feedback`: Show visual feedback of the operations. If set, visual feedback windows will be displayed.

```bash
python test_photometric_full_reference_metrics.py \
  -photos ./ground_truth_photos \
  -pext .jpg \
  -mesh ./models/mesh.ply \
  -K ./camera_intrinsics.txt \
  -o ./output \
  -show
```

***

## [test_topological_metrics.py](https://github.com/tiagomfmadeira/Meshtrics/blob/main/tests/test_topological_metrics.py)

This script computes various topological metrics for a given 3D mesh.

### Description

The script performs the following tasks:
1. Loads a 3D mesh file (in .ply, .obj, etc. format).
2. Computes general metrics such as number of vertices, faces, and mesh area.
3. Evaluates smoothness by analyzing adjacent face ratios.
4. Calculates aspect ratios for each face and determines percentage of faces within ideal ranges.
5. Computes skewness for each face and generates a histogram of skewness distribution.
6. Detects holes in the mesh outline and provides metrics about the outline perimeter.

### Usage 

#### Command-line Arguments

- `-mesh`, `--input_mesh` (required): Path to the 3D mesh input file (e.g., .ply, .obj).
- `-o`, `--output_path`: Path to save output files. Defaults to the current directory.

```bash
python test_topological_metrics.py \
  -mesh ./models/mesh.ply \
  -o ./output
```

***

## [test_photometric_no_reference_metrics.py](https://github.com/tiagomfmadeira/Meshtrics/blob/main/tests/test_photometric_no_reference_metrics.py)

This script uses image entropy to evaluate the visual quality of 3D models. It allows for the comparisson of the same area in two models.

### Description

The script performs the following tasks:
1. Loads two 3D meshes for comparison.
2. Renders viewpoints for both meshes using the same camera parameters.
3. Allows the user to select a region of interest (ROI) in the rendered images.
4. Calculates entropy metrics for the selected ROIs of both meshes.
5. Outputs the results to the specified directory.

### Usage 

#### Command-line Arguments

- `-mesh1`, `--input_mesh1` (required): Path to the first 3D mesh input file (e.g., .ply, .obj).
- `-mesh2`, `--input_mesh2` (required): Path to the second 3D mesh input file (e.g., .ply, .obj).
- `-o`, `--output_path`: Path to save output files. Defaults to the current directory.
- `-show`, `--show_visual_feedback`: Show visual feedback of the operations. If set, visual feedback windows will be displayed.

```bash
python test_photometric_no_reference_metrics.py \
  -mesh1 ./models/mesh1.ply \
  -mesh2 ./models/mesh2.ply \
  -o ./output \
  -show
```
