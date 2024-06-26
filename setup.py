import os
from setuptools import setup, find_packages

# Utility function to read the content of a file file.
def read(fname):
    return open(os.path.join(os.path.dirname(__file__), fname)).read()

setup(
    name = "Meshtrics",
    version = "1.0.0",
    install_requires=[
        'opencv-python',
        'psnr_hvsm==0.1.2',
        'open3d',
        'pyrender',
        'trimesh',
        'matplotlib',
        'scikit-image',
        'scipy',
        'seaborn',
        'argparse'
    ],
    author = "Tiago Madeira",
    author_email = "tiagomadeira@ua.pt",
    description = ("Metrics for textured mesh comparison and quality evaluation"),
    license = "GPLv3",
    keywords = "Mesh, Metrics, Texture, Geometry, Quality Evaluation",
    url = "https://github.com/tiagomfmadeira/Meshtrics",
    packages=find_packages(exclude=['test*']),
    package_dir={'Meshtrics': 'Meshtrics'},
    long_description=read('README.md'),
    classifiers=[
        "Development Status :: 5 - Production/Stable",
        "Topic :: Utilities",
        "License :: OSI Approved :: GNU General Public License v3 (GPLv3)",
        "Programming Language :: Python :: 3.6"
    ]
)