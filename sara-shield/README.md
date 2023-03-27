# SaRA-shield
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

This package provides safety for human-robot interaction using reachability analysis.
We use [SaRA](https://github.com/Sven-Schepp/SaRA) to calculate the reachable sets of humans and robots.
The SaRA shield additionally provides the necessary trajectory control to stop the robot before any collision with the human could occur.

# Installation
### Clone the repo with submodules
```
git clone --recurse-submodules git@github.com:JakobThumm/sara-shield.git
```
### Install the shield [C++ only]
The installation requires `gcc`, `c++>=17`, and `Eigen3` version 3.4 (download it here: https://eigen.tuxfamily.org/index.php?title=Main_Page).
Set the path to your eigen3 installation to this env variable, e.g.,
```
export EIGEN3_INCLUDE_DIR="/usr/include/eigen3/eigen-3.4.0"
```

```
cd safety_shield
mkdir build && cd build
cmake ..
make -j 4
```
### Install the shield [With Python bindings]
```
pip install -r requirements.txt
python setup.py install
```
### Run the python binding tests
```
pytest safety_shield/tests
```