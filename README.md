![banner-logo](doc/imgs/banner-pleno.png)

---

The **plenoptic-cam-processing-lib** is a comprehensive open-source C++ library dedicated for plenoptic camera modeling and data handling.

Quick Start
===========

### Pre-requisites

The plenoptic-cam-processing-lib has a light dependency list:

 * [Eigen] version >=3, a modern C++ matrix and linear-algebra library,
 * [boost] version >=1.54 and up, portable C++ source libraries,
 * [OpenCV] version >=3.2, a collection of algorithms and sample code for various computer vision problems,
 * [libv], a general purpose computer vision library developed at Pascal Institute (used for graphic and serialization), modules _core_, _build_, _geometry_ and _graphic_,
 * [lma], a non-linear optimization library implementing the Levenberg Marquardt Algorithm,
 
and was compiled and tested on:
  * Ubuntu 18.04.4 LTS, GCC 7.5.0, with Eigen 3.3.4, Boost 1.65.1, and OpenCV 3.2.0,
  * Ubuntu 20.04.5 LTS, GCC 9.4.0, with Eigen 3.3.7, Boost 1.71.0, and OpenCV 4.2.0.
  
Please see the file `dependencies.txt` for more detailed dependencies list. 
Third-party libraries [libv] and [lma] can be found in the corresponding folder `./third_parties/`. You can install them from here (warning about ros, catkin and others can be ignored for installation).
  
### Compilation & Installation 

If you are comfortable with Linux and CMake and have already installed the prerequisites above, the following commands should install plenoptic-cam-processing-lib on your system.

```
mkdir build && cd build
cmake .. -DUSE_OPEN_MP=true
make -j6
sudo make install
```

Applications
============

Currently available applications using the plenoptic-cam-processing-lib:
 * [COMPOTE](https://github.com/dolphin2003/compote) (Calibration Of Multi-focus PlenOpTic camEra), a set of tools to pre-calibrate and calibrate (multifocus) plenoptic cameras.
 * [PRISM](https://github.com/dolphin2003/prism) (Plenoptic Raw Image Simulator), a set of tools to generate and simulate raw images from (multifocus) plenoptic cameras.
 * [BLADE](https://github.com/dolphin2003/blade) (Blur Aware Depth Estimation with a plenoptic camera), a set of tools to estimate depth map from raw images obtained by (multifocus) plenoptic cameras.
 * ...
 
Configuration file examples are given in the folder `./examples/config/` for the datasets `R12-A` (see below).
Observations for the dataset `R12-A` are also given in the folder `./examples/obs/`.
 
Datasets
========

* For calibration, datasets R12-A, R12-B, R12-C, R12-D and UPC-S can be downloaded [from here](https://github.com/dolphin2003/plenoptic-datasets).
* For depth estimation, datasets R12-E, ES and ELP20 can be downloaded [from here](https://github.com/dolphin2003/plenoptic-datasets).

Citing
======

If you use plenoptic-cam-processing-lib in an academic context, please cite the following publications:

    
    

License
=======

plenoptic-cam-processing-lib is licensed under the GNU General Public License v3.0. Enjoy!

[Ubuntu]: http://www.ubuntu.com
[CMake]: http://www.cmake.org
[CMake documentation]: http://www.cmake.org/cmake/help/cmake2.6docs.html
[git]: http://git-scm.com
[Eigen]: http://eigen.tuxfamily.org
[libv]: http://gitlab.ip.uca.fr/libv/libv
[lma]: http://gitlab.ip.uca.fr/libv/lma
[OpenCV]: https://opencv.org/
[Doxygen]: http://www.stack.nl/~dimitri/doxygen/
[boost]: http://www.boost.org/

---