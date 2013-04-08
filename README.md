CamOdoCal: Automatic Intrinsic and Extrinsic Calibration of a Rig with Multiple Generic Cameras and Odometry

===============================
 Build Instructions for Ubuntu
===============================

1. Before you compile the repository code, you need to install the required
   dependencies, and install the optional dependencies if required.

Required dependencies
=====================
1) BLAS (Ubuntu package: libblas-dev)
2) CUDA
3) Eigen3 (Ubuntu package: libeigen3-dev)
4) GLib (Ubuntu package: libglib2.0-dev)
5) gtkmm (Ubuntu package: libglibmm-2.4-dev)
6) GThread (Ubuntu package: libglib2.0-dev)
7) libsigc++ (Ubuntu package: libsigc++-2.0-dev)
8) Boost >= 1.4.0 (Ubuntu package: libboost-all-dev)
9) OpenCV >= 2.4.0
10) SuiteSparse (Ubuntu package: libsuitesparse-dev)

Optional dependencies
=====================
1) OpenMP

2. Build the code.
     mkdir build
     cd build
     cmake -DCMAKE_BUILD_TYPE=Release ..
     
3. If you wish to generate Eclipse project files, run:
     cmake -DCMAKE_BUILD_TYPE=Release -G"Eclipse CDT4 - Unix Makefiles" ..


==========
 Examples
==========

Go to the build folder where the executables corresponding to the examples are located in.

1. Intrinsic calibration (src/examples/intrinsic_calib.cc)
     bin/intrinsic_calib -i ../data/images/ -p img

2. Extrinsic calibration (src/examples/extrinsic_calib.cc)
   
