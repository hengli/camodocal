CamOdoCal: Automatic Intrinsic and Extrinsic Calibration of a Rig with Multiple Generic Cameras and Odometry
------------------------------------------------------------------------------------------------------------

This C++ library supports both intrinsic calibration of a generic camera and extrinsic calibration of a multi-camera rig for which odometry data is provided. The intrinsic calibration process computes the parameters for one of the following three camera models:
* Pinhole camera model
* Unified projection model (C. Mei, and P. Rives, Single View Point Omnidirectional Camera Calibration from Planar Grids, ICRA 2007)
* Equidistant fish-eye model (J. Kannala, and S. Brandt, A Generic Camera Model and Calibration Method for Conventional, Wide-Angle, and Fish-Eye Lenses, PAMI 2006)

By default, the unified projection model is used since this model approximates a wide range of cameras from normal cameras to catadioptric cameras. Note that in our equidistant fish-eye model, we use 8 parameters: k2, k3, k4, k5, mu, mv, u0, v0. k1 is set to 1.

The link to the paper which accompanies the C++ library will be posted here upon acceptance of the paper in a conference.

Build Instructions for Ubuntu
-----------------------------

*Required dependencies*
* BLAS (Ubuntu package: libblas-dev)
* CUDA
* Eigen3 (Ubuntu package: libeigen3-dev)
* GLib (Ubuntu package: libglib2.0-dev)
* gtkmm (Ubuntu package: libglibmm-2.4-dev)
* GThread (Ubuntu package: libglib2.0-dev)
* libsigc++ (Ubuntu package: libsigc++-2.0-dev)
* Boost >= 1.4.0 (Ubuntu package: libboost-all-dev)
* OpenCV >= 2.4.0
* SuiteSparse (Ubuntu package: libsuitesparse-dev)

*Optional dependencies*
* OpenMP

1. Before you compile the repository code, you need to install the required
   dependencies, and install the optional dependencies if required.

2. Build the code.

        mkdir build
        cd build
        cmake -DCMAKE_BUILD_TYPE=Release ..

3. If you wish to generate Eclipse project files, run:

        cmake -DCMAKE_BUILD_TYPE=Release -G"Eclipse CDT4 - Unix Makefiles" ..

Examples
--------

Go to the build folder where the executables corresponding to the examples are located in.

1. Intrinsic calibration ([src/examples/intrinsic_calib.cc] [1])

        bin/intrinsic_calib -i ../data/images/ -p img --camera-model mei

   The camera-model parameter takes one of the following three values: pinhole, mei, and kannala-brandt.

2. Extrinsic calibration ([src/examples/extrinsic_calib.cc] [2])
   
  [1]: https://github.com/hengli/camodocal/blob/master/src/examples/intrinsic_calib.cc "src/examples/intrinsic_calib.cc"
  [2]: https://github.com/hengli/camodocal/blob/master/src/examples/extrinsic_calib.cc "src/examples/extrinsic_calib.cc"
