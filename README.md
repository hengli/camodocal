CamOdoCal: Automatic Intrinsic and Extrinsic Calibration of a Rig with Multiple Generic Cameras and Odometry
------------------------------------------------------------------------------------------------------------

This C++ library supports both intrinsic calibration of a generic camera and extrinsic calibration of a multi-camera rig for which odometry data is provided. The intrinsic calibration process computes the parameters for one of the following three camera models:
* Pinhole camera model
* Unified projection model (C. Mei, and P. Rives, Single View Point Omnidirectional Camera Calibration from Planar Grids, ICRA 2007)
* Equidistant fish-eye model (J. Kannala, and S. Brandt, A Generic Camera Model and Calibration Method for Conventional, Wide-Angle, and Fish-Eye Lenses, PAMI 2006)

By default, the unified projection model is used since this model approximates a wide range of cameras from normal cameras to catadioptric cameras. Note that in our equidistant fish-eye model, we use 8 parameters: k2, k3, k4, k5, mu, mv, u0, v0. k1 is set to 1.

The landing page of the library is located at http://people.inf.ethz.ch/hengli/camodocal/

The workings of the library is described in the paper:

        Lionel Heng, Bo Li, and Marc Pollefeys,
        CamOdoCal: Automatic Intrinsic and Extrinsic Calibration of a Rig with Multiple Generic Cameras and Odometry,
        In Proc. IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2013.

If you use this library in an academic publication, please cite this paper.

#### Acknowledgements ####

The primary author, Lionel Heng, is funded by the DSO Postgraduate Scholarship. This work is supported in part by the European Community's Seventh Framework Programme (FP7/2007-2013) under grant #269916 (V-Charge).

The CamOdoCal library includes third-party code from the following sources:

        M. Rufli, D. Scaramuzza, and R. Siegwart,
        Automatic Detection of Checkerboards on Blurred and Distorted Images,
        In Proc. IEEE/RSJ International Conference on Intelligent Robots and Systems, 2008.

        Sameer Agarwal, Keir Mierle, and Others,
        Ceres Solver.
        https://code.google.com/p/ceres-solver/
        
        D. Galvez-Lopez, and J. Tardos,
        Bags of Binary Words for Fast Place Recognition in Image Sequences,
        IEEE Transactions on Robotics, 28(5):1188-1197, October 2012.
        
Parts of the CamOdoCal library are based on the following papers:

* Robust pose graph optimization

        G.H. Lee, F. Fraundorfer, and Marc Pollefeys,
        Robust Pose-Graph Loop-Closures with Expectation-Maximization,
        In Proc. IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2013.
        

Build Instructions for Ubuntu
-----------------------------

*Required dependencies*
* BLAS (Ubuntu package: libblas-dev)
* Boost >= 1.4.0 (Ubuntu package: libboost-all-dev)
* CUDA >= 4.2
* Eigen3 (Ubuntu package: libeigen3-dev)
* GLib (Ubuntu package: libglib2.0-dev)
* glog
* GThread (Ubuntu package: libglib2.0-dev)
* gtkmm (Ubuntu package: libglibmm-2.4-dev)
* libsigc++ (Ubuntu package: libsigc++-2.0-dev)
* OpenCV >= 2.4.0
* SuiteSparse (Ubuntu package: libsuitesparse-dev)

*Optional dependencies*
* GTest
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

Go to the build folder where the executables corresponding to the examples are located in. To see all allowed options for each executable, use the --help option which shows a description of all available options.

1. Intrinsic calibration ([src/examples/intrinsic_calib.cc] [1])

        bin/intrinsic_calib -i ../data/images/ -p img --camera-model mei

   The camera-model parameter takes one of the following three values: pinhole, mei, and kannala-brandt.

2. Stereo calibration  ([src/examples/stereo_calib.cc] [2])

        bin/stereo_calib -i ../data/images/ --prefix-l left --prefix-r right --camera-model mei

   The camera-model parameter takes one of the following three values: pinhole, mei, and kannala-brandt.

3. Extrinsic calibration ([src/examples/extrinsic_calib.cc] [3])

   Note: Extrinsic calibration requires the use of a vocabulary tree. The vocabulary data
         corresponding to 64-bit SURF descriptors can be found in data/vocabulary/surf64.yml.gz.
         This file has to be located in the directory from which you run the executable.

4. Infrastructure-based calibration

   Details to be released soon!
   
  [1]: https://github.com/hengli/camodocal/blob/master/src/examples/intrinsic_calib.cc "src/examples/intrinsic_calib.cc"
  [2]: https://github.com/hengli/camodocal/blob/master/src/examples/stereo_calib.cc "src/examples/stereo_calib.cc"
  [3]: https://github.com/hengli/camodocal/blob/master/src/examples/extrinsic_calib.cc "src/examples/extrinsic_calib.cc"
