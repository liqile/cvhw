# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/liqile/qtwork/myslam

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/liqile/qtwork/myslam/build

# Include any dependencies generated for this target.
include CMakeFiles/main.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/main.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/main.dir/flags.make

CMakeFiles/main.dir/main.cc.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/main.cc.o: ../main.cc
	$(CMAKE_COMMAND) -E cmake_progress_report /home/liqile/qtwork/myslam/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/main.dir/main.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/main.dir/main.cc.o -c /home/liqile/qtwork/myslam/main.cc

CMakeFiles/main.dir/main.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/main.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/liqile/qtwork/myslam/main.cc > CMakeFiles/main.dir/main.cc.i

CMakeFiles/main.dir/main.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/main.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/liqile/qtwork/myslam/main.cc -o CMakeFiles/main.dir/main.cc.s

CMakeFiles/main.dir/main.cc.o.requires:
.PHONY : CMakeFiles/main.dir/main.cc.o.requires

CMakeFiles/main.dir/main.cc.o.provides: CMakeFiles/main.dir/main.cc.o.requires
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/main.cc.o.provides.build
.PHONY : CMakeFiles/main.dir/main.cc.o.provides

CMakeFiles/main.dir/main.cc.o.provides.build: CMakeFiles/main.dir/main.cc.o

# Object files for target main
main_OBJECTS = \
"CMakeFiles/main.dir/main.cc.o"

# External object files for target main
main_EXTERNAL_OBJECTS =

main: CMakeFiles/main.dir/main.cc.o
main: CMakeFiles/main.dir/build.make
main: ../lib/libmyslam.so
main: /usr/local/lib/libopencv_videostab.so.2.4.9
main: /usr/local/lib/libopencv_ts.a
main: /usr/local/lib/libopencv_superres.so.2.4.9
main: /usr/local/lib/libopencv_stitching.so.2.4.9
main: /usr/local/lib/libopencv_contrib.so.2.4.9
main: /usr/local/lib/libopencv_nonfree.so.2.4.9
main: /usr/local/lib/libopencv_ocl.so.2.4.9
main: /usr/local/lib/libopencv_gpu.so.2.4.9
main: /usr/local/lib/libopencv_photo.so.2.4.9
main: /usr/local/lib/libopencv_objdetect.so.2.4.9
main: /usr/local/lib/libopencv_legacy.so.2.4.9
main: /usr/local/lib/libopencv_video.so.2.4.9
main: /usr/local/lib/libopencv_ml.so.2.4.9
main: /usr/local/lib/libopencv_calib3d.so.2.4.9
main: /usr/local/lib/libopencv_features2d.so.2.4.9
main: /usr/local/lib/libopencv_highgui.so.2.4.9
main: /usr/local/lib/libopencv_imgproc.so.2.4.9
main: /usr/local/lib/libopencv_flann.so.2.4.9
main: /usr/local/lib/libopencv_core.so.2.4.9
main: ../Thirdparty/DBoW2/lib/libDBoW2.so
main: ../Thirdparty/g2o/lib/libg2o.so
main: /usr/lib/libvtkGenericFiltering.so.5.8.0
main: /usr/lib/libvtkGeovis.so.5.8.0
main: /usr/lib/libvtkCharts.so.5.8.0
main: /usr/lib/libvtkViews.so.5.8.0
main: /usr/lib/libvtkInfovis.so.5.8.0
main: /usr/lib/libvtkWidgets.so.5.8.0
main: /usr/lib/libvtkVolumeRendering.so.5.8.0
main: /usr/lib/libvtkHybrid.so.5.8.0
main: /usr/lib/libvtkParallel.so.5.8.0
main: /usr/lib/libvtkRendering.so.5.8.0
main: /usr/lib/libvtkImaging.so.5.8.0
main: /usr/lib/libvtkGraphics.so.5.8.0
main: /usr/lib/libvtkIO.so.5.8.0
main: /usr/lib/libvtkFiltering.so.5.8.0
main: /usr/lib/libvtkCommon.so.5.8.0
main: /usr/lib/libvtksys.so.5.8.0
main: /usr/lib/x86_64-linux-gnu/libboost_system.so
main: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
main: /usr/lib/x86_64-linux-gnu/libboost_thread.so
main: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
main: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
main: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
main: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
main: /usr/lib/x86_64-linux-gnu/libpthread.so
main: /usr/lib/libpcl_common.so
main: /usr/lib/libpcl_octree.so
main: /usr/lib/libOpenNI.so
main: /usr/lib/libOpenNI2.so
main: /usr/lib/libpcl_io.so
main: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
main: /usr/lib/libpcl_kdtree.so
main: /usr/lib/libpcl_search.so
main: /usr/lib/libpcl_visualization.so
main: /usr/lib/libpcl_sample_consensus.so
main: /usr/lib/libpcl_filters.so
main: /usr/lib/x86_64-linux-gnu/libboost_system.so
main: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
main: /usr/lib/x86_64-linux-gnu/libboost_thread.so
main: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
main: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
main: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
main: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
main: /usr/lib/x86_64-linux-gnu/libpthread.so
main: /usr/lib/libpcl_common.so
main: /usr/lib/libpcl_octree.so
main: /usr/lib/libOpenNI.so
main: /usr/lib/libOpenNI2.so
main: /usr/lib/libpcl_io.so
main: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
main: /usr/lib/libpcl_kdtree.so
main: /usr/lib/libpcl_search.so
main: /usr/lib/libpcl_visualization.so
main: /usr/lib/libpcl_sample_consensus.so
main: /usr/lib/libpcl_filters.so
main: CMakeFiles/main.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable main"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/main.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/main.dir/build: main
.PHONY : CMakeFiles/main.dir/build

CMakeFiles/main.dir/requires: CMakeFiles/main.dir/main.cc.o.requires
.PHONY : CMakeFiles/main.dir/requires

CMakeFiles/main.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/main.dir/cmake_clean.cmake
.PHONY : CMakeFiles/main.dir/clean

CMakeFiles/main.dir/depend:
	cd /home/liqile/qtwork/myslam/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/liqile/qtwork/myslam /home/liqile/qtwork/myslam /home/liqile/qtwork/myslam/build /home/liqile/qtwork/myslam/build /home/liqile/qtwork/myslam/build/CMakeFiles/main.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/main.dir/depend

