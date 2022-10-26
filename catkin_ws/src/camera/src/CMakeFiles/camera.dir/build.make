# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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
CMAKE_SOURCE_DIR = /home/ubuntu/RoboCup2023/Building-a-map-using-SLAM-on-a-Raspberry-Pi/catkin_ws/src/camera/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/RoboCup2023/Building-a-map-using-SLAM-on-a-Raspberry-Pi/catkin_ws/src/camera/src

# Include any dependencies generated for this target.
include CMakeFiles/camera.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/camera.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/camera.dir/flags.make

CMakeFiles/camera.dir/camera.cpp.o: CMakeFiles/camera.dir/flags.make
CMakeFiles/camera.dir/camera.cpp.o: camera.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/RoboCup2023/Building-a-map-using-SLAM-on-a-Raspberry-Pi/catkin_ws/src/camera/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/camera.dir/camera.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/camera.dir/camera.cpp.o -c /home/ubuntu/RoboCup2023/Building-a-map-using-SLAM-on-a-Raspberry-Pi/catkin_ws/src/camera/src/camera.cpp

CMakeFiles/camera.dir/camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camera.dir/camera.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/RoboCup2023/Building-a-map-using-SLAM-on-a-Raspberry-Pi/catkin_ws/src/camera/src/camera.cpp > CMakeFiles/camera.dir/camera.cpp.i

CMakeFiles/camera.dir/camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camera.dir/camera.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/RoboCup2023/Building-a-map-using-SLAM-on-a-Raspberry-Pi/catkin_ws/src/camera/src/camera.cpp -o CMakeFiles/camera.dir/camera.cpp.s

# Object files for target camera
camera_OBJECTS = \
"CMakeFiles/camera.dir/camera.cpp.o"

# External object files for target camera
camera_EXTERNAL_OBJECTS =

camera: CMakeFiles/camera.dir/camera.cpp.o
camera: CMakeFiles/camera.dir/build.make
camera: /opt/ros/noetic/lib/libcv_bridge.so
camera: /usr/lib/aarch64-linux-gnu/libopencv_calib3d.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_dnn.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_features2d.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_flann.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_highgui.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_ml.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_objdetect.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_photo.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_stitching.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_video.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_videoio.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_aruco.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_bgsegm.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_bioinspired.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_ccalib.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_datasets.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_dnn_superres.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_dpm.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_face.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_freetype.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_fuzzy.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_hdf.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_hfs.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_img_hash.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_line_descriptor.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_optflow.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_plot.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_quality.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_reg.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_rgbd.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_saliency.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_shape.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_stereo.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_structured_light.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_superres.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_surface_matching.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_text.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_tracking.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_videostab.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_viz.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_ximgproc.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_xobjdetect.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_xphoto.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_core.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.4.2.0
camera: /usr/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.4.2.0
camera: /opt/ros/noetic/lib/libimage_transport.so
camera: /opt/ros/noetic/lib/libmessage_filters.so
camera: /opt/ros/noetic/lib/libclass_loader.so
camera: /usr/lib/aarch64-linux-gnu/libPocoFoundation.so
camera: /usr/lib/aarch64-linux-gnu/libdl.so
camera: /opt/ros/noetic/lib/libroscpp.so
camera: /usr/lib/aarch64-linux-gnu/libpthread.so
camera: /usr/lib/aarch64-linux-gnu/libboost_chrono.so.1.71.0
camera: /opt/ros/noetic/lib/librosconsole.so
camera: /opt/ros/noetic/lib/librosconsole_log4cxx.so
camera: /opt/ros/noetic/lib/librosconsole_backend_interface.so
camera: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
camera: /usr/lib/aarch64-linux-gnu/libboost_regex.so.1.71.0
camera: /opt/ros/noetic/lib/libxmlrpcpp.so
camera: /opt/ros/noetic/lib/libroslib.so
camera: /opt/ros/noetic/lib/librospack.so
camera: /usr/lib/aarch64-linux-gnu/libpython3.8.so
camera: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so.1.71.0
camera: /usr/lib/aarch64-linux-gnu/libboost_program_options.so.1.71.0
camera: /usr/lib/aarch64-linux-gnu/libtinyxml2.so
camera: /opt/ros/noetic/lib/libroscpp_serialization.so
camera: /opt/ros/noetic/lib/librostime.so
camera: /usr/lib/aarch64-linux-gnu/libboost_date_time.so.1.71.0
camera: /opt/ros/noetic/lib/libcpp_common.so
camera: /usr/lib/aarch64-linux-gnu/libboost_system.so.1.71.0
camera: /usr/lib/aarch64-linux-gnu/libboost_thread.so.1.71.0
camera: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
camera: CMakeFiles/camera.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ubuntu/RoboCup2023/Building-a-map-using-SLAM-on-a-Raspberry-Pi/catkin_ws/src/camera/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable camera"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/camera.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/camera.dir/build: camera

.PHONY : CMakeFiles/camera.dir/build

CMakeFiles/camera.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/camera.dir/cmake_clean.cmake
.PHONY : CMakeFiles/camera.dir/clean

CMakeFiles/camera.dir/depend:
	cd /home/ubuntu/RoboCup2023/Building-a-map-using-SLAM-on-a-Raspberry-Pi/catkin_ws/src/camera/src && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/RoboCup2023/Building-a-map-using-SLAM-on-a-Raspberry-Pi/catkin_ws/src/camera/src /home/ubuntu/RoboCup2023/Building-a-map-using-SLAM-on-a-Raspberry-Pi/catkin_ws/src/camera/src /home/ubuntu/RoboCup2023/Building-a-map-using-SLAM-on-a-Raspberry-Pi/catkin_ws/src/camera/src /home/ubuntu/RoboCup2023/Building-a-map-using-SLAM-on-a-Raspberry-Pi/catkin_ws/src/camera/src /home/ubuntu/RoboCup2023/Building-a-map-using-SLAM-on-a-Raspberry-Pi/catkin_ws/src/camera/src/CMakeFiles/camera.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/camera.dir/depend
