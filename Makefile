# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Default target executed when no arguments are given to make.
default_target: all

.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:


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
CMAKE_SOURCE_DIR = /home/ubuntu/RoboCup2023/Building-a-map-using-SLAM-on-a-Raspberry-Pi

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/RoboCup2023/Building-a-map-using-SLAM-on-a-Raspberry-Pi

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache

.PHONY : rebuild_cache/fast

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake cache editor..."
	/usr/bin/cmake-gui -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache

.PHONY : edit_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /home/ubuntu/RoboCup2023/Building-a-map-using-SLAM-on-a-Raspberry-Pi/CMakeFiles /home/ubuntu/RoboCup2023/Building-a-map-using-SLAM-on-a-Raspberry-Pi/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/ubuntu/RoboCup2023/Building-a-map-using-SLAM-on-a-Raspberry-Pi/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean

.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named streamVideo

# Build rule for target.
streamVideo: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 streamVideo
.PHONY : streamVideo

# fast build rule for target.
streamVideo/fast:
	$(MAKE) -f CMakeFiles/streamVideo.dir/build.make CMakeFiles/streamVideo.dir/build
.PHONY : streamVideo/fast

streamVideo.o: streamVideo.cpp.o

.PHONY : streamVideo.o

# target to build an object file
streamVideo.cpp.o:
	$(MAKE) -f CMakeFiles/streamVideo.dir/build.make CMakeFiles/streamVideo.dir/streamVideo.cpp.o
.PHONY : streamVideo.cpp.o

streamVideo.i: streamVideo.cpp.i

.PHONY : streamVideo.i

# target to preprocess a source file
streamVideo.cpp.i:
	$(MAKE) -f CMakeFiles/streamVideo.dir/build.make CMakeFiles/streamVideo.dir/streamVideo.cpp.i
.PHONY : streamVideo.cpp.i

streamVideo.s: streamVideo.cpp.s

.PHO