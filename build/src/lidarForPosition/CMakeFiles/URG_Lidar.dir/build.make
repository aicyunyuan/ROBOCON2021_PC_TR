# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/action/code/2021/TR/ABU2021_PC_TR

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/action/code/2021/TR/ABU2021_PC_TR/build

# Include any dependencies generated for this target.
include src/lidarForPosition/CMakeFiles/URG_Lidar.dir/depend.make

# Include the progress variables for this target.
include src/lidarForPosition/CMakeFiles/URG_Lidar.dir/progress.make

# Include the compile flags for this target's objects.
include src/lidarForPosition/CMakeFiles/URG_Lidar.dir/flags.make

src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/data.cpp.o: src/lidarForPosition/CMakeFiles/URG_Lidar.dir/flags.make
src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/data.cpp.o: ../src/lidarForPosition/src/data.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/action/code/2021/TR/ABU2021_PC_TR/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/data.cpp.o"
	cd /home/action/code/2021/TR/ABU2021_PC_TR/build/src/lidarForPosition && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/URG_Lidar.dir/src/data.cpp.o -c /home/action/code/2021/TR/ABU2021_PC_TR/src/lidarForPosition/src/data.cpp

src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/data.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/URG_Lidar.dir/src/data.cpp.i"
	cd /home/action/code/2021/TR/ABU2021_PC_TR/build/src/lidarForPosition && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/action/code/2021/TR/ABU2021_PC_TR/src/lidarForPosition/src/data.cpp > CMakeFiles/URG_Lidar.dir/src/data.cpp.i

src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/data.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/URG_Lidar.dir/src/data.cpp.s"
	cd /home/action/code/2021/TR/ABU2021_PC_TR/build/src/lidarForPosition && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/action/code/2021/TR/ABU2021_PC_TR/src/lidarForPosition/src/data.cpp -o CMakeFiles/URG_Lidar.dir/src/data.cpp.s

src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/data.cpp.o.requires:

.PHONY : src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/data.cpp.o.requires

src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/data.cpp.o.provides: src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/data.cpp.o.requires
	$(MAKE) -f src/lidarForPosition/CMakeFiles/URG_Lidar.dir/build.make src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/data.cpp.o.provides.build
.PHONY : src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/data.cpp.o.provides

src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/data.cpp.o.provides.build: src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/data.cpp.o


src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/main.cpp.o: src/lidarForPosition/CMakeFiles/URG_Lidar.dir/flags.make
src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/main.cpp.o: ../src/lidarForPosition/src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/action/code/2021/TR/ABU2021_PC_TR/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/main.cpp.o"
	cd /home/action/code/2021/TR/ABU2021_PC_TR/build/src/lidarForPosition && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/URG_Lidar.dir/src/main.cpp.o -c /home/action/code/2021/TR/ABU2021_PC_TR/src/lidarForPosition/src/main.cpp

src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/URG_Lidar.dir/src/main.cpp.i"
	cd /home/action/code/2021/TR/ABU2021_PC_TR/build/src/lidarForPosition && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/action/code/2021/TR/ABU2021_PC_TR/src/lidarForPosition/src/main.cpp > CMakeFiles/URG_Lidar.dir/src/main.cpp.i

src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/URG_Lidar.dir/src/main.cpp.s"
	cd /home/action/code/2021/TR/ABU2021_PC_TR/build/src/lidarForPosition && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/action/code/2021/TR/ABU2021_PC_TR/src/lidarForPosition/src/main.cpp -o CMakeFiles/URG_Lidar.dir/src/main.cpp.s

src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/main.cpp.o.requires:

.PHONY : src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/main.cpp.o.requires

src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/main.cpp.o.provides: src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/main.cpp.o.requires
	$(MAKE) -f src/lidarForPosition/CMakeFiles/URG_Lidar.dir/build.make src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/main.cpp.o.provides.build
.PHONY : src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/main.cpp.o.provides

src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/main.cpp.o.provides.build: src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/main.cpp.o


src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/map.cpp.o: src/lidarForPosition/CMakeFiles/URG_Lidar.dir/flags.make
src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/map.cpp.o: ../src/lidarForPosition/src/map.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/action/code/2021/TR/ABU2021_PC_TR/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/map.cpp.o"
	cd /home/action/code/2021/TR/ABU2021_PC_TR/build/src/lidarForPosition && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/URG_Lidar.dir/src/map.cpp.o -c /home/action/code/2021/TR/ABU2021_PC_TR/src/lidarForPosition/src/map.cpp

src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/map.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/URG_Lidar.dir/src/map.cpp.i"
	cd /home/action/code/2021/TR/ABU2021_PC_TR/build/src/lidarForPosition && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/action/code/2021/TR/ABU2021_PC_TR/src/lidarForPosition/src/map.cpp > CMakeFiles/URG_Lidar.dir/src/map.cpp.i

src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/map.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/URG_Lidar.dir/src/map.cpp.s"
	cd /home/action/code/2021/TR/ABU2021_PC_TR/build/src/lidarForPosition && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/action/code/2021/TR/ABU2021_PC_TR/src/lidarForPosition/src/map.cpp -o CMakeFiles/URG_Lidar.dir/src/map.cpp.s

src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/map.cpp.o.requires:

.PHONY : src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/map.cpp.o.requires

src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/map.cpp.o.provides: src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/map.cpp.o.requires
	$(MAKE) -f src/lidarForPosition/CMakeFiles/URG_Lidar.dir/build.make src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/map.cpp.o.provides.build
.PHONY : src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/map.cpp.o.provides

src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/map.cpp.o.provides.build: src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/map.cpp.o


src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/pos.cpp.o: src/lidarForPosition/CMakeFiles/URG_Lidar.dir/flags.make
src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/pos.cpp.o: ../src/lidarForPosition/src/pos.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/action/code/2021/TR/ABU2021_PC_TR/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/pos.cpp.o"
	cd /home/action/code/2021/TR/ABU2021_PC_TR/build/src/lidarForPosition && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/URG_Lidar.dir/src/pos.cpp.o -c /home/action/code/2021/TR/ABU2021_PC_TR/src/lidarForPosition/src/pos.cpp

src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/pos.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/URG_Lidar.dir/src/pos.cpp.i"
	cd /home/action/code/2021/TR/ABU2021_PC_TR/build/src/lidarForPosition && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/action/code/2021/TR/ABU2021_PC_TR/src/lidarForPosition/src/pos.cpp > CMakeFiles/URG_Lidar.dir/src/pos.cpp.i

src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/pos.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/URG_Lidar.dir/src/pos.cpp.s"
	cd /home/action/code/2021/TR/ABU2021_PC_TR/build/src/lidarForPosition && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/action/code/2021/TR/ABU2021_PC_TR/src/lidarForPosition/src/pos.cpp -o CMakeFiles/URG_Lidar.dir/src/pos.cpp.s

src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/pos.cpp.o.requires:

.PHONY : src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/pos.cpp.o.requires

src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/pos.cpp.o.provides: src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/pos.cpp.o.requires
	$(MAKE) -f src/lidarForPosition/CMakeFiles/URG_Lidar.dir/build.make src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/pos.cpp.o.provides.build
.PHONY : src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/pos.cpp.o.provides

src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/pos.cpp.o.provides.build: src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/pos.cpp.o


src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/pps.cpp.o: src/lidarForPosition/CMakeFiles/URG_Lidar.dir/flags.make
src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/pps.cpp.o: ../src/lidarForPosition/src/pps.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/action/code/2021/TR/ABU2021_PC_TR/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/pps.cpp.o"
	cd /home/action/code/2021/TR/ABU2021_PC_TR/build/src/lidarForPosition && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/URG_Lidar.dir/src/pps.cpp.o -c /home/action/code/2021/TR/ABU2021_PC_TR/src/lidarForPosition/src/pps.cpp

src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/pps.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/URG_Lidar.dir/src/pps.cpp.i"
	cd /home/action/code/2021/TR/ABU2021_PC_TR/build/src/lidarForPosition && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/action/code/2021/TR/ABU2021_PC_TR/src/lidarForPosition/src/pps.cpp > CMakeFiles/URG_Lidar.dir/src/pps.cpp.i

src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/pps.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/URG_Lidar.dir/src/pps.cpp.s"
	cd /home/action/code/2021/TR/ABU2021_PC_TR/build/src/lidarForPosition && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/action/code/2021/TR/ABU2021_PC_TR/src/lidarForPosition/src/pps.cpp -o CMakeFiles/URG_Lidar.dir/src/pps.cpp.s

src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/pps.cpp.o.requires:

.PHONY : src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/pps.cpp.o.requires

src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/pps.cpp.o.provides: src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/pps.cpp.o.requires
	$(MAKE) -f src/lidarForPosition/CMakeFiles/URG_Lidar.dir/build.make src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/pps.cpp.o.provides.build
.PHONY : src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/pps.cpp.o.provides

src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/pps.cpp.o.provides.build: src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/pps.cpp.o


src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/serial.cpp.o: src/lidarForPosition/CMakeFiles/URG_Lidar.dir/flags.make
src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/serial.cpp.o: ../src/lidarForPosition/src/serial.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/action/code/2021/TR/ABU2021_PC_TR/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/serial.cpp.o"
	cd /home/action/code/2021/TR/ABU2021_PC_TR/build/src/lidarForPosition && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/URG_Lidar.dir/src/serial.cpp.o -c /home/action/code/2021/TR/ABU2021_PC_TR/src/lidarForPosition/src/serial.cpp

src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/serial.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/URG_Lidar.dir/src/serial.cpp.i"
	cd /home/action/code/2021/TR/ABU2021_PC_TR/build/src/lidarForPosition && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/action/code/2021/TR/ABU2021_PC_TR/src/lidarForPosition/src/serial.cpp > CMakeFiles/URG_Lidar.dir/src/serial.cpp.i

src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/serial.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/URG_Lidar.dir/src/serial.cpp.s"
	cd /home/action/code/2021/TR/ABU2021_PC_TR/build/src/lidarForPosition && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/action/code/2021/TR/ABU2021_PC_TR/src/lidarForPosition/src/serial.cpp -o CMakeFiles/URG_Lidar.dir/src/serial.cpp.s

src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/serial.cpp.o.requires:

.PHONY : src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/serial.cpp.o.requires

src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/serial.cpp.o.provides: src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/serial.cpp.o.requires
	$(MAKE) -f src/lidarForPosition/CMakeFiles/URG_Lidar.dir/build.make src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/serial.cpp.o.provides.build
.PHONY : src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/serial.cpp.o.provides

src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/serial.cpp.o.provides.build: src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/serial.cpp.o


# Object files for target URG_Lidar
URG_Lidar_OBJECTS = \
"CMakeFiles/URG_Lidar.dir/src/data.cpp.o" \
"CMakeFiles/URG_Lidar.dir/src/main.cpp.o" \
"CMakeFiles/URG_Lidar.dir/src/map.cpp.o" \
"CMakeFiles/URG_Lidar.dir/src/pos.cpp.o" \
"CMakeFiles/URG_Lidar.dir/src/pps.cpp.o" \
"CMakeFiles/URG_Lidar.dir/src/serial.cpp.o"

# External object files for target URG_Lidar
URG_Lidar_EXTERNAL_OBJECTS =

../bin/URG_Lidar: src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/data.cpp.o
../bin/URG_Lidar: src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/main.cpp.o
../bin/URG_Lidar: src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/map.cpp.o
../bin/URG_Lidar: src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/pos.cpp.o
../bin/URG_Lidar: src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/pps.cpp.o
../bin/URG_Lidar: src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/serial.cpp.o
../bin/URG_Lidar: src/lidarForPosition/CMakeFiles/URG_Lidar.dir/build.make
../bin/URG_Lidar: ../src/lidarForPosition/urg_library-1.2.5/src/liburg_cpp.so
../bin/URG_Lidar: ../src/lidarForPosition/urg_library-1.2.5/src/liburg_c.so
../bin/URG_Lidar: /usr/lib/aarch64-linux-gnu/libopencv_dnn.so.4.1.1
../bin/URG_Lidar: /usr/lib/aarch64-linux-gnu/libopencv_gapi.so.4.1.1
../bin/URG_Lidar: /usr/lib/aarch64-linux-gnu/libopencv_highgui.so.4.1.1
../bin/URG_Lidar: /usr/lib/aarch64-linux-gnu/libopencv_ml.so.4.1.1
../bin/URG_Lidar: /usr/lib/aarch64-linux-gnu/libopencv_objdetect.so.4.1.1
../bin/URG_Lidar: /usr/lib/aarch64-linux-gnu/libopencv_photo.so.4.1.1
../bin/URG_Lidar: /usr/lib/aarch64-linux-gnu/libopencv_stitching.so.4.1.1
../bin/URG_Lidar: /usr/lib/aarch64-linux-gnu/libopencv_video.so.4.1.1
../bin/URG_Lidar: /usr/lib/aarch64-linux-gnu/libopencv_videoio.so.4.1.1
../bin/URG_Lidar: wrappers/process_communication/libprocess_communication.a
../bin/URG_Lidar: /usr/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.4.1.1
../bin/URG_Lidar: /usr/lib/aarch64-linux-gnu/libopencv_calib3d.so.4.1.1
../bin/URG_Lidar: /usr/lib/aarch64-linux-gnu/libopencv_features2d.so.4.1.1
../bin/URG_Lidar: /usr/lib/aarch64-linux-gnu/libopencv_flann.so.4.1.1
../bin/URG_Lidar: /usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.4.1.1
../bin/URG_Lidar: /usr/lib/aarch64-linux-gnu/libopencv_core.so.4.1.1
../bin/URG_Lidar: src/lidarForPosition/CMakeFiles/URG_Lidar.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/action/code/2021/TR/ABU2021_PC_TR/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX executable ../../../bin/URG_Lidar"
	cd /home/action/code/2021/TR/ABU2021_PC_TR/build/src/lidarForPosition && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/URG_Lidar.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/lidarForPosition/CMakeFiles/URG_Lidar.dir/build: ../bin/URG_Lidar

.PHONY : src/lidarForPosition/CMakeFiles/URG_Lidar.dir/build

src/lidarForPosition/CMakeFiles/URG_Lidar.dir/requires: src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/data.cpp.o.requires
src/lidarForPosition/CMakeFiles/URG_Lidar.dir/requires: src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/main.cpp.o.requires
src/lidarForPosition/CMakeFiles/URG_Lidar.dir/requires: src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/map.cpp.o.requires
src/lidarForPosition/CMakeFiles/URG_Lidar.dir/requires: src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/pos.cpp.o.requires
src/lidarForPosition/CMakeFiles/URG_Lidar.dir/requires: src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/pps.cpp.o.requires
src/lidarForPosition/CMakeFiles/URG_Lidar.dir/requires: src/lidarForPosition/CMakeFiles/URG_Lidar.dir/src/serial.cpp.o.requires

.PHONY : src/lidarForPosition/CMakeFiles/URG_Lidar.dir/requires

src/lidarForPosition/CMakeFiles/URG_Lidar.dir/clean:
	cd /home/action/code/2021/TR/ABU2021_PC_TR/build/src/lidarForPosition && $(CMAKE_COMMAND) -P CMakeFiles/URG_Lidar.dir/cmake_clean.cmake
.PHONY : src/lidarForPosition/CMakeFiles/URG_Lidar.dir/clean

src/lidarForPosition/CMakeFiles/URG_Lidar.dir/depend:
	cd /home/action/code/2021/TR/ABU2021_PC_TR/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/action/code/2021/TR/ABU2021_PC_TR /home/action/code/2021/TR/ABU2021_PC_TR/src/lidarForPosition /home/action/code/2021/TR/ABU2021_PC_TR/build /home/action/code/2021/TR/ABU2021_PC_TR/build/src/lidarForPosition /home/action/code/2021/TR/ABU2021_PC_TR/build/src/lidarForPosition/CMakeFiles/URG_Lidar.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/lidarForPosition/CMakeFiles/URG_Lidar.dir/depend

