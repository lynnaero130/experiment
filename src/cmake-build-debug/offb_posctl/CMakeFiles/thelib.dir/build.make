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
CMAKE_COMMAND = /home/uav/CLion-2019.2.4/clion-2019.2.4/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/uav/CLion-2019.2.4/clion-2019.2.4/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/uav/lzy_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/uav/lzy_ws/src/cmake-build-debug

# Include any dependencies generated for this target.
include offb_posctl/CMakeFiles/thelib.dir/depend.make

# Include the progress variables for this target.
include offb_posctl/CMakeFiles/thelib.dir/progress.make

# Include the compile flags for this target's objects.
include offb_posctl/CMakeFiles/thelib.dir/flags.make

offb_posctl/CMakeFiles/thelib.dir/src/PID.cpp.o: offb_posctl/CMakeFiles/thelib.dir/flags.make
offb_posctl/CMakeFiles/thelib.dir/src/PID.cpp.o: ../offb_posctl/src/PID.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/uav/lzy_ws/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object offb_posctl/CMakeFiles/thelib.dir/src/PID.cpp.o"
	cd /home/uav/lzy_ws/src/cmake-build-debug/offb_posctl && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/thelib.dir/src/PID.cpp.o -c /home/uav/lzy_ws/src/offb_posctl/src/PID.cpp

offb_posctl/CMakeFiles/thelib.dir/src/PID.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/thelib.dir/src/PID.cpp.i"
	cd /home/uav/lzy_ws/src/cmake-build-debug/offb_posctl && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/uav/lzy_ws/src/offb_posctl/src/PID.cpp > CMakeFiles/thelib.dir/src/PID.cpp.i

offb_posctl/CMakeFiles/thelib.dir/src/PID.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/thelib.dir/src/PID.cpp.s"
	cd /home/uav/lzy_ws/src/cmake-build-debug/offb_posctl && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/uav/lzy_ws/src/offb_posctl/src/PID.cpp -o CMakeFiles/thelib.dir/src/PID.cpp.s

offb_posctl/CMakeFiles/thelib.dir/src/Parameter.cpp.o: offb_posctl/CMakeFiles/thelib.dir/flags.make
offb_posctl/CMakeFiles/thelib.dir/src/Parameter.cpp.o: ../offb_posctl/src/Parameter.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/uav/lzy_ws/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object offb_posctl/CMakeFiles/thelib.dir/src/Parameter.cpp.o"
	cd /home/uav/lzy_ws/src/cmake-build-debug/offb_posctl && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/thelib.dir/src/Parameter.cpp.o -c /home/uav/lzy_ws/src/offb_posctl/src/Parameter.cpp

offb_posctl/CMakeFiles/thelib.dir/src/Parameter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/thelib.dir/src/Parameter.cpp.i"
	cd /home/uav/lzy_ws/src/cmake-build-debug/offb_posctl && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/uav/lzy_ws/src/offb_posctl/src/Parameter.cpp > CMakeFiles/thelib.dir/src/Parameter.cpp.i

offb_posctl/CMakeFiles/thelib.dir/src/Parameter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/thelib.dir/src/Parameter.cpp.s"
	cd /home/uav/lzy_ws/src/cmake-build-debug/offb_posctl && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/uav/lzy_ws/src/offb_posctl/src/Parameter.cpp -o CMakeFiles/thelib.dir/src/Parameter.cpp.s

# Object files for target thelib
thelib_OBJECTS = \
"CMakeFiles/thelib.dir/src/PID.cpp.o" \
"CMakeFiles/thelib.dir/src/Parameter.cpp.o"

# External object files for target thelib
thelib_EXTERNAL_OBJECTS =

devel/lib/libthelib.so: offb_posctl/CMakeFiles/thelib.dir/src/PID.cpp.o
devel/lib/libthelib.so: offb_posctl/CMakeFiles/thelib.dir/src/Parameter.cpp.o
devel/lib/libthelib.so: offb_posctl/CMakeFiles/thelib.dir/build.make
devel/lib/libthelib.so: offb_posctl/CMakeFiles/thelib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/uav/lzy_ws/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX shared library ../devel/lib/libthelib.so"
	cd /home/uav/lzy_ws/src/cmake-build-debug/offb_posctl && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/thelib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
offb_posctl/CMakeFiles/thelib.dir/build: devel/lib/libthelib.so

.PHONY : offb_posctl/CMakeFiles/thelib.dir/build

offb_posctl/CMakeFiles/thelib.dir/clean:
	cd /home/uav/lzy_ws/src/cmake-build-debug/offb_posctl && $(CMAKE_COMMAND) -P CMakeFiles/thelib.dir/cmake_clean.cmake
.PHONY : offb_posctl/CMakeFiles/thelib.dir/clean

offb_posctl/CMakeFiles/thelib.dir/depend:
	cd /home/uav/lzy_ws/src/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/uav/lzy_ws/src /home/uav/lzy_ws/src/offb_posctl /home/uav/lzy_ws/src/cmake-build-debug /home/uav/lzy_ws/src/cmake-build-debug/offb_posctl /home/uav/lzy_ws/src/cmake-build-debug/offb_posctl/CMakeFiles/thelib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : offb_posctl/CMakeFiles/thelib.dir/depend

