# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

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
CMAKE_COMMAND = /home/wenyan/Downloads/CLion-2020.3/clion-2020.3/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/wenyan/Downloads/CLion-2020.3/clion-2020.3/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug

# Include any dependencies generated for this target.
include app/CMakeFiles/testCurveFitting.dir/depend.make

# Include the progress variables for this target.
include app/CMakeFiles/testCurveFitting.dir/progress.make

# Include the compile flags for this target's objects.
include app/CMakeFiles/testCurveFitting.dir/flags.make

app/CMakeFiles/testCurveFitting.dir/CurveFitting.cpp.o: app/CMakeFiles/testCurveFitting.dir/flags.make
app/CMakeFiles/testCurveFitting.dir/CurveFitting.cpp.o: ../app/CurveFitting.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object app/CMakeFiles/testCurveFitting.dir/CurveFitting.cpp.o"
	cd /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/app && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/testCurveFitting.dir/CurveFitting.cpp.o -c /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/app/CurveFitting.cpp

app/CMakeFiles/testCurveFitting.dir/CurveFitting.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testCurveFitting.dir/CurveFitting.cpp.i"
	cd /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/app && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/app/CurveFitting.cpp > CMakeFiles/testCurveFitting.dir/CurveFitting.cpp.i

app/CMakeFiles/testCurveFitting.dir/CurveFitting.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testCurveFitting.dir/CurveFitting.cpp.s"
	cd /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/app && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/app/CurveFitting.cpp -o CMakeFiles/testCurveFitting.dir/CurveFitting.cpp.s

# Object files for target testCurveFitting
testCurveFitting_OBJECTS = \
"CMakeFiles/testCurveFitting.dir/CurveFitting.cpp.o"

# External object files for target testCurveFitting
testCurveFitting_EXTERNAL_OBJECTS =

app/testCurveFitting: app/CMakeFiles/testCurveFitting.dir/CurveFitting.cpp.o
app/testCurveFitting: app/CMakeFiles/testCurveFitting.dir/build.make
app/testCurveFitting: backend/libslam_course_backend.a
app/testCurveFitting: app/CMakeFiles/testCurveFitting.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable testCurveFitting"
	cd /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/app && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/testCurveFitting.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
app/CMakeFiles/testCurveFitting.dir/build: app/testCurveFitting

.PHONY : app/CMakeFiles/testCurveFitting.dir/build

app/CMakeFiles/testCurveFitting.dir/clean:
	cd /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/app && $(CMAKE_COMMAND) -P CMakeFiles/testCurveFitting.dir/cmake_clean.cmake
.PHONY : app/CMakeFiles/testCurveFitting.dir/clean

app/CMakeFiles/testCurveFitting.dir/depend:
	cd /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/app /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/app /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/app/CMakeFiles/testCurveFitting.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : app/CMakeFiles/testCurveFitting.dir/depend

