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
CMAKE_SOURCE_DIR = /home/wenyan/Documents/shenlan/VIO/AS/course6_hw

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/wenyan/Documents/shenlan/VIO/AS/course6_hw/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/estimate_depth.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/estimate_depth.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/estimate_depth.dir/flags.make

CMakeFiles/estimate_depth.dir/triangulate.cpp.o: CMakeFiles/estimate_depth.dir/flags.make
CMakeFiles/estimate_depth.dir/triangulate.cpp.o: ../triangulate.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wenyan/Documents/shenlan/VIO/AS/course6_hw/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/estimate_depth.dir/triangulate.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/estimate_depth.dir/triangulate.cpp.o -c /home/wenyan/Documents/shenlan/VIO/AS/course6_hw/triangulate.cpp

CMakeFiles/estimate_depth.dir/triangulate.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/estimate_depth.dir/triangulate.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wenyan/Documents/shenlan/VIO/AS/course6_hw/triangulate.cpp > CMakeFiles/estimate_depth.dir/triangulate.cpp.i

CMakeFiles/estimate_depth.dir/triangulate.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/estimate_depth.dir/triangulate.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wenyan/Documents/shenlan/VIO/AS/course6_hw/triangulate.cpp -o CMakeFiles/estimate_depth.dir/triangulate.cpp.s

# Object files for target estimate_depth
estimate_depth_OBJECTS = \
"CMakeFiles/estimate_depth.dir/triangulate.cpp.o"

# External object files for target estimate_depth
estimate_depth_EXTERNAL_OBJECTS =

estimate_depth: CMakeFiles/estimate_depth.dir/triangulate.cpp.o
estimate_depth: CMakeFiles/estimate_depth.dir/build.make
estimate_depth: CMakeFiles/estimate_depth.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/wenyan/Documents/shenlan/VIO/AS/course6_hw/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable estimate_depth"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/estimate_depth.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/estimate_depth.dir/build: estimate_depth

.PHONY : CMakeFiles/estimate_depth.dir/build

CMakeFiles/estimate_depth.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/estimate_depth.dir/cmake_clean.cmake
.PHONY : CMakeFiles/estimate_depth.dir/clean

CMakeFiles/estimate_depth.dir/depend:
	cd /home/wenyan/Documents/shenlan/VIO/AS/course6_hw/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/wenyan/Documents/shenlan/VIO/AS/course6_hw /home/wenyan/Documents/shenlan/VIO/AS/course6_hw /home/wenyan/Documents/shenlan/VIO/AS/course6_hw/cmake-build-debug /home/wenyan/Documents/shenlan/VIO/AS/course6_hw/cmake-build-debug /home/wenyan/Documents/shenlan/VIO/AS/course6_hw/cmake-build-debug/CMakeFiles/estimate_depth.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/estimate_depth.dir/depend

