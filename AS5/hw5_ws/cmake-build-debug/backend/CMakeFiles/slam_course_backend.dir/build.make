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
include backend/CMakeFiles/slam_course_backend.dir/depend.make

# Include the progress variables for this target.
include backend/CMakeFiles/slam_course_backend.dir/progress.make

# Include the compile flags for this target's objects.
include backend/CMakeFiles/slam_course_backend.dir/flags.make

backend/CMakeFiles/slam_course_backend.dir/vertex.cc.o: backend/CMakeFiles/slam_course_backend.dir/flags.make
backend/CMakeFiles/slam_course_backend.dir/vertex.cc.o: ../backend/vertex.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object backend/CMakeFiles/slam_course_backend.dir/vertex.cc.o"
	cd /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/backend && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/slam_course_backend.dir/vertex.cc.o -c /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/backend/vertex.cc

backend/CMakeFiles/slam_course_backend.dir/vertex.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/slam_course_backend.dir/vertex.cc.i"
	cd /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/backend && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/backend/vertex.cc > CMakeFiles/slam_course_backend.dir/vertex.cc.i

backend/CMakeFiles/slam_course_backend.dir/vertex.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/slam_course_backend.dir/vertex.cc.s"
	cd /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/backend && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/backend/vertex.cc -o CMakeFiles/slam_course_backend.dir/vertex.cc.s

backend/CMakeFiles/slam_course_backend.dir/edge.cc.o: backend/CMakeFiles/slam_course_backend.dir/flags.make
backend/CMakeFiles/slam_course_backend.dir/edge.cc.o: ../backend/edge.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object backend/CMakeFiles/slam_course_backend.dir/edge.cc.o"
	cd /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/backend && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/slam_course_backend.dir/edge.cc.o -c /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/backend/edge.cc

backend/CMakeFiles/slam_course_backend.dir/edge.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/slam_course_backend.dir/edge.cc.i"
	cd /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/backend && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/backend/edge.cc > CMakeFiles/slam_course_backend.dir/edge.cc.i

backend/CMakeFiles/slam_course_backend.dir/edge.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/slam_course_backend.dir/edge.cc.s"
	cd /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/backend && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/backend/edge.cc -o CMakeFiles/slam_course_backend.dir/edge.cc.s

backend/CMakeFiles/slam_course_backend.dir/problem.cc.o: backend/CMakeFiles/slam_course_backend.dir/flags.make
backend/CMakeFiles/slam_course_backend.dir/problem.cc.o: ../backend/problem.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object backend/CMakeFiles/slam_course_backend.dir/problem.cc.o"
	cd /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/backend && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/slam_course_backend.dir/problem.cc.o -c /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/backend/problem.cc

backend/CMakeFiles/slam_course_backend.dir/problem.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/slam_course_backend.dir/problem.cc.i"
	cd /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/backend && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/backend/problem.cc > CMakeFiles/slam_course_backend.dir/problem.cc.i

backend/CMakeFiles/slam_course_backend.dir/problem.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/slam_course_backend.dir/problem.cc.s"
	cd /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/backend && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/backend/problem.cc -o CMakeFiles/slam_course_backend.dir/problem.cc.s

backend/CMakeFiles/slam_course_backend.dir/vertex_pose.cc.o: backend/CMakeFiles/slam_course_backend.dir/flags.make
backend/CMakeFiles/slam_course_backend.dir/vertex_pose.cc.o: ../backend/vertex_pose.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object backend/CMakeFiles/slam_course_backend.dir/vertex_pose.cc.o"
	cd /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/backend && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/slam_course_backend.dir/vertex_pose.cc.o -c /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/backend/vertex_pose.cc

backend/CMakeFiles/slam_course_backend.dir/vertex_pose.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/slam_course_backend.dir/vertex_pose.cc.i"
	cd /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/backend && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/backend/vertex_pose.cc > CMakeFiles/slam_course_backend.dir/vertex_pose.cc.i

backend/CMakeFiles/slam_course_backend.dir/vertex_pose.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/slam_course_backend.dir/vertex_pose.cc.s"
	cd /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/backend && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/backend/vertex_pose.cc -o CMakeFiles/slam_course_backend.dir/vertex_pose.cc.s

backend/CMakeFiles/slam_course_backend.dir/edge_reprojection.cc.o: backend/CMakeFiles/slam_course_backend.dir/flags.make
backend/CMakeFiles/slam_course_backend.dir/edge_reprojection.cc.o: ../backend/edge_reprojection.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object backend/CMakeFiles/slam_course_backend.dir/edge_reprojection.cc.o"
	cd /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/backend && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/slam_course_backend.dir/edge_reprojection.cc.o -c /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/backend/edge_reprojection.cc

backend/CMakeFiles/slam_course_backend.dir/edge_reprojection.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/slam_course_backend.dir/edge_reprojection.cc.i"
	cd /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/backend && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/backend/edge_reprojection.cc > CMakeFiles/slam_course_backend.dir/edge_reprojection.cc.i

backend/CMakeFiles/slam_course_backend.dir/edge_reprojection.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/slam_course_backend.dir/edge_reprojection.cc.s"
	cd /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/backend && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/backend/edge_reprojection.cc -o CMakeFiles/slam_course_backend.dir/edge_reprojection.cc.s

backend/CMakeFiles/slam_course_backend.dir/edge_imu.cc.o: backend/CMakeFiles/slam_course_backend.dir/flags.make
backend/CMakeFiles/slam_course_backend.dir/edge_imu.cc.o: ../backend/edge_imu.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object backend/CMakeFiles/slam_course_backend.dir/edge_imu.cc.o"
	cd /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/backend && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/slam_course_backend.dir/edge_imu.cc.o -c /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/backend/edge_imu.cc

backend/CMakeFiles/slam_course_backend.dir/edge_imu.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/slam_course_backend.dir/edge_imu.cc.i"
	cd /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/backend && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/backend/edge_imu.cc > CMakeFiles/slam_course_backend.dir/edge_imu.cc.i

backend/CMakeFiles/slam_course_backend.dir/edge_imu.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/slam_course_backend.dir/edge_imu.cc.s"
	cd /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/backend && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/backend/edge_imu.cc -o CMakeFiles/slam_course_backend.dir/edge_imu.cc.s

backend/CMakeFiles/slam_course_backend.dir/edge_prior.cpp.o: backend/CMakeFiles/slam_course_backend.dir/flags.make
backend/CMakeFiles/slam_course_backend.dir/edge_prior.cpp.o: ../backend/edge_prior.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object backend/CMakeFiles/slam_course_backend.dir/edge_prior.cpp.o"
	cd /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/backend && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/slam_course_backend.dir/edge_prior.cpp.o -c /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/backend/edge_prior.cpp

backend/CMakeFiles/slam_course_backend.dir/edge_prior.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/slam_course_backend.dir/edge_prior.cpp.i"
	cd /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/backend && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/backend/edge_prior.cpp > CMakeFiles/slam_course_backend.dir/edge_prior.cpp.i

backend/CMakeFiles/slam_course_backend.dir/edge_prior.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/slam_course_backend.dir/edge_prior.cpp.s"
	cd /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/backend && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/backend/edge_prior.cpp -o CMakeFiles/slam_course_backend.dir/edge_prior.cpp.s

backend/CMakeFiles/slam_course_backend.dir/loss_function.cc.o: backend/CMakeFiles/slam_course_backend.dir/flags.make
backend/CMakeFiles/slam_course_backend.dir/loss_function.cc.o: ../backend/loss_function.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object backend/CMakeFiles/slam_course_backend.dir/loss_function.cc.o"
	cd /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/backend && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/slam_course_backend.dir/loss_function.cc.o -c /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/backend/loss_function.cc

backend/CMakeFiles/slam_course_backend.dir/loss_function.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/slam_course_backend.dir/loss_function.cc.i"
	cd /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/backend && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/backend/loss_function.cc > CMakeFiles/slam_course_backend.dir/loss_function.cc.i

backend/CMakeFiles/slam_course_backend.dir/loss_function.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/slam_course_backend.dir/loss_function.cc.s"
	cd /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/backend && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/backend/loss_function.cc -o CMakeFiles/slam_course_backend.dir/loss_function.cc.s

backend/CMakeFiles/slam_course_backend.dir/imu_integration.cc.o: backend/CMakeFiles/slam_course_backend.dir/flags.make
backend/CMakeFiles/slam_course_backend.dir/imu_integration.cc.o: ../backend/imu_integration.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object backend/CMakeFiles/slam_course_backend.dir/imu_integration.cc.o"
	cd /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/backend && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/slam_course_backend.dir/imu_integration.cc.o -c /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/backend/imu_integration.cc

backend/CMakeFiles/slam_course_backend.dir/imu_integration.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/slam_course_backend.dir/imu_integration.cc.i"
	cd /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/backend && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/backend/imu_integration.cc > CMakeFiles/slam_course_backend.dir/imu_integration.cc.i

backend/CMakeFiles/slam_course_backend.dir/imu_integration.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/slam_course_backend.dir/imu_integration.cc.s"
	cd /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/backend && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/backend/imu_integration.cc -o CMakeFiles/slam_course_backend.dir/imu_integration.cc.s

# Object files for target slam_course_backend
slam_course_backend_OBJECTS = \
"CMakeFiles/slam_course_backend.dir/vertex.cc.o" \
"CMakeFiles/slam_course_backend.dir/edge.cc.o" \
"CMakeFiles/slam_course_backend.dir/problem.cc.o" \
"CMakeFiles/slam_course_backend.dir/vertex_pose.cc.o" \
"CMakeFiles/slam_course_backend.dir/edge_reprojection.cc.o" \
"CMakeFiles/slam_course_backend.dir/edge_imu.cc.o" \
"CMakeFiles/slam_course_backend.dir/edge_prior.cpp.o" \
"CMakeFiles/slam_course_backend.dir/loss_function.cc.o" \
"CMakeFiles/slam_course_backend.dir/imu_integration.cc.o"

# External object files for target slam_course_backend
slam_course_backend_EXTERNAL_OBJECTS =

backend/libslam_course_backend.a: backend/CMakeFiles/slam_course_backend.dir/vertex.cc.o
backend/libslam_course_backend.a: backend/CMakeFiles/slam_course_backend.dir/edge.cc.o
backend/libslam_course_backend.a: backend/CMakeFiles/slam_course_backend.dir/problem.cc.o
backend/libslam_course_backend.a: backend/CMakeFiles/slam_course_backend.dir/vertex_pose.cc.o
backend/libslam_course_backend.a: backend/CMakeFiles/slam_course_backend.dir/edge_reprojection.cc.o
backend/libslam_course_backend.a: backend/CMakeFiles/slam_course_backend.dir/edge_imu.cc.o
backend/libslam_course_backend.a: backend/CMakeFiles/slam_course_backend.dir/edge_prior.cpp.o
backend/libslam_course_backend.a: backend/CMakeFiles/slam_course_backend.dir/loss_function.cc.o
backend/libslam_course_backend.a: backend/CMakeFiles/slam_course_backend.dir/imu_integration.cc.o
backend/libslam_course_backend.a: backend/CMakeFiles/slam_course_backend.dir/build.make
backend/libslam_course_backend.a: backend/CMakeFiles/slam_course_backend.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Linking CXX static library libslam_course_backend.a"
	cd /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/backend && $(CMAKE_COMMAND) -P CMakeFiles/slam_course_backend.dir/cmake_clean_target.cmake
	cd /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/backend && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/slam_course_backend.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
backend/CMakeFiles/slam_course_backend.dir/build: backend/libslam_course_backend.a

.PHONY : backend/CMakeFiles/slam_course_backend.dir/build

backend/CMakeFiles/slam_course_backend.dir/clean:
	cd /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/backend && $(CMAKE_COMMAND) -P CMakeFiles/slam_course_backend.dir/cmake_clean.cmake
.PHONY : backend/CMakeFiles/slam_course_backend.dir/clean

backend/CMakeFiles/slam_course_backend.dir/depend:
	cd /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/backend /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/backend /home/wenyan/Documents/shenlan/VIO/AS/hw_course5_new/cmake-build-debug/backend/CMakeFiles/slam_course_backend.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : backend/CMakeFiles/slam_course_backend.dir/depend

