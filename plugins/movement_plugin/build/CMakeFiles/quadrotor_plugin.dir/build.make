# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/guido/Scaricati/Gazebosim-main/plugins/movement_plugin

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/guido/Scaricati/Gazebosim-main/plugins/movement_plugin/build

# Include any dependencies generated for this target.
include CMakeFiles/quadrotor_plugin.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/quadrotor_plugin.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/quadrotor_plugin.dir/flags.make

CMakeFiles/quadrotor_plugin.dir/quadrotor_plugin.cpp.o: CMakeFiles/quadrotor_plugin.dir/flags.make
CMakeFiles/quadrotor_plugin.dir/quadrotor_plugin.cpp.o: ../quadrotor_plugin.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/guido/Scaricati/Gazebosim-main/plugins/movement_plugin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/quadrotor_plugin.dir/quadrotor_plugin.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/quadrotor_plugin.dir/quadrotor_plugin.cpp.o -c /home/guido/Scaricati/Gazebosim-main/plugins/movement_plugin/quadrotor_plugin.cpp

CMakeFiles/quadrotor_plugin.dir/quadrotor_plugin.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/quadrotor_plugin.dir/quadrotor_plugin.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/guido/Scaricati/Gazebosim-main/plugins/movement_plugin/quadrotor_plugin.cpp > CMakeFiles/quadrotor_plugin.dir/quadrotor_plugin.cpp.i

CMakeFiles/quadrotor_plugin.dir/quadrotor_plugin.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/quadrotor_plugin.dir/quadrotor_plugin.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/guido/Scaricati/Gazebosim-main/plugins/movement_plugin/quadrotor_plugin.cpp -o CMakeFiles/quadrotor_plugin.dir/quadrotor_plugin.cpp.s

CMakeFiles/quadrotor_plugin.dir/quadrotor_plugin.cpp.o.requires:

.PHONY : CMakeFiles/quadrotor_plugin.dir/quadrotor_plugin.cpp.o.requires

CMakeFiles/quadrotor_plugin.dir/quadrotor_plugin.cpp.o.provides: CMakeFiles/quadrotor_plugin.dir/quadrotor_plugin.cpp.o.requires
	$(MAKE) -f CMakeFiles/quadrotor_plugin.dir/build.make CMakeFiles/quadrotor_plugin.dir/quadrotor_plugin.cpp.o.provides.build
.PHONY : CMakeFiles/quadrotor_plugin.dir/quadrotor_plugin.cpp.o.provides

CMakeFiles/quadrotor_plugin.dir/quadrotor_plugin.cpp.o.provides.build: CMakeFiles/quadrotor_plugin.dir/quadrotor_plugin.cpp.o


# Object files for target quadrotor_plugin
quadrotor_plugin_OBJECTS = \
"CMakeFiles/quadrotor_plugin.dir/quadrotor_plugin.cpp.o"

# External object files for target quadrotor_plugin
quadrotor_plugin_EXTERNAL_OBJECTS =

libquadrotor_plugin.so: CMakeFiles/quadrotor_plugin.dir/quadrotor_plugin.cpp.o
libquadrotor_plugin.so: CMakeFiles/quadrotor_plugin.dir/build.make
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libSimTKsimbody.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libSimTKmath.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libSimTKcommon.so
libquadrotor_plugin.so: /usr/lib/libblas.so
libquadrotor_plugin.so: /usr/lib/liblapack.so
libquadrotor_plugin.so: /usr/lib/libblas.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_client.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_ode.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_transport.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_util.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_common.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_ccd.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libpthread.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libsdformat.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libpthread.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libignition-transport4.so.4.0.0
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libignition-msgs1.so.1.0.0
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libignition-common1.so.1.1.1
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libignition-fuel_tools1.so.1.2.0
libquadrotor_plugin.so: /usr/lib/liblapack.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_client.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_ode.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_transport.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_util.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_common.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_ccd.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libpthread.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libsdformat.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libpthread.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libsdformat.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libignition-math4.so.4.0.0
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libuuid.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libuuid.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libswscale-ffmpeg.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libswscale-ffmpeg.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libavdevice-ffmpeg.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libavdevice-ffmpeg.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libavformat-ffmpeg.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libavformat-ffmpeg.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libavcodec-ffmpeg.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libavcodec-ffmpeg.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libavutil-ffmpeg.so
libquadrotor_plugin.so: /usr/lib/x86_64-linux-gnu/libavutil-ffmpeg.so
libquadrotor_plugin.so: CMakeFiles/quadrotor_plugin.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/guido/Scaricati/Gazebosim-main/plugins/movement_plugin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libquadrotor_plugin.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/quadrotor_plugin.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/quadrotor_plugin.dir/build: libquadrotor_plugin.so

.PHONY : CMakeFiles/quadrotor_plugin.dir/build

CMakeFiles/quadrotor_plugin.dir/requires: CMakeFiles/quadrotor_plugin.dir/quadrotor_plugin.cpp.o.requires

.PHONY : CMakeFiles/quadrotor_plugin.dir/requires

CMakeFiles/quadrotor_plugin.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/quadrotor_plugin.dir/cmake_clean.cmake
.PHONY : CMakeFiles/quadrotor_plugin.dir/clean

CMakeFiles/quadrotor_plugin.dir/depend:
	cd /home/guido/Scaricati/Gazebosim-main/plugins/movement_plugin/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/guido/Scaricati/Gazebosim-main/plugins/movement_plugin /home/guido/Scaricati/Gazebosim-main/plugins/movement_plugin /home/guido/Scaricati/Gazebosim-main/plugins/movement_plugin/build /home/guido/Scaricati/Gazebosim-main/plugins/movement_plugin/build /home/guido/Scaricati/Gazebosim-main/plugins/movement_plugin/build/CMakeFiles/quadrotor_plugin.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/quadrotor_plugin.dir/depend
