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
CMAKE_SOURCE_DIR = /home/kollaada/MyPlugin

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kollaada/MyPlugin/build

# Include any dependencies generated for this target.
include src/gazeboplugin/CMakeFiles/GazeboCoSimPlugin.dir/depend.make

# Include the progress variables for this target.
include src/gazeboplugin/CMakeFiles/GazeboCoSimPlugin.dir/progress.make

# Include the compile flags for this target's objects.
include src/gazeboplugin/CMakeFiles/GazeboCoSimPlugin.dir/flags.make

src/gazeboplugin/CMakeFiles/GazeboCoSimPlugin.dir/GazeboPlugin.cpp.o: src/gazeboplugin/CMakeFiles/GazeboCoSimPlugin.dir/flags.make
src/gazeboplugin/CMakeFiles/GazeboCoSimPlugin.dir/GazeboPlugin.cpp.o: ../src/gazeboplugin/GazeboPlugin.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kollaada/MyPlugin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/gazeboplugin/CMakeFiles/GazeboCoSimPlugin.dir/GazeboPlugin.cpp.o"
	cd /home/kollaada/MyPlugin/build/src/gazeboplugin && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/GazeboCoSimPlugin.dir/GazeboPlugin.cpp.o -c /home/kollaada/MyPlugin/src/gazeboplugin/GazeboPlugin.cpp

src/gazeboplugin/CMakeFiles/GazeboCoSimPlugin.dir/GazeboPlugin.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/GazeboCoSimPlugin.dir/GazeboPlugin.cpp.i"
	cd /home/kollaada/MyPlugin/build/src/gazeboplugin && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kollaada/MyPlugin/src/gazeboplugin/GazeboPlugin.cpp > CMakeFiles/GazeboCoSimPlugin.dir/GazeboPlugin.cpp.i

src/gazeboplugin/CMakeFiles/GazeboCoSimPlugin.dir/GazeboPlugin.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/GazeboCoSimPlugin.dir/GazeboPlugin.cpp.s"
	cd /home/kollaada/MyPlugin/build/src/gazeboplugin && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kollaada/MyPlugin/src/gazeboplugin/GazeboPlugin.cpp -o CMakeFiles/GazeboCoSimPlugin.dir/GazeboPlugin.cpp.s

# Object files for target GazeboCoSimPlugin
GazeboCoSimPlugin_OBJECTS = \
"CMakeFiles/GazeboCoSimPlugin.dir/GazeboPlugin.cpp.o"

# External object files for target GazeboCoSimPlugin
GazeboCoSimPlugin_EXTERNAL_OBJECTS =

../export/lib/libGazeboCoSimPlugin.so: src/gazeboplugin/CMakeFiles/GazeboCoSimPlugin.dir/GazeboPlugin.cpp.o
../export/lib/libGazeboCoSimPlugin.so: src/gazeboplugin/CMakeFiles/GazeboCoSimPlugin.dir/build.make
../export/lib/libGazeboCoSimPlugin.so: /usr/local/lib/libgazebo.so
../export/lib/libGazeboCoSimPlugin.so: /usr/local/lib/libgazebo_client.so
../export/lib/libGazeboCoSimPlugin.so: /usr/local/lib/libgazebo_gui.so
../export/lib/libGazeboCoSimPlugin.so: /usr/local/lib/libgazebo_sensors.so
../export/lib/libGazeboCoSimPlugin.so: /usr/local/lib/libgazebo_rendering.so
../export/lib/libGazeboCoSimPlugin.so: /usr/local/lib/libgazebo_physics.so
../export/lib/libGazeboCoSimPlugin.so: /usr/local/lib/libgazebo_ode.so
../export/lib/libGazeboCoSimPlugin.so: /usr/local/lib/libgazebo_transport.so
../export/lib/libGazeboCoSimPlugin.so: /usr/local/lib/libgazebo_msgs.so
../export/lib/libGazeboCoSimPlugin.so: /usr/local/lib/libgazebo_util.so
../export/lib/libGazeboCoSimPlugin.so: /usr/local/lib/libgazebo_common.so
../export/lib/libGazeboCoSimPlugin.so: /usr/local/lib/libgazebo_gimpact.so
../export/lib/libGazeboCoSimPlugin.so: /usr/local/lib/libgazebo_opcode.so
../export/lib/libGazeboCoSimPlugin.so: /usr/local/lib/libgazebo_opende_ou.so
../export/lib/libGazeboCoSimPlugin.so: /usr/local/lib/libgazebo_ccd.so
../export/lib/libGazeboCoSimPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
../export/lib/libGazeboCoSimPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
../export/lib/libGazeboCoSimPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
../export/lib/libGazeboCoSimPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
../export/lib/libGazeboCoSimPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
../export/lib/libGazeboCoSimPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so.1.71.0
../export/lib/libGazeboCoSimPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
../export/lib/libGazeboCoSimPlugin.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
../export/lib/libGazeboCoSimPlugin.so: /usr/lib/x86_64-linux-gnu/libsdformat.so
../export/lib/libGazeboCoSimPlugin.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
../export/lib/libGazeboCoSimPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
../export/lib/libGazeboCoSimPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
../export/lib/libGazeboCoSimPlugin.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
../export/lib/libGazeboCoSimPlugin.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
../export/lib/libGazeboCoSimPlugin.so: /usr/local/lib/libignition-transport4.so.4.0.0
../export/lib/libGazeboCoSimPlugin.so: /usr/local/lib/libignition-msgs1.so.1.0.0
../export/lib/libGazeboCoSimPlugin.so: /usr/local/lib/libignition-fuel_tools1.so.1.2.0
../export/lib/libGazeboCoSimPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
../export/lib/libGazeboCoSimPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
../export/lib/libGazeboCoSimPlugin.so: ../export/lib/libGazeboCoSimServer.so
../export/lib/libGazeboCoSimPlugin.so: ../export/lib/libGazeboCoSimTransport.so
../export/lib/libGazeboCoSimPlugin.so: ../export/lib/libmsgproto.so
../export/lib/libGazeboCoSimPlugin.so: ../export/lib/libGazeboCoSimCustom.so
../export/lib/libGazeboCoSimPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so.1.71.0
../export/lib/libGazeboCoSimPlugin.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
../export/lib/libGazeboCoSimPlugin.so: /usr/local/lib/libignition-common1.so.1.1.1
../export/lib/libGazeboCoSimPlugin.so: /usr/local/lib/libignition-math4.so.4.0.0
../export/lib/libGazeboCoSimPlugin.so: /usr/lib/x86_64-linux-gnu/libuuid.so
../export/lib/libGazeboCoSimPlugin.so: /usr/lib/x86_64-linux-gnu/libuuid.so
../export/lib/libGazeboCoSimPlugin.so: /usr/lib/x86_64-linux-gnu/libswscale.so
../export/lib/libGazeboCoSimPlugin.so: /usr/lib/x86_64-linux-gnu/libswscale.so
../export/lib/libGazeboCoSimPlugin.so: /usr/lib/x86_64-linux-gnu/libavdevice.so
../export/lib/libGazeboCoSimPlugin.so: /usr/lib/x86_64-linux-gnu/libavdevice.so
../export/lib/libGazeboCoSimPlugin.so: /usr/lib/x86_64-linux-gnu/libavformat.so
../export/lib/libGazeboCoSimPlugin.so: /usr/lib/x86_64-linux-gnu/libavformat.so
../export/lib/libGazeboCoSimPlugin.so: /usr/lib/x86_64-linux-gnu/libavcodec.so
../export/lib/libGazeboCoSimPlugin.so: /usr/lib/x86_64-linux-gnu/libavcodec.so
../export/lib/libGazeboCoSimPlugin.so: /usr/lib/x86_64-linux-gnu/libavutil.so
../export/lib/libGazeboCoSimPlugin.so: /usr/lib/x86_64-linux-gnu/libavutil.so
../export/lib/libGazeboCoSimPlugin.so: src/gazeboplugin/CMakeFiles/GazeboCoSimPlugin.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kollaada/MyPlugin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library ../../../export/lib/libGazeboCoSimPlugin.so"
	cd /home/kollaada/MyPlugin/build/src/gazeboplugin && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/GazeboCoSimPlugin.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/gazeboplugin/CMakeFiles/GazeboCoSimPlugin.dir/build: ../export/lib/libGazeboCoSimPlugin.so

.PHONY : src/gazeboplugin/CMakeFiles/GazeboCoSimPlugin.dir/build

src/gazeboplugin/CMakeFiles/GazeboCoSimPlugin.dir/clean:
	cd /home/kollaada/MyPlugin/build/src/gazeboplugin && $(CMAKE_COMMAND) -P CMakeFiles/GazeboCoSimPlugin.dir/cmake_clean.cmake
.PHONY : src/gazeboplugin/CMakeFiles/GazeboCoSimPlugin.dir/clean

src/gazeboplugin/CMakeFiles/GazeboCoSimPlugin.dir/depend:
	cd /home/kollaada/MyPlugin/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kollaada/MyPlugin /home/kollaada/MyPlugin/src/gazeboplugin /home/kollaada/MyPlugin/build /home/kollaada/MyPlugin/build/src/gazeboplugin /home/kollaada/MyPlugin/build/src/gazeboplugin/CMakeFiles/GazeboCoSimPlugin.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/gazeboplugin/CMakeFiles/GazeboCoSimPlugin.dir/depend

