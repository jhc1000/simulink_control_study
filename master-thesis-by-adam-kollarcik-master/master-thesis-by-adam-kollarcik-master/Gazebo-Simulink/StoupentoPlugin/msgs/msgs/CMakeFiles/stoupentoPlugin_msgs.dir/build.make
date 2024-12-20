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
CMAKE_SOURCE_DIR = /home/kollaada/stoupentoPlugin

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kollaada/stoupentoPlugin/msgs

# Include any dependencies generated for this target.
include msgs/CMakeFiles/stoupentoPlugin_msgs.dir/depend.make

# Include the progress variables for this target.
include msgs/CMakeFiles/stoupentoPlugin_msgs.dir/progress.make

# Include the compile flags for this target's objects.
include msgs/CMakeFiles/stoupentoPlugin_msgs.dir/flags.make

msgs/stoupento_mesurement.pb.h: stoupento_mesurement.proto
msgs/stoupento_mesurement.pb.h: /usr/bin/protoc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kollaada/stoupentoPlugin/msgs/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Running cpp protocol buffer compiler on stoupento_mesurement.proto"
	cd /home/kollaada/stoupentoPlugin/msgs/msgs && /usr/bin/protoc --cpp_out /home/kollaada/stoupentoPlugin/msgs/msgs -I /home/kollaada/stoupentoPlugin/msgs -I /usr/local/include/gazebo-10/gazebo/msgs/proto /home/kollaada/stoupentoPlugin/msgs/stoupento_mesurement.proto

msgs/stoupento_mesurement.pb.cc: msgs/stoupento_mesurement.pb.h
	@$(CMAKE_COMMAND) -E touch_nocreate msgs/stoupento_mesurement.pb.cc

msgs/controlCommand.pb.h: controlCommand.proto
msgs/controlCommand.pb.h: /usr/bin/protoc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kollaada/stoupentoPlugin/msgs/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Running cpp protocol buffer compiler on controlCommand.proto"
	cd /home/kollaada/stoupentoPlugin/msgs/msgs && /usr/bin/protoc --cpp_out /home/kollaada/stoupentoPlugin/msgs/msgs -I /home/kollaada/stoupentoPlugin/msgs -I /usr/local/include/gazebo-10/gazebo/msgs/proto /home/kollaada/stoupentoPlugin/msgs/controlCommand.proto

msgs/controlCommand.pb.cc: msgs/controlCommand.pb.h
	@$(CMAKE_COMMAND) -E touch_nocreate msgs/controlCommand.pb.cc

msgs/vector2d.pb.h: /usr/local/include/gazebo-10/gazebo/msgs/proto/vector2d.proto
msgs/vector2d.pb.h: /usr/bin/protoc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kollaada/stoupentoPlugin/msgs/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Running cpp protocol buffer compiler on /usr/local/include/gazebo-10/gazebo/msgs/proto/vector2d.proto"
	cd /home/kollaada/stoupentoPlugin/msgs/msgs && /usr/bin/protoc --cpp_out /home/kollaada/stoupentoPlugin/msgs/msgs -I /home/kollaada/stoupentoPlugin/msgs -I /usr/local/include/gazebo-10/gazebo/msgs/proto /usr/local/include/gazebo-10/gazebo/msgs/proto/vector2d.proto

msgs/vector2d.pb.cc: msgs/vector2d.pb.h
	@$(CMAKE_COMMAND) -E touch_nocreate msgs/vector2d.pb.cc

msgs/header.pb.h: /usr/local/include/gazebo-10/gazebo/msgs/proto/header.proto
msgs/header.pb.h: /usr/bin/protoc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kollaada/stoupentoPlugin/msgs/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Running cpp protocol buffer compiler on /usr/local/include/gazebo-10/gazebo/msgs/proto/header.proto"
	cd /home/kollaada/stoupentoPlugin/msgs/msgs && /usr/bin/protoc --cpp_out /home/kollaada/stoupentoPlugin/msgs/msgs -I /home/kollaada/stoupentoPlugin/msgs -I /usr/local/include/gazebo-10/gazebo/msgs/proto /usr/local/include/gazebo-10/gazebo/msgs/proto/header.proto

msgs/header.pb.cc: msgs/header.pb.h
	@$(CMAKE_COMMAND) -E touch_nocreate msgs/header.pb.cc

msgs/time.pb.h: /usr/local/include/gazebo-10/gazebo/msgs/proto/time.proto
msgs/time.pb.h: /usr/bin/protoc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kollaada/stoupentoPlugin/msgs/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Running cpp protocol buffer compiler on /usr/local/include/gazebo-10/gazebo/msgs/proto/time.proto"
	cd /home/kollaada/stoupentoPlugin/msgs/msgs && /usr/bin/protoc --cpp_out /home/kollaada/stoupentoPlugin/msgs/msgs -I /home/kollaada/stoupentoPlugin/msgs -I /usr/local/include/gazebo-10/gazebo/msgs/proto /usr/local/include/gazebo-10/gazebo/msgs/proto/time.proto

msgs/time.pb.cc: msgs/time.pb.h
	@$(CMAKE_COMMAND) -E touch_nocreate msgs/time.pb.cc

msgs/CMakeFiles/stoupentoPlugin_msgs.dir/stoupento_mesurement.pb.cc.o: msgs/CMakeFiles/stoupentoPlugin_msgs.dir/flags.make
msgs/CMakeFiles/stoupentoPlugin_msgs.dir/stoupento_mesurement.pb.cc.o: msgs/stoupento_mesurement.pb.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kollaada/stoupentoPlugin/msgs/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object msgs/CMakeFiles/stoupentoPlugin_msgs.dir/stoupento_mesurement.pb.cc.o"
	cd /home/kollaada/stoupentoPlugin/msgs/msgs && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stoupentoPlugin_msgs.dir/stoupento_mesurement.pb.cc.o -c /home/kollaada/stoupentoPlugin/msgs/msgs/stoupento_mesurement.pb.cc

msgs/CMakeFiles/stoupentoPlugin_msgs.dir/stoupento_mesurement.pb.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stoupentoPlugin_msgs.dir/stoupento_mesurement.pb.cc.i"
	cd /home/kollaada/stoupentoPlugin/msgs/msgs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kollaada/stoupentoPlugin/msgs/msgs/stoupento_mesurement.pb.cc > CMakeFiles/stoupentoPlugin_msgs.dir/stoupento_mesurement.pb.cc.i

msgs/CMakeFiles/stoupentoPlugin_msgs.dir/stoupento_mesurement.pb.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stoupentoPlugin_msgs.dir/stoupento_mesurement.pb.cc.s"
	cd /home/kollaada/stoupentoPlugin/msgs/msgs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kollaada/stoupentoPlugin/msgs/msgs/stoupento_mesurement.pb.cc -o CMakeFiles/stoupentoPlugin_msgs.dir/stoupento_mesurement.pb.cc.s

msgs/CMakeFiles/stoupentoPlugin_msgs.dir/controlCommand.pb.cc.o: msgs/CMakeFiles/stoupentoPlugin_msgs.dir/flags.make
msgs/CMakeFiles/stoupentoPlugin_msgs.dir/controlCommand.pb.cc.o: msgs/controlCommand.pb.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kollaada/stoupentoPlugin/msgs/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object msgs/CMakeFiles/stoupentoPlugin_msgs.dir/controlCommand.pb.cc.o"
	cd /home/kollaada/stoupentoPlugin/msgs/msgs && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stoupentoPlugin_msgs.dir/controlCommand.pb.cc.o -c /home/kollaada/stoupentoPlugin/msgs/msgs/controlCommand.pb.cc

msgs/CMakeFiles/stoupentoPlugin_msgs.dir/controlCommand.pb.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stoupentoPlugin_msgs.dir/controlCommand.pb.cc.i"
	cd /home/kollaada/stoupentoPlugin/msgs/msgs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kollaada/stoupentoPlugin/msgs/msgs/controlCommand.pb.cc > CMakeFiles/stoupentoPlugin_msgs.dir/controlCommand.pb.cc.i

msgs/CMakeFiles/stoupentoPlugin_msgs.dir/controlCommand.pb.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stoupentoPlugin_msgs.dir/controlCommand.pb.cc.s"
	cd /home/kollaada/stoupentoPlugin/msgs/msgs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kollaada/stoupentoPlugin/msgs/msgs/controlCommand.pb.cc -o CMakeFiles/stoupentoPlugin_msgs.dir/controlCommand.pb.cc.s

msgs/CMakeFiles/stoupentoPlugin_msgs.dir/vector2d.pb.cc.o: msgs/CMakeFiles/stoupentoPlugin_msgs.dir/flags.make
msgs/CMakeFiles/stoupentoPlugin_msgs.dir/vector2d.pb.cc.o: msgs/vector2d.pb.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kollaada/stoupentoPlugin/msgs/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object msgs/CMakeFiles/stoupentoPlugin_msgs.dir/vector2d.pb.cc.o"
	cd /home/kollaada/stoupentoPlugin/msgs/msgs && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stoupentoPlugin_msgs.dir/vector2d.pb.cc.o -c /home/kollaada/stoupentoPlugin/msgs/msgs/vector2d.pb.cc

msgs/CMakeFiles/stoupentoPlugin_msgs.dir/vector2d.pb.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stoupentoPlugin_msgs.dir/vector2d.pb.cc.i"
	cd /home/kollaada/stoupentoPlugin/msgs/msgs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kollaada/stoupentoPlugin/msgs/msgs/vector2d.pb.cc > CMakeFiles/stoupentoPlugin_msgs.dir/vector2d.pb.cc.i

msgs/CMakeFiles/stoupentoPlugin_msgs.dir/vector2d.pb.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stoupentoPlugin_msgs.dir/vector2d.pb.cc.s"
	cd /home/kollaada/stoupentoPlugin/msgs/msgs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kollaada/stoupentoPlugin/msgs/msgs/vector2d.pb.cc -o CMakeFiles/stoupentoPlugin_msgs.dir/vector2d.pb.cc.s

msgs/CMakeFiles/stoupentoPlugin_msgs.dir/header.pb.cc.o: msgs/CMakeFiles/stoupentoPlugin_msgs.dir/flags.make
msgs/CMakeFiles/stoupentoPlugin_msgs.dir/header.pb.cc.o: msgs/header.pb.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kollaada/stoupentoPlugin/msgs/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object msgs/CMakeFiles/stoupentoPlugin_msgs.dir/header.pb.cc.o"
	cd /home/kollaada/stoupentoPlugin/msgs/msgs && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stoupentoPlugin_msgs.dir/header.pb.cc.o -c /home/kollaada/stoupentoPlugin/msgs/msgs/header.pb.cc

msgs/CMakeFiles/stoupentoPlugin_msgs.dir/header.pb.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stoupentoPlugin_msgs.dir/header.pb.cc.i"
	cd /home/kollaada/stoupentoPlugin/msgs/msgs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kollaada/stoupentoPlugin/msgs/msgs/header.pb.cc > CMakeFiles/stoupentoPlugin_msgs.dir/header.pb.cc.i

msgs/CMakeFiles/stoupentoPlugin_msgs.dir/header.pb.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stoupentoPlugin_msgs.dir/header.pb.cc.s"
	cd /home/kollaada/stoupentoPlugin/msgs/msgs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kollaada/stoupentoPlugin/msgs/msgs/header.pb.cc -o CMakeFiles/stoupentoPlugin_msgs.dir/header.pb.cc.s

msgs/CMakeFiles/stoupentoPlugin_msgs.dir/time.pb.cc.o: msgs/CMakeFiles/stoupentoPlugin_msgs.dir/flags.make
msgs/CMakeFiles/stoupentoPlugin_msgs.dir/time.pb.cc.o: msgs/time.pb.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kollaada/stoupentoPlugin/msgs/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object msgs/CMakeFiles/stoupentoPlugin_msgs.dir/time.pb.cc.o"
	cd /home/kollaada/stoupentoPlugin/msgs/msgs && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stoupentoPlugin_msgs.dir/time.pb.cc.o -c /home/kollaada/stoupentoPlugin/msgs/msgs/time.pb.cc

msgs/CMakeFiles/stoupentoPlugin_msgs.dir/time.pb.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stoupentoPlugin_msgs.dir/time.pb.cc.i"
	cd /home/kollaada/stoupentoPlugin/msgs/msgs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kollaada/stoupentoPlugin/msgs/msgs/time.pb.cc > CMakeFiles/stoupentoPlugin_msgs.dir/time.pb.cc.i

msgs/CMakeFiles/stoupentoPlugin_msgs.dir/time.pb.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stoupentoPlugin_msgs.dir/time.pb.cc.s"
	cd /home/kollaada/stoupentoPlugin/msgs/msgs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kollaada/stoupentoPlugin/msgs/msgs/time.pb.cc -o CMakeFiles/stoupentoPlugin_msgs.dir/time.pb.cc.s

# Object files for target stoupentoPlugin_msgs
stoupentoPlugin_msgs_OBJECTS = \
"CMakeFiles/stoupentoPlugin_msgs.dir/stoupento_mesurement.pb.cc.o" \
"CMakeFiles/stoupentoPlugin_msgs.dir/controlCommand.pb.cc.o" \
"CMakeFiles/stoupentoPlugin_msgs.dir/vector2d.pb.cc.o" \
"CMakeFiles/stoupentoPlugin_msgs.dir/header.pb.cc.o" \
"CMakeFiles/stoupentoPlugin_msgs.dir/time.pb.cc.o"

# External object files for target stoupentoPlugin_msgs
stoupentoPlugin_msgs_EXTERNAL_OBJECTS =

msgs/libstoupentoPlugin_msgs.so: msgs/CMakeFiles/stoupentoPlugin_msgs.dir/stoupento_mesurement.pb.cc.o
msgs/libstoupentoPlugin_msgs.so: msgs/CMakeFiles/stoupentoPlugin_msgs.dir/controlCommand.pb.cc.o
msgs/libstoupentoPlugin_msgs.so: msgs/CMakeFiles/stoupentoPlugin_msgs.dir/vector2d.pb.cc.o
msgs/libstoupentoPlugin_msgs.so: msgs/CMakeFiles/stoupentoPlugin_msgs.dir/header.pb.cc.o
msgs/libstoupentoPlugin_msgs.so: msgs/CMakeFiles/stoupentoPlugin_msgs.dir/time.pb.cc.o
msgs/libstoupentoPlugin_msgs.so: msgs/CMakeFiles/stoupentoPlugin_msgs.dir/build.make
msgs/libstoupentoPlugin_msgs.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
msgs/libstoupentoPlugin_msgs.so: msgs/CMakeFiles/stoupentoPlugin_msgs.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kollaada/stoupentoPlugin/msgs/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Linking CXX shared library libstoupentoPlugin_msgs.so"
	cd /home/kollaada/stoupentoPlugin/msgs/msgs && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/stoupentoPlugin_msgs.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
msgs/CMakeFiles/stoupentoPlugin_msgs.dir/build: msgs/libstoupentoPlugin_msgs.so

.PHONY : msgs/CMakeFiles/stoupentoPlugin_msgs.dir/build

msgs/CMakeFiles/stoupentoPlugin_msgs.dir/clean:
	cd /home/kollaada/stoupentoPlugin/msgs/msgs && $(CMAKE_COMMAND) -P CMakeFiles/stoupentoPlugin_msgs.dir/cmake_clean.cmake
.PHONY : msgs/CMakeFiles/stoupentoPlugin_msgs.dir/clean

msgs/CMakeFiles/stoupentoPlugin_msgs.dir/depend: msgs/stoupento_mesurement.pb.h
msgs/CMakeFiles/stoupentoPlugin_msgs.dir/depend: msgs/stoupento_mesurement.pb.cc
msgs/CMakeFiles/stoupentoPlugin_msgs.dir/depend: msgs/controlCommand.pb.h
msgs/CMakeFiles/stoupentoPlugin_msgs.dir/depend: msgs/controlCommand.pb.cc
msgs/CMakeFiles/stoupentoPlugin_msgs.dir/depend: msgs/vector2d.pb.h
msgs/CMakeFiles/stoupentoPlugin_msgs.dir/depend: msgs/vector2d.pb.cc
msgs/CMakeFiles/stoupentoPlugin_msgs.dir/depend: msgs/header.pb.h
msgs/CMakeFiles/stoupentoPlugin_msgs.dir/depend: msgs/header.pb.cc
msgs/CMakeFiles/stoupentoPlugin_msgs.dir/depend: msgs/time.pb.h
msgs/CMakeFiles/stoupentoPlugin_msgs.dir/depend: msgs/time.pb.cc
	cd /home/kollaada/stoupentoPlugin/msgs && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kollaada/stoupentoPlugin /home/kollaada/stoupentoPlugin/msgs /home/kollaada/stoupentoPlugin/msgs /home/kollaada/stoupentoPlugin/msgs/msgs /home/kollaada/stoupentoPlugin/msgs/msgs/CMakeFiles/stoupentoPlugin_msgs.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : msgs/CMakeFiles/stoupentoPlugin_msgs.dir/depend

