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
CMAKE_SOURCE_DIR = /home/justin/class/ROB530/ROB-530_DVO

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/justin/class/ROB530/ROB-530_DVO/build

# Include any dependencies generated for this target.
include CMakeFiles/helper.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/helper.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/helper.dir/flags.make

CMakeFiles/helper.dir/src/dvo_class.cpp.o: CMakeFiles/helper.dir/flags.make
CMakeFiles/helper.dir/src/dvo_class.cpp.o: ../src/dvo_class.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/justin/class/ROB530/ROB-530_DVO/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/helper.dir/src/dvo_class.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/helper.dir/src/dvo_class.cpp.o -c /home/justin/class/ROB530/ROB-530_DVO/src/dvo_class.cpp

CMakeFiles/helper.dir/src/dvo_class.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/helper.dir/src/dvo_class.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/justin/class/ROB530/ROB-530_DVO/src/dvo_class.cpp > CMakeFiles/helper.dir/src/dvo_class.cpp.i

CMakeFiles/helper.dir/src/dvo_class.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/helper.dir/src/dvo_class.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/justin/class/ROB530/ROB-530_DVO/src/dvo_class.cpp -o CMakeFiles/helper.dir/src/dvo_class.cpp.s

CMakeFiles/helper.dir/src/dvo_class.cpp.o.requires:

.PHONY : CMakeFiles/helper.dir/src/dvo_class.cpp.o.requires

CMakeFiles/helper.dir/src/dvo_class.cpp.o.provides: CMakeFiles/helper.dir/src/dvo_class.cpp.o.requires
	$(MAKE) -f CMakeFiles/helper.dir/build.make CMakeFiles/helper.dir/src/dvo_class.cpp.o.provides.build
.PHONY : CMakeFiles/helper.dir/src/dvo_class.cpp.o.provides

CMakeFiles/helper.dir/src/dvo_class.cpp.o.provides.build: CMakeFiles/helper.dir/src/dvo_class.cpp.o


CMakeFiles/helper.dir/src/image_alignment.cpp.o: CMakeFiles/helper.dir/flags.make
CMakeFiles/helper.dir/src/image_alignment.cpp.o: ../src/image_alignment.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/justin/class/ROB530/ROB-530_DVO/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/helper.dir/src/image_alignment.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/helper.dir/src/image_alignment.cpp.o -c /home/justin/class/ROB530/ROB-530_DVO/src/image_alignment.cpp

CMakeFiles/helper.dir/src/image_alignment.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/helper.dir/src/image_alignment.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/justin/class/ROB530/ROB-530_DVO/src/image_alignment.cpp > CMakeFiles/helper.dir/src/image_alignment.cpp.i

CMakeFiles/helper.dir/src/image_alignment.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/helper.dir/src/image_alignment.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/justin/class/ROB530/ROB-530_DVO/src/image_alignment.cpp -o CMakeFiles/helper.dir/src/image_alignment.cpp.s

CMakeFiles/helper.dir/src/image_alignment.cpp.o.requires:

.PHONY : CMakeFiles/helper.dir/src/image_alignment.cpp.o.requires

CMakeFiles/helper.dir/src/image_alignment.cpp.o.provides: CMakeFiles/helper.dir/src/image_alignment.cpp.o.requires
	$(MAKE) -f CMakeFiles/helper.dir/build.make CMakeFiles/helper.dir/src/image_alignment.cpp.o.provides.build
.PHONY : CMakeFiles/helper.dir/src/image_alignment.cpp.o.provides

CMakeFiles/helper.dir/src/image_alignment.cpp.o.provides.build: CMakeFiles/helper.dir/src/image_alignment.cpp.o


CMakeFiles/helper.dir/src/util.cpp.o: CMakeFiles/helper.dir/flags.make
CMakeFiles/helper.dir/src/util.cpp.o: ../src/util.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/justin/class/ROB530/ROB-530_DVO/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/helper.dir/src/util.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/helper.dir/src/util.cpp.o -c /home/justin/class/ROB530/ROB-530_DVO/src/util.cpp

CMakeFiles/helper.dir/src/util.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/helper.dir/src/util.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/justin/class/ROB530/ROB-530_DVO/src/util.cpp > CMakeFiles/helper.dir/src/util.cpp.i

CMakeFiles/helper.dir/src/util.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/helper.dir/src/util.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/justin/class/ROB530/ROB-530_DVO/src/util.cpp -o CMakeFiles/helper.dir/src/util.cpp.s

CMakeFiles/helper.dir/src/util.cpp.o.requires:

.PHONY : CMakeFiles/helper.dir/src/util.cpp.o.requires

CMakeFiles/helper.dir/src/util.cpp.o.provides: CMakeFiles/helper.dir/src/util.cpp.o.requires
	$(MAKE) -f CMakeFiles/helper.dir/build.make CMakeFiles/helper.dir/src/util.cpp.o.provides.build
.PHONY : CMakeFiles/helper.dir/src/util.cpp.o.provides

CMakeFiles/helper.dir/src/util.cpp.o.provides.build: CMakeFiles/helper.dir/src/util.cpp.o


CMakeFiles/helper.dir/src/Optimizer.cpp.o: CMakeFiles/helper.dir/flags.make
CMakeFiles/helper.dir/src/Optimizer.cpp.o: ../src/Optimizer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/justin/class/ROB530/ROB-530_DVO/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/helper.dir/src/Optimizer.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/helper.dir/src/Optimizer.cpp.o -c /home/justin/class/ROB530/ROB-530_DVO/src/Optimizer.cpp

CMakeFiles/helper.dir/src/Optimizer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/helper.dir/src/Optimizer.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/justin/class/ROB530/ROB-530_DVO/src/Optimizer.cpp > CMakeFiles/helper.dir/src/Optimizer.cpp.i

CMakeFiles/helper.dir/src/Optimizer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/helper.dir/src/Optimizer.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/justin/class/ROB530/ROB-530_DVO/src/Optimizer.cpp -o CMakeFiles/helper.dir/src/Optimizer.cpp.s

CMakeFiles/helper.dir/src/Optimizer.cpp.o.requires:

.PHONY : CMakeFiles/helper.dir/src/Optimizer.cpp.o.requires

CMakeFiles/helper.dir/src/Optimizer.cpp.o.provides: CMakeFiles/helper.dir/src/Optimizer.cpp.o.requires
	$(MAKE) -f CMakeFiles/helper.dir/build.make CMakeFiles/helper.dir/src/Optimizer.cpp.o.provides.build
.PHONY : CMakeFiles/helper.dir/src/Optimizer.cpp.o.provides

CMakeFiles/helper.dir/src/Optimizer.cpp.o.provides.build: CMakeFiles/helper.dir/src/Optimizer.cpp.o


CMakeFiles/helper.dir/src/Converter.cc.o: CMakeFiles/helper.dir/flags.make
CMakeFiles/helper.dir/src/Converter.cc.o: ../src/Converter.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/justin/class/ROB530/ROB-530_DVO/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/helper.dir/src/Converter.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/helper.dir/src/Converter.cc.o -c /home/justin/class/ROB530/ROB-530_DVO/src/Converter.cc

CMakeFiles/helper.dir/src/Converter.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/helper.dir/src/Converter.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/justin/class/ROB530/ROB-530_DVO/src/Converter.cc > CMakeFiles/helper.dir/src/Converter.cc.i

CMakeFiles/helper.dir/src/Converter.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/helper.dir/src/Converter.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/justin/class/ROB530/ROB-530_DVO/src/Converter.cc -o CMakeFiles/helper.dir/src/Converter.cc.s

CMakeFiles/helper.dir/src/Converter.cc.o.requires:

.PHONY : CMakeFiles/helper.dir/src/Converter.cc.o.requires

CMakeFiles/helper.dir/src/Converter.cc.o.provides: CMakeFiles/helper.dir/src/Converter.cc.o.requires
	$(MAKE) -f CMakeFiles/helper.dir/build.make CMakeFiles/helper.dir/src/Converter.cc.o.provides.build
.PHONY : CMakeFiles/helper.dir/src/Converter.cc.o.provides

CMakeFiles/helper.dir/src/Converter.cc.o.provides.build: CMakeFiles/helper.dir/src/Converter.cc.o


# Object files for target helper
helper_OBJECTS = \
"CMakeFiles/helper.dir/src/dvo_class.cpp.o" \
"CMakeFiles/helper.dir/src/image_alignment.cpp.o" \
"CMakeFiles/helper.dir/src/util.cpp.o" \
"CMakeFiles/helper.dir/src/Optimizer.cpp.o" \
"CMakeFiles/helper.dir/src/Converter.cc.o"

# External object files for target helper
helper_EXTERNAL_OBJECTS =

libhelper.a: CMakeFiles/helper.dir/src/dvo_class.cpp.o
libhelper.a: CMakeFiles/helper.dir/src/image_alignment.cpp.o
libhelper.a: CMakeFiles/helper.dir/src/util.cpp.o
libhelper.a: CMakeFiles/helper.dir/src/Optimizer.cpp.o
libhelper.a: CMakeFiles/helper.dir/src/Converter.cc.o
libhelper.a: CMakeFiles/helper.dir/build.make
libhelper.a: CMakeFiles/helper.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/justin/class/ROB530/ROB-530_DVO/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX static library libhelper.a"
	$(CMAKE_COMMAND) -P CMakeFiles/helper.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/helper.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/helper.dir/build: libhelper.a

.PHONY : CMakeFiles/helper.dir/build

CMakeFiles/helper.dir/requires: CMakeFiles/helper.dir/src/dvo_class.cpp.o.requires
CMakeFiles/helper.dir/requires: CMakeFiles/helper.dir/src/image_alignment.cpp.o.requires
CMakeFiles/helper.dir/requires: CMakeFiles/helper.dir/src/util.cpp.o.requires
CMakeFiles/helper.dir/requires: CMakeFiles/helper.dir/src/Optimizer.cpp.o.requires
CMakeFiles/helper.dir/requires: CMakeFiles/helper.dir/src/Converter.cc.o.requires

.PHONY : CMakeFiles/helper.dir/requires

CMakeFiles/helper.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/helper.dir/cmake_clean.cmake
.PHONY : CMakeFiles/helper.dir/clean

CMakeFiles/helper.dir/depend:
	cd /home/justin/class/ROB530/ROB-530_DVO/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/justin/class/ROB530/ROB-530_DVO /home/justin/class/ROB530/ROB-530_DVO /home/justin/class/ROB530/ROB-530_DVO/build /home/justin/class/ROB530/ROB-530_DVO/build /home/justin/class/ROB530/ROB-530_DVO/build/CMakeFiles/helper.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/helper.dir/depend

