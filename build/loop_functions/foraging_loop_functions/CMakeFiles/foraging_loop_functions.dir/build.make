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
CMAKE_SOURCE_DIR = /home/john/Documents/argos3-examples

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/john/Documents/argos3-examples/build

# Include any dependencies generated for this target.
include loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/depend.make

# Include the progress variables for this target.
include loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/progress.make

# Include the compile flags for this target's objects.
include loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/flags.make

loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.o: loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/flags.make
loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.o: ../loop_functions/foraging_loop_functions/foraging_loop_functions.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/john/Documents/argos3-examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.o"
	cd /home/john/Documents/argos3-examples/build/loop_functions/foraging_loop_functions && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.o -c /home/john/Documents/argos3-examples/loop_functions/foraging_loop_functions/foraging_loop_functions.cpp

loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.i"
	cd /home/john/Documents/argos3-examples/build/loop_functions/foraging_loop_functions && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/john/Documents/argos3-examples/loop_functions/foraging_loop_functions/foraging_loop_functions.cpp > CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.i

loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.s"
	cd /home/john/Documents/argos3-examples/build/loop_functions/foraging_loop_functions && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/john/Documents/argos3-examples/loop_functions/foraging_loop_functions/foraging_loop_functions.cpp -o CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.s

loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.o.requires:

.PHONY : loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.o.requires

loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.o.provides: loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.o.requires
	$(MAKE) -f loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/build.make loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.o.provides.build
.PHONY : loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.o.provides

loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.o.provides.build: loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.o


loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.o: loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/flags.make
loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.o: ../loop_functions/foraging_loop_functions/foraging_qt_user_functions.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/john/Documents/argos3-examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.o"
	cd /home/john/Documents/argos3-examples/build/loop_functions/foraging_loop_functions && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.o -c /home/john/Documents/argos3-examples/loop_functions/foraging_loop_functions/foraging_qt_user_functions.cpp

loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.i"
	cd /home/john/Documents/argos3-examples/build/loop_functions/foraging_loop_functions && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/john/Documents/argos3-examples/loop_functions/foraging_loop_functions/foraging_qt_user_functions.cpp > CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.i

loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.s"
	cd /home/john/Documents/argos3-examples/build/loop_functions/foraging_loop_functions && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/john/Documents/argos3-examples/loop_functions/foraging_loop_functions/foraging_qt_user_functions.cpp -o CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.s

loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.o.requires:

.PHONY : loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.o.requires

loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.o.provides: loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.o.requires
	$(MAKE) -f loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/build.make loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.o.provides.build
.PHONY : loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.o.provides

loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.o.provides.build: loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.o


loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.o: loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/flags.make
loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.o: loop_functions/foraging_loop_functions/foraging_loop_functions_automoc.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/john/Documents/argos3-examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.o"
	cd /home/john/Documents/argos3-examples/build/loop_functions/foraging_loop_functions && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.o -c /home/john/Documents/argos3-examples/build/loop_functions/foraging_loop_functions/foraging_loop_functions_automoc.cpp

loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.i"
	cd /home/john/Documents/argos3-examples/build/loop_functions/foraging_loop_functions && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/john/Documents/argos3-examples/build/loop_functions/foraging_loop_functions/foraging_loop_functions_automoc.cpp > CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.i

loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.s"
	cd /home/john/Documents/argos3-examples/build/loop_functions/foraging_loop_functions && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/john/Documents/argos3-examples/build/loop_functions/foraging_loop_functions/foraging_loop_functions_automoc.cpp -o CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.s

loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.o.requires:

.PHONY : loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.o.requires

loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.o.provides: loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.o.requires
	$(MAKE) -f loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/build.make loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.o.provides.build
.PHONY : loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.o.provides

loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.o.provides.build: loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.o


# Object files for target foraging_loop_functions
foraging_loop_functions_OBJECTS = \
"CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.o" \
"CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.o" \
"CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.o"

# External object files for target foraging_loop_functions
foraging_loop_functions_EXTERNAL_OBJECTS =

loop_functions/foraging_loop_functions/libforaging_loop_functions.so: loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.o
loop_functions/foraging_loop_functions/libforaging_loop_functions.so: loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.o
loop_functions/foraging_loop_functions/libforaging_loop_functions.so: loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.o
loop_functions/foraging_loop_functions/libforaging_loop_functions.so: loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/build.make
loop_functions/foraging_loop_functions/libforaging_loop_functions.so: controllers/footbot_foraging/libfootbot_foraging.so
loop_functions/foraging_loop_functions/libforaging_loop_functions.so: loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/john/Documents/argos3-examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX shared module libforaging_loop_functions.so"
	cd /home/john/Documents/argos3-examples/build/loop_functions/foraging_loop_functions && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/foraging_loop_functions.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/build: loop_functions/foraging_loop_functions/libforaging_loop_functions.so

.PHONY : loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/build

loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/requires: loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions.cpp.o.requires
loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/requires: loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_qt_user_functions.cpp.o.requires
loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/requires: loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/foraging_loop_functions_automoc.cpp.o.requires

.PHONY : loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/requires

loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/clean:
	cd /home/john/Documents/argos3-examples/build/loop_functions/foraging_loop_functions && $(CMAKE_COMMAND) -P CMakeFiles/foraging_loop_functions.dir/cmake_clean.cmake
.PHONY : loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/clean

loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/depend:
	cd /home/john/Documents/argos3-examples/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/john/Documents/argos3-examples /home/john/Documents/argos3-examples/loop_functions/foraging_loop_functions /home/john/Documents/argos3-examples/build /home/john/Documents/argos3-examples/build/loop_functions/foraging_loop_functions /home/john/Documents/argos3-examples/build/loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : loop_functions/foraging_loop_functions/CMakeFiles/foraging_loop_functions.dir/depend

