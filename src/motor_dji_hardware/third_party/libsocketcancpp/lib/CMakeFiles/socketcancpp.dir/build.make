# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.28

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

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/chenyu/dartros2workspace/src/dart_launcher/libsockcancpp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/chenyu/dartros2workspace/src/dart_launcher/libsockcancpp/lib

# Include any dependencies generated for this target.
include CMakeFiles/socketcancpp.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/socketcancpp.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/socketcancpp.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/socketcancpp.dir/flags.make

CMakeFiles/socketcancpp.dir/src/CanDriver.cpp.o: CMakeFiles/socketcancpp.dir/flags.make
CMakeFiles/socketcancpp.dir/src/CanDriver.cpp.o: /home/chenyu/dartros2workspace/src/dart_launcher/libsockcancpp/src/CanDriver.cpp
CMakeFiles/socketcancpp.dir/src/CanDriver.cpp.o: CMakeFiles/socketcancpp.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/chenyu/dartros2workspace/src/dart_launcher/libsockcancpp/lib/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/socketcancpp.dir/src/CanDriver.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/socketcancpp.dir/src/CanDriver.cpp.o -MF CMakeFiles/socketcancpp.dir/src/CanDriver.cpp.o.d -o CMakeFiles/socketcancpp.dir/src/CanDriver.cpp.o -c /home/chenyu/dartros2workspace/src/dart_launcher/libsockcancpp/src/CanDriver.cpp

CMakeFiles/socketcancpp.dir/src/CanDriver.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/socketcancpp.dir/src/CanDriver.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chenyu/dartros2workspace/src/dart_launcher/libsockcancpp/src/CanDriver.cpp > CMakeFiles/socketcancpp.dir/src/CanDriver.cpp.i

CMakeFiles/socketcancpp.dir/src/CanDriver.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/socketcancpp.dir/src/CanDriver.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chenyu/dartros2workspace/src/dart_launcher/libsockcancpp/src/CanDriver.cpp -o CMakeFiles/socketcancpp.dir/src/CanDriver.cpp.s

# Object files for target socketcancpp
socketcancpp_OBJECTS = \
"CMakeFiles/socketcancpp.dir/src/CanDriver.cpp.o"

# External object files for target socketcancpp
socketcancpp_EXTERNAL_OBJECTS =

libsocketcancpp.so: CMakeFiles/socketcancpp.dir/src/CanDriver.cpp.o
libsocketcancpp.so: CMakeFiles/socketcancpp.dir/build.make
libsocketcancpp.so: CMakeFiles/socketcancpp.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/chenyu/dartros2workspace/src/dart_launcher/libsockcancpp/lib/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libsocketcancpp.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/socketcancpp.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/socketcancpp.dir/build: libsocketcancpp.so
.PHONY : CMakeFiles/socketcancpp.dir/build

CMakeFiles/socketcancpp.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/socketcancpp.dir/cmake_clean.cmake
.PHONY : CMakeFiles/socketcancpp.dir/clean

CMakeFiles/socketcancpp.dir/depend:
	cd /home/chenyu/dartros2workspace/src/dart_launcher/libsockcancpp/lib && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/chenyu/dartros2workspace/src/dart_launcher/libsockcancpp /home/chenyu/dartros2workspace/src/dart_launcher/libsockcancpp /home/chenyu/dartros2workspace/src/dart_launcher/libsockcancpp/lib /home/chenyu/dartros2workspace/src/dart_launcher/libsockcancpp/lib /home/chenyu/dartros2workspace/src/dart_launcher/libsockcancpp/lib/CMakeFiles/socketcancpp.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/socketcancpp.dir/depend

