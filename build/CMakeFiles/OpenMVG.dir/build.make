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
CMAKE_SOURCE_DIR = /home/wayz/文档/code/postprocesssfmdata

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/wayz/文档/code/postprocesssfmdata/build

# Include any dependencies generated for this target.
include CMakeFiles/OpenMVG.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/OpenMVG.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/OpenMVG.dir/flags.make

CMakeFiles/OpenMVG.dir/src/image_io.cpp.o: CMakeFiles/OpenMVG.dir/flags.make
CMakeFiles/OpenMVG.dir/src/image_io.cpp.o: ../src/image_io.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wayz/文档/code/postprocesssfmdata/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/OpenMVG.dir/src/image_io.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/OpenMVG.dir/src/image_io.cpp.o -c /home/wayz/文档/code/postprocesssfmdata/src/image_io.cpp

CMakeFiles/OpenMVG.dir/src/image_io.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/OpenMVG.dir/src/image_io.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wayz/文档/code/postprocesssfmdata/src/image_io.cpp > CMakeFiles/OpenMVG.dir/src/image_io.cpp.i

CMakeFiles/OpenMVG.dir/src/image_io.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/OpenMVG.dir/src/image_io.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wayz/文档/code/postprocesssfmdata/src/image_io.cpp -o CMakeFiles/OpenMVG.dir/src/image_io.cpp.s

CMakeFiles/OpenMVG.dir/src/numeric.cpp.o: CMakeFiles/OpenMVG.dir/flags.make
CMakeFiles/OpenMVG.dir/src/numeric.cpp.o: ../src/numeric.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wayz/文档/code/postprocesssfmdata/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/OpenMVG.dir/src/numeric.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/OpenMVG.dir/src/numeric.cpp.o -c /home/wayz/文档/code/postprocesssfmdata/src/numeric.cpp

CMakeFiles/OpenMVG.dir/src/numeric.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/OpenMVG.dir/src/numeric.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wayz/文档/code/postprocesssfmdata/src/numeric.cpp > CMakeFiles/OpenMVG.dir/src/numeric.cpp.i

CMakeFiles/OpenMVG.dir/src/numeric.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/OpenMVG.dir/src/numeric.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wayz/文档/code/postprocesssfmdata/src/numeric.cpp -o CMakeFiles/OpenMVG.dir/src/numeric.cpp.s

CMakeFiles/OpenMVG.dir/src/colmap_reader.cpp.o: CMakeFiles/OpenMVG.dir/flags.make
CMakeFiles/OpenMVG.dir/src/colmap_reader.cpp.o: ../src/colmap_reader.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wayz/文档/code/postprocesssfmdata/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/OpenMVG.dir/src/colmap_reader.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/OpenMVG.dir/src/colmap_reader.cpp.o -c /home/wayz/文档/code/postprocesssfmdata/src/colmap_reader.cpp

CMakeFiles/OpenMVG.dir/src/colmap_reader.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/OpenMVG.dir/src/colmap_reader.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wayz/文档/code/postprocesssfmdata/src/colmap_reader.cpp > CMakeFiles/OpenMVG.dir/src/colmap_reader.cpp.i

CMakeFiles/OpenMVG.dir/src/colmap_reader.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/OpenMVG.dir/src/colmap_reader.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wayz/文档/code/postprocesssfmdata/src/colmap_reader.cpp -o CMakeFiles/OpenMVG.dir/src/colmap_reader.cpp.s

# Object files for target OpenMVG
OpenMVG_OBJECTS = \
"CMakeFiles/OpenMVG.dir/src/image_io.cpp.o" \
"CMakeFiles/OpenMVG.dir/src/numeric.cpp.o" \
"CMakeFiles/OpenMVG.dir/src/colmap_reader.cpp.o"

# External object files for target OpenMVG
OpenMVG_EXTERNAL_OBJECTS =

libOpenMVG.so: CMakeFiles/OpenMVG.dir/src/image_io.cpp.o
libOpenMVG.so: CMakeFiles/OpenMVG.dir/src/numeric.cpp.o
libOpenMVG.so: CMakeFiles/OpenMVG.dir/src/colmap_reader.cpp.o
libOpenMVG.so: CMakeFiles/OpenMVG.dir/build.make
libOpenMVG.so: /usr/lib/x86_64-linux-gnu/libtiff.so
libOpenMVG.so: /usr/lib/x86_64-linux-gnu/libpng.so
libOpenMVG.so: /usr/lib/x86_64-linux-gnu/libz.so
libOpenMVG.so: /usr/lib/x86_64-linux-gnu/libjpeg.so
libOpenMVG.so: third_party/stlplus3/libopenMVG_stlplus.a
libOpenMVG.so: CMakeFiles/OpenMVG.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/wayz/文档/code/postprocesssfmdata/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX shared library libOpenMVG.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/OpenMVG.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/OpenMVG.dir/build: libOpenMVG.so

.PHONY : CMakeFiles/OpenMVG.dir/build

CMakeFiles/OpenMVG.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/OpenMVG.dir/cmake_clean.cmake
.PHONY : CMakeFiles/OpenMVG.dir/clean

CMakeFiles/OpenMVG.dir/depend:
	cd /home/wayz/文档/code/postprocesssfmdata/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/wayz/文档/code/postprocesssfmdata /home/wayz/文档/code/postprocesssfmdata /home/wayz/文档/code/postprocesssfmdata/build /home/wayz/文档/code/postprocesssfmdata/build /home/wayz/文档/code/postprocesssfmdata/build/CMakeFiles/OpenMVG.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/OpenMVG.dir/depend

