# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_COMMAND = "/Applications/CMake 2.8-12.app/Contents/bin/cmake"

# The command to remove a file.
RM = "/Applications/CMake 2.8-12.app/Contents/bin/cmake" -E remove -f

# Escaping for special characters.
EQUALS = =

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = "/Applications/CMake 2.8-12.app/Contents/bin/ccmake"

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/hemingge/Desktop/Heming-Bella/CS4670/P5

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/hemingge/Desktop/Heming-Bella/CS4670/P5

# Include any dependencies generated for this target.
include thirdparty/ImageLib/CMakeFiles/image.dir/depend.make

# Include the progress variables for this target.
include thirdparty/ImageLib/CMakeFiles/image.dir/progress.make

# Include the compile flags for this target's objects.
include thirdparty/ImageLib/CMakeFiles/image.dir/flags.make

thirdparty/ImageLib/CMakeFiles/image.dir/Convolve.cpp.o: thirdparty/ImageLib/CMakeFiles/image.dir/flags.make
thirdparty/ImageLib/CMakeFiles/image.dir/Convolve.cpp.o: thirdparty/ImageLib/Convolve.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object thirdparty/ImageLib/CMakeFiles/image.dir/Convolve.cpp.o"
	cd /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/image.dir/Convolve.cpp.o -c /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib/Convolve.cpp

thirdparty/ImageLib/CMakeFiles/image.dir/Convolve.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/image.dir/Convolve.cpp.i"
	cd /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib/Convolve.cpp > CMakeFiles/image.dir/Convolve.cpp.i

thirdparty/ImageLib/CMakeFiles/image.dir/Convolve.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/image.dir/Convolve.cpp.s"
	cd /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib/Convolve.cpp -o CMakeFiles/image.dir/Convolve.cpp.s

thirdparty/ImageLib/CMakeFiles/image.dir/Convolve.cpp.o.requires:
.PHONY : thirdparty/ImageLib/CMakeFiles/image.dir/Convolve.cpp.o.requires

thirdparty/ImageLib/CMakeFiles/image.dir/Convolve.cpp.o.provides: thirdparty/ImageLib/CMakeFiles/image.dir/Convolve.cpp.o.requires
	$(MAKE) -f thirdparty/ImageLib/CMakeFiles/image.dir/build.make thirdparty/ImageLib/CMakeFiles/image.dir/Convolve.cpp.o.provides.build
.PHONY : thirdparty/ImageLib/CMakeFiles/image.dir/Convolve.cpp.o.provides

thirdparty/ImageLib/CMakeFiles/image.dir/Convolve.cpp.o.provides.build: thirdparty/ImageLib/CMakeFiles/image.dir/Convolve.cpp.o

thirdparty/ImageLib/CMakeFiles/image.dir/FileIO.cpp.o: thirdparty/ImageLib/CMakeFiles/image.dir/flags.make
thirdparty/ImageLib/CMakeFiles/image.dir/FileIO.cpp.o: thirdparty/ImageLib/FileIO.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object thirdparty/ImageLib/CMakeFiles/image.dir/FileIO.cpp.o"
	cd /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/image.dir/FileIO.cpp.o -c /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib/FileIO.cpp

thirdparty/ImageLib/CMakeFiles/image.dir/FileIO.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/image.dir/FileIO.cpp.i"
	cd /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib/FileIO.cpp > CMakeFiles/image.dir/FileIO.cpp.i

thirdparty/ImageLib/CMakeFiles/image.dir/FileIO.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/image.dir/FileIO.cpp.s"
	cd /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib/FileIO.cpp -o CMakeFiles/image.dir/FileIO.cpp.s

thirdparty/ImageLib/CMakeFiles/image.dir/FileIO.cpp.o.requires:
.PHONY : thirdparty/ImageLib/CMakeFiles/image.dir/FileIO.cpp.o.requires

thirdparty/ImageLib/CMakeFiles/image.dir/FileIO.cpp.o.provides: thirdparty/ImageLib/CMakeFiles/image.dir/FileIO.cpp.o.requires
	$(MAKE) -f thirdparty/ImageLib/CMakeFiles/image.dir/build.make thirdparty/ImageLib/CMakeFiles/image.dir/FileIO.cpp.o.provides.build
.PHONY : thirdparty/ImageLib/CMakeFiles/image.dir/FileIO.cpp.o.provides

thirdparty/ImageLib/CMakeFiles/image.dir/FileIO.cpp.o.provides.build: thirdparty/ImageLib/CMakeFiles/image.dir/FileIO.cpp.o

thirdparty/ImageLib/CMakeFiles/image.dir/Image.cpp.o: thirdparty/ImageLib/CMakeFiles/image.dir/flags.make
thirdparty/ImageLib/CMakeFiles/image.dir/Image.cpp.o: thirdparty/ImageLib/Image.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object thirdparty/ImageLib/CMakeFiles/image.dir/Image.cpp.o"
	cd /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/image.dir/Image.cpp.o -c /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib/Image.cpp

thirdparty/ImageLib/CMakeFiles/image.dir/Image.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/image.dir/Image.cpp.i"
	cd /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib/Image.cpp > CMakeFiles/image.dir/Image.cpp.i

thirdparty/ImageLib/CMakeFiles/image.dir/Image.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/image.dir/Image.cpp.s"
	cd /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib/Image.cpp -o CMakeFiles/image.dir/Image.cpp.s

thirdparty/ImageLib/CMakeFiles/image.dir/Image.cpp.o.requires:
.PHONY : thirdparty/ImageLib/CMakeFiles/image.dir/Image.cpp.o.requires

thirdparty/ImageLib/CMakeFiles/image.dir/Image.cpp.o.provides: thirdparty/ImageLib/CMakeFiles/image.dir/Image.cpp.o.requires
	$(MAKE) -f thirdparty/ImageLib/CMakeFiles/image.dir/build.make thirdparty/ImageLib/CMakeFiles/image.dir/Image.cpp.o.provides.build
.PHONY : thirdparty/ImageLib/CMakeFiles/image.dir/Image.cpp.o.provides

thirdparty/ImageLib/CMakeFiles/image.dir/Image.cpp.o.provides.build: thirdparty/ImageLib/CMakeFiles/image.dir/Image.cpp.o

thirdparty/ImageLib/CMakeFiles/image.dir/ImageProc.cpp.o: thirdparty/ImageLib/CMakeFiles/image.dir/flags.make
thirdparty/ImageLib/CMakeFiles/image.dir/ImageProc.cpp.o: thirdparty/ImageLib/ImageProc.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object thirdparty/ImageLib/CMakeFiles/image.dir/ImageProc.cpp.o"
	cd /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/image.dir/ImageProc.cpp.o -c /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib/ImageProc.cpp

thirdparty/ImageLib/CMakeFiles/image.dir/ImageProc.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/image.dir/ImageProc.cpp.i"
	cd /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib/ImageProc.cpp > CMakeFiles/image.dir/ImageProc.cpp.i

thirdparty/ImageLib/CMakeFiles/image.dir/ImageProc.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/image.dir/ImageProc.cpp.s"
	cd /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib/ImageProc.cpp -o CMakeFiles/image.dir/ImageProc.cpp.s

thirdparty/ImageLib/CMakeFiles/image.dir/ImageProc.cpp.o.requires:
.PHONY : thirdparty/ImageLib/CMakeFiles/image.dir/ImageProc.cpp.o.requires

thirdparty/ImageLib/CMakeFiles/image.dir/ImageProc.cpp.o.provides: thirdparty/ImageLib/CMakeFiles/image.dir/ImageProc.cpp.o.requires
	$(MAKE) -f thirdparty/ImageLib/CMakeFiles/image.dir/build.make thirdparty/ImageLib/CMakeFiles/image.dir/ImageProc.cpp.o.provides.build
.PHONY : thirdparty/ImageLib/CMakeFiles/image.dir/ImageProc.cpp.o.provides

thirdparty/ImageLib/CMakeFiles/image.dir/ImageProc.cpp.o.provides.build: thirdparty/ImageLib/CMakeFiles/image.dir/ImageProc.cpp.o

thirdparty/ImageLib/CMakeFiles/image.dir/RefCntMem.cpp.o: thirdparty/ImageLib/CMakeFiles/image.dir/flags.make
thirdparty/ImageLib/CMakeFiles/image.dir/RefCntMem.cpp.o: thirdparty/ImageLib/RefCntMem.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object thirdparty/ImageLib/CMakeFiles/image.dir/RefCntMem.cpp.o"
	cd /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/image.dir/RefCntMem.cpp.o -c /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib/RefCntMem.cpp

thirdparty/ImageLib/CMakeFiles/image.dir/RefCntMem.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/image.dir/RefCntMem.cpp.i"
	cd /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib/RefCntMem.cpp > CMakeFiles/image.dir/RefCntMem.cpp.i

thirdparty/ImageLib/CMakeFiles/image.dir/RefCntMem.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/image.dir/RefCntMem.cpp.s"
	cd /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib/RefCntMem.cpp -o CMakeFiles/image.dir/RefCntMem.cpp.s

thirdparty/ImageLib/CMakeFiles/image.dir/RefCntMem.cpp.o.requires:
.PHONY : thirdparty/ImageLib/CMakeFiles/image.dir/RefCntMem.cpp.o.requires

thirdparty/ImageLib/CMakeFiles/image.dir/RefCntMem.cpp.o.provides: thirdparty/ImageLib/CMakeFiles/image.dir/RefCntMem.cpp.o.requires
	$(MAKE) -f thirdparty/ImageLib/CMakeFiles/image.dir/build.make thirdparty/ImageLib/CMakeFiles/image.dir/RefCntMem.cpp.o.provides.build
.PHONY : thirdparty/ImageLib/CMakeFiles/image.dir/RefCntMem.cpp.o.provides

thirdparty/ImageLib/CMakeFiles/image.dir/RefCntMem.cpp.o.provides.build: thirdparty/ImageLib/CMakeFiles/image.dir/RefCntMem.cpp.o

thirdparty/ImageLib/CMakeFiles/image.dir/Transform.cpp.o: thirdparty/ImageLib/CMakeFiles/image.dir/flags.make
thirdparty/ImageLib/CMakeFiles/image.dir/Transform.cpp.o: thirdparty/ImageLib/Transform.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object thirdparty/ImageLib/CMakeFiles/image.dir/Transform.cpp.o"
	cd /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/image.dir/Transform.cpp.o -c /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib/Transform.cpp

thirdparty/ImageLib/CMakeFiles/image.dir/Transform.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/image.dir/Transform.cpp.i"
	cd /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib/Transform.cpp > CMakeFiles/image.dir/Transform.cpp.i

thirdparty/ImageLib/CMakeFiles/image.dir/Transform.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/image.dir/Transform.cpp.s"
	cd /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib/Transform.cpp -o CMakeFiles/image.dir/Transform.cpp.s

thirdparty/ImageLib/CMakeFiles/image.dir/Transform.cpp.o.requires:
.PHONY : thirdparty/ImageLib/CMakeFiles/image.dir/Transform.cpp.o.requires

thirdparty/ImageLib/CMakeFiles/image.dir/Transform.cpp.o.provides: thirdparty/ImageLib/CMakeFiles/image.dir/Transform.cpp.o.requires
	$(MAKE) -f thirdparty/ImageLib/CMakeFiles/image.dir/build.make thirdparty/ImageLib/CMakeFiles/image.dir/Transform.cpp.o.provides.build
.PHONY : thirdparty/ImageLib/CMakeFiles/image.dir/Transform.cpp.o.provides

thirdparty/ImageLib/CMakeFiles/image.dir/Transform.cpp.o.provides.build: thirdparty/ImageLib/CMakeFiles/image.dir/Transform.cpp.o

# Object files for target image
image_OBJECTS = \
"CMakeFiles/image.dir/Convolve.cpp.o" \
"CMakeFiles/image.dir/FileIO.cpp.o" \
"CMakeFiles/image.dir/Image.cpp.o" \
"CMakeFiles/image.dir/ImageProc.cpp.o" \
"CMakeFiles/image.dir/RefCntMem.cpp.o" \
"CMakeFiles/image.dir/Transform.cpp.o"

# External object files for target image
image_EXTERNAL_OBJECTS =

thirdparty/ImageLib/libimage.a: thirdparty/ImageLib/CMakeFiles/image.dir/Convolve.cpp.o
thirdparty/ImageLib/libimage.a: thirdparty/ImageLib/CMakeFiles/image.dir/FileIO.cpp.o
thirdparty/ImageLib/libimage.a: thirdparty/ImageLib/CMakeFiles/image.dir/Image.cpp.o
thirdparty/ImageLib/libimage.a: thirdparty/ImageLib/CMakeFiles/image.dir/ImageProc.cpp.o
thirdparty/ImageLib/libimage.a: thirdparty/ImageLib/CMakeFiles/image.dir/RefCntMem.cpp.o
thirdparty/ImageLib/libimage.a: thirdparty/ImageLib/CMakeFiles/image.dir/Transform.cpp.o
thirdparty/ImageLib/libimage.a: thirdparty/ImageLib/CMakeFiles/image.dir/build.make
thirdparty/ImageLib/libimage.a: thirdparty/ImageLib/CMakeFiles/image.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library libimage.a"
	cd /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib && $(CMAKE_COMMAND) -P CMakeFiles/image.dir/cmake_clean_target.cmake
	cd /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/image.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
thirdparty/ImageLib/CMakeFiles/image.dir/build: thirdparty/ImageLib/libimage.a
.PHONY : thirdparty/ImageLib/CMakeFiles/image.dir/build

thirdparty/ImageLib/CMakeFiles/image.dir/requires: thirdparty/ImageLib/CMakeFiles/image.dir/Convolve.cpp.o.requires
thirdparty/ImageLib/CMakeFiles/image.dir/requires: thirdparty/ImageLib/CMakeFiles/image.dir/FileIO.cpp.o.requires
thirdparty/ImageLib/CMakeFiles/image.dir/requires: thirdparty/ImageLib/CMakeFiles/image.dir/Image.cpp.o.requires
thirdparty/ImageLib/CMakeFiles/image.dir/requires: thirdparty/ImageLib/CMakeFiles/image.dir/ImageProc.cpp.o.requires
thirdparty/ImageLib/CMakeFiles/image.dir/requires: thirdparty/ImageLib/CMakeFiles/image.dir/RefCntMem.cpp.o.requires
thirdparty/ImageLib/CMakeFiles/image.dir/requires: thirdparty/ImageLib/CMakeFiles/image.dir/Transform.cpp.o.requires
.PHONY : thirdparty/ImageLib/CMakeFiles/image.dir/requires

thirdparty/ImageLib/CMakeFiles/image.dir/clean:
	cd /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib && $(CMAKE_COMMAND) -P CMakeFiles/image.dir/cmake_clean.cmake
.PHONY : thirdparty/ImageLib/CMakeFiles/image.dir/clean

thirdparty/ImageLib/CMakeFiles/image.dir/depend:
	cd /Users/hemingge/Desktop/Heming-Bella/CS4670/P5 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/hemingge/Desktop/Heming-Bella/CS4670/P5 /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib /Users/hemingge/Desktop/Heming-Bella/CS4670/P5 /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/thirdparty/ImageLib/CMakeFiles/image.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : thirdparty/ImageLib/CMakeFiles/image.dir/depend

