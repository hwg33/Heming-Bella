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
include CMakeFiles/objdet.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/objdet.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/objdet.dir/flags.make

CMakeFiles/objdet.dir/main.cpp.o: CMakeFiles/objdet.dir/flags.make
CMakeFiles/objdet.dir/main.cpp.o: main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/objdet.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/objdet.dir/main.cpp.o -c /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/main.cpp

CMakeFiles/objdet.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/objdet.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/main.cpp > CMakeFiles/objdet.dir/main.cpp.i

CMakeFiles/objdet.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/objdet.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/main.cpp -o CMakeFiles/objdet.dir/main.cpp.s

CMakeFiles/objdet.dir/main.cpp.o.requires:
.PHONY : CMakeFiles/objdet.dir/main.cpp.o.requires

CMakeFiles/objdet.dir/main.cpp.o.provides: CMakeFiles/objdet.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/objdet.dir/build.make CMakeFiles/objdet.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/objdet.dir/main.cpp.o.provides

CMakeFiles/objdet.dir/main.cpp.o.provides.build: CMakeFiles/objdet.dir/main.cpp.o

CMakeFiles/objdet.dir/GUIController.cpp.o: CMakeFiles/objdet.dir/flags.make
CMakeFiles/objdet.dir/GUIController.cpp.o: GUIController.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/objdet.dir/GUIController.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/objdet.dir/GUIController.cpp.o -c /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/GUIController.cpp

CMakeFiles/objdet.dir/GUIController.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/objdet.dir/GUIController.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/GUIController.cpp > CMakeFiles/objdet.dir/GUIController.cpp.i

CMakeFiles/objdet.dir/GUIController.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/objdet.dir/GUIController.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/GUIController.cpp -o CMakeFiles/objdet.dir/GUIController.cpp.s

CMakeFiles/objdet.dir/GUIController.cpp.o.requires:
.PHONY : CMakeFiles/objdet.dir/GUIController.cpp.o.requires

CMakeFiles/objdet.dir/GUIController.cpp.o.provides: CMakeFiles/objdet.dir/GUIController.cpp.o.requires
	$(MAKE) -f CMakeFiles/objdet.dir/build.make CMakeFiles/objdet.dir/GUIController.cpp.o.provides.build
.PHONY : CMakeFiles/objdet.dir/GUIController.cpp.o.provides

CMakeFiles/objdet.dir/GUIController.cpp.o.provides.build: CMakeFiles/objdet.dir/GUIController.cpp.o

CMakeFiles/objdet.dir/GUIModel.cpp.o: CMakeFiles/objdet.dir/flags.make
CMakeFiles/objdet.dir/GUIModel.cpp.o: GUIModel.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/objdet.dir/GUIModel.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/objdet.dir/GUIModel.cpp.o -c /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/GUIModel.cpp

CMakeFiles/objdet.dir/GUIModel.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/objdet.dir/GUIModel.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/GUIModel.cpp > CMakeFiles/objdet.dir/GUIModel.cpp.i

CMakeFiles/objdet.dir/GUIModel.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/objdet.dir/GUIModel.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/GUIModel.cpp -o CMakeFiles/objdet.dir/GUIModel.cpp.s

CMakeFiles/objdet.dir/GUIModel.cpp.o.requires:
.PHONY : CMakeFiles/objdet.dir/GUIModel.cpp.o.requires

CMakeFiles/objdet.dir/GUIModel.cpp.o.provides: CMakeFiles/objdet.dir/GUIModel.cpp.o.requires
	$(MAKE) -f CMakeFiles/objdet.dir/build.make CMakeFiles/objdet.dir/GUIModel.cpp.o.provides.build
.PHONY : CMakeFiles/objdet.dir/GUIModel.cpp.o.provides

CMakeFiles/objdet.dir/GUIModel.cpp.o.provides.build: CMakeFiles/objdet.dir/GUIModel.cpp.o

CMakeFiles/objdet.dir/ImageView.cpp.o: CMakeFiles/objdet.dir/flags.make
CMakeFiles/objdet.dir/ImageView.cpp.o: ImageView.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/objdet.dir/ImageView.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/objdet.dir/ImageView.cpp.o -c /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/ImageView.cpp

CMakeFiles/objdet.dir/ImageView.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/objdet.dir/ImageView.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/ImageView.cpp > CMakeFiles/objdet.dir/ImageView.cpp.i

CMakeFiles/objdet.dir/ImageView.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/objdet.dir/ImageView.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/ImageView.cpp -o CMakeFiles/objdet.dir/ImageView.cpp.s

CMakeFiles/objdet.dir/ImageView.cpp.o.requires:
.PHONY : CMakeFiles/objdet.dir/ImageView.cpp.o.requires

CMakeFiles/objdet.dir/ImageView.cpp.o.provides: CMakeFiles/objdet.dir/ImageView.cpp.o.requires
	$(MAKE) -f CMakeFiles/objdet.dir/build.make CMakeFiles/objdet.dir/ImageView.cpp.o.provides.build
.PHONY : CMakeFiles/objdet.dir/ImageView.cpp.o.provides

CMakeFiles/objdet.dir/ImageView.cpp.o.provides.build: CMakeFiles/objdet.dir/ImageView.cpp.o

CMakeFiles/objdet.dir/ImageViewGL.cpp.o: CMakeFiles/objdet.dir/flags.make
CMakeFiles/objdet.dir/ImageViewGL.cpp.o: ImageViewGL.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/objdet.dir/ImageViewGL.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/objdet.dir/ImageViewGL.cpp.o -c /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/ImageViewGL.cpp

CMakeFiles/objdet.dir/ImageViewGL.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/objdet.dir/ImageViewGL.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/ImageViewGL.cpp > CMakeFiles/objdet.dir/ImageViewGL.cpp.i

CMakeFiles/objdet.dir/ImageViewGL.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/objdet.dir/ImageViewGL.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/ImageViewGL.cpp -o CMakeFiles/objdet.dir/ImageViewGL.cpp.s

CMakeFiles/objdet.dir/ImageViewGL.cpp.o.requires:
.PHONY : CMakeFiles/objdet.dir/ImageViewGL.cpp.o.requires

CMakeFiles/objdet.dir/ImageViewGL.cpp.o.provides: CMakeFiles/objdet.dir/ImageViewGL.cpp.o.requires
	$(MAKE) -f CMakeFiles/objdet.dir/build.make CMakeFiles/objdet.dir/ImageViewGL.cpp.o.provides.build
.PHONY : CMakeFiles/objdet.dir/ImageViewGL.cpp.o.provides

CMakeFiles/objdet.dir/ImageViewGL.cpp.o.provides.build: CMakeFiles/objdet.dir/ImageViewGL.cpp.o

CMakeFiles/objdet.dir/ControlsBox.cpp.o: CMakeFiles/objdet.dir/flags.make
CMakeFiles/objdet.dir/ControlsBox.cpp.o: ControlsBox.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/objdet.dir/ControlsBox.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/objdet.dir/ControlsBox.cpp.o -c /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/ControlsBox.cpp

CMakeFiles/objdet.dir/ControlsBox.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/objdet.dir/ControlsBox.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/ControlsBox.cpp > CMakeFiles/objdet.dir/ControlsBox.cpp.i

CMakeFiles/objdet.dir/ControlsBox.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/objdet.dir/ControlsBox.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/ControlsBox.cpp -o CMakeFiles/objdet.dir/ControlsBox.cpp.s

CMakeFiles/objdet.dir/ControlsBox.cpp.o.requires:
.PHONY : CMakeFiles/objdet.dir/ControlsBox.cpp.o.requires

CMakeFiles/objdet.dir/ControlsBox.cpp.o.provides: CMakeFiles/objdet.dir/ControlsBox.cpp.o.requires
	$(MAKE) -f CMakeFiles/objdet.dir/build.make CMakeFiles/objdet.dir/ControlsBox.cpp.o.provides.build
.PHONY : CMakeFiles/objdet.dir/ControlsBox.cpp.o.provides

CMakeFiles/objdet.dir/ControlsBox.cpp.o.provides.build: CMakeFiles/objdet.dir/ControlsBox.cpp.o

# Object files for target objdet
objdet_OBJECTS = \
"CMakeFiles/objdet.dir/main.cpp.o" \
"CMakeFiles/objdet.dir/GUIController.cpp.o" \
"CMakeFiles/objdet.dir/GUIModel.cpp.o" \
"CMakeFiles/objdet.dir/ImageView.cpp.o" \
"CMakeFiles/objdet.dir/ImageViewGL.cpp.o" \
"CMakeFiles/objdet.dir/ControlsBox.cpp.o"

# External object files for target objdet
objdet_EXTERNAL_OBJECTS =

objdet: CMakeFiles/objdet.dir/main.cpp.o
objdet: CMakeFiles/objdet.dir/GUIController.cpp.o
objdet: CMakeFiles/objdet.dir/GUIModel.cpp.o
objdet: CMakeFiles/objdet.dir/ImageView.cpp.o
objdet: CMakeFiles/objdet.dir/ImageViewGL.cpp.o
objdet: CMakeFiles/objdet.dir/ControlsBox.cpp.o
objdet: CMakeFiles/objdet.dir/build.make
objdet: libod.a
objdet: /usr/local/lib/libfltk_images.a
objdet: /usr/local/lib/libfltk_forms.a
objdet: /usr/local/lib/libfltk_gl.a
objdet: /usr/local/lib/libfltk.a
objdet: /opt/local/lib/libpng.dylib
objdet: /usr/lib/libz.dylib
objdet: /usr/local/lib/libjpeg.dylib
objdet: thirdparty/ImageLib/libimage.a
objdet: thirdparty/libsvm-3.14/libsvm.a
objdet: thirdparty/ImageLib/thirdparty/JPEG/libjpegrw.a
objdet: /usr/local/lib/libfltk.a
objdet: /opt/local/lib/libpng.dylib
objdet: /usr/lib/libz.dylib
objdet: /usr/local/lib/libjpeg.dylib
objdet: CMakeFiles/objdet.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable objdet"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/objdet.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/objdet.dir/build: objdet
.PHONY : CMakeFiles/objdet.dir/build

CMakeFiles/objdet.dir/requires: CMakeFiles/objdet.dir/main.cpp.o.requires
CMakeFiles/objdet.dir/requires: CMakeFiles/objdet.dir/GUIController.cpp.o.requires
CMakeFiles/objdet.dir/requires: CMakeFiles/objdet.dir/GUIModel.cpp.o.requires
CMakeFiles/objdet.dir/requires: CMakeFiles/objdet.dir/ImageView.cpp.o.requires
CMakeFiles/objdet.dir/requires: CMakeFiles/objdet.dir/ImageViewGL.cpp.o.requires
CMakeFiles/objdet.dir/requires: CMakeFiles/objdet.dir/ControlsBox.cpp.o.requires
.PHONY : CMakeFiles/objdet.dir/requires

CMakeFiles/objdet.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/objdet.dir/cmake_clean.cmake
.PHONY : CMakeFiles/objdet.dir/clean

CMakeFiles/objdet.dir/depend:
	cd /Users/hemingge/Desktop/Heming-Bella/CS4670/P5 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/hemingge/Desktop/Heming-Bella/CS4670/P5 /Users/hemingge/Desktop/Heming-Bella/CS4670/P5 /Users/hemingge/Desktop/Heming-Bella/CS4670/P5 /Users/hemingge/Desktop/Heming-Bella/CS4670/P5 /Users/hemingge/Desktop/Heming-Bella/CS4670/P5/CMakeFiles/objdet.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/objdet.dir/depend

