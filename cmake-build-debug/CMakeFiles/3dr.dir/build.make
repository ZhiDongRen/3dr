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
CMAKE_COMMAND = /home/huake/Downloads/CLion-2020.1.1/clion-2020.1.1/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/huake/Downloads/CLion-2020.1.1/clion-2020.1.1/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/huake/CLionProjects/3dr

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/huake/CLionProjects/3dr/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/3dr.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/3dr.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/3dr.dir/flags.make

CMakeFiles/3dr.dir/main.cpp.o: CMakeFiles/3dr.dir/flags.make
CMakeFiles/3dr.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/huake/CLionProjects/3dr/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/3dr.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/3dr.dir/main.cpp.o -c /home/huake/CLionProjects/3dr/main.cpp

CMakeFiles/3dr.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/3dr.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/huake/CLionProjects/3dr/main.cpp > CMakeFiles/3dr.dir/main.cpp.i

CMakeFiles/3dr.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/3dr.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/huake/CLionProjects/3dr/main.cpp -o CMakeFiles/3dr.dir/main.cpp.s

# Object files for target 3dr
3dr_OBJECTS = \
"CMakeFiles/3dr.dir/main.cpp.o"

# External object files for target 3dr
3dr_EXTERNAL_OBJECTS =

3dr: CMakeFiles/3dr.dir/main.cpp.o
3dr: CMakeFiles/3dr.dir/build.make
3dr: /usr/local/lib/libpcl_surface.so
3dr: /usr/local/lib/libpcl_keypoints.so
3dr: /usr/local/lib/libpcl_tracking.so
3dr: /usr/local/lib/libpcl_recognition.so
3dr: /usr/local/lib/libpcl_stereo.so
3dr: /usr/local/lib/libpcl_outofcore.so
3dr: /usr/local/lib/libpcl_people.so
3dr: /usr/lib/x86_64-linux-gnu/libboost_system.so
3dr: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
3dr: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
3dr: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
3dr: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
3dr: /usr/lib/x86_64-linux-gnu/libboost_regex.so
3dr: /usr/lib/x86_64-linux-gnu/libqhull.so
3dr: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkInfovisCore-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libfreetype.so
3dr: /usr/lib/x86_64-linux-gnu/libz.so
3dr: /usr/lib/x86_64-linux-gnu/libjpeg.so
3dr: /usr/lib/x86_64-linux-gnu/libpng.so
3dr: /usr/lib/x86_64-linux-gnu/libtiff.so
3dr: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkRenderingQt-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkFiltersTexture-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkGUISupportQt-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkRenderingLabel-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
3dr: /usr/lib/x86_64-linux-gnu/libboost_thread.so
3dr: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
3dr: /usr/lib/x86_64-linux-gnu/libboost_system.so
3dr: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
3dr: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
3dr: /usr/local/lib/libpcl_registration.so
3dr: /usr/local/lib/libpcl_segmentation.so
3dr: /usr/local/lib/libpcl_features.so
3dr: /usr/local/lib/libpcl_filters.so
3dr: /usr/local/lib/libpcl_sample_consensus.so
3dr: /usr/local/lib/libpcl_ml.so
3dr: /usr/local/lib/libpcl_visualization.so
3dr: /usr/local/lib/libpcl_search.so
3dr: /usr/local/lib/libpcl_kdtree.so
3dr: /usr/local/lib/libpcl_io.so
3dr: /usr/local/lib/libpcl_octree.so
3dr: /usr/local/lib/libpcl_common.so
3dr: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
3dr: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
3dr: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
3dr: /usr/lib/x86_64-linux-gnu/libboost_regex.so
3dr: /usr/lib/x86_64-linux-gnu/libqhull.so
3dr: /usr/lib/x86_64-linux-gnu/libjpeg.so
3dr: /usr/lib/x86_64-linux-gnu/libpng.so
3dr: /usr/lib/x86_64-linux-gnu/libtiff.so
3dr: /usr/lib/x86_64-linux-gnu/libboost_thread.so
3dr: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
3dr: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
3dr: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkImagingColor-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkIOImage-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkDICOMParser-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkIOCore-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkmetaio-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libz.so
3dr: /usr/lib/x86_64-linux-gnu/libGLU.so
3dr: /usr/lib/x86_64-linux-gnu/libSM.so
3dr: /usr/lib/x86_64-linux-gnu/libICE.so
3dr: /usr/lib/x86_64-linux-gnu/libX11.so
3dr: /usr/lib/x86_64-linux-gnu/libXext.so
3dr: /usr/lib/x86_64-linux-gnu/libXt.so
3dr: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkImagingFourier-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkalglib-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkCommonSystem-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtksys-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libvtkftgl-6.3.so.6.3.0
3dr: /usr/lib/x86_64-linux-gnu/libfreetype.so
3dr: /usr/lib/x86_64-linux-gnu/libGL.so
3dr: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.9.5
3dr: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.9.5
3dr: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.9.5
3dr: CMakeFiles/3dr.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/huake/CLionProjects/3dr/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable 3dr"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/3dr.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/3dr.dir/build: 3dr

.PHONY : CMakeFiles/3dr.dir/build

CMakeFiles/3dr.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/3dr.dir/cmake_clean.cmake
.PHONY : CMakeFiles/3dr.dir/clean

CMakeFiles/3dr.dir/depend:
	cd /home/huake/CLionProjects/3dr/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/huake/CLionProjects/3dr /home/huake/CLionProjects/3dr /home/huake/CLionProjects/3dr/cmake-build-debug /home/huake/CLionProjects/3dr/cmake-build-debug /home/huake/CLionProjects/3dr/cmake-build-debug/CMakeFiles/3dr.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/3dr.dir/depend

