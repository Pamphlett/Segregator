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
CMAKE_SOURCE_DIR = /home/pam/catkin_ws/src/Segreagator

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pam/catkin_ws/src/Segreagator/build

# Include any dependencies generated for this target.
include pmc-build/CMakeFiles/pmc.dir/depend.make

# Include the progress variables for this target.
include pmc-build/CMakeFiles/pmc.dir/progress.make

# Include the compile flags for this target's objects.
include pmc-build/CMakeFiles/pmc.dir/flags.make

pmc-build/CMakeFiles/pmc.dir/pmc_heu.cpp.o: pmc-build/CMakeFiles/pmc.dir/flags.make
pmc-build/CMakeFiles/pmc.dir/pmc_heu.cpp.o: pmc-src/pmc_heu.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pam/catkin_ws/src/Segreagator/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object pmc-build/CMakeFiles/pmc.dir/pmc_heu.cpp.o"
	cd /home/pam/catkin_ws/src/Segreagator/build/pmc-build && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pmc.dir/pmc_heu.cpp.o -c /home/pam/catkin_ws/src/Segreagator/build/pmc-src/pmc_heu.cpp

pmc-build/CMakeFiles/pmc.dir/pmc_heu.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pmc.dir/pmc_heu.cpp.i"
	cd /home/pam/catkin_ws/src/Segreagator/build/pmc-build && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pam/catkin_ws/src/Segreagator/build/pmc-src/pmc_heu.cpp > CMakeFiles/pmc.dir/pmc_heu.cpp.i

pmc-build/CMakeFiles/pmc.dir/pmc_heu.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pmc.dir/pmc_heu.cpp.s"
	cd /home/pam/catkin_ws/src/Segreagator/build/pmc-build && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pam/catkin_ws/src/Segreagator/build/pmc-src/pmc_heu.cpp -o CMakeFiles/pmc.dir/pmc_heu.cpp.s

pmc-build/CMakeFiles/pmc.dir/pmc_maxclique.cpp.o: pmc-build/CMakeFiles/pmc.dir/flags.make
pmc-build/CMakeFiles/pmc.dir/pmc_maxclique.cpp.o: pmc-src/pmc_maxclique.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pam/catkin_ws/src/Segreagator/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object pmc-build/CMakeFiles/pmc.dir/pmc_maxclique.cpp.o"
	cd /home/pam/catkin_ws/src/Segreagator/build/pmc-build && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pmc.dir/pmc_maxclique.cpp.o -c /home/pam/catkin_ws/src/Segreagator/build/pmc-src/pmc_maxclique.cpp

pmc-build/CMakeFiles/pmc.dir/pmc_maxclique.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pmc.dir/pmc_maxclique.cpp.i"
	cd /home/pam/catkin_ws/src/Segreagator/build/pmc-build && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pam/catkin_ws/src/Segreagator/build/pmc-src/pmc_maxclique.cpp > CMakeFiles/pmc.dir/pmc_maxclique.cpp.i

pmc-build/CMakeFiles/pmc.dir/pmc_maxclique.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pmc.dir/pmc_maxclique.cpp.s"
	cd /home/pam/catkin_ws/src/Segreagator/build/pmc-build && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pam/catkin_ws/src/Segreagator/build/pmc-src/pmc_maxclique.cpp -o CMakeFiles/pmc.dir/pmc_maxclique.cpp.s

pmc-build/CMakeFiles/pmc.dir/pmcx_maxclique.cpp.o: pmc-build/CMakeFiles/pmc.dir/flags.make
pmc-build/CMakeFiles/pmc.dir/pmcx_maxclique.cpp.o: pmc-src/pmcx_maxclique.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pam/catkin_ws/src/Segreagator/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object pmc-build/CMakeFiles/pmc.dir/pmcx_maxclique.cpp.o"
	cd /home/pam/catkin_ws/src/Segreagator/build/pmc-build && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pmc.dir/pmcx_maxclique.cpp.o -c /home/pam/catkin_ws/src/Segreagator/build/pmc-src/pmcx_maxclique.cpp

pmc-build/CMakeFiles/pmc.dir/pmcx_maxclique.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pmc.dir/pmcx_maxclique.cpp.i"
	cd /home/pam/catkin_ws/src/Segreagator/build/pmc-build && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pam/catkin_ws/src/Segreagator/build/pmc-src/pmcx_maxclique.cpp > CMakeFiles/pmc.dir/pmcx_maxclique.cpp.i

pmc-build/CMakeFiles/pmc.dir/pmcx_maxclique.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pmc.dir/pmcx_maxclique.cpp.s"
	cd /home/pam/catkin_ws/src/Segreagator/build/pmc-build && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pam/catkin_ws/src/Segreagator/build/pmc-src/pmcx_maxclique.cpp -o CMakeFiles/pmc.dir/pmcx_maxclique.cpp.s

pmc-build/CMakeFiles/pmc.dir/pmcx_maxclique_basic.cpp.o: pmc-build/CMakeFiles/pmc.dir/flags.make
pmc-build/CMakeFiles/pmc.dir/pmcx_maxclique_basic.cpp.o: pmc-src/pmcx_maxclique_basic.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pam/catkin_ws/src/Segreagator/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object pmc-build/CMakeFiles/pmc.dir/pmcx_maxclique_basic.cpp.o"
	cd /home/pam/catkin_ws/src/Segreagator/build/pmc-build && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pmc.dir/pmcx_maxclique_basic.cpp.o -c /home/pam/catkin_ws/src/Segreagator/build/pmc-src/pmcx_maxclique_basic.cpp

pmc-build/CMakeFiles/pmc.dir/pmcx_maxclique_basic.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pmc.dir/pmcx_maxclique_basic.cpp.i"
	cd /home/pam/catkin_ws/src/Segreagator/build/pmc-build && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pam/catkin_ws/src/Segreagator/build/pmc-src/pmcx_maxclique_basic.cpp > CMakeFiles/pmc.dir/pmcx_maxclique_basic.cpp.i

pmc-build/CMakeFiles/pmc.dir/pmcx_maxclique_basic.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pmc.dir/pmcx_maxclique_basic.cpp.s"
	cd /home/pam/catkin_ws/src/Segreagator/build/pmc-build && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pam/catkin_ws/src/Segreagator/build/pmc-src/pmcx_maxclique_basic.cpp -o CMakeFiles/pmc.dir/pmcx_maxclique_basic.cpp.s

pmc-build/CMakeFiles/pmc.dir/pmc_cores.cpp.o: pmc-build/CMakeFiles/pmc.dir/flags.make
pmc-build/CMakeFiles/pmc.dir/pmc_cores.cpp.o: pmc-src/pmc_cores.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pam/catkin_ws/src/Segreagator/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object pmc-build/CMakeFiles/pmc.dir/pmc_cores.cpp.o"
	cd /home/pam/catkin_ws/src/Segreagator/build/pmc-build && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pmc.dir/pmc_cores.cpp.o -c /home/pam/catkin_ws/src/Segreagator/build/pmc-src/pmc_cores.cpp

pmc-build/CMakeFiles/pmc.dir/pmc_cores.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pmc.dir/pmc_cores.cpp.i"
	cd /home/pam/catkin_ws/src/Segreagator/build/pmc-build && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pam/catkin_ws/src/Segreagator/build/pmc-src/pmc_cores.cpp > CMakeFiles/pmc.dir/pmc_cores.cpp.i

pmc-build/CMakeFiles/pmc.dir/pmc_cores.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pmc.dir/pmc_cores.cpp.s"
	cd /home/pam/catkin_ws/src/Segreagator/build/pmc-build && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pam/catkin_ws/src/Segreagator/build/pmc-src/pmc_cores.cpp -o CMakeFiles/pmc.dir/pmc_cores.cpp.s

pmc-build/CMakeFiles/pmc.dir/pmc_utils.cpp.o: pmc-build/CMakeFiles/pmc.dir/flags.make
pmc-build/CMakeFiles/pmc.dir/pmc_utils.cpp.o: pmc-src/pmc_utils.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pam/catkin_ws/src/Segreagator/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object pmc-build/CMakeFiles/pmc.dir/pmc_utils.cpp.o"
	cd /home/pam/catkin_ws/src/Segreagator/build/pmc-build && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pmc.dir/pmc_utils.cpp.o -c /home/pam/catkin_ws/src/Segreagator/build/pmc-src/pmc_utils.cpp

pmc-build/CMakeFiles/pmc.dir/pmc_utils.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pmc.dir/pmc_utils.cpp.i"
	cd /home/pam/catkin_ws/src/Segreagator/build/pmc-build && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pam/catkin_ws/src/Segreagator/build/pmc-src/pmc_utils.cpp > CMakeFiles/pmc.dir/pmc_utils.cpp.i

pmc-build/CMakeFiles/pmc.dir/pmc_utils.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pmc.dir/pmc_utils.cpp.s"
	cd /home/pam/catkin_ws/src/Segreagator/build/pmc-build && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pam/catkin_ws/src/Segreagator/build/pmc-src/pmc_utils.cpp -o CMakeFiles/pmc.dir/pmc_utils.cpp.s

pmc-build/CMakeFiles/pmc.dir/pmc_graph.cpp.o: pmc-build/CMakeFiles/pmc.dir/flags.make
pmc-build/CMakeFiles/pmc.dir/pmc_graph.cpp.o: pmc-src/pmc_graph.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pam/catkin_ws/src/Segreagator/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object pmc-build/CMakeFiles/pmc.dir/pmc_graph.cpp.o"
	cd /home/pam/catkin_ws/src/Segreagator/build/pmc-build && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pmc.dir/pmc_graph.cpp.o -c /home/pam/catkin_ws/src/Segreagator/build/pmc-src/pmc_graph.cpp

pmc-build/CMakeFiles/pmc.dir/pmc_graph.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pmc.dir/pmc_graph.cpp.i"
	cd /home/pam/catkin_ws/src/Segreagator/build/pmc-build && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pam/catkin_ws/src/Segreagator/build/pmc-src/pmc_graph.cpp > CMakeFiles/pmc.dir/pmc_graph.cpp.i

pmc-build/CMakeFiles/pmc.dir/pmc_graph.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pmc.dir/pmc_graph.cpp.s"
	cd /home/pam/catkin_ws/src/Segreagator/build/pmc-build && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pam/catkin_ws/src/Segreagator/build/pmc-src/pmc_graph.cpp -o CMakeFiles/pmc.dir/pmc_graph.cpp.s

pmc-build/CMakeFiles/pmc.dir/pmc_clique_utils.cpp.o: pmc-build/CMakeFiles/pmc.dir/flags.make
pmc-build/CMakeFiles/pmc.dir/pmc_clique_utils.cpp.o: pmc-src/pmc_clique_utils.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pam/catkin_ws/src/Segreagator/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object pmc-build/CMakeFiles/pmc.dir/pmc_clique_utils.cpp.o"
	cd /home/pam/catkin_ws/src/Segreagator/build/pmc-build && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pmc.dir/pmc_clique_utils.cpp.o -c /home/pam/catkin_ws/src/Segreagator/build/pmc-src/pmc_clique_utils.cpp

pmc-build/CMakeFiles/pmc.dir/pmc_clique_utils.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pmc.dir/pmc_clique_utils.cpp.i"
	cd /home/pam/catkin_ws/src/Segreagator/build/pmc-build && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pam/catkin_ws/src/Segreagator/build/pmc-src/pmc_clique_utils.cpp > CMakeFiles/pmc.dir/pmc_clique_utils.cpp.i

pmc-build/CMakeFiles/pmc.dir/pmc_clique_utils.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pmc.dir/pmc_clique_utils.cpp.s"
	cd /home/pam/catkin_ws/src/Segreagator/build/pmc-build && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pam/catkin_ws/src/Segreagator/build/pmc-src/pmc_clique_utils.cpp -o CMakeFiles/pmc.dir/pmc_clique_utils.cpp.s

# Object files for target pmc
pmc_OBJECTS = \
"CMakeFiles/pmc.dir/pmc_heu.cpp.o" \
"CMakeFiles/pmc.dir/pmc_maxclique.cpp.o" \
"CMakeFiles/pmc.dir/pmcx_maxclique.cpp.o" \
"CMakeFiles/pmc.dir/pmcx_maxclique_basic.cpp.o" \
"CMakeFiles/pmc.dir/pmc_cores.cpp.o" \
"CMakeFiles/pmc.dir/pmc_utils.cpp.o" \
"CMakeFiles/pmc.dir/pmc_graph.cpp.o" \
"CMakeFiles/pmc.dir/pmc_clique_utils.cpp.o"

# External object files for target pmc
pmc_EXTERNAL_OBJECTS =

devel/lib/libpmc.so: pmc-build/CMakeFiles/pmc.dir/pmc_heu.cpp.o
devel/lib/libpmc.so: pmc-build/CMakeFiles/pmc.dir/pmc_maxclique.cpp.o
devel/lib/libpmc.so: pmc-build/CMakeFiles/pmc.dir/pmcx_maxclique.cpp.o
devel/lib/libpmc.so: pmc-build/CMakeFiles/pmc.dir/pmcx_maxclique_basic.cpp.o
devel/lib/libpmc.so: pmc-build/CMakeFiles/pmc.dir/pmc_cores.cpp.o
devel/lib/libpmc.so: pmc-build/CMakeFiles/pmc.dir/pmc_utils.cpp.o
devel/lib/libpmc.so: pmc-build/CMakeFiles/pmc.dir/pmc_graph.cpp.o
devel/lib/libpmc.so: pmc-build/CMakeFiles/pmc.dir/pmc_clique_utils.cpp.o
devel/lib/libpmc.so: pmc-build/CMakeFiles/pmc.dir/build.make
devel/lib/libpmc.so: /usr/lib/gcc/x86_64-linux-gnu/9/libgomp.so
devel/lib/libpmc.so: /usr/lib/x86_64-linux-gnu/libpthread.so
devel/lib/libpmc.so: pmc-build/CMakeFiles/pmc.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pam/catkin_ws/src/Segreagator/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Linking CXX shared library ../devel/lib/libpmc.so"
	cd /home/pam/catkin_ws/src/Segreagator/build/pmc-build && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pmc.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
pmc-build/CMakeFiles/pmc.dir/build: devel/lib/libpmc.so

.PHONY : pmc-build/CMakeFiles/pmc.dir/build

pmc-build/CMakeFiles/pmc.dir/clean:
	cd /home/pam/catkin_ws/src/Segreagator/build/pmc-build && $(CMAKE_COMMAND) -P CMakeFiles/pmc.dir/cmake_clean.cmake
.PHONY : pmc-build/CMakeFiles/pmc.dir/clean

pmc-build/CMakeFiles/pmc.dir/depend:
	cd /home/pam/catkin_ws/src/Segreagator/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pam/catkin_ws/src/Segreagator /home/pam/catkin_ws/src/Segreagator/build/pmc-src /home/pam/catkin_ws/src/Segreagator/build /home/pam/catkin_ws/src/Segreagator/build/pmc-build /home/pam/catkin_ws/src/Segreagator/build/pmc-build/CMakeFiles/pmc.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : pmc-build/CMakeFiles/pmc.dir/depend
