# Specify the minimum required version of CMake to build the project.
cmake_minimum_required( VERSION 3.4 )
project( subdeform )

list( APPEND CMAKE_PREFIX_PATH "$ENV{HT}/cmake")
find_package( Houdini REQUIRED )

# Eigen time
set( EIGEN_INCLUDE_DIR "$ENV{EIGEN_INCLUDE_DIR}")
if (NOT EXISTS ${EIGEN_INCLUDE_DIR})
    message(FATAL_ERROR "Specify EIGEN location with $EIGEN_INCLUDE_DIR. CMake will exit now.")
else()
    message(STATUS "EIGEN_INCLUDE_DIR: ${EIGEN_INCLUDE_DIR}")
endif()

include_directories(${EIGEN_INCLUDE_DIR})
# external root path 
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/external)

# Can't link to hboost with standalone apps without that. 
link_directories($ENV{HFS}/dsolib)

# Add a executable.
set( executable_name subdeform )

# Add a SOP dso.
set( library_name SOP_subdeform )

add_executable( ${executable_name}
    # external/cxxopts/include/cxxopts.hpp
    # $ENV{HT}/include/hboost/program_options.hpp 
    src/utils.hpp
    src/subdeform.cpp
)


add_library( ${library_name} SHARED
    src/utils.hpp
    src/SOP_Subdeform.hpp
    src/SOP_Subdeform.cpp
)

# Link against the Houdini libraries, and add required include directories and compile definitions.
target_link_libraries( ${library_name} Houdini )
# Configure several common target properties, such as its output directory.
houdini_configure_target( ${library_name} )

# Link against the Houdini libraries, and add required include directories and compile definitions.
target_link_libraries( ${executable_name} Houdini )
target_link_libraries( ${executable_name} hboost_program_options)
# Configure several common target properties, such as its output directory.
houdini_configure_target( ${executable_name} )



add_executable( playground 
    src/test.cpp
)
# Link against the Houdini libraries, and add required include directories and compile definitions.
target_link_libraries( playground Houdini )
# Configure several common target properties, such as its output directory.
houdini_configure_target( playground )