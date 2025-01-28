cmake_minimum_required(VERSION 3.1)
project(planner)

# Set IPO policy before checking for IPO support
cmake_policy(SET CMP0069 NEW)

# Enable most aggressive optimizations
set(CMAKE_BUILD_TYPE Release)

# Use modern C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add ALL Homebrew paths for Apple Silicon
include_directories(/opt/homebrew/include)
link_directories(/opt/homebrew/lib)
link_directories(/opt/homebrew/Cellar)

# Platform-specific optimizations
if(APPLE)
    # macOS specific flags
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG -march=native -mtune=native -flto -ffast-math")
else()
    # Linux/other flags
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG -march=native -mtune=native -flto -ffast-math -fopenmp")
endif()

# Enable interprocedural optimization if available
include(CheckIPOSupported)
check_ipo_supported(RESULT ipo_supported OUTPUT ipo_error)
if(ipo_supported)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

# Find OMPL
find_package(OMPL REQUIRED)

# Create executable
add_executable(planner planner.cpp)

# Link libraries
target_link_libraries(planner 
    PRIVATE 
    ${OMPL_LIBRARIES}
)

# Include directories
target_include_directories(planner 
    PRIVATE 
    ${OMPL_INCLUDE_DIRS}
)

# Enable maximum optimization for this target
set_target_properties(planner 
    PROPERTIES 
    COMPILE_FLAGS "-O3"
    POSITION_INDEPENDENT_CODE ON
)