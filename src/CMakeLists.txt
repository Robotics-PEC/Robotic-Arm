cmake_minimum_required(VERSION 3.5)
project(manipulator)

# Use ament_cmake
find_package(ament_cmake REQUIRED)

# Install directories
foreach(dir config launch meshes urdf)
    install(DIRECTORY ${dir}/
        DESTINATION share/${PROJECT_NAME}/${dir})
endforeach()

# Include directories if needed
# include_directories(include)

# Add executable targets and link libraries if needed
# add_executable(your_node src/your_node.cpp)
# ament_target_dependencies(your_node rclcpp std_msgs)

# Export the package
ament_export_dependencies()
ament_export_include_directories(include)
ament_export_libraries(libraries)

# Install additional files or directories if needed
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
)

# Declare the ament package (must be the last command)
ament_package()
