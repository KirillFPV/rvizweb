cmake_minimum_required(VERSION 3.8)
project(rvizweb)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Copy needed files to www build folder
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/www)
configure_file(package.json www/package.json COPYONLY)
configure_file(index.html www/index.html COPYONLY)
configure_file(bower.json www/bower.json COPYONLY)

# Install webapp build tools
execute_process(
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/www
  COMMAND npm install
)

# Build the webapp with bower and polymer
execute_process(
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/www
  COMMAND npm run build
)

ament_package()

install(
  DIRECTORY
    launch
    config
  DESTINATION
    share/${PROJECT_NAME}
)

install(
  DIRECTORY
    ${CMAKE_CURRENT_BINARY_DIR}/www/build/www
  DESTINATION
    share/${PROJECT_NAME}/www
)
