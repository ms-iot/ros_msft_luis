cmake_minimum_required(VERSION 2.8.3)
project(azure_cs_luis)

## Compile as at least C++17 for cppwinrt
add_compile_options(/Od)
add_compile_options(/std:c++latest)
add_compile_options(/await)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES azure_cs_luis
  CATKIN_DEPENDS roscpp roslib std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/azure_cs_luis.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(MSVC)
find_program(NUGET nuget)
add_custom_target(nuget-restore
  COMMAND ${NUGET} restore ${CMAKE_CURRENT_BINARY_DIR}/packages.config -PackagesDirectory ${CMAKE_CURRENT_BINARY_DIR}
)

include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}/Microsoft.CognitiveServices.Speech.1.1.0/build/native/include/c_api
  ${CMAKE_CURRENT_BINARY_DIR}/Microsoft.CognitiveServices.Speech.1.1.0/build/native/include/cxx_api
)
target_link_libraries(${PROJECT_NAME}_node ${CMAKE_CURRENT_BINARY_DIR}/Microsoft.CognitiveServices.Speech.1.1.0/build/native/x64/Release/Microsoft.CognitiveServices.Speech.core.lib)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/src/packages.config ${CMAKE_CURRENT_BINARY_DIR}/packages.config COPYONLY)
configure_file(${CMAKE_CURRENT_BINARY_DIR}/Microsoft.CognitiveServices.Speech.1.1.0/build/native/x64/Release/Microsoft.CognitiveServices.Speech.core.dll ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Microsoft.CognitiveServices.Speech.core.dll COPYONLY)



add_dependencies(${PROJECT_NAME}_node nuget-restore)
endif()