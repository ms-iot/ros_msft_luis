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
  message("Installing Microsoft Cognitive Services Nuget package")

  find_program(NUGET nuget)
  if(NOT NUGET)
      message(FATAL "CMake could not find the nuget command line tool. Please install it from nuget.org")
  else()
    execute_process(COMMAND ${NUGET} restore ${CMAKE_CURRENT_BINARY_DIR}/packages.config -PackagesDirectory ${CMAKE_CURRENT_BINARY_DIR})

  include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}/Microsoft.CognitiveServices.Speech.1.13.0/build/native/include/c_api
    ${CMAKE_CURRENT_BINARY_DIR}/Microsoft.CognitiveServices.Speech.1.13.0/build/native/include/cxx_api
  )
  target_link_libraries(${PROJECT_NAME}_node ${CMAKE_CURRENT_BINARY_DIR}/Microsoft.CognitiveServices.Speech.1.13.0/build/native/x64/Release/Microsoft.CognitiveServices.Speech.core.lib)

  add_dependencies(${PROJECT_NAME}_node nuget-restore)

  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/src/packages.config ${CMAKE_CURRENT_BINARY_DIR}/packages.config COPYONLY)
  configure_file(${CMAKE_CURRENT_BINARY_DIR}/Microsoft.CognitiveServices.Speech.1.13.0/runtimes/win-x64/native/Microsoft.CognitiveServices.Speech.core.dll ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Microsoft.CognitiveServices.Speech.core.dll COPYONLY)
endif()
endif()