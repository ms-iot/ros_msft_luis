cmake_minimum_required(VERSION 2.8.3)
project(ros_msft_luis)

if(MSVC)
  ## Compile as at least C++17 for cppwinrt
  add_compile_options(/Od)
  add_compile_options(/std:c++latest)
  add_compile_options(/await)

  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
  add_definitions(-D_SILENCE_ALL_CXX17_DEPRECATION_WARNINGS)
  
  # WORKAROUND: ros_type_introspection/utils/variant.hpp(235): warning C4267: '=': conversion from 'size_t' to 'uint32_t', possible loss of data
  add_compile_options(/wd4267)
  # WORKAROUND: ros/message_event.h(217): warning C4127: conditional expression is constant
  add_compile_options(/wd4127)
endif()


find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  audio_common_msgs
  ros_msft_luis_msgs
)

find_package(unofficial-parson)
if(unofficial-parson_FOUND)
  message(STATUS "Found Parson")
endif(unofficial-parson_FOUND)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES 
    ros_msft_luis 
  CATKIN_DEPENDS 
    roscpp 
    roslib 
    std_msgs 
    audio_common_msgs
    ros_msft_luis_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/azure_cs_luis.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

if(MSVC)
  target_link_libraries(${PROJECT_NAME}_node 
    ${catkin_LIBRARIES}
    unofficial::parson::parson
    )

  message("Installing Microsoft Cognitive Services Nuget package")

  find_program(NUGET nuget PATHS C:\\opt\\ros\\melodic\\x64\\bin)
  if(NOT NUGET)
      message(FATAL "CMake could not find the nuget command line tool. Please install it from nuget.org")
  else()
    execute_process(COMMAND ${NUGET} restore ${CMAKE_CURRENT_SOURCE_DIR}/src/packages.config -PackagesDirectory ${CMAKE_CURRENT_BINARY_DIR})
    #message(STATUS "the cmake current binary directory  " ${CMAKE_CURRENT_BINARY_DIR} "the binary directory" ${CMAKE_BINARY_DIR})
    #message(STATUS "the cmake prefix path" ${CMAKE_PREFIX_PATH})

    include_directories(
      ${CMAKE_CURRENT_BINARY_DIR}/Microsoft.CognitiveServices.Speech.1.13.0/build/native/include/c_api
      ${CMAKE_CURRENT_BINARY_DIR}/Microsoft.CognitiveServices.Speech.1.13.0/build/native/include/cxx_api
    )

    target_link_libraries(${PROJECT_NAME}_node ${CMAKE_CURRENT_BINARY_DIR}/Microsoft.CognitiveServices.Speech.1.13.0/build/native/x64/Release/Microsoft.CognitiveServices.Speech.core.lib)

    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/src/packages.config ${CMAKE_CURRENT_BINARY_DIR}/packages.config COPYONLY)
    configure_file(${CMAKE_CURRENT_BINARY_DIR}/Microsoft.CognitiveServices.Speech.1.13.0/runtimes/win-x64/native/Microsoft.CognitiveServices.Speech.core.dll ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Microsoft.CognitiveServices.Speech.core.dll COPYONLY)
  endif()
else()
  if(EXISTS "$ENV{SPEECHSDK_ROOT}/include/c_api/speechapi_c.h")
 
      include_directories(
        $ENV{SPEECHSDK_ROOT}/include/c_api
        $ENV{SPEECHSDK_ROOT}/include/cxx_api
      )

      if ("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "aarch64")
        set(COG_ARCH "arm64")
      else()
        set(COG_ARCH "x64")
      endif()

      find_package( PATHS )


      message("Speech SDK Link - $ENV{SPEECHSDK_ROOT}/lib/${COG_ARCH}")

      target_link_libraries(${PROJECT_NAME}_node 
        ${catkin_LIBRARIES}
        unofficial::parson::parson
        libasound.so.2
        $ENV{SPEECHSDK_ROOT}/lib/${COG_ARCH}/libMicrosoft.CognitiveServices.Speech.core.so
        )

  else()
        message(FATAL "Please install and define SPEECHSDK_ROOT per the instructions at https://github.com/Azure-Samples/cognitive-services-speech-sdk/tree/master/quickstart/cpp/linux/from-microphone")
  endif()
endif()


install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
