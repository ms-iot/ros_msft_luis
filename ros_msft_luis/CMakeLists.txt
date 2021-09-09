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
  # WORKAROUND: parson/parson.c(62): warning C4232: nonstandard extension used: 'parson_malloc'
  add_compile_options(/wd4232)
endif()


find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  audio_common_msgs
  ros_msft_luis_msgs
  resource_retriever
)


add_library(parson STATIC
  parson/parson.c
  parson/parson.h
)

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
    resource_retriever
)

include_directories(
  include
  parson
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/azure_cs_luis.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node 
${catkin_LIBRARIES}
parson
)

message("Installing Microsoft Cognitive Services Nuget package")

set(SPEECH_RUNTIME "Microsoft.CognitiveServices.Speech.1.16.0")
set(PACKAGE_URL "https://www.nuget.org/api/v2/package/Microsoft.CognitiveServices.Speech/1.16.0")
set(PACKAGE_SHA512 "395973ad434efac7d33166ad1f749ab1b20cd6d516faa789351f45895a4ded8dff3ee9d666bf686358fb062dc65cb48ad3ada8dc8a495910e9211aa32058d64a")

file(DOWNLOAD
    ${PACKAGE_URL}
    ${CMAKE_CURRENT_BINARY_DIR}/speech.nuget
    EXPECTED_HASH SHA512=${PACKAGE_SHA512}
    SHOW_PROGRESS
)

file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/${SPEECH_RUNTIME}")

if(MSVC)
  set(ARCH "win-x64" )
  execute_process(COMMAND tar xvzf "${CMAKE_CURRENT_BINARY_DIR}/speech.nuget" 
               WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/${SPEECH_RUNTIME}"
               )

else()
  set(ARCH "linux-x64")
  execute_process(COMMAND unzip "${CMAKE_CURRENT_BINARY_DIR}/speech.nuget" 
               WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/${SPEECH_RUNTIME}"
               )

endif()

include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}/${SPEECH_RUNTIME}/build/native/include/c_api
  ${CMAKE_CURRENT_BINARY_DIR}/${SPEECH_RUNTIME}/build/native/include/cxx_api
)

if(MSVC)
  target_link_libraries(${PROJECT_NAME}_node ${CMAKE_CURRENT_BINARY_DIR}/${SPEECH_RUNTIME}/build/native/x64/Release/Microsoft.CognitiveServices.Speech.core.lib)
  configure_file(${CMAKE_CURRENT_BINARY_DIR}/${SPEECH_RUNTIME}/runtimes/${ARCH}/native/Microsoft.CognitiveServices.Speech.core.dll ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Microsoft.CognitiveServices.Speech.core.dll COPYONLY)
  configure_file(${CMAKE_CURRENT_BINARY_DIR}/${SPEECH_RUNTIME}/runtimes/${ARCH}/native/Microsoft.CognitiveServices.Speech.extension.audio.sys.dll ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Microsoft.CognitiveServices.Speech.extension.audio.sys.dll COPYONLY)
  configure_file(${CMAKE_CURRENT_BINARY_DIR}/${SPEECH_RUNTIME}/runtimes/${ARCH}/native/Microsoft.CognitiveServices.Speech.extension.codec.dll ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Microsoft.CognitiveServices.Speech.extension.codec.dll COPYONLY)
  configure_file(${CMAKE_CURRENT_BINARY_DIR}/${SPEECH_RUNTIME}/runtimes/${ARCH}/native/Microsoft.CognitiveServices.Speech.extension.kws.dll ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Microsoft.CognitiveServices.Speech.extension.kws.dll COPYONLY)
  configure_file(${CMAKE_CURRENT_BINARY_DIR}/${SPEECH_RUNTIME}/runtimes/${ARCH}/native/Microsoft.CognitiveServices.Speech.extension.silk_codec.dll ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Microsoft.CognitiveServices.Speech.extension.silk_codec.dll COPYONLY)
  configure_file(${CMAKE_CURRENT_BINARY_DIR}/${SPEECH_RUNTIME}/build/native/x64/Release/Microsoft.CognitiveServices.Speech.core.lib ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Microsoft.CognitiveServices.Speech.core.lib COPYONLY)
else()
  target_link_libraries(${PROJECT_NAME}_node ${CMAKE_CURRENT_BINARY_DIR}/${SPEECH_RUNTIME}/runtimes/${ARCH}/native/libMicrosoft.CognitiveServices.Speech.core.so)
  target_link_libraries(${PROJECT_NAME}_node ${CMAKE_CURRENT_BINARY_DIR}/${SPEECH_RUNTIME}/runtimes/${ARCH}/native/libMicrosoft.CognitiveServices.Speech.extension.audio.sys.so)
  target_link_libraries(${PROJECT_NAME}_node ${CMAKE_CURRENT_BINARY_DIR}/${SPEECH_RUNTIME}/runtimes/${ARCH}/native/libMicrosoft.CognitiveServices.Speech.extension.codec.so)
  target_link_libraries(${PROJECT_NAME}_node ${CMAKE_CURRENT_BINARY_DIR}/${SPEECH_RUNTIME}/runtimes/${ARCH}/native/libMicrosoft.CognitiveServices.Speech.extension.kws.so)
  configure_file(${CMAKE_CURRENT_BINARY_DIR}/${SPEECH_RUNTIME}/runtimes/${ARCH}/native/libMicrosoft.CognitiveServices.Speech.core.so ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/libMicrosoft.CognitiveServices.Speech.core.so COPYONLY)
  configure_file(${CMAKE_CURRENT_BINARY_DIR}/${SPEECH_RUNTIME}/runtimes/${ARCH}/native/libMicrosoft.CognitiveServices.Speech.extension.audio.sys.so ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/libMicrosoft.CognitiveServices.Speech.extension.audio.sys.so COPYONLY)
  configure_file(${CMAKE_CURRENT_BINARY_DIR}/${SPEECH_RUNTIME}/runtimes/${ARCH}/native/libMicrosoft.CognitiveServices.Speech.extension.codec.so ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/libMicrosoft.CognitiveServices.Speech.extension.codec.so COPYONLY)
  configure_file(${CMAKE_CURRENT_BINARY_DIR}/${SPEECH_RUNTIME}/runtimes/${ARCH}/native/libMicrosoft.CognitiveServices.Speech.extension.kws.so ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/libMicrosoft.CognitiveServices.Speech.extension.kws.so COPYONLY)
endif()