# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Debug")
  file(REMOVE_RECURSE
  "CMakeFiles\\QtTest_autogen.dir\\AutogenUsed.txt"
  "CMakeFiles\\QtTest_autogen.dir\\ParseCache.txt"
  "QtTest_autogen"
  )
endif()
