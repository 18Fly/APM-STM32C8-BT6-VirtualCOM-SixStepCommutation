# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Release")
  file(REMOVE_RECURSE
  "APM32F103-VirtualCOM-SixStepCommutation-ESC.map"
  )
endif()
