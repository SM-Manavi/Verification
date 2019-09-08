add_library(Lib
Robot.cpp
Environment.cpp
Arrangement.cpp
Event.cpp
dispatch.cpp
M_Utility.cpp)

# state that anybody linking to us needs to include the current source dir
# to find MathFunctions.h, while we don't.
target_include_directories(Lib
          PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
          )
