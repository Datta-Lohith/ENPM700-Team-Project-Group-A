add_library (PID_Controller
  # list of cpp source files:
  pid.cpp
  )

target_include_directories(PID_Controller PUBLIC
  # list of directories:
  .
  )
