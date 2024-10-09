add_library (PID_Controller 
  # list of cpp source files:
  src.cpp
  )

target_include_directories(PID_Controller PUBLIC
  # list of directories:
  include
  )
