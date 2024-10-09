#include <iostream>
#include "pid.hpp"

int main() {
  PIDController pid(1.0, 1.0, 1.0);
  std::cout << "PID Output: " << pid.compute(20.0, 10.0) << std::endl;
  return 0;
}
