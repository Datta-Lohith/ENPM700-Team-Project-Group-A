#include <iostream>
#include "pid.hpp"

int main() {
  PIDController pid(0.1, 0.01, 0.5, 1.0);
  std::cout << "PID Output: " << pid.compute(100.0, 90.0) << std::endl;
  return 0;
}
