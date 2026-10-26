#include <cassert>
#include <limits>
#include <iostream>
class CYdLidar { public: float m_MinRange=.15f, m_MaxRange=12.f; bool isRangeValid(double reading) const; };
bool CYdLidar::isRangeValid(double reading) const
{
  if (reading >= m_MinRange && reading <= m_MaxRange)
  {
    return true;
  }

  return false;
}
int main() {
 CYdLidar lidar;
 assert(!lidar.isRangeValid(.149));
 assert(lidar.isRangeValid(lidar.m_MinRange));
 assert(lidar.isRangeValid(.151));
 assert(lidar.isRangeValid(2.));
 assert(lidar.isRangeValid(12.));
 assert(!lidar.isRangeValid(12.001));
 assert(!lidar.isRangeValid(0.));
 assert(!lidar.isRangeValid(std::numeric_limits<double>::infinity()));
 assert(!lidar.isRangeValid(std::numeric_limits<double>::quiet_NaN()));
 std::cout << "PASS 9 boundary/nonfinite cases using unchanged SDK isRangeValid source; no ROS data published\n";
}
