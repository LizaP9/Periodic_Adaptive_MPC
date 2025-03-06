/*!
 * @file utilities.cpp
 * @brief Common utility functions
 */

#include <ctime>
#include <iomanip>
#include <iostream>

#include "Configuration.h"
#include "Utilities/utilities.h"

/*!
 * Write std::string to file with given name
 */
void writeStringToFile(const std::string& fileName, const std::string& fileData)
{
  FILE* fp = fopen(fileName.c_str(), "w");
  if (!fp)
  {
    printf("Failed to fopen %s\n", fileName.c_str());
    throw std::runtime_error("Failed to open file");
  }
  fprintf(fp, "%s", fileData.c_str());
  fclose(fp);
}

/*!
 * Get the current time and date as a string
 */
std::string getCurrentTimeAndDate()
{
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  std::ostringstream ss;
  ss << std::put_time(&tm, "%c");
  return ss.str();
}

geometry_msgs::Point ros::toMsg(const Vec3<float>& data)
{
  geometry_msgs::Point out;
  out.x = data(0);
  out.y = data(1);
  out.z = data(2);
  return out;
}

Vec3<float> ros::fromMsg(const geometry_msgs::Point& data)
{
  Vec3<float> out;
  out(0) = data.x;
  out(1) = data.y;
  out(2) = data.z;
  return out;
}
