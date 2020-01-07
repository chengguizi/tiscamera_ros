#ifndef UTILS_H
#define UTILS_H

#include <ros/ros.h>
#include <string>
#include <opencv2/core/core.hpp>

namespace utils {

cv::Mat getTransformCV(const ros::NodeHandle &nh,
                       const std::string &field);

cv::Mat getVec16Transform(const ros::NodeHandle &nh,
                          const std::string &field);

cv::Mat getKalibrStyleTransform(const ros::NodeHandle &nh,
                                const std::string &field);
}
#endif