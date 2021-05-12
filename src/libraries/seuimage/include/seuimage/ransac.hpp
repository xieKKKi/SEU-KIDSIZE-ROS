#ifndef _RANSAC_HPP
#define _RANSAC_HPP

#include <vector>
#include <opencv2/opencv.hpp>

float Ransac(const std::vector<cv::Point2f> &points, cv::Vec4f &_line);
float Ransac(const std::vector<cv::Point2f> &points, cv::Vec4f &_line, int numForEstimate, 
             float successProbability, float maxOutliersPercentage);
#endif
