#ifndef FITLINE_HPP
#define FITLINE_HPP

#include <cmath>
#include <vector>
#include <algorithm>
#include <opencv2/opencv.hpp>

int FitLine2D(const std::vector<cv::Point2f> &points, 
    const std::vector<float> &weights, cv::Vec4f &_line);
double CalcDist2D(const std::vector<cv::Point2f> &points, 
    const cv::Vec4f &_line, std::vector<float> &dist);
int FitLine2D(const std::vector<cv::Point2f> &points, cv::Vec4f &_line);
void WeightL1(const std::vector<float> &d, std::vector<float> &w);

#endif
