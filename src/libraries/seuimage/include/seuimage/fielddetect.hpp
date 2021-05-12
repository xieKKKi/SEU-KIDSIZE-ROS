#ifndef __FEILD_DETECT_HPP
#define __FEILD_DETECT_HPP


#include <opencv2/opencv.hpp>
#include <algorithm>

namespace seuimage
{
    #define myStep 4
    struct Range
    {
        int upper_limit;
        int low_limit;
    };

    const double eps = 1e-6;
    class FieldDetector
    {
        bool _nogreen;
        int COUNT;//sum of bound(bound.size)
        int inlierCnt ;
        int outlierCnt ;
        std::vector<cv::Point2f> points;
        float lines[4] ;
        float A, B, Al, Bl, Ar, Br; //三条直线的参数
        CvPoint left, right;

        int _greenMinIntensity;
    private:
        std::vector<int>boundfinal;
        std::vector<int> scanline(IplImage *sample, IplImage *src, const bool &debug);
        std::vector<int> newScanLine(cv::Mat &src);
        bool image_sample(IplImage *dst, IplImage *src); //sample
        bool test_sample(IplImage *dst, IplImage *src);
        bool Contours_bound(IplImage *img);
        bool Find_Contours(IplImage *dst, IplImage *src);
        std::vector<int> ConvexHull(cv::Mat &dst, std::vector<int> boundfirst, const bool &debug);
        std::vector<int> ConvexHullFitler(IplImage *src, std::vector<int> candidate, std::vector<int> fitler);
        CvPoint findMatch(int currentPosition, bool leftOrRight, std::vector<bool> &ifMatch, std::vector<Point> &candidatePoints);
        int fitlerCompute(int currentPosition, Point matchLeft, Point matchRight);
    public:
        std::vector<cv::Point> getCrossPoint()//initPoint:left(0,0),right(639,0)
        {
            std::vector<cv::Point> CrossPoint;
            CrossPoint.push_back(left);
            CrossPoint.push_back(right);
            return CrossPoint;
        }
        int getPointsSize()
        {
            return COUNT;
        }
        void boundInit(std::vector<int>bound , int begin) //begin(0~63):the beginning x coordinate of boundfinal
        {
            COUNT = bound.size();
            inlierCnt = 0.5 * bound.size();
            outlierCnt = COUNT - inlierCnt;

            points.clear();
            points.resize(COUNT);

            for (int i = 0; i < COUNT; i++)
            {
                points[i].x = (begin + i) * myStep + 4;
                points[i].y = bound[i];
            }

            for (int i = 0; i < 4; i++)
            {
                lines[i] = 0.0;
            }
        }
        bool isNoGreen() const
        {
            return _nogreen;
        }
        std::vector<int> GetBound(cv::Mat &cvSrc, int minIntensity);
        FieldDetector();

        //ransac functions
        std::vector<int>myScanLine(IplImage *src);
        std::vector<int> allransac(std::vector<int> bound1, 
                        bool IfRecursive, bool IsRansacRight, int begin);

        int FitLine2D(const std::vector<cv::Point2f> &points, int count, const std::vector<float> &weights, cv::Vec4f &_line);
        double CalcDist2D(const std::vector<cv::Point2f> &points, int count,
                        const cv::Vec4f &_line, std::vector<float> &dist);
        int FitLine2D(const std::vector<cv::Point2f> &points, int count, cv::Vec4f &_line);
        void  WeightL1(const std::vector<float> &d, int count, std::vector<float> &w );

        float Ransac(const std::vector<cv::Point2f> &points, int count, cv::Vec4f &_line);
        Range Ransac(const std::vector<cv::Point2f> &points, int count, cv::Vec4f &_line, int numForEstimate,
                        float successProbability, float maxOutliersPercentage,
                        bool IsRansacRight, int begin);
    };

}
#endif