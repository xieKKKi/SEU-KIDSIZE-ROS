#include <seuimage/fielddetect.hpp>

namespace seuimage
{
    using namespace std;
    using namespace cv;
    #define distance 2

    FieldDetector::FieldDetector()
    {
    }

    vector<int> FieldDetector::newScanLine(cv::Mat &src)
    {
        int width = src.cols;
        int height = src.rows;

        int heightOfEnoughGreen = height * 3 / 7;
        int heightStartCountPenalty = height * 7 / 10;
        int heightOfFault = 10;
        bool firstGreen = false;

        int greenPointsCount = 0;
        int boundPointsCount = 0;
        bool isGreen;

        double weight = 0;
        double maxWeight = 0;
        int upperBound = height - 1;
        int finalBound = height - 1;
        double reward = 1;
        double penalty = -1;
        
        int continuousNotGreenCount = 0;
        bool continuousNotGreenFLAG = false;
        bool breakFLAG = false;
        int divideLine = 250;
        
        double pixelDistance = 0.0;
        Vec3f position;
        
        int debug = 0;
        vector<int> bound(width / myStep);

        for (int i = 1; i < width; i += myStep)
        {
            for (int j = height - 1; j >= 0; j--)
            {
                Vec3b v = src.at<Vec3b>(j, i);
                isGreen = (v[1] == 255);

                if (isGreen)
                {
                        weight += reward;
                        greenPointsCount++;
                        continuousNotGreenCount = 0;
                        continuousNotGreenFLAG = false;
                        debug = 1;
                        firstGreen = true;
                }
                else
                {
                    if (continuousNotGreenFLAG)
                    {
                        continuousNotGreenCount++;
                    }
                    continuousNotGreenFLAG = true;
                    if ((j < heightStartCountPenalty) && (firstGreen = true))
                    {
                        weight += penalty;
                    }
                    debug = 0;
                }

                if (weight > maxWeight)
                {
                    maxWeight = weight;
                    upperBound = j;
                }
            }
            
            if(breakFLAG){
                upperBound = finalBound;
            }
            breakFLAG = false;
            bound[boundPointsCount++] = upperBound;
            upperBound = height - 1;
            finalBound = height - 1;

            greenPointsCount = 0;
            continuousNotGreenCount = 0;
            continuousNotGreenFLAG = false;
            firstGreen = false;

            divideLine = 250;
            reward = 1;
            penalty = -1;

            weight = 0;
            maxWeight = 0;
        }

        return bound;
     }

    bool FieldDetector::Contours_bound(IplImage *img)
    {
        CvScalar cs;
        int w = img->width;
        int h = img->height;
        int val;

        for (int i = 0; i != w; i++)
        {
            val = 0;

            for (int j = 0; j != h; j++)
            {
                cs = cvGet2D(img, j, i);

                if (cs.val[0] == 255)
                {
                    val = 255;
                }

                cvSet2D(img, j, i, cvScalar(val));
            }
        }

        return true;
    }

    /**
    * @brief 对于不在凸包线上的点，将其上移到左右两个在凸包线上的候选点形成的点
    * @param src
    * @param candidate　候选边界点
    * @param fitler　凸包线上的点
    * @return　滤波后的边界点
    **/
    vector<int> FieldDetector::ConvexHullFitler(IplImage *src, vector<int> candidate, vector<int> fitler)
    {
        int width = src->width;
        int fitlerLet = 0;
        int fitlerRight = 0;
        vector<Point> candidatePoints(width);
        vector<Point> fitlerPoints(width);
        vector<bool> ifMatch(width);

        CvPoint emptyPoint;
        emptyPoint.x = -1;
        emptyPoint.y = -1;

        for (int i = 0; i < width; i++)
        {
            candidatePoints[i] = emptyPoint;
            fitlerPoints[i] = emptyPoint;
            ifMatch[i] = false;
        }

        // Mark - 将横坐标转成CvPoint
        int whatCount = 0;

        for (int i = 1; i < width; i += myStep)
        {
            candidatePoints[i].x = i;
            fitlerPoints[i].x = i;
            candidatePoints[i].y = candidate[whatCount];
            fitlerPoints[i].y = fitler[whatCount];

            if (candidate[whatCount] == fitler[whatCount])
            {
                ifMatch[i] = true;
            }
            else
            {
                ifMatch[i] = false;
            }

            whatCount++;
        }

        Point matchLeft;
        Point matchRight;
        int fitlerY = 0;

        for (int i = 1; i < width; i += myStep)
        {
            if (ifMatch[i] == false)
            {
                matchLeft = findMatch(i, 0, ifMatch, candidatePoints);

                matchRight = findMatch(i, 1, ifMatch, candidatePoints);
                if ((matchLeft.x == -1) && (matchRight.x != -1))
                {
                    if (((matchRight.x - i) <= myStep * 5) && (matchRight.y > fitlerPoints[i].y))
                    {
                        candidatePoints[i].y = matchRight.y;
                    }
                }
                else if ((matchLeft.x != -1) && (matchRight.x == -1))
                {
                    if (((matchLeft.x - i) <= myStep * 5) && (matchLeft.y > fitlerPoints[i].y))
                    {
                        candidatePoints[i].y = matchLeft.y;
                    }

                }
                else if (matchLeft.x != -1 && matchRight.x != -1)
                {
                    fitlerY = fitlerCompute(i, matchLeft, matchRight);

                    if (candidatePoints[i].y > fitlerY)
                    {
                        candidatePoints[i].y = fitlerY;
                    }
                }
            }
        }

        vector<int> boundFitler(width / myStep);
        whatCount = 0;

        for (int i = 1; i < width; i += myStep)
        {
            boundFitler[whatCount] = candidatePoints[i].y;
            whatCount++;
        }
        return boundFitler;
    }

    /**
     * @brief　寻找在凸包线上的候选点
     * @param currentPosition 当前横坐标
     * @param leftOrRight 往左或往右寻找 0是左边，1是右边
     * @param ifMatch 候选点是否在凸包线上
     * @param　candidatePoints 候选点集
     * @return　返回匹配到的点，若匹配不到，返回的点的x和y为-1
     **/
    CvPoint FieldDetector::findMatch(int currentPosition, bool leftOrRight, vector<bool> &ifMatch,
                                     vector<Point> &candidatePoints)
    {
        CvPoint returnPoint;
        returnPoint.x = -1;
        returnPoint.y = -1;

        if (leftOrRight)
        {
            for (int i = currentPosition; i < candidatePoints.size(); i += myStep)
            {
//                cout<<"正在判断横坐标"<<i<<"是否匹配"<<endl;
                if (ifMatch[i])
                {
                    returnPoint.x = candidatePoints[i].x;
                    returnPoint.y = candidatePoints[i].y;
                    break;
                }
            }
        }
        else
        {
            for (int i = currentPosition; i >= 1; i -= myStep)
            {
                if (ifMatch[i])
                {
                    returnPoint.x = candidatePoints[i].x;
                    returnPoint.y = candidatePoints[i].y;
                    break;
                }
            }
        }
        return returnPoint;
    }

    /**
     * @brief　已经找到左右匹配到的点，计算形成的点的纵坐标
     * @param currentPosition 当前横坐标
     * @param matchLeft 左边匹配到的点
     * @param matchRight 右边匹配到的点
     * @return　形成的点的纵坐标
     **/
    int FieldDetector::fitlerCompute(int currentPosition, Point matchLeft, Point matchRight)
    {
        int fitlerY = 0;

        if (matchRight.x - matchLeft.x == 0)
        {
            cout << "除于０" << endl;
        }

        double k = (double) (matchRight.y - matchLeft.y) / (matchRight.x - matchLeft.x);
        double b = (double) matchRight.y - k * matchRight.x;
        fitlerY = k * currentPosition + b;
//        IMAGEPROC_LOG<<"Bound fitler Compute finish"<<endl;
        return fitlerY;
    }


    bool FieldDetector::Find_Contours(IplImage *dst, IplImage *src)//(cvdst1,sample)
    {
        CvMemStorage *storage = cvCreateMemStorage(0);
        CvSeq *contour = 0;

        //clock_t start=clock();
        cvFindContours(src, storage, &contour, sizeof(CvContour), CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
        //clock_t end=clock();
        //cout<<"time:"<<(end-start)*1.0/(CLOCKS_PER_SEC)<<endl;

        cvZero(dst);//清空数组
        CvSeq *_contour = contour;
        double maxarea = 0;
        double minarea = 100;
        int n = -1, m = 0;//n为面积最大轮廓索引，m为迭代索引 \

        //cout<<"here!!1"<<endl;
        for (; contour != 0; contour = contour->h_next)
        {

            double tmparea = fabs(cvContourArea(contour));

//       cout<<"tmparea:"<<tmparea<<endl;
//       if(tmparea < minarea)
//       {
//    cvSeqRemove(contour, 0); // 删除面积小于设定值的轮廓
//    continue;
//       }
            //cout<<"here!!2"<<endl;
//       CvRect aRect = cvBoundingRect( contour, 0 );
//       if ((aRect.width/aRect.height)<1)
//       {
//    cvSeqRemove(contour, 0); //删除宽高比例小于设定值的轮廓
//    continue;
//       }
//       cout<<"here!!3"<<endl;
            if (tmparea > maxarea)
            {
                maxarea = tmparea;
            }

            m++;
            // 创建一个色彩值
            CvScalar color = CV_RGB(255, 255, 255);

            //max_level 绘制轮廓的最大等级。如果等级为0，绘制单独的轮廓。如果为1，绘制轮廓及在其后的相同的级别下轮廓
            //如果值为2，所有的轮廓。如果等级为2，绘制所有同级轮廓及所有低一级轮廓，诸此种种
            //如果值为负数，函数不绘制同级轮廓，但会升序绘制直到级别为abs(max_level)-1的子轮廓
            //cvDrawContours(dst, contour, color, color, 1, 1, 8);   //绘制外部和内部的轮廓
        }

        //cout<<"maxarea:"<<maxarea<<endl;
        contour = _contour;
        int count = 0;

        for (; contour != 0; contour = contour->h_next)
        {
            count++;
            double tmparea = fabs(cvContourArea(contour));

            if (tmparea == maxarea)
            {
                //cout<<"max"<<endl;
                CvScalar extern_color = cvScalar(255);
                CvScalar hole_color = cvScalar(120);
                cvDrawContours(dst, contour, extern_color, hole_color, 0, 1, 8);

            }
        }

        cvReleaseMemStorage(&storage);
//   printf("The total number of contours is:%d", count);
//   cout<<endl;
        Contours_bound(dst);
        return true;
    }

    vector<int> FieldDetector::ConvexHull(cv::Mat &dst, vector<int> boundfirst, const bool &debug)
    {
        // cvZero(imgdst);
        vector<int> bound(dst.cols);

        CvScalar cs;
        uint8_t val = 50;
        CvPoint pt0;

        int count = 0, hullcount = 0;
        vector<CvPoint> greenpoints;

        int w = dst.cols;
        int h = dst.rows;
        int stepW = w / boundfirst.size();
        int stepH = 1;//sample->height=480
        int whatCount = 0;

        for (int i = 1; i < w; i += myStep)
        {
            for (int j = h - 1; j > boundfirst.at(whatCount); j--)
            {
                pt0.x = i;
                pt0.y = j * stepH;
                greenpoints.push_back(pt0); //获得所有绿点
                count++;
            }

            whatCount++;
        }

        _nogreen = false;

        if (greenpoints.size() == 0) //若没有绿色候选点
        {
            _nogreen = true;
            return bound;
        }

        count = greenpoints.size();
        vector<Point> points(count); //分配内存空间
        vector<Point> hull;

        for (int i = 0; i < greenpoints.size(); i++)
        {
            points[i] = greenpoints[i];
        }

        /**
        * @brief 凸包函数  OpenCV
        * @param point_mat 需要凸包的点集，这里为两通道的向量
        * @param hull_mat 获得的凸包的点集
        * @param CV_CLOCKWISE 逆时针
        * @return
        * */
        convexHull(points, hull, CV_CLOCKWISE, false);

        for (int i = 0; i < hullcount; i++)
        {
            Point pt = points[hull[i]];

            if (pt0.x - pt.x > 0)
            {
                if (debug)
                {
                    line(dst, pt0, pt, Scalar(255, 255, 255), 1);
                }

                float k = (pt0.y - pt.y) * 1.0 / (pt0.x - pt.x);
                int y = 0;

                for (int j = pt.x; j < pt0.x || j == dst.cols - 1; j = j + 1)
                {
                    y = k * (j - pt0.x) + pt0.y;
                    bound[j] = y;

                    if (debug)
                    {
                        CvPoint boundline;
                        boundline.x = j;
                        boundline.y = y;
                    }
                }
            }

            pt0 = pt;
        }

        for (int i = 0; i < stepW; i++)
        {
            bound[i] = bound[stepW + 1];
        }

        for (int i = bound.size() - 2 * stepW; i < bound.size(); i++)
        {
            bound[i] = bound[bound.size() - 2 * stepW - 1];
        }
        vector<int> bound1;

        for (int i = 0; i < boundfirst.size(); i++)
        {
            bound1.push_back(bound[i * stepW]);
        }
        return bound1;
    }

    vector<int> FieldDetector::GetBound(cv::Mat &cvSrc, int minIntensity)
    {
        _greenMinIntensity = minIntensity;
        IplImage *sample, *cvDst1;
        A = B = Al = Bl = Ar = Br = 1000;
        auto t = (double)cvGetTickCount();
        vector<int> boundScan = newScanLine(cvSrc);
        t = (double)cvGetTickCount() - t;
        int width = cvSrc.cols;
        int height = cvSrc.rows;
        vector<int> boundHull = ConvexHull(cvSrc, boundScan, false);
        vector<int> boundFitler = ConvexHullFitler(cvSrc, boundScan, boundHull);

        if (boundfinal.size() < width)
            for (int i = 0; i < width; i++)
            {
                boundfinal.push_back(0);
            }


        allransac(boundFitler, true, false, 0);//bound:每一列的边界点坐标（宽度为sample->width）
        
        left.x = left.y = 0;
        right.x = width - 1;
        right.y = 0;

        //cout<<"al　"<<Al<<"　bl　"<<Bl<<endl;
        //cout<<"a　"<<A<<"　b　"<<B<<endl;
        //cout<<"ar　"<<Ar<<"　br　"<<Br<<endl;
        if (Al != 1000 && Bl != 1000)
        {
            left.x = int((Bl - B) / (A - Al));

            if (left.x < 0 | left.x > width - 1)
            {
                left.x = 0;
            }
        }
        else
        {
            left.x = 0;
        }

        if (Ar != 1000 && Br != 1000)
        {
            right.x = int((Br - B) / (A - Ar));

            if (right.x < 0 | right.x > width - 1)
            {
                right.x = width - 1;
            }
        }
        else
        {
            right.x = width - 1;
        }

        //cout<<"left "<<left.x<<" right"<<right.x<<endl;
        IMAGEPROC_LOG << "calculate the cross finish" << endl;

        for (int i = left.x; i < right.x; i++)
        {
            int coordinate = (int) (A * i + B);

            if (coordinate < 0)
            {
                coordinate = 0;
            }

            if (coordinate > height - 1)
            {
                coordinate = height - 1;
            }

            boundfinal[i] = coordinate;
        }

        if (left.x > 0)
        {
            for (int i = 0; i < left.x; i++)
            {
                int coordinate = (int) (Al * i + Bl);

                if (coordinate < 0)
                {
                    coordinate = 0;
                }

                if (coordinate > height - 1)
                {
                    coordinate = height - 1;
                }

                boundfinal[i] = coordinate;
            }
        }

        if (right.x < width - 1)
        {
            for (int i = right.x; i < width; i++)
            {
                int coordinate = (int) (Ar * i + Br);

                if (coordinate < 0)
                {
                    coordinate = 0;
                }

                if (coordinate > height - 1)
                {
                    coordinate = height - 1;
                }

                boundfinal[i] = coordinate;
            }
        }

        IMAGEPROC_LOG << "Draw line finish" << endl;
        delete points;
        vector<int> bound1(boundfinal);
        IMAGEPROC_LOG << "Bound detect finish" << endl;
        return bound1;
    }

    vector<int> FieldDetector::allransac(vector<int> bound1, bool IfRecursive, bool IsRansacRight, int begin)
    {
        boundInit(bound1, begin);
//         FitLine2D(points, inlierCnt, NULL, lines);
//         float a = lines[1] / lines[0];
//         float b = lines[3] - a * lines[2];

        //weighted least square fit with only inliers
        FitLine2D(points, inlierCnt, lines);
        float a = lines[1] / lines[0];
        float b = lines[3] - a * lines[2];

        //Ransac fit with inlier and outlier; note: the result maybe wrong
        Ransac(points, COUNT, lines);
        a = lines[1] / lines[0];
        b = lines[3] - a * lines[2];
        Range range;
        int numForEstimate = 5;
        float successProbability = 0.999f;
        float maxOutliersPercentage = (float) outlierCnt / COUNT;
        range = Ransac(points, COUNT, lines, numForEstimate, successProbability, maxOutliersPercentage, IsRansacRight,
                       begin);
        a = lines[1] / lines[0];
        b = lines[3] - a * lines[2];

        //cout<<IsRansacRight<<" "<<"low"<<range.low_limit<<" "<<"upper"<<range.upper_limit<<endl;
        //cout<<a<<"  "<<b<<endl;
        if (IfRecursive)
        {
            A = a;
            B = b;
//        cout<<"middle"<<a<<" "<<b<<endl;
        }
        else if (IsRansacRight)
        {
            Ar = a;
            Br = b;
//        cout<<"right"<<a<<" "<<b<<endl;
        }
        else
        {
            Al = a;
            Bl = b;
//        cout<<"left"<<a<<" "<<b<<endl;
        }

        if (IfRecursive)
        {
            if (range.low_limit > 5)
            {
                vector<int> boundleft;

                for (int i = 0; i < range.low_limit; i++)
                {
                    boundleft.push_back(bound1.at(i));    //为拟合的第二条直线的bound赋值
                }

                allransac(boundleft, false, false, 0);
            }
            else
            {
                range.low_limit = 0;
            }

            //右边
            if (range.upper_limit < bound1.size() - 6)
            {
                vector<int> boundright;

                for (int i = range.upper_limit; i < bound1.size(); i++)
                {
                    boundright.push_back(bound1.at(i));    //为拟合的第二条直线的bound赋值
                }
                allransac(boundright, false, true, range.upper_limit);

            }
            else
            {
                range.upper_limit = bound1.size();
            }
        }
        vector<int> boundsend(boundfinal);
        return boundsend;
    }

    float FieldDetector::Ransac(const std::vector<cv::Point2f> &points, int count, cv::Vec4f &_line)
    {
        int numDataObjects = count;
        int numForEstimate = count * 0.1;
        int maxVoteCnt = 0;
        Vec4f tempLine;
        float inliersPercentage = 0.0;

        //随机抽取一定比例的点
        int ransac_times = 500;
        vector<int> Chosen(numDataObjects, 0);

        std::vector<Point2f> subPoints(numForEstimate);
        int pointCnt = 0;
        int voteCnt = 0;

        for (int i = 0; i < ransac_times; i++)
        {
            //随机抽取
            //randomly select data for exact model fit ('numForEstimate' objects).
            int maxIndex = numDataObjects - 1;

            for (int j = 0; j < numForEstimate; j++)
            {
                int selectedIndex = rand() % numDataObjects;
                Chosen[selectedIndex] = 1;
            }

            //拟合
            pointCnt = 0;

            for (int k = 0; k < numDataObjects; k++)
            {
                if (Chosen[k])
                {
                    subPoints[pointCnt].x = points[k].x;
                    subPoints[pointCnt].y = points[k].y;
                    pointCnt++;
                }
            }

            FitLine2D(subPoints, pointCnt, tempLine);
            float a = tempLine[1] / tempLine[0];
            float b = tempLine[3] - a * tempLine[2];


            //拟合完整之后要对拟合的结果进行鉴定，选出最优的结果
            voteCnt = 0;

            for (int k = 0; k < count; k++)
            {
                //如果在直线上或者附近
                if (abs(points[k].y - a * points[k].x - b) < distance)
                {
                    voteCnt++;
                }
            }

            if (voteCnt > maxVoteCnt)
            {
                maxVoteCnt = voteCnt;
                inliersPercentage = (float) maxVoteCnt / count;

                for (int m = 0; m < 4; m++)
                {
                    _line[m] = tempLine[m];
                }

            }

            //当inliers的比例比较高的时候就可以直接取该值作为最优解
            if (inliersPercentage > 0.4)
            {
                return inliersPercentage;
            }
        }

        return inliersPercentage;

    }

    Range FieldDetector::Ransac(const std::vector<cv::Point2f> &points, int count, Vec4f &_line, 
                int numForEstimate, float successProbability,
                float maxOutliersPercentage, bool IsRansacRight, int begin)
    {
        //1 ? p = (1 ? w^n)^k
        //p =
        //float outlierPercentage = maxOutliersPercentage;//估计值
        int count = points.size();
        float numerator = log(1.0f - successProbability);
        float denominator = log(1.0f - pow(1.0 - maxOutliersPercentage, numForEstimate));
        //随机抽取一定比例的点
        int ransac_times = (int) (numerator / denominator + 0.5);

        //printf("ransac_times： %d\n", ransac_times);
        int numDataObjects = count;
        //int numForEstimate = count*0.1;
        int maxVoteCnt = 0;
        Vec4f tempLine;
        float inliersPercentage = 0.0;


        vector<int> Chosen(numDataObjects, 0);
        Range range;
        std::vector<cv::Point2f> subPoints(numForEstimate);
        int pointCnt = 0;
        int voteCnt = 0;


        for (int i = 0; i < ransac_times; i++)
        {
            //随机抽取
            //randomly select data for exact model fit ('numForEstimate' objects).
            int maxIndex = numDataObjects - 1;

            for (int j = 0; j < numForEstimate; j++)
            {
                int selectedIndex = rand() % numDataObjects;
                Chosen[selectedIndex] = 1;
            }

            //拟合
            pointCnt = 0;

            for (int k = 0; k < numDataObjects; k++)
            {
                if (Chosen[k])
                {
                    subPoints[pointCnt].x = points[k].x;
                    subPoints[pointCnt].y = points[k].y;
                    pointCnt++;
                }
            }

            FitLine2D(subPoints, pointCnt, tempLine);
            float a = tempLine[1] / tempLine[0];
            float b = tempLine[3] - a * tempLine[2];

            //拟合完整之后要对拟合的结果进行鉴定，选出最优的结果
            voteCnt = 0;

            for (int k = 0; k < count; k++)
            {
                //如果在直线上或者附近
                if (abs(points[k].y - a * points[k].x - b) < distance)
                {
                    voteCnt++;
                }
            }

            if (voteCnt > maxVoteCnt)
            {
                int num = 0;
                int num1 = 0;

                for (int k = 0; k < count; k++)
                {
                    //如果在直线上或者附近
                    if (abs(points[k].y - a * points[k].x - b) < distance)
                    {
                        num++;

                        if (num == 1)
                        {
                            range.low_limit = k;
                        }

                        num1 = k;
                    }
                }

                range.upper_limit = num1;
                maxVoteCnt = voteCnt;
                inliersPercentage = (float) maxVoteCnt / count;

                for (int m = 0; m < 4; m++)
                {
                    _line[m] = tempLine[m];
                }
            }

       

            //当inliers的比例比较高的时候就可以直接取该值作为最优解
            if (inliersPercentage > 0.4)
            {
                for (int i = max(0, range.low_limit - 5); i < range.low_limit; i++)
                {
                    if (abs(points[i].y - a * points[i].x - b) < distance)
                    {
                        range.low_limit = i;
                        break;
                    }
                }

                for (int i = min(COUNT - 1, range.upper_limit + 5); i > range.upper_limit; i--)
                {

                    if (abs(points[i].y - a * points[i].x - b) < distance)
                    {
                        range.upper_limit = i;
                        break;
                    }
                }

                if (IsRansacRight)
                {
                    range.low_limit += begin;
                    range.upper_limit += begin;
                }
                return range;
            }
        }

        float a = tempLine[1] / tempLine[0];
        float b = tempLine[3] - a * tempLine[2];

        for (int i = max(0, range.low_limit - 5); i < range.low_limit; i++)
        {
            if (abs(points[i].y - a * points[i].x - b) < distance)
            {
                range.low_limit = i;
                break;
            }
        }

        for (int i = min(COUNT - 1, range.upper_limit + 5); i > range.upper_limit; i--)
        {
            if (abs(points[i].y - a * points[i].x - b) < distance)
            {
                range.upper_limit = i;
                break;
            }
        }

        if (IsRansacRight)
        {
            range.low_limit += begin;
            range.upper_limit += begin;
        }

        return range;
    }

    int FieldDetector::FitLine2D(const std::vector<cv::Point2f> &points, int count,
        const std::vector<float> &weights, Vec4f &line)
    {
        double x = 0, y = 0
        , x2 = 0, y2 = 0, xy = 0, w = 0;
        double dx2, dy2, dxy;
        int i;
        float t;

        /* Calculating the average of x and y... */
        if (weights.size()==0)
        {
            for (i = 0; i < count; i += 1)
            {
                x += points[i].x;
                y += points[i].y;
                x2 += points[i].x * points[i].x;
                y2 += points[i].y * points[i].y;
                xy += points[i].x * points[i].y;
            }

            w = (float) count;
        }
        else
        {
            for (i = 0; i < count; i += 1)
            {
                x += weights[i] * points[i].x;
                y += weights[i] * points[i].y;
                x2 += weights[i] * points[i].x * points[i].x;
                y2 += weights[i] * points[i].y * points[i].y;
                xy += weights[i] * points[i].x * points[i].y;
                w += weights[i];
            }
        }

        x /= w;
        y /= w;
        x2 /= w;
        y2 /= w;
        xy /= w;

        dx2 = x2 - x * x;
        dy2 = y2 - y * y;
        dxy = xy - x * y;

        t = (float) atan2(2 * dxy, dx2 - dy2) / 2;
        line[0] = (float) cos(t);
        line[1] = (float) sin(t);

        line[2] = (float) x;
        line[3] = (float) y;

        return 0;
    }


    double FieldDetector::CalcDist2D(const std::vector<cv::Point2f> &points, int count,
            const Vec4f &_line, std::vector<float> &dist)
    {
        int j;
        float px = _line[2], py = _line[3];
        float nx = _line[1], ny = -_line[0];
        double sum_dist = 0.;

        for (j = 0; j < count; j++)
        {
            float x, y;

            x = points[j].x - px;
            y = points[j].y - py;

            dist[j] = (float) fabs(nx * x + ny * y);
            sum_dist += dist[j];
        }

        return sum_dist;
    }


    int FieldDetector::FitLine2D(const std::vector<cv::Point2f> &points, int count, cv::Vec4f &line)
    {
        //进行一遍拟合，获取直线参数
        vector<float> tw;
        FitLine2D(points, count, tw, line);
        //计算权值，再进行一次拟合
        std::vector<float> dist(count);
        std::vector<float> W(count);

        //迭代进行加权拟合，迭代次数不小于三次
        for (int i = 0; i < 5; i++)
        {
            CalcDist2D(points, count, line, dist);
            WeightL1(dist, count, W);
            FitLine2D(points, count, W, line);
        }
    }

    void FieldDetector::WeightL1(const std::vector<float> &d, int count, std::vector<float> &w)
    {
        int i;
        for (i = 0; i < count; i++)
        {
            double t = fabs((double) d[i]);
            w[i] = (float) (1. / max(t, eps));
        }
    }
}