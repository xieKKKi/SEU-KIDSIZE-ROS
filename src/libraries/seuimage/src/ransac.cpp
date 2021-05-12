#include <seuimage/ransac.hpp>
#include <seuimage/fitline.hpp>

using namespace std;

//返回有效点的比例
float Ransac(const std::vector<cv::Point2f> &points, std::vector<float> &line)
{
    line.clear();
    line.resize(4);
    size_t Cnt = points.size();
	int numDataObjects = Cnt;
	int numForEstimate = Cnt*0.1;
	int maxVoteCnt = 0;
	cv::Vec4f tempLine;
	float inliersPercentage = 0.0;
	
	//随机抽取一定比例的点
	int ransac_times = 500;
	vector<int> Chosen(numDataObjects, 0);

	std::vector<cv::Point2f> subPoints;
	int voteCnt = 0;
	for(int i = 0; i < ransac_times; i++)
	{
		//随机抽取 
		//randomly select data for exact model fit ('numForEstimate' objects).
        int maxIndex = numDataObjects-1;
		for(int j = 0; j < numForEstimate; j++)
		{
			int selectedIndex = rand() % numDataObjects;
			Chosen[selectedIndex] = 1;
		}
		//拟合
        subPoints.clear();
		for(int k = 0; k < numDataObjects; k++)
		{
			if(Chosen[k])
			{
                subPoints.push_back(points[k]);
			}
		}
		FitLine2D(subPoints, tempLine);
		float a = tempLine[1]/tempLine[0];
		float b = tempLine[3] - a*tempLine[2];
		
		//拟合完整之后要对拟合的结果进行鉴定，选出最优的结果
		voteCnt = 0;
		for(int k = 0; k < Cnt; k++)
		{
			//如果在直线上或者附近
			if(abs(points[k].y - a*points[k].x - b) < 2)
			{
				voteCnt++;
			}
		}

		if(voteCnt > maxVoteCnt)
		{
			maxVoteCnt = voteCnt;
			inliersPercentage = (float)maxVoteCnt/Cnt;
			for(int m = 0; m < 4; m++)
			{
				line[m] = tempLine[m];
			}
			
		}	
		//当inliers的比例比较高的时候就可以直接取该值作为最优解
//		if(inliersPercentage > 0.2)
//		{
//			return inliersPercentage;
//		}
	}
	return inliersPercentage;
}

float Ransac(const std::vector<cv::Point2f> &points, cv::Vec4f &_line,
	int numForEstimate, float successProbability, float maxOutliersPercentage)
{
	//1 − p = (1 − w^n)^k
	//p = 
	//float outlierPercentage = maxOutliersPercentage;//估计值
    size_t Cnt = points.size();
	float numerator = log(1.0f-successProbability);
	float denominator = log(1.0f- pow(1.0-maxOutliersPercentage, numForEstimate));
	//随机抽取一定比例的点
	int ransac_times = (int)(numerator/denominator + 0.5);
	
 	printf("ransac_times： %d\n", ransac_times);
	int numDataObjects = Cnt;
	//int numForEstimate = Cnt*0.1;
	int maxVoteCnt = 0;
	cv::Vec4f tempLine;
	float inliersPercentage = 0.0;
	
	std::vector<int> Chosen(numDataObjects, 0);

	std::vector<cv::Point2f> subPoints;
	int voteCnt = 0;
	for(int i = 0; i < ransac_times; i++)
	{
		//随机抽取 
		//randomly select data for exact model fit ('numForEstimate' objects).
        int maxIndex = numDataObjects-1;
		for(int j = 0; j < numForEstimate; j++)
		{
			int selectedIndex = rand() % numDataObjects;
			Chosen[selectedIndex] = 1;
		}
		//拟合
        subPoints.clear();
		for(int k = 0; k < numDataObjects; k++)
		{
			if(Chosen[k])
			{
                subPoints.push_back(points[k]);
			}
		}
		FitLine2D(subPoints, tempLine);
		float a = tempLine[1]/tempLine[0];
		float b = tempLine[3] - a*tempLine[2];
		
		//拟合完整之后要对拟合的结果进行鉴定，选出最优的结果
		voteCnt = 0;
		for(int k = 0; k < Cnt; k++)
		{
			//如果在直线上或者附近
			if(abs(points[k].y - a*points[k].x - b) < 2)
			{
				voteCnt++;
			}
		}

		if(voteCnt > maxVoteCnt)
		{
			maxVoteCnt = voteCnt;
			inliersPercentage = (float)maxVoteCnt/Cnt;
//			printf("a: %f\tb%f\tpercent: %f\n", a, b, inliersPercentage);
			for(int m = 0; m < 4; m++)
			{
				_line[m] = tempLine[m];
			}
			
		}	
		//当inliers的比例比较高的时候就可以直接取该值作为最优解
//		if(inliersPercentage > 0.2)
//		{
//			return inliersPercentage;
//		}
	}
	return inliersPercentage;
}
            
            
            