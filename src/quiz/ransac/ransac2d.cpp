/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

struct RANSAC_result
{
    std::unordered_set<int> inliers;
    std::pair<int,int> keyPoints;
};

RANSAC_result Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    RANSAC_result result;
    int numPoints = cloud->points.size();
    std::vector < std::pair<int, int> > lines;
    std::pair <int, int> line;
    std::vector <int> inlierCount;
    int numInliers = 0;
    int maxInliers = 0;
    int maxInlierLocation = 0;
    std::vector <pcl::PointXYZ> inliers;
    int rand_a = rand() % numPoints;
    int rand_b = rand() % numPoints;
    float A=0.0, B=0.0, C=0.0;
    float distance = 0.0;
    float denominator = 0.0;
    pcl::PointXYZ pointA, pointB;
    int i = 0, j = 0, k = 0;

    srand(time(NULL));

    for (int i = 0; i < maxIterations; i++)
    {
        rand_a = rand() % numPoints;
        rand_b = rand() % numPoints;
        while (rand_a == rand_b) rand_b = rand() % numPoints;
        line.first = rand_a;
        line.second = rand_b;
        std::cout << "Iteration #" << i << ":(Point_1=" << rand_a << ") & (Point_2=" << rand_b << ")" << std::endl;
        pointA = cloud->points[rand_a];
        pointB = cloud->points[rand_b];
        std::cout << "Point_1=(X:" << pointA.x << ",Y:" << pointA.y << ")//Point_2=(X:" << pointB.x << ",Y:" << pointB.y << ")" << std::endl;
        A = pointA.y - pointB.y;
        B = pointB.x - pointA.x;
        C = ((pointA.x - pointB.x) * pointA.y) + ((pointB.y - pointA.y) * pointA.x);
        std::cout << "A: " << A << " B: " << B << " C: " << C << std::endl;

        denominator = sqrt(pow(A, 2.0f) + pow(B, 2.0f));
        for (j = 0; j < numPoints; j++)
        {
            distance = abs(A * cloud->points[j].x + B * cloud->points[j].y + C) / denominator;
            if (distance < distanceTol)
            {
                inliers.push_back(cloud->points[j]);
            }
        }
        numInliers = inliers.size();
        std::cout << "Number of inliers: " << numInliers << std::endl;
        if (numInliers > maxInliers)
        {
            maxInliers = numInliers;
            maxInlierLocation = i;
            std::cout << "New maximum! " << std::endl;
        }
        else
        {
            std::cout << "Maximum still: " << maxInlierLocation << " - Point 1: " << lines[maxInlierLocation].first
                << " Point 2: " << lines[maxInlierLocation].second << std::endl;
        }
        inlierCount.push_back(numInliers);
        inliers.clear();
        lines.push_back(line);
    }
    
    line.first = lines[maxInlierLocation].first;
    line.second = lines[maxInlierLocation].second;
    result.keyPoints = line;
    pointA = cloud->points[line.first];
    pointB = cloud->points[line.second];
    A = pointA.y - pointB.y;
    B = pointB.x - pointA.x;
    C = ((pointA.x - pointB.x) * pointA.y) + ((pointB.y - pointA.y) * pointA.x);
    denominator = sqrt(pow(A, 2.0f) + pow(B, 2.0f));
    for (i = 0; i < numPoints; i++)
    {
        distance = abs(A * cloud->points[i].x + B * cloud->points[i].y + C) / denominator;
        if (distance < distanceTol)
        {
            result.inliers.insert(i);
        }
    }

	return result;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
    std::unordered_set<int> inliers;
    std::pair<int, int> keypoints;
    
    RANSAC_result result = Ransac(cloud, 50, .5f);
    inliers = result.inliers;
    keypoints = result.keyPoints;

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudKeyPoints(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
        if (index == keypoints.first || index == keypoints.second)
            cloudKeyPoints->points.push_back(point);
        else if (inliers.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
        renderPointCloud(viewer, cloudKeyPoints, "keypoints", Color(0, 0, 1));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
