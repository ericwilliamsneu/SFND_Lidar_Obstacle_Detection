/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool optRenderScene = false;
    bool optRenderRays = false;
    bool optRenderCloud = false;
    bool optRenderFullObstCloud = false;
    bool optRenderClusters = true;
    bool optRenderBoundingBoxes = true;
    bool optRenderPlane = false;

    std::vector<Car> cars = initHighway(optRenderScene, viewer);
    
    //Create LIDAR sensor
    Lidar* lidar = new Lidar(cars,0.0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    if(optRenderRays)
        renderRays(viewer,lidar->position,inputCloud);
    if(optRenderCloud)
        renderPointCloud(viewer,inputCloud,"inputCloud");

    //Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud =
        pointProcessor.SegmentPlane(inputCloud, 20, 0.2);
    if (optRenderFullObstCloud)
    {
        renderPointCloud(viewer, segmentCloud.second, "obstCloud", Color(1, 0, 0));
    }

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = pointProcessor.Clustering(segmentCloud.second, 1.0, 3, 50);
    std::vector<Color> Colors = { Color(1,0,0), Color(0,1,0), Color(0,0,1) };

    int i = 0;
    for (auto& cluster : clusters)
    {
        Box box = pointProcessor.BoundingBox(cluster);
        if(optRenderClusters)
            renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(i), Colors[i % Colors.size()]);
        if(optRenderBoundingBoxes)
            renderBox(viewer, box, i);
        i++;
    }

    if(optRenderPlane)
        renderPointCloud(viewer, segmentCloud.first, "planeCloud", Color(0, 1, 0));

    

    
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, pcl::visualization::PCLVisualizer::Ptr& viewer2)
{
    // RENDER OPTIONS
    bool optRenderFullObstCloud = false;
    bool optRenderClusters = true;
    bool optRenderBoundingBoxes = true;
    bool optRenderPlane = true;

    
    ProcessPointClouds<pcl::PointXYZI>* pointProcessor = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessor->loadPcd("C:\\source\\SFND_Lidar_Obstacle_Detection\\src\\sensors\\data\\pcd\\data_1\\0000000000.pcd");
    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    renderPointCloud(viewer, inputCloud, "Input");
    
    //Downsample and filter
    Eigen::Vector4f min, max;
    min << -10, -7.5, -5, 0;
    max << 30, 7.5, 10, 1;
    float filterRes = 0.2;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessor->FilterCloud(inputCloud, 0.2, min, max);

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_segments =
        pointProcessor->SegmentPlane(filteredCloud, 20, 0.25);
    
    if (optRenderPlane)
        renderPointCloud(viewer2, cloud_segments.first, "GroundPlane", Color(0, 1, 0));
    if (optRenderFullObstCloud)
        renderPointCloud(viewer2, cloud_segments.second, "Obstacles", Color(1, 0, 0));

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = pointProcessor->Clustering(cloud_segments.second, 2.0, 3, 500);
    std::vector<Color> Colors = { Color(1,0,0), Color(0,1,0), Color(0,0,1) };

    int i = 0;
    for (auto& cluster : clusters)
    {
        Box box = pointProcessor->BoundingBox(cluster);
        if (optRenderClusters)
            renderPointCloud(viewer2, cluster, "obstCloud" + std::to_string(i), Colors[i % Colors.size()]);
        if (optRenderBoundingBoxes)
            renderBox(viewer2, box, i);
        i++;
    }

}

int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer  (new pcl::visualization::PCLVisualizer ("Input Viewer"));
    pcl::visualization::PCLVisualizer::Ptr viewer2 (new pcl::visualization::PCLVisualizer("Filtered Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    initCamera(setAngle, viewer2);
    //simpleHighway(viewer);
    cityBlock(viewer,viewer2);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}