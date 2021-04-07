// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <pcl/filters/extract_indices.h>


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    //Filter cloud using VoxelGrid
    typename pcl::PointCloud<PointT>::Ptr filteredCloud(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    Eigen::Vector4f leafsize;
    leafsize << filterRes, filterRes, filterRes, filterRes;
    sor.setLeafSize(leafsize);
    sor.filter(*filteredCloud);

    //Apply Region of Interest
    typename pcl::PointCloud<PointT>::Ptr croppedCloud(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> roi(true);
    roi.setMax(maxPoint);
    roi.setMin(minPoint);
    roi.setInputCloud(filteredCloud);
    roi.filter(*croppedCloud);

    //Remove Rooftop Points
    typename pcl::PointCloud<PointT>::Ptr roofRemoved(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof.setInputCloud(croppedCloud);
    roof.setNegative(true);
    roof.filter(*roofRemoved);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return roofRemoved;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr positives(new pcl::PointCloud<PointT>), negatives(new pcl::PointCloud<PointT>);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*positives);
    extract.setNegative(true);
    extract.filter(*negatives);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(positives, negatives);
    return segResult;
}

template <typename PointT>
void ProcessPointClouds<PointT>::proximity(const int id, const typename pcl::PointCloud<PointT>::Ptr inputCloud, typename pcl::PointCloud<PointT>::Ptr cluster, std::vector<bool>& processed,
    KdTree* tree, const float distanceTol)
{
#ifdef PRINT_DEBUG
    std::cout << "Processing node [" << id << "]..." << std::endl;
#endif

    processed[id] = true;
    PointT point = inputCloud->at(id);
    cluster->push_back(point);
 
    std::vector<float> xyz = { point.x, point.y, point.z };
    std::vector<int> nearby = tree->search(xyz, distanceTol);

#ifdef PRINT_DEBUG
    std::cout << "Neighby neighbors found: [";
    for (int i = 0; i < nearby.size(); i++)
    {
        std::cout << nearby[i] << ",";
    }
    std::cout << "]" << std::endl;
#endif

    for (int i = 0; i < nearby.size(); i++)
    {
        if (!processed[nearby[i]])
        {
#ifdef PRINT_DEBUG
            std::cout << "Neighbor [" << nearby[i] << "] not processed, clustering..." << std::endl;
#endif
            ProcessPointClouds::proximity(nearby[i], inputCloud, cluster, processed, tree, distanceTol);
        }
    }
}

template <typename PointT>
typename std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr inputCloud,
    float distanceTol, int minSize, int maxSize)
{
    // TODO: Fill out this function to return list of indices for each cluster
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<bool> processed(inputCloud->points.size(), false);

    //Build KdTree for Cloud Points
    KdTree* tree = new KdTree();
    int i = 0;
    for (auto point : inputCloud->points)
    {
        std::vector<float> xyz = { point.x, point.y, point.z };
        tree->insert(xyz, i);
        i++;
    }

#ifdef PRINT_DEBUG
    std::cout << "Clustering points..." << std::endl;
#endif

    for (int i = 0; i < processed.size(); i++)
    {
#ifdef PRINT_DEBUG
        std::cout << "Checking ID [" << i << "]..." << std::endl;
#endif

        if (!processed[i])
        {
#ifdef PRINT_DEBUG
            std::cout << "Node not yet processed, clustering..." << std::endl;
#endif

            typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>());
            ProcessPointClouds<PointT>::proximity(i,inputCloud, cluster, processed, tree, distanceTol);

#ifdef PRINT_DEBUG
            std::cout << "Cluster created with nodes: ";
            for (std::vector<int>::const_iterator i = cluster.begin(); i != cluster.end(); ++i) std::cout << *i << ",";
            std::cout << std::endl;
#endif
            if(cluster->points.size() <= maxSize && cluster->points.size() >= minSize)
                clusters.push_back(cluster);
        }
    }
    return clusters;
}

template <typename PointT>
pcl::PointIndices::Ptr RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    pcl::PointIndices::Ptr result(new pcl::PointIndices());
    std::tuple<int, int, int> keyPoints;
    int numPoints = cloud->points.size();
    std::vector < std::tuple<int, int, int> > planes;
    std::tuple <int, int, int> plane;
    std::vector <int> inlierCount;
    int numInliers = 0;
    int maxInliers = 0;
    int maxInlierLocation = 0;
    std::vector<PointT> inliers;
    int rand_a, rand_b, rand_c;
    float A = 0.0, B = 0.0, C = 0.0, D = 0.0;
    float distance = 0.0;
    float denominator = 0.0;
    PointT pointA, pointB, pointC;
    int i = 0, j = 0, k = 0;
    bool validPoints = false;

    srand(time(NULL));

    for (int i = 0; i < maxIterations; i++)
    {
        while (!validPoints)
        {
            rand_a = rand() % numPoints;
            rand_b = rand() % numPoints;
            rand_c = rand() % numPoints;
            if (rand_a != rand_b && rand_a != rand_c && rand_b != rand_c) validPoints = true;
        }
        validPoints = false;
        plane = std::make_tuple(rand_a, rand_b, rand_c);
        planes.push_back(plane);
#ifdef PRINT_DEBUG
        std::cout << "Iteration #" << i << ":(Point_1=" << std::get<0>(plane) << ") & (Point_2="
            << std::get<1>(plane) << ") & (Point_3=" << std::get<2>(plane) << ")" << std::endl;
#endif
        pointA = cloud->points[std::get<0>(plane)];
        pointB = cloud->points[std::get<1>(plane)];
        pointC = cloud->points[std::get<2>(plane)];
#ifdef PRINT_DEBUG
        std::cout << "Point_1=(X:" << pointA.x << ",Y:" << pointA.y << ")//Point_2=(X:" << pointB.x
            << ",Y:" << pointB.y << ")//Point_3=(X:" << pointC.x << ",Y:" << pointC.y << ")" << std::endl;
#endif
        A = ((pointB.y - pointA.y) * (pointC.z - pointA.z)) - ((pointB.z - pointA.z) * (pointC.y - pointA.y));
        B = ((pointB.z - pointA.z) * (pointC.x - pointA.x)) - ((pointB.x - pointA.x) * (pointC.z - pointA.z));
        C = ((pointB.x - pointA.x) * (pointC.y - pointA.y)) - ((pointB.y - pointA.y) * (pointC.x - pointA.x));
        D = -(A * pointA.x + B * pointA.y + C * pointA.z);
#ifdef PRINT_DEBUG
        std::cout << "A: " << A << " B: " << B << " C: " << C << " D: " << D << std::endl;
#endif

        denominator = sqrt(pow(A, 2.0f) + pow(B, 2.0f) + pow(C, 2.0f));
        for (j = 0; j < numPoints; j++)
        {
            distance = abs((A * cloud->points[j].x) + (B * cloud->points[j].y) + (C * cloud->points[j].z) + D) / denominator;
            if (distance < distanceTol)
            {
                inliers.push_back(cloud->points[j]);
            }
        }
        numInliers = inliers.size();
#ifdef PRINT_DEBUG
        std::cout << "Number of inliers: " << numInliers << std::endl;
#endif
        if (numInliers > maxInliers)
        {
            maxInliers = numInliers;
            maxInlierLocation = i;
#ifdef PRINT_DEBUG
            std::cout << "New maximum! " << std::endl;
#endif
        }
        else
        {
#ifdef PRINT_DEBUG
            std::cout << "Maximum still: " << maxInlierLocation << " - Point 1: " << std::get<0>(planes[maxInlierLocation])
                << " Point 2: " << std::get<1>(planes[maxInlierLocation]) << "Point 3: " <<
                std::get<2>(planes[maxInlierLocation]) << std::endl;
#endif
        }
        inlierCount.push_back(numInliers);
        inliers.clear();
    }

    plane = std::make_tuple(std::get<0>(planes[maxInlierLocation]), std::get<1>(planes[maxInlierLocation]), std::get<2>(planes[maxInlierLocation]));
    keyPoints = plane;

    pointA = cloud->points[std::get<0>(plane)];
    pointB = cloud->points[std::get<1>(plane)];
    pointC = cloud->points[std::get<2>(plane)];
    A = ((pointB.y - pointA.y) * (pointC.z - pointA.z)) - ((pointB.z - pointA.z) * (pointC.y - pointA.y));
    B = ((pointB.z - pointA.z) * (pointC.x - pointA.x)) - ((pointB.x - pointA.x) * (pointC.z - pointA.z));
    C = ((pointB.x - pointA.x) * (pointC.y - pointA.y)) - ((pointB.y - pointA.y) * (pointC.x - pointA.x));
    D = -(A * pointA.x + B * pointA.y + C * pointA.z);
    denominator = sqrt(pow(A, 2.0f) + pow(B, 2.0f) + pow(C, 2.0f));
#ifdef PRINT_DEBUG
    std::cout << "Best plane found: ";
    std::cout << "A: " << A << " B: " << B << " C: " << C << " D: " << D << std::endl;
#endif

    for (i = 0; i < numPoints; i++)
    {
        distance = abs((A * cloud->points[i].x) + (B * cloud->points[i].y) + (C * cloud->points[i].z) + D) / denominator;
        if (distance < distanceTol)
        {
            result->indices.push_back(i);
        }
    }

    return result;

}

//#define _USING_PCL_TOOLS_
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
    ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
#ifdef _USING_PCL_TOOLS_
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers,*coefficients);
    
    if(inliers->indices.size()==0)
    {
        std::cout << "Could not estimate a planar model" << std::endl;
    }
#else
    pcl::PointIndices::Ptr inliers = RansacPlane<PointT>(cloud, maxIterations, distanceThreshold);
#endif
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult;
    segResult = SeparateClouds(inliers, cloud);
    return segResult;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

#ifdef _USING_PCL_TOOLS_
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    for (pcl::PointIndices getIndices : clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);

        for (int index : getIndices.indices)
            cloudCluster->points.push_back(cloud->points[index]);

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;
        clusters.push_back(cloudCluster);
    }
#else
    clusters = ProcessPointClouds::euclideanCluster(cloud, clusterTolerance,minSize,maxSize);
#endif

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}

template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}