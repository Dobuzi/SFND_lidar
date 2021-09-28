// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "selfmade/kdtree.h"
#include <unordered_set>
#include <random>
#include <cmath>

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

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr cloud_crop_region (new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr cloud_crop_final (new pcl::PointCloud<PointT>);
    
    typename pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered);

    typename pcl::CropBox<PointT> cb;
    cb.setInputCloud (cloud_filtered);
    cb.setMin (minPoint);
    cb.setMax (maxPoint);
    cb.filter (*cloud_crop_region);

    Eigen::Vector4f minPoint_roof (-3, -2, -1, 1);
    Eigen::Vector4f maxPoint_roof (3, 2, 0.5, 1);
    
    cb.setInputCloud (cloud_crop_region);
    cb.setMin (minPoint_roof);
    cb.setMax (maxPoint_roof);
    cb.setNegative (true);
    cb.filter (*cloud_crop_final);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_crop_final;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr cloud_p (new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr cloud_o (new pcl::PointCloud<PointT>);

    for (int index : inliers->indices)
        cloud_p->points.push_back(cloud->points[index]);
    
    pcl::ExtractIndices<PointT> extract;

    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloud_o);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_o, cloud_p);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.
    bool useBuiltIn = false;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    if (useBuiltIn)
    {
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

        // PCL built-in function : segmentation
        pcl::SACSegmentation<PointT> seg;

        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (maxIterations);
        seg.setDistanceThreshold (distanceThreshold);

        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);
    }
    else
    {
        std::unordered_set<int> inliersResult;
        srand(time(NULL));

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<int> dis(0, cloud->points.size());
        
        PointT point1, point2, point3;
        float A, B, C, D, distance;
        std::unordered_set<int> indices;

        while (maxIterations--)
        {
            point1 = cloud->points[dis(gen)];
            point2 = cloud->points[dis(gen)];
            point3 = cloud->points[dis(gen)];

            A = (point2.y - point1.y) * (point3.z - point1.z) - (point2.z - point1.z) * (point3.y - point1.y);
            B = (point2.z - point1.z) * (point3.x - point1.x) - (point2.x - point1.x) * (point3.z - point1.z);
            C = (point2.x - point1.x) * (point3.y - point1.y) - (point2.y - point1.y) * (point3.x - point1.x);
            D = -1 * (A * point1.x + B * point1.y + C * point1.z);

            indices.clear ();

            for (int j = 0; j < cloud->points.size(); j++)
            {
                distance = abs(A * cloud->points[j].x + B * cloud->points[j].y + C * cloud->points[j].z + D) / sqrt(A * A + B * B + C * C);
                if (distance < distanceThreshold)
                {
                    indices.insert(j);
                }
            }

            if (indices.size() > inliersResult.size())
            {
                inliersResult = indices;
            }
        }

        for (int idx : inliersResult)
        {
            inliers->indices.push_back(idx);
        }
    }

    if (inliers->indices.size() == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    bool useBuiltIn = false;

    if (useBuiltIn)
    {
        typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
        tree->setInputCloud (cloud);

        std::vector<pcl::PointIndices> cluster_indices;

        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance (clusterTolerance);
        ec.setMinClusterSize (minSize);
        ec.setMaxClusterSize (maxSize);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud);
        ec.extract (cluster_indices);

        int j = 0;
        
        for (pcl::PointIndices getIndices : cluster_indices)
        {
            typename pcl::PointCloud<PointT>::Ptr cluster_cloud (new pcl::PointCloud<PointT>);
            for (int idx : getIndices.indices)
            {
                cluster_cloud->push_back ((*cloud)[idx]);
            }
            cluster_cloud->width = cluster_cloud->size ();
            cluster_cloud->height = 1;
            cluster_cloud->is_dense = true;

            clusters.push_back (cluster_cloud);
        }
    }
    else
    {
        // make tree
        KdTree* tree = new KdTree;
		std::vector<std::vector<float>> points;
        for (int i = 0; i < cloud->points.size(); i++)
        {
			std::vector<float> point = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
			points.push_back (point);
            tree->insert(point, i);
        }

        // make clusters
        std::vector<std::vector<int>> clusters_indices = euclideanCluster(points, tree, clusterTolerance, minSize, maxSize);
        for (std::vector<int> cluster_indices : clusters_indices)
        {
            typename pcl::PointCloud<PointT>::Ptr cluster_cloud (new pcl::PointCloud<PointT>);
            for (int idx : cluster_indices)
            {
                cluster_cloud->push_back((*cloud)[idx]);
            }
            cluster_cloud->width = cluster_cloud->size ();
            cluster_cloud->height = 1;
            cluster_cloud->is_dense = true;

            clusters.push_back (cluster_cloud);
        }
    }

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
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cluster, pcaCentroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

    /*
    // Easy way via the PCL-PCA interface
    typename pcl::PointCloud<PointT>::Ptr clusterPCAprojection (new pcl::PointCloud<PointT>);
    typename pcl::PCA<PointT> pca;
    pca.setInputCloud(cluster);
    pca.project(*cluster, *clusterPCAprojection);
    std::cerr << std::endl << "Eigen Vectors: " << pca.getEigenVectors() << std::endl;
    std::cerr << std::endl << "Eigen Values: " << pca.getEigenValues() << std::endl;
    */

    // Transform the original cloud to the origin where the principal components correspond to the axes
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());
    typename pcl::PointCloud<PointT>::Ptr clusterProjected (new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cluster, *clusterProjected, projectionTransform);
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*clusterProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // Final transform
    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA);
    const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

    // Make the BoxQ
    BoxQ boxq;
    boxq.bboxQuaternion = bboxQuaternion;
    boxq.bboxTransform = bboxTransform;
    boxq.cube_width = abs(maxPoint.x - minPoint.x);
    boxq.cube_length = abs(maxPoint.y - minPoint.y);
    boxq.cube_height = abs(maxPoint.z - minPoint.z);

    return boxq;
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
