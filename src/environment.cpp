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
    bool renderScene = false; // if you want to see only sensing data, then set false.
    bool render_Rays = false;
    bool render_PCD_raw = false;
    bool render_obstacle_raw = false;
    bool render_plane = true;
    bool render_clusters_as_point = true;
    bool render_clusters_as_Box = true;
    bool render_clusters_as_BoxQ = false;

    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar->scan();
    Color yellow(1, 1, 0);

    if (render_Rays)
    {
        renderRays(viewer, lidar->position, cloud);
    }

    if (render_PCD_raw)
    {
        std::string name = "pcd";
        renderPointCloud(viewer, cloud, name, yellow);
    }
    
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>(); 
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(cloud, 10, 0.2);
    if (render_obstacle_raw)
    {
        renderPointCloud(viewer, segmentCloud.first, "obstacle", Color(1, 0, 0));
    }

    if (render_plane)
    {
        renderPointCloud(viewer, segmentCloud.second, "plane", yellow);
    }
    
    std::vector<typename pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering (segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessor->numPoints (cluster);
        
        if (render_clusters_as_point)
        {
            renderPointCloud(viewer, cluster, "obstacle Cloud"+std::to_string(clusterId), colors[clusterId]);
        }
        
        if (render_clusters_as_Box)
        {
            Box box = pointProcessor->BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }

        if (render_clusters_as_BoxQ)
        {
            BoxQ boxq = pointProcessor->BoundingBoxQ(cluster);
            renderBox(viewer, boxq, clusterId);
        }
        
        ++clusterId;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // Open 3D Viewer and display City Block
    bool render_PCD_raw = false;
    bool render_filtered_cloud = true;

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    if (render_PCD_raw)
    {
        renderPointCloud(viewer, inputCloud, "inputCloud");
    }

    float filterRes = 0.5;
    Eigen::Vector4f minPoint (0.5, 0.5, 0.5, 1);
    Eigen::Vector4f maxPoint (0.8, 0.8, 0.8, 1);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, filterRes, minPoint, maxPoint);
    if (render_filtered_cloud)
    {
        renderPointCloud(viewer, filterCloud, "filterCloud");
    }

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


int main (int argc, char** argv)
{
    bool render_simple_highway = false;
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    if (render_simple_highway)
    {
        simpleHighway(viewer);
    }
    else
    {
        cityBlock(viewer);
    }
    

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}
