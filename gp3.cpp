//
// Created by dorian on 22/04/2021.
//

#include <pcl/point_types.h>                    
#include <pcl/io/pcd_io.h>                      
#include <pcl/kdtree/kdtree_flann.h>            
#include <pcl/features/normal_3d.h>             
#include <pcl/surface/gp3.h>                    
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
using namespace std;

int
main(int argc, char *argv[])
{
    string pcd_file;
    if (argc < 1)
    {
        printf("\nUsage:  pcd<PointXYZ>-in-file \n");
        exit(0);
    }
    pcd_file = argv[1];
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 cloud_blob;
    printf("  loading %s\n", pcd_file.c_str());
    pcl::io::loadPCDFile(pcd_file, cloud_blob); 
    pcl::fromPCLPointCloud2(cloud_blob, *cloud);                          
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);     
    n.setInputCloud(cloud);         
    n.setSearchMethod(tree);        
    n.setKSearch(20);               
    n.compute(*normals);            
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals); 
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3; 
    pcl::PolygonMesh triangles;

    gp3.setSearchRadius(20); 
    gp3.setMu(2.5);                 
    gp3.setMaximumNearestNeighbors(5);   
    gp3.setMaximumSurfaceAngle(M_PI / 4);   
    gp3.setMinimumAngle(M_PI / 18);        
    gp3.setMaximumAngle(2 * M_PI / 3);      
    gp3.setNormalConsistency(false);     

    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);          
    gp3.reconstruct(triangles);          
    std::cout<<"traiangles:"<<triangles<<std::endl;


    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(cloud, 255, 0, 0);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPolygonMesh(triangles,color_handler, "poly_mesh");
    viewer->setBackgroundColor(255, 255, 255);
    viewer->setSize(800, 600);
    viewer->initCameraParameters();
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    return (0);
}

