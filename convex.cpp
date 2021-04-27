//
// Created by dorian on 22/04/2021.
//

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/visualization/pcl_visualizer.h>
using namespace pcl::console;
using namespace std;

int main(int argc, char *argv[]){
    pcl::PCDReader reader;
    string pcd_file;
    if (argc < 1)
    {
        printf("\nUsage:  pcd<PointXYZ>-in-file \n");
        exit(0);
    }
    pcd_file = argv[1];

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
    reader.read(pcd_file, *cloud);
    printf("  loading %s\n", pcd_file.c_str());
    cout << "PointCloud before filtering has: " << cloud->points.size() << " data points." << endl;


    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setInputCloud(cloud);
    hull.setDimension(3);

    vector<pcl::Vertices> polygons;
    pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>);
    hull.reconstruct(*surface_hull, polygons);

    cout<<"surface_hull_size:" << surface_hull->size();

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);
    viewer->setBackgroundColor(255, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(cloud, 176, 224, 230);
    viewer->addPointCloud(cloud, color_handler, "point_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "point_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handlerK(surface_hull, 255, 0, 0);
    viewer->addPointCloud(surface_hull, color_handlerK, "point_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "point_cloud");

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }

    return (0);

}
