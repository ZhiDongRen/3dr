#include <pcl/point_types.h>                    //PCL中所有点类型定义的头文件
#include <pcl/io/pcd_io.h>                      //打开关闭pcd文件的类定义的头文件
#include <pcl/kdtree/kdtree_flann.h>            //kdtree搜索对象的类定义的头文件
#include <pcl/features/normal_3d.h>             //法向量特征估计相关类定义的头文件
#include <pcl/surface/gp3.h>                    //贪婪投影三角化算法类定义的头文件
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
    // 将一个xyz点类型的pcd文件打开并存储到对象PointCloud中
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 cloud_blob;
    printf("  loading %s\n", pcd_file.c_str());
    pcl::io::loadPCDFile(pcd_file, cloud_blob); //加载dragon.pcd文件
    pcl::fromPCLPointCloud2(cloud_blob, *cloud);                            //*数据最终存储在cloud中
    /**********由于例子中用到的pcd文件只有XYZ坐标，所以我们把它加载到对象
    PointCloud< PointXYZ>中，上面代码打开pcd文件，并将点云存储到cloud指针对象中。 **********/

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;  //法线估计对象
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);   //存储估计的法线
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);  //定义kd树指针
    tree->setInputCloud(cloud);     //用cloud构建tree对象
    n.setInputCloud(cloud);         //为法线估计对象设置输入点云
    n.setSearchMethod(tree);        //设置搜索方法
    n.setKSearch(20);               //设置k搜索的k值为20
    n.compute(*normals);            //估计法线存储结果到normals中
    /***********由于本例中使用的三角化算法输入必须为有向点云，
    所以需要使用PCL中的法线估计方法预先估计出数据中每个点的法线，
    上面的代码就完成该预处理。*********/

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals); //连接字段，cloud_with_normals存储有向点云
    //* cloud_with_normals = cloud + normals
    /*************由于XYZ坐标字段和法线字段需要在相同PointCloud对象中，
    所以创建一个新的PointNormal类型的点云来存储坐标字段和法线连接后的点云。***********/

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);//定义搜索树对象
    tree2->setInputCloud(cloud_with_normals); //利用点云构建搜索树
    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3; //定义三角化对象
    pcl::PolygonMesh triangles;  //存储最终三角化的网格模型
    /******以上代码是对三角化对象相关变量进行定义。******/

    gp3.setSearchRadius(0.025); //设置连接点之间的最大距离（即为三角形最大边长）为0.025
    //设置各参数特征值
    gp3.setMu(2.5);                         //设置被样本点搜索其邻近点的最远距离为2.5，为了适应点云密度的变化
    gp3.setMaximumNearestNeighbors(100);    //设置样本点可搜索的邻域个数为100
    gp3.setMaximumSurfaceAngle(M_PI / 4);   //设置某点法线方向偏离样本点法线方向的最大角度为45度
    gp3.setMinimumAngle(M_PI / 18);         //设置三角化后得到三角形内角最小角度为10度
    gp3.setMaximumAngle(2 * M_PI / 3);      //设置三角化后得到三角形内角最大角度为120度
    gp3.setNormalConsistency(false);        //设置该参数保证法线朝向一致

    // Get result
    gp3.setInputCloud(cloud_with_normals);//设置输入点云为有向点云cloud_with_normals
    gp3.setSearchMethod(tree2);           //设置搜索方式为tree2
    gp3.reconstruct(triangles);           //重建提取三角化
    // std::cout << triangles;

    // 附加顶点信息
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();
    /*********对每个点来说，ID字段中中含有连接组件和该点本身的“状态”
    (例如 gp3.FREE, gp3.BOUNDARY或 gp3.COMPLETED)可以被检索到。**********/

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPolygonMesh(triangles, "my_mesh");
    viewer->setBackgroundColor(255, 255, 255);
    viewer->setSize(800, 600);

    //viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    // 主循环
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    // Finish
    return (0);
}

