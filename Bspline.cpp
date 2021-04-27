//
// Created by dorian on 22/04/2021.
//

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <math.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
#include <pcl/surface/on_nurbs/triangulation.h>
#include <pcl/console/parse.h>


using namespace std;
using namespace pcl;
using namespace pcl::console;
typedef pcl::PointXYZ Point;

void
PointCloud2Vector3d(PointCloud<Point>::Ptr cloud, on_nurbs::vector_vec3d &data);

void
visualizeCurve(ON_NurbsCurve &curve,
               ON_NurbsSurface &surface,
               visualization::PCLVisualizer &viewer);

int
main(int argc, char *argv[])
{
    string pcd_file, file_3dm;

    if (argc < 2)
    {
        printf("\nUsage:  pcd<PointXYZ>-in-file -order 3 -refinement 4 -iter 10 -mesh_resolution 128 -two_dim 1\n\n");
        exit(0);
    }
    pcd_file = argv[1];

    visualization::PCLVisualizer viewer("Bspline");
    viewer.setBackgroundColor(255, 255, 255);
    viewer.setSize(900, 900);

   // loading

    printf("  loading %s\n", pcd_file.c_str());
    PointCloud<Point>::Ptr cloud(new PointCloud<Point>);
    PCLPointCloud2 cloud2;
    on_nurbs::NurbsDataSurface data;

    if (io::loadPCDFile(pcd_file, cloud2) == -1)
        throw runtime_error("PCD file not found or exit.");

    fromPCLPointCloud2(cloud2, *cloud);
    PointCloud2Vector3d(cloud, data.interior);
    visualization::PointCloudColorHandlerCustom<Point> handler(cloud, 0, 255, 0);
    viewer.addPointCloud<Point>(cloud, handler, "cloud_cylinder");
    printf("  %lu cloud->size():\n", cloud->size());


    unsigned order(3);
    unsigned refinement(4);
    unsigned iterations(10);
    unsigned mesh_resolution(128);
    bool two_dim = true;
    parse_argument(argc, argv, "-oder", order);
    parse_argument(argc, argv, "-refinement", refinement);
    parse_argument(argc, argv, "-iter", iterations);
    parse_argument(argc, argv, "-mesh_resolution", mesh_resolution);
    parse_argument(argc, argv, "-two_dim", two_dim);
    on_nurbs::FittingSurface::Parameter params;
    params.interior_smoothness = 0.2;
    params.interior_weight = 1.0;
    params.boundary_smoothness = 0.2;
    params.boundary_weight = 0.0;

    // initialize
    printf("  init ...\n");
    ON_NurbsSurface nurbs = on_nurbs::FittingSurface::initNurbsPCABoundingBox(order, &data);
    on_nurbs::FittingSurface fit(&data, nurbs);
    PolygonMesh mesh;
    PointCloud<pcl::PointXYZ>::Ptr mesh_cloud(new PointCloud<pcl::PointXYZ>);
    vector<Vertices> mesh_vertices;
    string mesh_id = "mesh_nurbs";
    on_nurbs::Triangulation::convertSurface2PolygonMesh(fit.m_nurbs, mesh, mesh_resolution);
    viewer.addPolygonMesh(mesh, mesh_id);
    cout << "Before refine" << endl;
    viewer.spinOnce(3000);
    // surface refinement
    for (unsigned i = 0; i < refinement; i++)
    {
        fit.refine(0);
        if (two_dim)fit.refine(1);
        fit.assemble(params);
        fit.solve();
        on_nurbs::Triangulation::convertSurface2Vertices(fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
        viewer.updatePolygonMesh<pcl::PointXYZ>(mesh_cloud, mesh_vertices, mesh_id);
        viewer.spinOnce(3000);
        cout << "refine: " << i << endl;
    }
    //fitting with final refinement level
    for (unsigned i = 0; i < iterations; i++)
    {
        fit.assemble(params);
        fit.solve();
        on_nurbs::Triangulation::convertSurface2Vertices(fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
        viewer.updatePolygonMesh<pcl::PointXYZ>(mesh_cloud, mesh_vertices, mesh_id);
        viewer.spinOnce(3000);
        cout << "iterations: " << i << endl;
    }

    on_nurbs::FittingCurve2dAPDM::FitParameter curve_params;
    curve_params.addCPsAccuracy = 5e-2;
    curve_params.addCPsIteration = 3;
    curve_params.maxCPs = 200;
    curve_params.accuracy = 1;
    curve_params.iterations = 100;

    curve_params.param.closest_point_resolution = 0;
    curve_params.param.closest_point_weight = 1.0;
    curve_params.param.closest_point_sigma2 = 0.1;
    curve_params.param.interior_sigma2 = 0.00001;
    curve_params.param.smooth_concavity = 1.0;
    curve_params.param.smoothness = 1.0;

    printf("  start fitting ...\n");
    on_nurbs::NurbsDataCurve2d curve_data;
    curve_data.interior = data.interior_param;
    curve_data.interior_weight_function.push_back(true);
    ON_NurbsCurve curve_nurbs = on_nurbs::FittingCurve2dAPDM::initNurbsCurve2D(order, curve_data.interior);
    on_nurbs::FittingCurve2dASDM curve_fit(&curve_data, curve_nurbs);
    curve_fit.fitting(curve_params);
    visualizeCurve(curve_fit.m_nurbs, fit.m_nurbs, viewer);

    // triangulation of trimmed surface

    printf("  triangulate trimmed surface ...\n");
    viewer.removePolygonMesh(mesh_id);
    on_nurbs::Triangulation::convertTrimmedSurface2PolygonMesh(fit.m_nurbs, curve_fit.m_nurbs, mesh,
                                                                    mesh_resolution);
    viewer.addPolygonMesh(mesh, mesh_id);

    printf("  ... done.\n");

    viewer.spin();
    return 0;
}

void
PointCloud2Vector3d(pcl::PointCloud<Point>::Ptr cloud, on_nurbs::vector_vec3d &data)
{
    for (unsigned i = 0; i < cloud->size(); i++)
    {
        Point &p = cloud->at(i);
        if (!__isnan(p.x) && !__isnan(p.y) && !__isnan(p.z))
            data.push_back(Eigen::Vector3d(p.x, p.y, p.z));
    }
}

void
visualizeCurve(ON_NurbsCurve &curve, ON_NurbsSurface &surface, visualization::PCLVisualizer &viewer)
{
    PointCloud<pcl::PointXYZRGB>::Ptr curve_cloud(new PointCloud<PointXYZRGB>);

    on_nurbs::Triangulation::convertCurve2PointCloud(curve, surface, curve_cloud, 4);
    for (size_t i = 0; i < curve_cloud->size() - 1; i++)
    {
        PointXYZRGB &p1 = curve_cloud->at(i);
        PointXYZRGB &p2 = curve_cloud->at(i + 1);
        ostringstream os;
        os << "line" << i;
        viewer.removeShape(os.str());
        viewer.addLine<pcl::PointXYZRGB>(p1, p2, 1.0, 0.0, 0.0, os.str());
    }

    PointCloud<pcl::PointXYZRGB>::Ptr curve_curvepoints(new PointCloud<PointXYZRGB>);
    for (int i = 0; i < curve.CVCount(); i++)
    {
        ON_3dPoint p1;
        curve.GetCV(i, p1);

        double pnt[3];
        surface.Evaluate(p1.x, p1.y, 0, 3, pnt);
        PointXYZRGB p2;
        p2.x = float(pnt[0]);
        p2.y = float(pnt[1]);
        p2.z = float(pnt[2]);

        p2.r = 255;
        p2.g = 0;
        p2.b = 0;

        curve_curvepoints->push_back(p2);
    }
    viewer.removePointCloud("point_cloud");
    viewer.addPointCloud(curve_curvepoints, "points");
}

//..............usage:.....................//
//   ./Bspline xxx.pcd parameters  ubuntu
