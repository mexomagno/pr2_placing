#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/PolygonMesh.h>

pcl::PolygonMesh getConvexHull(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    pcl::PolygonMesh mesh;
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud(cloud);
    chull.reconstruct(mesh);
    return mesh;
}
int main (int argc, char** argv){
    if (argc < 2){
        printf("Error: Debe especificar un archivo .pcd\n");
        exit(1);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if ( pcl::io::loadPCDFile <pcl::PointXYZ> (argv[1], *in_cloud) == -1)
    {
      std::cout << "Cloud reading failed." << std::endl;
      return (-1);
    }

    // pcl::IndicesPtr indices (new std::vector <int>);
    // pcl::PassThrough<pcl::PointXYZ> pass;
    // pass.setInputCloud (cloud);
    // pass.setFilterFieldName ("z");
    // pass.setFilterLimits (0.0, 1.0);
    // pass.filter (*indices);

    // Obtener convex hull
    pcl::PolygonMesh hull = getConvexHull(in_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>());
    fromPCLPointCloud2(hull.cloud, *cloud);

    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (in_cloud);
    normal_estimator.setKSearch (50);
    normal_estimator.compute (*normals);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize (1);
    reg.setMaxClusterSize (1000000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (5);
    reg.setInputCloud (cloud);
    //reg.setIndices (indices);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold (1.0);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

    std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
    std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
    std::cout << "These are the indices of the points of the initial" <<
      std::endl << "cloud that belong to the first cluster:" << std::endl;
    int counter = 0;
    while (counter < clusters[0].indices.size ())
    {
      std::cout << clusters[0].indices[counter] << ", ";
      counter++;
      if (counter % 10 == 0)
        std::cout << std::endl;
    }
    std::cout << std::endl;

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
    pcl::visualization::CloudViewer viewer ("Cluster viewer");
    viewer.showCloud(colored_cloud);
    while (!viewer.wasStopped ())
    {
    }
    return (0);
}