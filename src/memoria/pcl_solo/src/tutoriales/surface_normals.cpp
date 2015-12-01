#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
//#include <boost/thread/thread.hpp>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

int main (int argc, char **argv){
  // Crear visualizador
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

  // Leer nube
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::io::loadPCDFile ("pcds/kitchen/rf3.pcd"),*cloud;
  viewer.showCloud(cloud);

  // // Create the normal estimation class, and pass the input dataset to it
  // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  // ne.setInputCloud (cloud);

  // // Create an empty kdtree representation, and pass it to the normal estimation object.
  // // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  // ne.setSearchMethod (tree);

  // // Output datasets
  // pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // // Use all neighbors in a sphere of radius 3cm
  // ne.setRadiusSearch (0.03);

  // // Compute the features
  // ne.compute (*cloud_normals);

  // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
  while (!viewer->wasStopped()){
    //viewer->spinOnce(100);
    //boost::this_thread::sleep(boost::posix_time::microsecods (100000));
  }
  return 0;
}