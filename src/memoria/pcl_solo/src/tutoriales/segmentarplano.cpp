#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/centroid.h>
//#include "extras.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// Constantes
const double LEAFSIZE = 0.05;
const double SEG_THRESHOLD = 0.01;

// Globales
pcl::SACSegmentation<pcl::PointXYZ> segmentator;
pcl::VoxelGrid<pcl::PointXYZ> subsampler;

/*boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0.7,0.7,0.7);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}*/
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0.7,0.7,0.7);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


int main(int argc, char **argv){
    if (argc < 2){
        printf("Error: Debe especificar un archivo .pcd\n");
        exit(1);
    }
    // Cargar nube de puntos
    PointCloud::Ptr cloud (new PointCloud);
    pcl::io::loadPCDFile(argv[1],*cloud);
    // Crear visualizador
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = simpleVis(cloud);
    // Segmentar
    segmentator.setOptimizeCoefficients(true);
    segmentator.setModelType(pcl::SACMODEL_PLANE);
    segmentator.setMethodType(pcl::SAC_RANSAC);
    segmentator.setDistanceThreshold(SEG_THRESHOLD);
    segmentator.setInputCloud(cloud);
    pcl::ModelCoefficients::Ptr coefs (new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    segmentator.segment(*inliers,*coefs);
    // Ver qué se obtuvo
    printf("Se obtienen %d puntos\n",(int)inliers->indices.size());
    if (inliers->indices.size() == 0){
      printf("Segmentación no sirvió de nada.\n");
      exit(1);
    }
    viewer->addPlane(*coefs, "planito");
    // Obtener centroide y dibujarlo
    Eigen::Vector4f centroid;
    compute3DCentroid(*cloud,centroid);
    viewer->addSphere(pcl::PointXYZ(centroid(0),centroid(1),centroid(2)), 0.2, "centroide");
    // Dibujar un plano en el viewer para indicar más menos qué se obtuvo
    // http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_p_c_l_visualizer.html
    while (!viewer->wasStopped()){
        viewer->spinOnce(100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}