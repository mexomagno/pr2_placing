#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
// boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
// {
//   // --------------------------------------------
//   // -----Open 3D viewer and add point cloud-----
//   // --------------------------------------------
//   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//   viewer->setBackgroundColor (0.7,0.7,0.7);
//   pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
//   viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
//   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
//   viewer->addCoordinateSystem (1.0);
//   viewer->initCameraParameters ();
//   return (viewer);
// }

int main(int argc, char **argv){
    if (argc < 2){
        printf("Error: Debe especificar un archivo .pcd\n");
        exit(1);
    }
    double axis_scale = 1;
    if (argc == 3){
        axis_scale = atof(argv[2]);
        printf("Axis scale: %f\n", axis_scale);
    }
    // Cargar nube de puntos
    PointCloud::Ptr cloud (new PointCloud);
    pcl::io::loadPCDFile(argv[1],*cloud);
    // Crear visualizador
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0.7,0.7,0.7);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
    viewer->addCoordinateSystem (axis_scale);
    viewer->initCameraParameters ();
    while (!viewer->wasStopped()){
        viewer->spinOnce(100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}