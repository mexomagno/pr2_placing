#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (PointCloud::ConstPtr cloud)
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
    PointCloud::Ptr cloud (new PointCloud), cloud_filt (new PointCloud);
    pcl::io::loadPCDFile(argv[1],*cloud);

    // Crear filtro. Remueve todos los puntos que se salen del campo especificado.
    // En este caso, quita puntos tales que "z" está fuera de [0,1.1].
    // Se puede setear como filtro inverso con   pass.setFilterLimitsNegative (true);
/*    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, 5);
    pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filt);*/
    // Crear convex hull alrededor den nube
    PointCloud::Ptr hull(new PointCloud);
    pcl::ConcaveHull<pcl::PointXYZ> chull;
    chull.setInputCloud(cloud);
    chull.setAlpha(0.1);
    chull.reconstruct(*hull);
    // Crear visualizador
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = simpleVis(hull);
    while (!viewer->wasStopped()){
        viewer->spinOnce(100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}