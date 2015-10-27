#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
//#include <boost/thread/thread.hpp>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

void runOnce(pcl::visualization::PCLVisualizer &viewer){
    viewer.setBackgroundColor(0.1,0.9,0,9);
    viewer.addSphere(pcl::PointXYZ(0,0,0), 0.25,"sphere", 0);
    
}

int main (int argc, char **argv){
    if (argc < 2){
        printf("Error: Debe ingresar archivo\n");
        exit(1);
    }
    // Crear visualizador
    pcl::visualization::CloudViewer viewer("SimpleCloudViewer");
    // Leer nube
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile (argv[1],*cloud);
    viewer.showCloud(cloud);
    viewer.runOnVisualizationThreadOnce(runOnce);
    while (!viewer.wasStopped()){
    }
    return 0;
}