/* Creado específicamente para crear nubes de puntos de objetos 
básicos, figuras primarias como cubos, esferas, cilindros, pirámides, etc.

*/
#include <string>
//#include <iostream>
#include <sstream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <tf/transform_listener.h>

using namespace std;
void panic(string mensaje){
    printf("Error: %s\n",mensaje.c_str());
    exit(1);
}
void printUsage(){
    printf("Argumentos:\n");
    printf("    nombre figura (cubo, esfera, cilindro)\n");

}
void esfera(pcl::PointCloud<pcl::PointXYZ> &cloud, float radio, float paso){
    tf::Vector3 v(0,0,radio), z_axis(0,0,1), y_axis(0,1,0);
    // Calcular paso yaw y paso pitch
    int nyaws = floor((2 * 3.1415 * radio)/(paso));
    float dyaw = 360.0/nyaws;
    int npitchs = floor(nyaws/2);
    float dpitch = 180.0/npitchs;
    // Calcular cantidad de puntos resultantes
    //int npuntos = 
    int point_index = 0;
    for (int pitch = 0; pitch < npitchs*dpitch; pitch++ ){
        // setear nuevo pitch de vector
        for (int yaw = 0; yaw < nyaws*dyaw; yaw++){
            // cloud.points[point_index].x
            v.rotate(dyaw,z_axis);
        }
        v.rotate(pitch,y_axis);
    }
}
void cubo(pcl::PointCloud<pcl::PointXYZ> &cloud, float lado, float paso){
    int pasos = floor(lado/paso + 1);
    // Calcular cantidad de puntos totales que habrá
    // 2 tapas + (largo -2 pasos) cuadrados
    // int npuntos = 2*(pasos*pasos) + (2*pasos + 2*(pasos-2));
    int npuntos2 = (pasos*pasos*pasos)-((pasos-2)*(pasos-2)*(pasos-2));
    printf("Se construirán %d puntos\n", npuntos2);
    cloud.width = npuntos2;
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize (cloud.width * cloud.height);
    // Iterar por plano
    int point_index = 0;
    for (int zz = 0; zz< pasos ; zz++){
        // si es tapa, hacer toda la tapa
            // hacer tapa
        for (int yy = 0; yy < pasos; yy++){
            for (int xx = 0; xx < pasos ; xx++){
                if (zz == 0  or zz == pasos -1){
                    cloud.points[point_index].x = xx*paso;
                    cloud.points[point_index].y = yy*paso;
                    cloud.points[point_index++].z = zz*paso;
                    printf("point_index = %d\n",point_index);
                }
                else 
                    if (xx == 0 or xx == (pasos -1) or yy == 0 or yy == (pasos -1)){
                        cloud.points[point_index].x = xx*paso;
                        cloud.points[point_index].y = yy*paso;
                        cloud.points[point_index++].z = zz*paso;
                    printf("point_index = %d\n",point_index);
                    }
            }
        }
    }

}
void cilindro(pcl::PointCloud<pcl::PointXYZ> &cloud, float radio, float alto, float paso){

}
int main(int argc, char **argv){
    // Verificar argumentos (nombre figura)
    if (argc < 2){
        printf("Error: Debe ingresar argumentos\n");
        printUsage();
        exit(1);
    }
    string figura (argv[1]);
    printf("figura: %s\n",figura.c_str());

    // Contenedor de figura
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // Verificar argumentos 
    /* 
    esfera: radio, paso
    cubo: Lado, paso
    cilindro: radio, alto, paso
    */
    ostringstream filename;
    filename << "figura_";
    if (figura.compare("esfera") == 0 and argc == 4){
        float radio = atof(argv[2]);
        float paso = atof(argv[3]);
        esfera(*cloud, radio, paso);
        filename  << radio << "_" << paso;
    }
    else if (figura.compare("cubo") == 0 and argc == 4){
        float lado = atof(argv[2]);
        float paso = atof(argv[3]);
        cubo(*cloud, lado, paso);
        filename << lado << "_" << paso;
    }
    else if (figura.compare("cilindro") == 0){
        float radio = atof(argv[2]);
        float alto = atof(argv[3]);
        float paso = atof(argv[4]);
        cilindro(*cloud, radio, alto, paso);
        filename << radio << "_" << alto << "_" << paso;
    }
    else panic("Figura no implementada");
    // Guardar figura en directorio
    filename << ".pcd";
    pcl::io::savePCDFileASCII(filename.str(), *cloud);
    printf("Listo\n");
    return 0;
}