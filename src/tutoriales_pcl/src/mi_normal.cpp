#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// Para publicar ciertos mensajes
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
// Para tf
#include <tf/transform_listener.h>

float toRad(int grados){
    return grados*2*3.1415/360.0;
}
float toGrad(float rad){
    return rad*360/(2*3.1415);
}
geometry_msgs::Quaternion coefsToQuaternionMsg(float a, float b, float c){
    float vmod=sqrt(a*a+b*b+c*c);
    float pitch = -(1.0*asin(1.0*c/vmod));
    float yaw = (a==0?toRad(90):atan(b/a)) + (a>=0?0:toRad(180));
    return tf::createQuaternionMsgFromRollPitchYaw(0,pitch,yaw);
}
int grados=0;
int paso=1;
int eje=0;
void rotarEnCadaEje(geometry_msgs::PoseStamped *vector){
    vector->pose.position.x=vector->pose.position.y=vector->pose.position.z=0;
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(eje==0?toRad(grados):0,eje==1?toRad(grados):0,eje==2?toRad(grados):0);
    vector->pose.orientation=quat;
    int ang_prev=grados;
    grados=(grados+paso)%360;
    if (grados<ang_prev)
        eje=(eje+1)%3;
}
int caso=0;
void rotarRPY(geometry_msgs::PoseStamped *vector){
    int roll=0,pitch=0,yaw=0;
    vector->pose.position.x=vector->pose.position.y=vector->pose.position.z=0;
    switch (caso){
        case 0:
            roll=pitch=yaw=0;
            break;
        case 1:
            pitch=yaw=0;
            roll=45;
            break;
        case 2:
            roll=pitch=45;
            yaw=0;
            break;
        case 3:
            roll=pitch=yaw=45;
            break;
    }
    printf("Caso: %d\n",caso);
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(toRad(roll),toRad(pitch),toRad(yaw));
    vector->pose.orientation = quat;
    caso=(caso+1)%4;
}
void rotarYPR(geometry_msgs::PoseStamped *vector){
    int roll=0,pitch=0,yaw=0;
    vector->pose.position.x=vector->pose.position.y=vector->pose.position.z=0;
    switch (caso){
        case 0:
            roll=pitch=yaw=0;
            break;
        case 1:
            roll=pitch=0;
            yaw=45;
            break;
        case 2:
            yaw=pitch=45;
            roll=0;
            break;
        case 3:
            roll=pitch=yaw=45;
            break;
    }
    printf("Caso: %d\n",caso);
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(toRad(roll),toRad(pitch),toRad(yaw));
    vector->pose.orientation = quat;
    caso=(caso+1)%4;
}
int cuadrantes[][3]={{1,1,1},{-1,1,1},{-1,-1,1},{1,-1,1},{1,1,-1},{-1,1,-1},{-1,-1,-1},{1,-1,-1}};
int cuadrante=0;
void rotarSegunCoefs(geometry_msgs::PoseStamped *vector){
    /* usa distintos vectores, definidos por sus coeficientes */
    printf("Cuadrante %d\n",(cuadrante+1));
    int a=cuadrantes[cuadrante][0],b=cuadrantes[cuadrante][1],c=cuadrantes[cuadrante][2];
    //float a2b2=sqrt(a*a+b*b);
/*    float yaw= (a==0?toRad(90):atan(b/a)) + (a>=0?0:toRad(180));//atan da valores de -pi/2 a pi/2
    float vmod=sqrt(a*a+b*b+c*c);
    float pitch= -(1.0*asin(1.0*c/vmod));// + (c>=0?0:toRad(180)) );
    printf("Yaw: %f, Pitch: %f\n",toGrad(yaw),toGrad(pitch));*/
    vector->pose.position.x=vector->pose.position.y=vector->pose.position.z=0;
    /*vector->pose.position.x=cuadrantes[cuadrante][0];
    vector->pose.position.y=cuadrantes[cuadrante][1];
    vector->pose.position.z=cuadrantes[cuadrante][2];*/
    //geometry_msgs::Quaternion quat=tf::createQuaternionMsgFromRollPitchYaw(0,pitch,yaw);
    vector->pose.orientation = coefsToQuaternionMsg(a,b,c);
    cuadrante=(cuadrante+1)%8;
}
int gr_cuadrantes[][3]={{45,-45,0},{135,-45,0},{225,-45,0},{315,-45,0},{45,45,0},{135,45,0},{225,45,0},{315,45,0}};
int gr_cuadrante=0;
void rotarSegunGrados(geometry_msgs::PoseStamped *vector){
    /* usa distintos vectores, definidos por sus coeficientes */
    printf("gr_Cuadrante %d\n",(gr_cuadrante+1));
    vector->pose.position.x=vector->pose.position.y=vector->pose.position.z=0;
    geometry_msgs::Quaternion quat=tf::createQuaternionMsgFromRollPitchYaw(0,toRad(gr_cuadrantes[gr_cuadrante][1]),toRad(gr_cuadrantes[gr_cuadrante][0]));
    vector->pose.orientation = quat;
    gr_cuadrante=(gr_cuadrante+1)%8;
}
int main (int argc, char** argv){
    // Crear nodo con el nombre del paquete al que pertenece (obligatorio)
    ros::init (argc, argv, "tutoriales_pcl");
    // Crear punto de acceso al sistema ROS (obligatorio)
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped> ("mi_pose", 1);
    // Frecuencia
    ros::Rate loop_rate(1);
    // Spin
    while (ros::ok()){
        //printf("Eje: %c: R=%d°(%frad), P=%d°(%frad), Y=%d°(%frad)\n", eje==0?'x':(eje==1?'y':'z'),eje==0?grados:0,eje==0?toRad(grados):0,eje==1?grados:0,eje==1?toRad(grados):0,eje==2?grados:0,eje==2?toRad(grados):0);
        geometry_msgs::PoseStamped vector;
        if (argc==2)
            vector.header.frame_id=argv[1];
        else
            vector.header.frame_id="/base_footprint";
        vector.header.stamp = ros::Time::now();
        //rotarEnCadaEje(&vector);
        //rotarRPY(&vector);
        //rotarYPR(&vector);
        rotarSegunCoefs(&vector);
        //rotarSegunGrados(&vector);
        pub.publish(vector);

        ros::spinOnce ();
        loop_rate.sleep();
    }
}