#include "../RobotBaseDriver.h"
// CONSTANTES
const float WAIT_TF_TIMEOUT = 1.0;
const float LOOP_FREQ = 10.0;
const float TWIST_VELOCITY = 2.5;
const float ANGULAR_VELOCITY = 3;
const float DISTANCE_CORRECTION = 0.18;
const float DISTANCE_PRE_TURN = 0.15;
const float ANGLE_CORRECTION = 0.09;
bool store_odom = false;
tf::Vector3 last_position(0,0,0);
tf::Quaternion last_orientation(0,0,0,1);
ros::Subscriber odom_sub_;
ros::Publisher base_cmd_pub_;

// PUBLIC
// Métodos
void odomCallback(const nav_msgs::Odometry::ConstPtr& odom){
	if (not store_odom)
        return;
    // ROS_DEBUG("RobotBaseDriver: Recibida odometría: (%f,%f,%f) (%f,%f,%f,%f)",odom->pose.pose.position.x,odom->pose.pose.position.y,odom->pose.pose.position.z,odom->pose.pose.orientation.x,odom->pose.pose.orientation.y,odom->pose.pose.orientation.z,odom->pose.pose.orientation.w);
    // Actualizar última posición
    last_position.setX(odom->pose.pose.position.x);
    last_position.setY(odom->pose.pose.position.y);
    last_position.setZ(odom->pose.pose.position.z);

    // Actualizar última orientación
    last_orientation.setX(odom->pose.pose.orientation.x);
    last_orientation.setY(odom->pose.pose.orientation.y);
    last_orientation.setZ(odom->pose.pose.orientation.z);
    last_orientation.setW(odom->pose.pose.orientation.w);
}
// Constructor y Destructor
RobotBaseDriver::RobotBaseDriver(){
	// ros::init("RobotBaseDriver_node");
	ROS_DEBUG("RobotBaseDriver: Creando nodeHandle");
	nh_ = new ros::NodeHandle;
	// Subscribir a mensajes de odometría
	ROS_DEBUG("RobotBaseDriver: Creando subscriber odom");
	odom_sub_ = nh_->subscribe<nav_msgs::Odometry>(Util::ODOM_TOPIC, 1, odomCallback);
	// Publicar mensajes para base cmd
	ROS_DEBUG("RobotBaseDriver: Creando publisher twist");
	base_cmd_pub_ = nh_->advertise<geometry_msgs::Twist>(Util::BASE_CONTROLLER_TOPIC, 1);
}
RobotBaseDriver::~RobotBaseDriver(){
	ROS_DEBUG("RobotBaseDriver: Destruyendo nodeHandle");
	delete nh_;
}
/*bool RobotBaseDriver::goToPose(const string frame_id, geometry_msgs::PoseStamped pose){
	// Ver si es el mismo frame, sino, transformarlo
}*/
bool RobotBaseDriver::goToPose(geometry_msgs::PoseStamped pose_goal){
	/**
	 * 1) Mover base hasta un poco antes de llegar al punto final
	 * 2) Girar la base a orientación requerida
	 * 3) Mover un poco más (lo que falta)
	 *
	 * En el proceso aplicar correcciones. Por naturaleza, turn y travel se pasan de largo una cierta cantidad.
	 */
	if (Util::BASE_FRAME.compare(pose_goal.header.frame_id) != 0){
		ROS_ERROR("Se ingresó pose final con un frame no válido: '%s'", pose_goal.header.frame_id.c_str());
		return false;
	}
	geometry_msgs::Pose robot_pose = pose_goal.pose;
	ROS_DEBUG("RobotBaseDriver: Pose buscada: (%f, %f, %f)", robot_pose.position.x, robot_pose.position.y, robot_pose.position.z);
	// 1) DESPLAZAR BASE
	if (robot_pose.position.x != 0 or robot_pose.position.y != 0){
		// Calcular distancia desde robot a nueva pose
		float distance = sqrt(robot_pose.position.x*robot_pose.position.x + robot_pose.position.y*robot_pose.position.y);
		ROS_DEBUG("RobotBaseDriver: Trasladando base");
		tf::Vector3 xend( robot_pose.position.x, robot_pose.position.y, robot_pose.position.z);
		tf::Vector3 anglestart (1,0,0);
		float angle = anglestart.angle(xend)*(robot_pose.position.y < 0 ? -1 : 1); // Corrige problema de ángulo siempre positivo
		ROS_DEBUG("RobotBaseDriver: Angulo dirección desplazamiento: %f", Util::toGrad(angle));
		// Corrección de distancia
		distance = distance - DISTANCE_CORRECTION - DISTANCE_PRE_TURN;
		if (not travel(distance, angle))
			return false;
	}
	else{
		ROS_DEBUG("RobotBaseDriver: Recibida pose en mismo lugar");
	}
	// 2) ROTAR BASE
	ROS_DEBUG("RobotBaseDriver: Rotando base");
	geometry_msgs::Quaternion q = robot_pose.orientation;
	float angle = atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z));
	// Corrección de ángulo
	//angle = (angle > 0 ? angle - ANGLE_CORRECTION : angle + ANGLE_CORRECTION);
	if (not turn(angle))
		return false;
	// 3) MOVER ULTIMO POCO
	if (robot_pose.position.x != 0 or robot_pose.position.y != 0){
		ROS_DEBUG("RobotBaseDriver: Terminando de acercarse");
		if (not travel(DISTANCE_PRE_TURN, 0))
			return false;
	}
	ROS_DEBUG("RobotBaseDriver: Fin goToPose");
	return true;
}
// PRIVATE
bool RobotBaseDriver::turn(float angle){
	ROS_DEBUG("RobotBaseDriver: Girar %f grados", Util::toGrad(angle));
	ros::Rate rate(LOOP_FREQ);
	store_odom = true;
	rate.sleep();
	// Actualizar última posición
	ros::spinOnce();
	tf::Quaternion start_orientation = last_orientation;
	geometry_msgs::Twist base_cmd;
	base_cmd.angular.z = (angle < 0 ? -1 : 1)*ANGULAR_VELOCITY;
	bool done = false;
	float real_angle = 0;
	float traveled_angle;
	while (!done and ros::ok()){
		base_cmd_pub_.publish(base_cmd);
		rate.sleep();
		// Actualizar posicíón odométrica
		ros::spinOnce();
		traveled_angle = 2*start_orientation.angle(last_orientation);
		if (traveled_angle > (angle < 0 ? -angle : angle)) done = true;
	}
	if (done){
		ROS_DEBUG("Giro real fue: %f grados", Util::toGrad(traveled_angle));
		return true;
	}
	else{
		ROS_ERROR("RobotBaseDriver: What wea dis error, i am fallan2");
		return false;
	}
}
bool RobotBaseDriver::travel(float distance, float angle){
	ROS_DEBUG("RobotBaseDriver: Desplazar distancia: %f, angulo: %f", distance, Util::toGrad(angle));
	ros::Rate rate(LOOP_FREQ);
	store_odom = true;
	rate.sleep();
	ros::spinOnce();
	tf::Vector3 start_position = last_position;
	// mensaje a enviar a la base
	geometry_msgs::Twist base_cmd;
	// Calcular comando a enviar
	tf::Vector3 itongo((distance < 0 ? -1 : 1)*TWIST_VELOCITY, 0, 0);
	tf::Vector3 zaxis(0,0,1);
	// Rotar vector unitario
	itongo = itongo.rotate(zaxis, angle);
	// Guardar en twist
	base_cmd.linear.x = itongo.x();
	base_cmd.linear.y = itongo.y();
	base_cmd.linear.z = 0; // No queremos flotar ni enterrarnos en el suelo
	bool done = false;
	float real_distance = 0;
	float traveled_dist;
	while (!done and ros::ok()){
		// Enviar comando (ponderado)
		base_cmd_pub_.publish(base_cmd);
		rate.sleep();
		// Actualizar posición odométrica
		ros::spinOnce();
		traveled_dist = start_position.distance(last_position);
		ROS_DEBUG("RobotBaseDriver: Distancia navegada = %fm", traveled_dist);
		if (traveled_dist > (distance < 0 ? -distance : distance)) done = true;
	}
	store_odom = false;
	if (done){
		ROS_DEBUG("RobotBaseDriver: Desplazamiento real fue: %fm", traveled_dist);
		return true;
	}
	else{
		ROS_ERROR("RobotBaseDriver: What wea dis error, i am fallan2.");
		return false;
	}
}

/**
 *	TODO:
 *		- Evaluar posibilidad de desacelerar al acercarse a posición final (para turn y travel).
 *  	- Evaluar generalidad de goToPose. Está pensado para el placing, pero para otras aplicaciones se comporta de forma poco práctica.
 */