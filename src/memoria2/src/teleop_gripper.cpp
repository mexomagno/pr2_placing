/**
 * grippernode es un nodo ROS que funciona como un teleop pero para mover grippers
 * Para su funcionamiento utiliza MoveIt
 *
 * Descripción de funcionamiento
 */
#include <ros/ros.h>
#include <string>
#include <iostream>
#include <unistd.h>
#include <termios.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group.h>
#include <tf/transform_listener.h>
using namespace std;
// CONSTANTES
const double PI = 3.1416;
const string ODOM_FRAME = "/odom_combined"; 
// Variables globales
double LINEAR_STEP = 0.05;
double ANGULAR_STEP =  45; // grados
char WHICH_GRIPPER;
string GRIPPER_FRAME;
string WRIST_JOINT;
string FLEX_JOINT;

ros::Publisher current_pose_pub;
ros::Publisher new_pose_pub;
moveit::planning_interface::MoveGroup *group;
moveit::planning_interface::MoveGroup::Plan plan;
double gripper_orientation[] = {0,0,0}; //RPY
double current_roll = 0;
double current_flex = 0;
// MÉTODOS
double gradToRad(double grad){
	return PI*grad/180.0;
}
double radToGrad(double rad){
	return 180.0*rad/PI;
}
void endProgram(int retcode){
	printf("Finaliza programa con código %d\n", retcode);
	exit(retcode);
}
char getch() {
    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0)
            perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0)
            perror("tcsetattr ICANON");
    if (read(0, &buf, 1) < 0)
            perror ("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0)
            perror ("tcsetattr ~ICANON");
    return (buf);
}
void printUsage(){
	printf("Uso:\n");
	printf("\t'q/a': Moverse en X (relativo a odom)\n");
	printf("\t'w/s': Moverse en Y (relativo a odom)\n");
	printf("\t'e/d': Moverse en Z (relativo a odom)\n");
	printf("\t'Q/A': Moverse en X (relativo a gripper)\n");
	printf("\t'W/S': Moverse en Y (relativo a gripper)\n");
	printf("\t'E/D': Moverse en Z (relativo a gripper)\n");
	printf("\t'i/k': Roll (X)\n");
	printf("\t'j/l': Flex (Y)\n");
	printf("\t'1': Cambiar paso lineal (metros)\n");
	printf("\t'2': Cambiar paso angular (grados)\n");
	printf("\t'h': Ayuda (este menú)\n");
	printf("\t'0': Salir\n");
}
geometry_msgs::Quaternion coefsToQuaternionMsg(float a, float b, float c){
    /* 
    Recibe: a, b, c, coeficientes de un plano
    Retorna: Quaternion con la orientación de la normal del plano.
    */
    float vmod=sqrt(a*a+b*b+c*c);
    float pitch = -(1.0*asin(1.0*c/vmod));
    float yaw = (a==0?gradToRad(90):atan(b/a)) + (a>=0?0:gradToRad(180));
    return tf::createQuaternionMsgFromRollPitchYaw(0,pitch,yaw);
}
Eigen::Vector3f transformVector(Eigen::Vector3f vector_in, Eigen::Matrix4f transf){
    Eigen::Vector4f vector_in_4f(vector_in[0], vector_in[1], vector_in[2], 1);
    // Obtener rotación desde matriz de transformación
    Eigen::Matrix4f rot;
    rot = transf;
    rot.col(3) = Eigen::Vector4f(0, 0, 0, 1);
    Eigen::Vector4f vector_out_4f = rot*Eigen::Vector4f(vector_in[0], vector_in[1], vector_in[2], 1);
    Eigen::Vector3f vector_out (vector_out_4f[0], vector_out_4f[1], vector_out_4f[2]);
    return vector_out;
}
geometry_msgs::Point transformPoint(geometry_msgs::Point point_in, Eigen::Matrix4f transf){
    geometry_msgs::Point point_out;
    Eigen::Vector4f point_4f;
    point_4f << point_in.x, point_in.y, point_in.z, 1;
    point_4f = transf*point_4f;
    point_out.x = point_4f[0];
    point_out.y = point_4f[1];
    point_out.z = point_4f[2];
    return point_out;
}
geometry_msgs::Pose transformPose(geometry_msgs::Pose pose_in, Eigen::Matrix4f transf){
    geometry_msgs::Pose pose_out;
    // Obtener rotación desde matriz de transformación
    Eigen::Matrix4f rot;
    rot = transf;
    rot.col(3) = Eigen::Vector4f(0, 0, 0, 1);
    // Crear normal desde quaternion
    tf::Quaternion tf_q;
    tf::quaternionMsgToTF(pose_in.orientation, tf_q);
    tf::Vector3 normal(1, 0, 0);
    normal = tf::quatRotate(tf_q, normal);
    normal.normalize();
    // Rotar normal
    Eigen::Vector3f normal_vector (normal.x(), normal.y(), normal.z());
    Eigen::Vector3f normal_rotated = transformVector(normal_vector, transf);
    normal_rotated.normalize();
    // Obtener quaternion desde normal rotada
    pose_out.orientation = coefsToQuaternionMsg(normal_rotated[0], normal_rotated[1], normal_rotated[2]);
    // Transportar posición
    pose_out.position = transformPoint(pose_in.position, transf);
    return pose_out;
}
Eigen::Vector3f quaternionMsgToVector(geometry_msgs::Quaternion ros_q){
    // Convertir a quaternion de TF
    tf::Quaternion tf_q;
    tf::quaternionMsgToTF(ros_q, tf_q);
    // Convertir quaternion a vector TF
    tf::Vector3 unitv(1,0,0);
    unitv = tf::quatRotate(tf_q, unitv);
    unitv.normalize();
    // Convertir a vector Eigen
    return Eigen::Vector3f(unitv.x(), unitv.y(), unitv.z());
}
Eigen::Matrix4f getTransformation(string frame_ini, string frame_end){
    tf::TransformListener tf_listener;
    tf::StampedTransform stamped_tf;
    ros::Time transform_time = ros::Time(0);
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    double WAIT_TF_TIMEOUT = 1.0;
    if (frame_ini.compare(frame_end) == 0)
        return transformation;
    try{
        ROS_INFO("Esperando transformacion disponible...");
        if (not tf_listener.waitForTransform(frame_end, frame_ini, transform_time, ros::Duration(WAIT_TF_TIMEOUT))){
            ROS_ERROR("Transformacion no pudo ser obtenida antes del timeout (%fs)", WAIT_TF_TIMEOUT);
            return transformation;
        }
        ROS_INFO("Guardando transformacion");
        tf_listener.lookupTransform(frame_end, frame_ini, ros::Time(0), stamped_tf);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("Excepcion al obtener transformacion: %s", ex.what());
        return transformation;
    }
    tf::Matrix3x3 rotation = stamped_tf.getBasis();
    tf::Vector3 translation = stamped_tf.getOrigin();
    Eigen::Matrix3f upleft3x3;
    upleft3x3 << rotation[0][0], rotation[0][1], rotation[0][2],
                 rotation[1][0], rotation[1][1], rotation[1][2],
                 rotation[2][0], rotation[2][1], rotation[2][2];
    transformation.block<3,3>(0,0) = upleft3x3;
    transformation.col(3) = Eigen::Vector4f(translation[0], translation[1], translation[2], 1);
    return transformation;
}

// Funcionamiento movimiento gripper
/**
 * linear mueve el gripper de forma lineal en la dirección del step indicado en el eje indicado.
 * Esta cantidad es ponderada por el LINEAR_STEP.
 * @param  x_step paso en X
 * @param  y_step paso en Y
 * @param  z_step paso en Z
 * @return        true si todo ok, false si no se puede mover a la posición
 */	
bool linear(double x_step, double y_step, double z_step, bool odom){
	ROS_INFO("Desplazamiento lineal según %s", (odom ? "odom" : "frame del gripper"));
	if (not odom){
		// ROS_INFO("Desplazamiento lineal (%f, %f, %f)", x_step*LINEAR_STEP, y_step*LINEAR_STEP, z_step*LINEAR_STEP);
		Eigen::Matrix4f tf = getTransformation(GRIPPER_FRAME, ODOM_FRAME);
		// Crear pose nueva
		geometry_msgs::Pose new_pose_gripper;
		new_pose_gripper.position.x = x_step*LINEAR_STEP;
		new_pose_gripper.position.y = y_step*LINEAR_STEP;
		new_pose_gripper.position.z = z_step*LINEAR_STEP;
		new_pose_gripper.orientation.x = 0;
		new_pose_gripper.orientation.y = 0;
		new_pose_gripper.orientation.z = 0;
		new_pose_gripper.orientation.w = 1;
		// ROS_INFO("Nueva pose respecto a gripper: (%f, %f, %f) (%f, %f, %f, %f)", new_pose_gripper.position.x, new_pose_gripper.position.y, new_pose_gripper.position.z, new_pose_gripper.orientation.x, new_pose_gripper.orientation.y, new_pose_gripper.orientation.z, new_pose_gripper.orientation.w);
		geometry_msgs::Pose new_pose_odom = transformPose(new_pose_gripper, tf);
		// ROS_INFO("Nueva pose respecto a odom: (%f, %f, %f) (%f, %f, %f, %f)", new_pose_odom.position.x, new_pose_odom.position.y, new_pose_odom.position.z, new_pose_odom.orientation.x, new_pose_odom.orientation.y, new_pose_odom.orientation.z, new_pose_odom.orientation.w);
		geometry_msgs::PoseStamped new_pose_stamped;
		new_pose_stamped.header.frame_id = ODOM_FRAME; // OBLIGATORIO
		// Obtener orientación actual de la pose actual del robot
		new_pose_stamped.pose.orientation = group->getCurrentPose().pose.orientation;
		new_pose_stamped.pose = new_pose_odom;
		new_pose_pub.publish(new_pose_stamped);
		new_pose_pub.publish(new_pose_stamped);
		new_pose_pub.publish(new_pose_stamped);
		// ROS_INFO("Pose buscada (%s): (%f, %f, %f) (%f, %f, %f, %f)",new_pose_stamped.header.frame_id.c_str(),new_pose_stamped.pose.position.x,new_pose_stamped.pose.position.y,new_pose_stamped.pose.position.z,new_pose_stamped.pose.orientation.x,new_pose_stamped.pose.orientation.y,new_pose_stamped.pose.orientation.z,new_pose_stamped.pose.orientation.w);
		group->setPoseTarget(new_pose_stamped);
		
	}
	else{
		geometry_msgs::PoseStamped current_pose = group->getCurrentPose();
		geometry_msgs::PoseStamped new_pose;
		new_pose.pose.position.x = current_pose.pose.position.x + x_step*LINEAR_STEP;
		new_pose.pose.position.y = current_pose.pose.position.y + y_step*LINEAR_STEP;
		new_pose.pose.position.z = current_pose.pose.position.z + z_step*LINEAR_STEP;
		new_pose.pose.orientation = current_pose.pose.orientation;
		new_pose.header.frame_id = current_pose.header.frame_id;
		new_pose_pub.publish(new_pose);
		new_pose_pub.publish(new_pose);
		new_pose_pub.publish(new_pose);
		group->setPoseTarget(new_pose);
	}
	bool plan_done = group->plan(plan);
	if (plan_done){
		ROS_INFO("Plan exitoso");
		group->move();
	}
	else{
		ROS_ERROR("Plan falla");
	}
	return plan_done;
}
// bool angular(double roll){
// 	geometry_msgs::PoseStamped current_pose, new_pose;
// 	current_pose = group->getCurrentPose();
// 	new_pose.pose.position = current_pose.pose.position;
// 	new_pose.pose.orientation
// }
bool angular(double roll, double flex){
	// gripper_orientation[0] += gradToRad(roll*ANGULAR_STEP);
	// gripper_orientation[1] += gradToRad(flex*ANGULAR_STEP);
	// geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(gripper_orientation[0], gripper_orientation[1], gripper_orientation[2]);
	// geometry_msgs::PoseStamped new_pose = group->getCurrentPose();
	// new_pose.pose.orientation = quat;
	// new_pose_pub.publish(new_pose);
	// group->setPoseTarget(new_pose);
	double new_roll = current_roll;
	double new_flex = current_flex;
	if (roll != 0){
		ROS_INFO("Haciendo ROLL");
		new_roll += gradToRad(roll*ANGULAR_STEP);
		// current_roll = (current_roll < 0 ? current_roll + 2*PI : (current_roll > 2*PI ? current_roll - 2*PI : current_roll));
		group->setPoseTarget(group->getCurrentPose());
		group->setJointValueTarget(WRIST_JOINT, new_roll);
	}
	else{
		ROS_INFO("Haciendo FLEX");
		new_flex += gradToRad(flex*ANGULAR_STEP);
		// current_flex = (current_flex < 0 ? current_flex + 2*PI : (current_flex > 2*PI ? current_flex - 2*PI : current_flex));
		group->setPoseTarget(group->getCurrentPose());
		group->setJointValueTarget(FLEX_JOINT, new_flex);
	}
	bool plan_done = group->plan(plan);
	if (plan_done){
		ROS_INFO("Plan exitoso");
		group->move();
		if (roll != 0)
			current_roll = new_roll;
		else
			current_flex = new_flex;
	}
	else{
		ROS_ERROR("Plan falla");
	}
	return plan_done;
}
bool angular_old2(double in){
	Eigen::Matrix4f tf_og = getTransformation(ODOM_FRAME, GRIPPER_FRAME);
	geometry_msgs::PoseStamped current_pose_odom = group->getCurrentPose();
	geometry_msgs::PoseStamped current_pose_gripper;
	current_pose_gripper.pose = transformPose(current_pose_odom.pose, tf_og);
	ROS_INFO("old quaternion: (%f, %f, %f, %f)",current_pose_gripper.pose.orientation.x, current_pose_gripper.pose.orientation.y, current_pose_gripper.pose.orientation.z, current_pose_gripper.pose.orientation.w);
	tf::Quaternion q(current_pose_gripper.pose.orientation.x, current_pose_gripper.pose.orientation.y, current_pose_gripper.pose.orientation.z, current_pose_gripper.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	ROS_INFO("RPY segun gripper: %f, %f, %f", radToGrad(roll), radToGrad(pitch), radToGrad(yaw));
	roll += gradToRad(in*ANGULAR_STEP);
	roll = (roll < -2*PI ? roll + 2*PI : (roll > 2*PI ? roll - 2*PI : roll));
	ROS_INFO("Nuevo roll: %f", radToGrad(roll));
	geometry_msgs::Pose new_pose_gripper;
	new_pose_gripper.position.x = new_pose_gripper.position.y = new_pose_gripper.position.z = 0;
	new_pose_gripper.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
	ROS_INFO("new quaternion: (%f, %f, %f, %f)",new_pose_gripper.orientation.x, new_pose_gripper.orientation.y, new_pose_gripper.orientation.z, new_pose_gripper.orientation.w);

	geometry_msgs::PoseStamped new_posestamped_odom;
	Eigen::Matrix4f tf_go = getTransformation(GRIPPER_FRAME, ODOM_FRAME);
	new_posestamped_odom.pose = transformPose(new_pose_gripper, tf_go);
	new_posestamped_odom.header.frame_id = ODOM_FRAME;

	new_pose_pub.publish(new_posestamped_odom);
	group->setPoseTarget(new_posestamped_odom);
	bool plan_done = group->plan(plan);
	if (plan_done){
		ROS_INFO("Plan exitoso");
		group->move();
	}
	else{
		ROS_ERROR("Plan falla");
	}
	return plan_done;
}
bool angular_old(double roll_step, double pitch_step){
	ROS_INFO("Rotacion");
	ROS_INFO("Roll step: %f grad", roll_step * ANGULAR_STEP);
	Eigen::Matrix4f tf = getTransformation(GRIPPER_FRAME, ODOM_FRAME);
	// Eigen::Matrix4f tf_inverse = getTransformation(ODOM_FRAME, GRIPPER_FRAME);
	// Intentar planear para 10cm adelante del gripper
	geometry_msgs::Pose new_pose_gripper;
	new_pose_gripper.position.x = 0;
	new_pose_gripper.position.y = 0;
	new_pose_gripper.position.z = 0;
	new_pose_gripper.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
	// Crear nueva orientación
	// tf::Quaternion tf_q = createQuaternionFromRPY(gradToRad(roll_step * ANGULAR_STEP), gradToRad(pitch_step * ANGULAR_STEP));
	// tf::Quaternion new_rotation = tf::createQuaternionFromRPY(gradToRad(roll_step * ANGULAR_STEP), gradToRad(pitch_step * ANGULAR_STEP), 0);
	// geometry_msgs::Pose old_pose_odom = group->getCurrentPose().pose;
	// geometry_msgs::Pose old_pose_gripper = transformPose(old_pose_odom, tf_inverse);
	// tf::Quaternion old_quaternion_gripper (old_pose_gripper.orientation.x, old_pose_gripper.orientation.y, old_pose_gripper.orientation.z, old_pose_gripper.orientation.w);
	// new_rotation *= old_quaternion_gripper;
	// new_pose_gripper.orientation.x = new_rotation.x();
	// new_pose_gripper.orientation.y = new_rotation.y();
	// new_pose_gripper.orientation.z = new_rotation.z();
	// new_pose_gripper.orientation.w = new_rotation.w();
	// new_pose_gripper.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,gradToRad(roll_step*ANGULAR_STEP), gradToRad(pitch_step*ANGULAR_STEP));
	geometry_msgs::Pose new_pose_odom = transformPose(new_pose_gripper, tf);
	geometry_msgs::PoseStamped new_pose_stamped;
	new_pose_stamped.header.frame_id = ODOM_FRAME; // OBLIGATORIO
	new_pose_stamped.pose = new_pose_odom;
	new_pose_pub.publish(new_pose_stamped);
	new_pose_pub.publish(new_pose_stamped);
	new_pose_pub.publish(new_pose_stamped);
	// ROS_INFO("Pose buscada (%s): (%f, %f, %f) (%f, %f, %f, %f)",new_pose_stamped.header.frame_id.c_str(),new_pose_stamped.pose.position.x,new_pose_stamped.pose.position.y,new_pose_stamped.pose.position.z,new_pose_stamped.pose.orientation.x,new_pose_stamped.pose.orientation.y,new_pose_stamped.pose.orientation.z,new_pose_stamped.pose.orientation.w);
	group->setPoseTarget(new_pose_stamped);
	bool plan_done = group->plan(plan);
	if (plan_done){
		ROS_INFO("Plan exitoso");
		group->move();
	}
	else{
		ROS_ERROR("Plan falla");
	}
	return plan_done;
}

bool processInput(char input){
	bool success;
	string new_step;
	switch (input){
		//  Según odom
		case 'a':
			success = linear(-1, 0, 0, true);
			break;
		case 'q':
			success = linear(1, 0, 0, true);
			break;
		case 's':
			success = linear(0, -1, 0, true);
			break;
		case 'w':
			success = linear(0, 1, 0, true);
			break;
		case 'd':
			success = linear(0, 0, -1, true);
			break;
		case 'e':
			success = linear(0, 0, 1, true);
			break;
		// Según gripper_frame
		case 'A':
			success = linear(-1, 0, 0, false);
			break;
		case 'Q':
			success = linear(1, 0, 0, false);
			break;
		case 'S':
			success = linear(0, -1, 0, false);
			break;
		case 'W':
			success = linear(0, 1, 0, false);
			break;
		case 'D':
			success = linear(0, 0, -1, false);
			break;
		case 'E':
			success = linear(0, 0, 1, false);
			break;
		// Angular
		case 'k':
			success = angular(-1, 0);
			break;
		case 'i':
			success = angular(1, 0);
			break;
		case 'j':
			success = angular(0, -1);
			break;
		case 'l':
			success = angular(0, 1);
			break;
		// Otros
		case '1':
			printf("Ingrese nuevo valor de paso lineal (actual: %fm):  ", LINEAR_STEP);
			cin >> new_step;
			LINEAR_STEP = atof(new_step.c_str());
			success = true;
			break;
		case '2':
			printf("Ingrese nuevo valor de paso angular (actual: %fm):  ", ANGULAR_STEP);
			cin >> new_step;
			ANGULAR_STEP = atof(new_step.c_str());
			success = true;
			break;
		case 'h':
			printUsage();
			success = true;
			break;
		default:
			return false;
	}
	if (not success)
		ROS_WARN("Oops! No se pudo desplazar el gripper!");
	return true;
}
void showCurrentPose(){
	geometry_msgs::PoseStamped pose = group->getCurrentPose();
	current_pose_pub.publish(pose);
	current_pose_pub.publish(pose);
	current_pose_pub.publish(pose);
	ROS_INFO("Pose actual (%s): (%f, %f, %f) (%f, %f, %f, %f)", pose.header.frame_id.c_str(), pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
}



/**
 * Al inicializar el programa, se especifica qué gripper se desea manipular, y opcionalmente
 * qué step se debe usar para desplazamiento lineal. Finalmente, se puede entregar como opción el step angular.
 * @param  argc 
 * @param  argv [l|r] [linear] [angular]
 * @return      estado de salida del programa
 */
int main(int argc, char **argv){
	// validación de inputs
	if (argc < 2){
		printf("Debe ingresar gripper a usar [l|r]\n");
		exit(0);
	}
	WHICH_GRIPPER = argv[1][0];
	if (WHICH_GRIPPER != 'l' and WHICH_GRIPPER != 'r'){
		printf("Debe ingresar l o r\n");
		exit(0);
	}
	if (argc >= 3){
		LINEAR_STEP = (double)atof(argv[2]);
		if (argc >=4){
			ANGULAR_STEP = (double)atof(argv[3]);
		}
	}
	printUsage();
	printf("Gripper: '%s', linear: %f, angular: %f\n", (WHICH_GRIPPER == 'l' ? "izquierdo" : "derecho"), LINEAR_STEP, ANGULAR_STEP);
	GRIPPER_FRAME = (WHICH_GRIPPER == 'l' ? "/l_wrist_flex_link" : "/r_wrist_flex_link");
	WRIST_JOINT = (WHICH_GRIPPER == 'l' ? "l_wrist_roll_joint" : "r_wrist_roll_joint");
	FLEX_JOINT = (WHICH_GRIPPER == 'l' ? "l_wrist_flex_joint" : "r_wrist_flex_joint");
	// Iniciar nodo ROS
	ros::init(argc, argv, "gripper_node");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();
	current_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose1", 1);
	new_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose2", 1);
	ROS_INFO("Inicializando brazo %s...", (WHICH_GRIPPER == 'l' ? "izquierdo" : "derecho"));
	ros::Duration(1).sleep();
	// Iniciar brazo correspondiente
	moveit::planning_interface::MoveGroup aux_group((WHICH_GRIPPER == 'l' ? "left_arm" : "right_arm"));
	group = &aux_group;
	group->setPoseReferenceFrame(ODOM_FRAME);
	// Mostrar pose actual
	showCurrentPose();
	printf("Esperando input...: \n");
	char input = getch();	
	while (input != '0'){
		printf("Se apreto la '%c'. Procesando comando...\n", input);
		if (not processInput(input)){
			printf("Comando desconocido\n");
		}
		else{
			showCurrentPose();
		}
		printf("Esperando input...: \n");
		input = getch();
	}
	printf("Programa termina\n");
	return 0;
	// ros::init()
}