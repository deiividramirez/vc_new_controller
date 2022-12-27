/*
Intel (Zapopan, Jal), Robotics Lab (CIMAT, Gto), Patricia Tavares & Gerardo Rodriguez.
November 20th, 2017
This ROS code is used to connect rotors_simulator hummingbird's data and
move it to a desired pose.
*/

/******************************************************* ROS libraries*/
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mav_msgs/conversions.h>
#include <geometry_msgs/PointStamped.h>

/****************************************************** c++ libraries */
#include <random>
#include <vector>
#include <string>
#include <stdio.h>
#include <fstream>
#include <stdlib.h>
#include <iostream>
#include <opencv2/core.hpp>
// # Roll pitch yawrate thrust controller parameters
// attitude_gain: {x: 2, y: 3, z: 0.035}
// angular_rate_gain: {x: 0.4, y: 0.52, z: 0.025}
using namespace std;

void positionCallback(const geometry_msgs::PointStamped::ConstPtr &msg);
void writeFile(vector<float> &vec, const string &name)
{
	ofstream myfile;
	myfile.open(name);
	for (int i = 0; i < vec.size(); i++)
		myfile << vec[i] << endl;
	myfile.close();
}

// void writeMatrix(Mat &mat, const string &name);
geometry_msgs::PointStamped pos_msg;


/* Main function */
int main(int argc, char **argv)
{

	/***************************************************************************************** INIT */
	ros::init(argc, argv, "placing");
	ros::NodeHandle nh;

	// if (argc < 2)
	// return 0; // if no name was given, nothing to do here-
	string slash("/");
	string name(argv[1]);
	// string name("iris");
	string publish("/command/trajectory");
	// string subscribe("/ground_truth/position");
	string subscribe("/ground_truth/position");
	string topic_pub = slash + name + publish;
	string topic_sub = slash + name + subscribe;

	std::cout << "Publishing to: " << topic_pub << std::endl;
	std::cout << "Subscribing to: " << topic_sub << std::endl;

	/************************************************************* CREATING PUBLISHER AND SUBSCRIBER*/
	ros::Publisher pos_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(topic_pub, 1);
	ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PointStamped>(topic_sub, 1, positionCallback);
	ros::Rate rate(50);

	/************************************************************* DEFINING THE POSE */
	std::random_device rd;														 // obtain a random number from hardware
	std::mt19937 gen(rd());														 // seed the generator
	std::uniform_real_distribution<> disXY(-3.0, 3.0); // define the range for X and Y
	std::uniform_real_distribution<> disZ(0.0, 3.0);	 // define the range for Z
	std::uniform_real_distribution<> disYaw(-2, 2);		 // define the range yaw

	float X, Y, Z, Yaw, error;
	X = argc > 2 ? atof(argv[2]) : disXY(gen);		// if x coordinate has been given
	Y = argc > 3 ? atof(argv[3]) : disXY(gen);		// if y coordinate has been given
	Z = argc > 4 ? atof(argv[4]) : disZ(gen);			// if z coordinate has been given
	Yaw = argc > 5 ? atof(argv[5]) : disYaw(gen); // if yaw has been given
	
	vector<float> error_vec;
	vector<float> x;
	vector<float> y;
	vector<float> z;
	vector<float> yaw;
	vector<float> params = {X, Y, Z, Yaw};

	Eigen::VectorXd position;
	position.resize(3);
	// position(0) = X;
	position(0) = X-0.124;
	position(1) = Y;
	position(2) = Z;

	int conteo = 0;

	/******************************************************************************* BUCLE START*/
	while (ros::ok())
	{
		// get a msg
		ros::spinOnce();
		// create message for the pose
		trajectory_msgs::MultiDOFJointTrajectory msg;

		// prepare msg
		msg.header.stamp = ros::Time::now();
		mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(position, Yaw, &msg);
		// publish
		pos_pub.publish(msg);
		// verify if it's over
		error = (pos_msg.point.x - X) * (pos_msg.point.x - X) + (pos_msg.point.y - Y) * (pos_msg.point.y - Y) + (pos_msg.point.z - Z) * (pos_msg.point.z - Z);

		std::cout << "Error: " << error << std::endl;
		std::cout << "=> X: " << pos_msg.point.x << " Y: " << pos_msg.point.y << " Z: " << pos_msg.point.z << std::endl;
		std::cout << ">> X: " << X << " Y: " << Y << " Z: " << Z << std::endl
							<< std::endl;

		error_vec.push_back(error);
		x.push_back(pos_msg.point.x);
		y.push_back(pos_msg.point.y);
		z.push_back(pos_msg.point.z);
		yaw.push_back(Yaw);

		// cout << pos_msg.orientation.x << endl;

		if (error < 0.01 || conteo++ > 250)
			break;

		rate.sleep();
	}

	std::cout << "Pose desired of the drone ==> " << endl
						<< "X: " << X << ", Y: " << Y << ", Z: " << Z << ", Yaw: " << Yaw << endl;

	std::cout << "Real pose of the drone <<== " << endl
						<< "X: " << pos_msg.point.x << ", Y: " << pos_msg.point.y << ", Z: " << pos_msg.point.z << ", Yaw: " << Yaw << endl;

	std::cout << "Error: " << error << std::endl;

	writeFile(error_vec, "error.txt");
	writeFile(x, "x.txt");
	writeFile(y, "y.txt");
	writeFile(z, "z.txt");
	writeFile(yaw, "yaw.txt");
	writeFile(params, "params.txt");

	return 0;
}

/*
	function: positionCallback
	description: gets the position of the drone and assigns it to the variable point_msg.
	params: ptr to msg.
*/
void positionCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
	pos_msg.point.x = msg->point.x;
	pos_msg.point.y = msg->point.y;
	pos_msg.point.z = msg->point.z;

	std::cout << "P: " << msg->point.x << " - " << msg->point.y << " - " << msg->point.z << std::endl;
}


// void writeMatrix(cv::Mat &mat, const string &name)
// {
// 	ofstream myfile;
// 	myfile.open(name);
// 	for (int i = 0; i < mat.rows; i++)
// 	{
// 		for (int j = 0; j < mat.cols; j++)
// 		{
// 			myfile << mat.at<float>(i, j) << " ";
// 		}
// 		myfile << endl;
// 	}
// 	myfile.close();
// }