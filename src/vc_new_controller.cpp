/*
Intel (Zapopan, Jal), Robotics Lab (CIMAT, Gto), Patricia Tavares & Gerardo Rodriguez.
November 20th, 2017
This ROS code is used to connect rotors_simulator hummingbird's camera
and process the images to obtain the homography.
*/

#include "vc_new_controller.h"
#include <opencv2/video/tracking.hpp>

/* Declaring namespaces */
using namespace cv;
using namespace std;

/* Declaring callbacks and other functions*/
void imageCallback(const sensor_msgs::Image::ConstPtr &msg);
void poseCallback(const geometry_msgs::Pose::ConstPtr &msg);
void writeFile(vector<float> &vec, const string &name);

/* Declaring objects to receive messages */
sensor_msgs::ImagePtr image_msg;

/* Workspace definition from CMake */
string workspace = WORKSPACE;

// Visual control state
vc_state state;

// Result of the matching operation
vc_homograpy_matching_result matching_result;

//  Selector de control
int contr_sel = 2;

// Conteo de imágenes
int contIMG = 0;

Mat img_old, img_points;

Mat Ordenamiento(Mat puntos, int opc)
{
	bool check;
	Mat orden = Mat::zeros(1, puntos.rows, CV_32SC1), p2 = puntos.clone();
	for (int i = 0; i < p2.rows; i++)
	{
		orden.at<int>(0, i) = i;
	}

	cout << "Orden: " << orden << endl;
	for (int i = 0; i < p2.rows; i++)
	{
		for (int j = 0; j < p2.rows - 1; j++)
		{
			if (opc == 1)
			{
				check = (p2.at<double>(i, 0) + p2.at<double>(i, 1) < p2.at<double>(j, 0) + p2.at<double>(j, 1));
			}
			else
			{
				check = (p2.at<double>(i, 0) - p2.at<double>(i, 1) < p2.at<double>(j, 0) - p2.at<double>(j, 1));
			}
			if (check)
			{
				double temp = p2.at<double>(i, 0);
				p2.at<double>(i, 0) = p2.at<double>(j, 0);
				p2.at<double>(j, 0) = temp;

				temp = p2.at<double>(i, 1);
				p2.at<double>(i, 1) = p2.at<double>(j, 1);
				p2.at<double>(j, 1) = temp;

				int temp2 = orden.at<int>(0, i);
				orden.at<int>(0, i) = orden.at<int>(0, j);
				orden.at<int>(0, j) = temp2;
			}
		}
	}

	cout << "Orden: " << orden << endl;
	return orden;
}

/* Main function */
int main(int argc, char **argv)
{
	/***************************************************************************************** INIT */
	ros::init(argc, argv, "vc_new_controller");
	ros::NodeHandle nh;
	state.load(nh);

	image_transport::ImageTransport it(nh);
	image_transport::Subscriber image_sub;
	/************************************************************* CREATING PUBLISHER AND SUBSCRIBER */
	if (state.params.camara == 0)
	{
		cout << "[INFO] Using hummingbird bottom camera" << endl;
		image_sub = it.subscribe("/hummingbird/camera_nadir/image_raw", 1, imageCallback);
	}
	else if (state.params.camara == 1)
	{
		cout << "[INFO] Using iris front camera" << endl;
		image_sub = it.subscribe("/iris/camera_front_camera/image_raw", 1, imageCallback);
	}
	else if (state.params.camara == 2)
	{
		cout << "[INFO] Using iris bottom camera" << endl;
		image_sub = it.subscribe("/iris/camera_under_camera/image_raw", 1, imageCallback);
	}
	else
	{
		cout << "[ERROR] There is no camera with that number" << endl;
		return -1;
	}

	image_transport::Publisher image_pub = it.advertise("matching", 1);
	ros::Rate rate(30);
	// ros::Rate rate(120);

	/************************************************************************** OPENING DESIRED IMAGE */
	string image_dir = "/src/vc_new_controller/src/desired.jpg";
	state.desired_configuration.img = imread(workspace + image_dir, IMREAD_COLOR);
	if (state.desired_configuration.img.empty())
	{
		cerr << "[ERR] Could not open or find the reference image" << std::endl;
		return -1;
	}
	else
	{
		cout << "[INFO] Reference image loaded" << std::endl;
	}

	Ptr<ORB> orb = ORB::create(state.params.nfeatures,
														 state.params.scaleFactor,
														 state.params.nlevels,
														 state.params.edgeThreshold,
														 state.params.firstLevel,
														 state.params.WTA_K,
														 state.params.scoreType,
														 state.params.patchSize,
														 state.params.fastThreshold);

	orb->detect(state.desired_configuration.img,
							state.desired_configuration.kp);
	orb->compute(state.desired_configuration.img,
							 state.desired_configuration.kp,
							 state.desired_configuration.descriptors);

	/******************************************************************************* MOVING TO A POSE */
	ros::Publisher pos_pub;
	ros::Subscriber pos_sub;

	if (state.params.camara == 0)
	{
		cout << "[INFO] Hummingbird trajectory and pose" << endl
				 << endl;
		pos_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/hummingbird/command/trajectory", 1);
		pos_sub = nh.subscribe<geometry_msgs::Pose>("/hummingbird/ground_truth/pose", 1, poseCallback);
	}
	else if (state.params.camara == 1 || state.params.camara == 2)
	{
		cout << "[INFO] Iris trajectory and pose" << endl
				 << endl;
		pos_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/iris/command/trajectory", 1);
		pos_sub = nh.subscribe<geometry_msgs::Pose>("/iris/ground_truth/pose", 1, poseCallback);
	}

	/**************************************************************************** data for graphics */
	vector<float> vel_x;
	vector<float> vel_y;
	vector<float> vel_z;
	vector<float> vel_yaw;
	vector<float> errors;
	vector<float> time;

	// Create message for the pose
	trajectory_msgs::MultiDOFJointTrajectory msg;
	string file_folder = "/src/vc_new_controller/src/data/";

	/******************************************************************************* CYCLE START*/
	while (ros::ok())
	{
		// get a msg
		ros::spinOnce();

		if (!state.initialized)
		{
			rate.sleep();
			continue;
		} // if we havent get the new pose

		// save data
		time.push_back(state.t);
		errors.push_back((float)matching_result.mean_feature_error);
		vel_x.push_back(state.Vx);
		vel_y.push_back(state.Vy);
		vel_z.push_back(state.Vz);
		vel_yaw.push_back(state.Vyaw);

		if (matching_result.mean_feature_error < state.params.feature_threshold)
		{
			cout << endl
					 << "[INFO] Target reached" << endl
					 << endl;
			waitKey(0);
			break;
		}

		// Publish image of the matching
		cout << endl
				 << "[INFO] Publishing image" << endl;
		image_pub.publish(image_msg);

		// Update state with the current control
		auto new_pose = state.update();

		// Prepare msg
		msg.header.stamp = ros::Time::now();
		mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(new_pose.first, new_pose.second, &msg);

		// Publish msg
		pos_pub.publish(msg);
		rate.sleep();
	}

	// save data
	writeFile(errors, workspace + file_folder + "errors.txt");
	writeFile(time, workspace + file_folder + "time.txt");
	writeFile(vel_x, workspace + file_folder + "Vx.txt");
	writeFile(vel_y, workspace + file_folder + "Vy.txt");
	writeFile(vel_z, workspace + file_folder + "Vz.txt");
	writeFile(vel_yaw, workspace + file_folder + "Vyaw.txt");

	return 0;
}

/*
	function: imageCallback
	description: uses the msg image and converts it to and opencv image to obtain the kp and
	descriptors, it is done if the drone has moved to the defined position. After that the resulting image and velocities are published.
	params:
		msg: ptr to the msg image.
*/

void imageCallback(const sensor_msgs::Image::ConstPtr &msg)
{
	cout << "[INFO] ImageCallback function" << endl;
	try
	{
		Mat actual = cv_bridge::toCvShare(msg, "bgr8")->image;
		cout << "[INFO] Image received" << endl;

		string saveIMG = "/src/vc_new_controller/src/data/img/" + to_string(contIMG++) + ".jpg";
		imwrite(workspace + saveIMG, actual);
		cout << "[INFO] << Image saved >>" << saveIMG << endl;

		// Mat actual_gray;
		// cvtColor(actual, actual_gray, COLOR_BGR2GRAY);

		// Mat corner;
		// goodFeaturesToTrack(actual_gray, corner, 100, 0.01, 10);
		// // print corner and size
		// cout << "[INFO] Corner size: " << corner.size() << endl;
		// cout << "[INFO] Corner: " << corner << endl;
		// cout << corner.at<float>(0, 0) << endl;
		// cout << corner.at<float>(0, 1) << endl;

		if (contIMG == 1)
		{

			cout << endl
					 << "[INFO] Detecting keypoints" << endl;

			if (compute_descriptors(actual, state.params, state.desired_configuration, matching_result) < 0)
			{
				cout << "[ERROR] Error en compute_descriptors" << endl;
				return;
			}

			Mat esquinas1 = Ordenamiento(matching_result.p2, 1);
			Mat esquinas2 = Ordenamiento(matching_result.p2, 2);

			int start = (int)rand() % 10 + 5;
			int end = matching_result.p2.rows - 1 - start;

			img_points = Mat(4, 2, CV_32F);
			img_points.at<Point2f>(0, 0) = Point2f(matching_result.p2.at<double>(esquinas1.at<int>(0, start), 0), matching_result.p2.at<double>(esquinas1.at<int>(0, start), 1));
			img_points.at<Point2f>(1, 0) = Point2f(matching_result.p2.at<double>(esquinas1.at<int>(0, end), 0), matching_result.p2.at<double>(esquinas1.at<int>(0, end), 1));
			img_points.at<Point2f>(2, 0) = Point2f(matching_result.p2.at<double>(esquinas2.at<int>(0, start), 0), matching_result.p2.at<double>(esquinas2.at<int>(0, start), 1));
			img_points.at<Point2f>(3, 0) = Point2f(matching_result.p2.at<double>(esquinas2.at<int>(0, end), 0), matching_result.p2.at<double>(esquinas2.at<int>(0, end), 1));

			Mat temporal = Mat::zeros(4, 2, CV_32F);
			temporal.at<Point2f>(0, 0) = Point2f(matching_result.p1.at<double>(esquinas1.at<int>(0, start), 0), matching_result.p1.at<double>(esquinas1.at<int>(0, start), 1));
			temporal.at<Point2f>(1, 0) = Point2f(matching_result.p1.at<double>(esquinas1.at<int>(0, end), 0), matching_result.p1.at<double>(esquinas1.at<int>(0, end), 1));
			temporal.at<Point2f>(2, 0) = Point2f(matching_result.p1.at<double>(esquinas2.at<int>(0, start), 0), matching_result.p1.at<double>(esquinas2.at<int>(0, start), 1));
			temporal.at<Point2f>(3, 0) = Point2f(matching_result.p1.at<double>(esquinas2.at<int>(0, end), 0), matching_result.p1.at<double>(esquinas2.at<int>(0, end), 1));
			cout << "[INFO] temporal: " << temporal << endl;
			temporal.convertTo(matching_result.p1, CV_64F);

			temporal = Mat::zeros(4, 2, CV_32F);
			temporal.at<Point2f>(0, 0) = Point2f(matching_result.p2.at<double>(esquinas1.at<int>(0, start), 0), matching_result.p2.at<double>(esquinas1.at<int>(0, start), 1));
			temporal.at<Point2f>(1, 0) = Point2f(matching_result.p2.at<double>(esquinas1.at<int>(0, end), 0), matching_result.p2.at<double>(esquinas1.at<int>(0, end), 1));
			temporal.at<Point2f>(2, 0) = Point2f(matching_result.p2.at<double>(esquinas2.at<int>(0, start), 0), matching_result.p2.at<double>(esquinas2.at<int>(0, start), 1));
			temporal.at<Point2f>(3, 0) = Point2f(matching_result.p2.at<double>(esquinas2.at<int>(0, end), 0), matching_result.p2.at<double>(esquinas2.at<int>(0, end), 1));
			cout << "[INFO] temporal: " << temporal << endl;
			temporal.convertTo(matching_result.p2, CV_64F);

			cout << "[INFO] img_points: " << img_points << endl;
			cout << "[INFO] matching_result.p1: " << matching_result.p1 << endl;
			cout << "[INFO] matching_result.p2: " << matching_result.p2 << endl;
			// exit(-1);

			// img_points = Mat(matching_result.p2.size(), CV_32F);
			// for (int i = 0; i < matching_result.p2.rows; i++)
			// {
			// 	img_points.at<Point2f>(i, 0) = Point2f(matching_result.p2.at<double>(i, 0), matching_result.p2.at<double>(i, 1));
			// }

			img_old = actual;
		}
		else
		{
			Mat img_new = actual;

			// AQUI ES DONDE SE EJECUTA TODOOOOO
			if (controllers[contr_sel](actual, state, matching_result) < 0)
			{
				cout << "[ERROR] Controller failed" << endl;
				return;
			}
			else
			{
				cout << "[INFO] Controller part has been executed" << endl;
			}

			// KLT tracker for the next iteration
			Mat new_points, status, error;
			Mat img_old_gray, img_new_gray;
			cvtColor(img_old, img_old_gray, COLOR_BGR2GRAY);
			cvtColor(img_new, img_new_gray, COLOR_BGR2GRAY);
			// matching_result.p2 = corner.clone();
			calcOpticalFlowPyrLK(img_old_gray, img_new_gray, img_points, new_points, status, error);

			// Clean matching_result.p2 and paste new_points
			// matching_result.p2 = Mat::zeros(new_points.size(), new_points.type());
			// new_points.copyTo(matching_result.p2);

			// matching_result.p2 = new_points.clone();

			// print the results
			// imshow("Matching", matching_result.img_matches);
			// waitKey(1);
			Mat desired_temp = state.desired_configuration.img.clone();
			for (int i = 0; i < matching_result.p1.rows; i++)
			{
				// cout << i << ": " << matching_result.p1.at<double>(i, 0) << " " << matching_result.p1.at<double>(i, 1) << endl;
				circle(desired_temp, Point2f(matching_result.p1.at<double>(i, 0), matching_result.p1.at<double>(i, 1)), 3, Scalar(0, 0, 255), -1);
				// cout << i << ": " << img_points.at<Point2f>(i, 0) << endl;
				// circle(img_old, img_points.at<Point2f>(i, 0), 3, Scalar(0, 0, 255), -1);
				// circle(img_old, new_points.at<Point2f>(i, 0), 3, Scalar(255, 0, 0), -1);
			}
			for (int i = 0; i < new_points.rows; i++)
			{
				// cout << i << ": " << new_points.at<Point2f>(i, 0) << endl;
				circle(desired_temp, new_points.at<Point2f>(i, 0), 3, Scalar(255, 0, 0), -1);
				circle(actual, new_points.at<Point2f>(i, 0), 3, Scalar(255, 0, 0), -1);
				circle(actual, img_points.at<Point2f>(i, 0), 3, Scalar(0, 0, 255), -1);
			}

			// cout << "[INFO] matching_result.p1: " << matching_result.p1.size() << endl;
			// cout << "[INFO] matching_result.p2: " << matching_result.p2.size() << endl;
			// exit(-1);

			// imshow("Image old", img_old);
			imshow("Image", actual);
			imshow("Desired", desired_temp);
			waitKey(1);

			// cout << "New points size: " << new_points.size() << endl;
			// cout << "New points: " << new_points << endl;
			// cout << "Status size: " << status.size() << endl;
			// cout << "Status: " << status.t() << endl;

			// reconvert from CV_32F to CV_64F
			// Mat img_points_temp = Mat(new_points.size(), CV_64F);
			// for (int i = 0; i < new_points.rows; i++)
			// {
			// 	img_points_temp.at<Point2d>(i, 0) = Point2d(new_points.at<Point2f>(i, 0).x, new_points.at<Point2f>(i, 0).y);
			// }

			new_points.convertTo(matching_result.p2, CV_64F);
			// matching_result.p2 = img_points_temp.clone();
			img_points = new_points.clone();
			actual.copyTo(img_old);

			// Mat a = Mat(matching_result.p1);
			// Mat b = Mat(matching_result.p2);
			// matching_result.mean_feature_error = norm(a, b) / ((double)matching_result.p1.rows);
			// // Finding homography
			// matching_result.H = findHomography(matching_result.p1, matching_result.p2, RANSAC, 0.5);
			// if (matching_result.H.rows == 0)
			// 	return -1;
			// /************************************************************* Draw matches */

			// matching_result.img_matches = Mat::zeros(img.rows, img.cols * 2, img.type());
			// drawMatches(state.desired_configuration.img, state.desired_configuration.kp, img, kp,
			// 						goodMatches, matching_result.img_matches,
			// 						Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

			// exit(-1);
		}
		/************************************************************* Prepare message */
		image_msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, matching_result.img_matches).toImageMsg();
		image_msg->header.frame_id = "matching_image";
		image_msg->width = matching_result.img_matches.cols;
		image_msg->height = matching_result.img_matches.rows;
		image_msg->is_bigendian = false;
		image_msg->step = sizeof(unsigned char) * matching_result.img_matches.cols * 3;
		image_msg->header.stamp = ros::Time::now();

		cout << "[INFO] Matching published" << endl;

		if (state.initialized)
			cout << "[VELS] Vx: " << state.Vx << ", Vy: " << state.Vy << ", Vz: " << state.Vz << "\nVroll: " << state.Vroll << ", Vpitch: " << state.Vpitch << ", Wyaw: " << state.Vyaw << "\n==> average error: " << matching_result.mean_feature_error << "<==" << endl
					 << "===================================================================\n\n";
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
							msg->encoding.c_str());
	}
}

/*
	Function: PoseCallback
	description: get the ppose info from the groundtruth of the drone and uses it in simulation
	params: message with pose info
*/
void poseCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
	// cout << endl << "[INFO] poseCallback function" << endl;

	// Creating quaternion
	tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
	// Creatring rotation matrix ffrom quaternion
	tf::Matrix3x3 mat(q);
	// obtaining euler angles
	double roll, pitch, yaw;
	mat.getEulerYPR(yaw, pitch, roll);
	// saving the data obtained
	state.Roll = (float)roll;
	state.Pitch = (float)pitch;

	// setting the position if its the first time
	if (!state.initialized)
	{
		cout << "[INFO] Setting initial position" << endl;
		state.initialize((float)msg->position.x, (float)msg->position.y, (float)msg->position.z, yaw);
	}
}

void writeFile(vector<float> &vec, const string &name)
{
	ofstream myfile;
	myfile.open(name);
	for (int i = 0; i < vec.size(); i++)
		myfile << vec[i] << endl;
	myfile.close();
}

void writeMatrix(Mat &mat, const string &name)
{
	ofstream myfile;
	myfile.open(name);
	for (int i = 0; i < mat.rows; i++)
	{
		for (int j = 0; j < mat.cols; j++)
		{
			myfile << mat.at<float>(i, j) << " ";
		}
		myfile << endl;
	}
	myfile.close();
}