#include "vc_state/vc_state.h"

using namespace cv;
using namespace std;

// Create the Struct which be able to
// store the information about the points
typedef struct vecDist
{
        int i;        // index i
        int j;        // index j p_i * p_j
        double dist;  // distance between p2_i and p2_j
        double dist2; // distance between p1_i and p1_j
} vecDist;

int toSphere(Mat p1,               // Points in the target image
             Mat p2,               // Points in the actual image
             Mat &p1s,             // Empty matrix for 3D recovery direction of p1 points
             Mat &p2s,             // Empty matrix for 3D recovery direction of p2 points
             vc_parameters &params // Parameters of the camera
)
{
        Mat temp = Mat::zeros(3, 1, CV_64F), tmp; // Temporal matrix for calculation

        for (int i = 0; i < p1.rows; i++)
        {
                // Take the points in target image and add 1 to the last row
                temp.at<double>(0, 0) = p1.at<double>(i, 0);
                temp.at<double>(1, 0) = p1.at<double>(i, 1);
                temp.at<double>(2, 0) = 1;
                // Invert the matrix of the camera and multiply by the points
                tmp = params.K.inv() * temp;
                // Normalize the points
                p1s.row(i) = tmp.t() / norm(tmp);

                // Take the points in actual image and add 1 to the last row
                temp.at<double>(0, 0) = p2.at<double>(i, 0);
                temp.at<double>(1, 0) = p2.at<double>(i, 1);
                temp.at<double>(2, 0) = 1;
                // Invert the matrix of the camera and multiply by the points
                tmp = params.K.inv() * temp;
                // Normalize the points
                p2s.row(i) = tmp.t() / norm(tmp);
        }
        // Free the memory
        temp.release();
        tmp.release();
        return 0;
}

int distances(Mat p1,                     // Points in the target image
              Mat p2,                     // Points in the actual image
              vector<vecDist> &distancias // Vector of distances struct
)
{
        vecDist tmpDist;    // Temporal struct for calculation
        double dist, dist2; // Temporal variables for distance calculation

        for (int i_temp = 0; i_temp < 16; i_temp++)
        {
                for (int j_temp = 0; j_temp < 16; j_temp++)
                {
                        int i = (int) rand() % p1.rows;
                        int j = (int) rand() % p2.rows;
                        if (i != j)
                        {
                                dist = sqrt(2 - 2 * (double)(p2.row(i).dot(p2.row(j))));
                                dist2 = sqrt(2 - 2 * (double)(p1.row(i).dot(p1.row(j))));
                                if (dist <= 1e-9 ||
                                    dist2 <= 1e-9 ||
                                    //     isnan(dist) ||
                                    //     isnan(dist2) ||
                                    //     isinf(dist) ||
                                    //     isinf(dist2) ||
                                    p2.row(i).dot(p2.row(j)) > .97 ||
                                    p1.row(i).dot(p1.row(j)) > .97)
                                {
                                        continue;
                                }

                                tmpDist.i = i;
                                tmpDist.j = j;
                                tmpDist.dist = 1 / dist;
                                tmpDist.dist2 = 1 / dist2;
                                distancias.push_back(tmpDist);
                        }
                }

        }
        return 0;
}

bool mayorQue(vecDist a, vecDist b)
{
        return a.dist > b.dist;
}

Mat ortoProj(Mat p1)
{
        Mat I = Mat::eye(3, 3, CV_64F);
        Mat p1Temp = Mat::zeros(3, 1, CV_64F);

        p1Temp.at<double>(0, 0) = p1.at<double>(0, 0);
        p1Temp.at<double>(1, 0) = p1.at<double>(0, 1);
        p1Temp.at<double>(2, 0) = p1.at<double>(0, 2);

        Mat OP = I - p1Temp * p1Temp.t();

        I.release();
        p1Temp.release();
        return OP;
}

Mat Lvl(Mat p2s,                   // Points of the actual image in the sphere
        vector<vecDist> &distances // Vector of distances struct with actual distances
)
{
        int n = distances.size(); // Number of distances
        // int n = 16; // Number of distances
        std::cout << ">> Size Interaction Matrix: [" << n << "x3]" << std::endl;

        Mat temp = Mat::zeros(3, 1, CV_64F); // Temp vector for calculation
        Mat L = Mat::zeros(n, 3, CV_64F);    // Interaction matrix
        Mat pi, pj;                          // Temporal points for calculation
        double s;
        for (int i = 0; i < n; i++)
        {
                pi = p2s.row(distances[i].i);
                pj = p2s.row(distances[i].j);

                s = -distances[i].dist * distances[i].dist * distances[i].dist;
                temp = s * (pi * ortoProj(pj) + pj * ortoProj(pi));
                temp.copyTo(L.row(i));
        }
        temp.release();
        pi.release();
        pj.release();
        return L;
}

int GUO(Mat img,                                      // Image to be processed
        vc_state &state,                              // State of the camera
        vc_homograpy_matching_result &matching_result // Result of the matcher matching
)
{
        cout << "--------------------> EMPIEZA <--------------------" << endl;

        // Compute the matching between the images using ORB as detector and descriptor
        if (compute_descriptors(img, state.params, state.desired_configuration, matching_result) < 0)
        {
                cout << "Error en compute_descriptors" << endl;
                return -1;
        }

        // Temporal matrixes for calculation
        Mat p1s, p2s, p23D, Lo, ERROR, U, U_temp, L;
        p1s = Mat::zeros(matching_result.p1.rows, 3, CV_64F);
        p2s = Mat::zeros(matching_result.p2.rows, 3, CV_64F);
        p23D = Mat::zeros(matching_result.p2.rows, 3, CV_64F);
        vector<vecDist> distancias;

        // Send images points to sphere model by generic camera model
        if (toSphere(matching_result.p1, matching_result.p2, p1s, p2s, state.params) < 0)
        {
                cout << "Error en toSphere" << endl;
                return -1;
        }

        // Calculate the distances between the points in the sphere
        // and sorting these distance for choose the greater ones
        distances(p1s, p2s, distancias);
        sort(distancias.begin(), distancias.end(), mayorQue);

        // Choosing the gain for the control law
        double lambda = 5.0;

        // Get interaction matrix and error vector with distances
        L = Lvl(p2s, distancias);
        for (int i = 0; i < distancias.size(); i++)
        // for (int i = 0; i < 16; i++)
        {
                ERROR.push_back(distancias[i].dist2 - distancias[i].dist);
        }

        // Get the Penrose pseudo-inverse of the interaction matrix
        double det = 0.0;
        Lo = Moore_Penrose_PInv(L, det);
        if (det < 1e-6)
        {
                cout << "ERROR DET ZERO: det = " << det << endl;
                return -1;
        }

        // Get the control law with dimentions 3x1 in translation
        U_temp = -lambda * Lo * ERROR;

        // FIll with zeros the control law in rotation 3x1
        U = Mat::zeros(6, 1, CV_64F);
        U_temp.copyTo(U.rowRange(0, 3));
        cout << "U = " << U.t() << endl;

        // Send the control law to the camera
        state.Vx = (float)U.at<double>(1, 0);
        state.Vy = (float)U.at<double>(0, 0);
        state.Vz = (float)U.at<double>(2, 0);
        // state.Vroll  = (float) U.at<double>(3,0);
        // state.Vpitch = (float) U.at<double>(4,0);
        state.Vyaw = (float)U.at<double>(5, 0);
        // cout << "Enviadas las velocidades..." << endl;

        U.release();
        U_temp.release();
        L.release();
        Lo.release();
        ERROR.release();
        p1s.release();
        p2s.release();
        p23D.release();
        distancias.clear();

        return 0;
}
