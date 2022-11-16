#include "vc_state/vc_state.h"

using namespace cv;
using namespace std;

int NUMERO;

// create struct
typedef struct vecDist
{
        int i;
        int j;
        double dist;
        double dist2;
} vecDist;

int toSphere(Mat p1,
             Mat p2,
             Mat &p1s,
             Mat &p2s,
             Mat &p23D,
             vc_parameters &params)
{
        // if (p1.rows != p2.rows || p1.cols != p2.cols || p1.rows == 0 || p1.cols == 0)
        // {
        //         cout << "Error: p1 and p2 must have the same size and not be empty" << endl;
        //         return -1;
        // }
        // recover 3d points from 2d points into sphere
        Mat temp = Mat::zeros(3, 1, CV_64F);
        Mat tmp;

        for (int i = 0; i < p1.rows; i++)
        {
                temp.at<double>(0, 0) = p1.at<double>(i, 0);
                temp.at<double>(1, 0) = p1.at<double>(i, 1);
                temp.at<double>(2, 0) = 1;
                tmp = params.K.inv() * temp;
                // tmp = tmp / norm(tmp);
                p1s.row(i) = tmp.t() / norm(tmp);
                // cout << "tmp1: " << tmp.t() << " p1s: " << p1s.row(i) << endl;

                temp.at<double>(0, 0) = p2.at<double>(i, 0);
                temp.at<double>(1, 0) = p2.at<double>(i, 1);
                temp.at<double>(2, 0) = 1;
                tmp = params.K.inv() * temp;
                p23D.row(i) = tmp.t();
                // cout << "tmp2.1: " << tmp.t() << " p23D: " << p23D.row(i) << endl;
                // tmp = tmp / norm(tmp);
                p2s.row(i) = tmp.t() / norm(tmp);
                // cout << "tmp2.2: " << tmp.t() << " p2s: " << p2s.row(i) << endl;
        }
        temp.release();
        tmp.release();
        // exit(-1);
        return 0;
}

int distances(Mat p1,
              Mat p2,
              vector<vecDist> &distancias)
{
        // if (p1.rows != p2.rows || p1.cols != p2.cols || p1.rows == 0 || p1.cols == 0)
        //         return -1;
        vecDist tmpDist;
        double dist, dist2;
        for (int i = 0; i < p1.rows; i++)
        {
                for (int j = 0; j < p1.rows; j++)
                {
                        if (i != j)
                        {
                                // cout << "p2i * p2j: " << p2.row(i).dot(p2.row(j)) << endl;
                                // cout << "p1i * p1j: " << p1.row(i).dot(p1.row(j)) << endl;
                                dist  = sqrt(2 - 2 * (double)(p2.row(i).dot(p2.row(j))));
                                dist2 = sqrt(2 - 2 * (double)(p1.row(i).dot(p1.row(j))));
                                if (dist <= 1e-9 || 
                                        dist2 <= 1e-9 || 
                                        isnan(dist) || 
                                        isnan(dist2) || 
                                        isinf(dist) || 
                                        isinf(dist2) || 
                                        p2.row(i).dot(p2.row(j)) > .97 || 
                                        p1.row(i).dot(p1.row(j)) > .97)
                                {
                                        // cout << "WTF dist: " << dist << " -- " << dist2 << " -> " << p2.row(i) << " <-> " << p2.row(j) << " - " << p1.row(i).dot(p1.row(j)) << endl;
                                        continue;
                                }
                                // cout << "dist: " << dist << " -- " << dist2 << " -> " << p2.row(i) << " <-> " << p2.row(j) << endl;
                                tmpDist.i = i;
                                tmpDist.j = j;
                                tmpDist.dist = 1 / dist;
                                tmpDist.dist2 = 1 / dist2;
                                // cout << "dist: " << dist << " / " << 1/dist << "p2.row(i): " << p2.row(i) << " -- p2.row(j): " << p2.row(j) << endl;
                                // cout << "dist2: " << dist2 << " / " << 1/dist2 << "p1.row(i): " << p1.row(i) << " -- p1.row(j): " << p1.row(j) << endl;
                                // error.push_back(tmpDist.dist2 - tmpDist.dist);
                                distancias.push_back(tmpDist);
                        }
                }
        }

        // for (int i = 0; i < 100; i++)
        // {
        //         cout << distancias[i].i << " " << distancias[i].j << " " << distancias[i].dist << " " << distancias[i].dist2 << endl;
        //         cout << "p2.row(i): " << p2.row(distancias[i].i) << " -- p2.row(j): " << p2.row(distancias[i].j) << endl;
        //         cout << p2.row(distancias[i].i).dot(p2.row(distancias[i].j)) << endl << endl;
        // }

        // exit(-1);
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
        // Mat OP = I - p1Temp.t() * p1Temp;
        I.release();
        p1Temp.release();
        return OP;
}

Mat Lvl(Mat p23D,
        Mat p2s,
        vector<vecDist> &distances)
{
        NUMERO = (int)(distances.size());
        int n = NUMERO;
        std::cout << ">> Interaction_Mat: n  = " << n << std::endl;
        cout << "NUMERO: " << NUMERO << endl;

        Mat L = Mat::zeros(n, 3, CV_64F);
        Mat temp = Mat::zeros(3, 1, CV_64F);
        Mat pi, pj, Pi, Pj;
        double s;
        for (int i = 0; i < n; i++)
        {
                pi = p2s.row(distances[i].i);
                pj = p2s.row(distances[i].j);
                // Pi = p23D.row(distances[i].i);
                // Pj = p23D.row(distances[i].j);

                s = -distances[i].dist * distances[i].dist * distances[i].dist;
                // temp = s * (pi * ortoProj(Pj) / norm(Pj) + pj * ortoProj(Pi) / norm(Pi));
                temp = s * (pi * ortoProj(pj) + pj * ortoProj(pi));
                // temp.copyTo(L.row(distances[i].i));
                temp.copyTo(L.row(i));
                // cout << i << " de " << n << " temp: " << temp << " len: " << temp.size() << endl;
                // cout << "L.row(i): " << L.row(i) << " len: " << L.row(i).size() << endl;
        }
        temp.release();
        pi.release();
        pj.release();
        Pi.release();
        Pj.release();
        // exit(-1);
        return L;
}

Mat p1s, p2s, p23D, Lo;
int GUO(Mat img,
        vc_state &state,
        vc_homograpy_matching_result &matching_result
        //                sensor_msgs::ImagePtr image_msg
)
{
        cout << "---------> EMPIEZA ! ..." << endl;

        if (compute_descriptors(img, state.params, state.desired_configuration, matching_result) < 0)

        {
                cout << "Error en compute_descriptors" << endl;
                return -1;
        }

        p1s = Mat::zeros(matching_result.p1.rows, 3, CV_64F);
        p2s = Mat::zeros(matching_result.p2.rows, 3, CV_64F);
        p23D = Mat::zeros(matching_result.p2.rows, 3, CV_64F);

        if (toSphere(matching_result.p1, matching_result.p2, p1s, p2s, p23D, state.params) < 0)
        {
                cout << "Error en toSphere" << endl;
                return -1;
        }

        vector<vecDist> distancias;
        Mat ERROR;
        distances(p1s, p2s, distancias);
        sort(distancias.begin(), distancias.end(), mayorQue);

        // for (int i = 0; i < 10; i++)
        // {
        //         cout << distancias[i].i << " " << distancias[i].j << " " << distancias[i].dist << " " << distancias[i].dist2 << endl;
        //         cout << "p2.row(i): " << p2s.row(distancias[i].i) << " -- p2.row(j): " << p2s.row(distancias[i].j) << endl;
        //         // cout << p2s.row(distancias[i].i).dot(p2s.row(distancias[i].j)) << endl << endl;
        // }
        // exit(-1);

        // cout << "distances = " << endl;
        // for (int i = 0; i < 10; i++)
        // {
        //         cout << i << ": " << n << " << " << distancias[i].i << " "
        //              << distancias[i].j << " " << distancias[i].dist << " "
        //              << distancias[i].dist2 << " -> ("
        //              << matching_result.p1.at<double>(distancias[i].i, 0) << ", "
        //              << matching_result.p1.at<double>(distancias[i].i, 1) << ") -- ("
        //              << matching_result.p2.at<double>(distancias[i].j, 0) << ", "
        //              << matching_result.p2.at<double>(distancias[i].j, 1) << ") " << endl;
        // }
        // cout << "Size of distances = " << distancias.size() << endl;
        // cout << "Size of error = " << ERROR.size() << endl;

        // //
        // cout << "DB2" << endl;
        // Descriptor control
        double lambda = 10.0;
        // Mat L = interaction_Mat(matching_result, 1.0);
        Mat L = Lvl(p23D, p2s, distancias);
        // cout << "L[0] = " << L.row(0) << endl;
        // exit(-1);
        for (int i = 0; i < NUMERO; i++)
        {
                ERROR.push_back(distancias[i].dist2 - distancias[i].dist);
        }

        double det = 0.0;
        Lo = Moore_Penrose_PInv(L, det);
        if (det < 1e-6)
        {
                cout << "ERROR DET ZERO: det = " << det << endl;
                return -1;
        }
        Mat U_temp = - lambda * Lo * ERROR;
        // cout << "U_temp = " << U_temp << endl;
        // cout << "LLEGA (?) 2" << endl;

        Mat U = Mat::zeros(6, 1, CV_64F);
        U_temp.copyTo(U.rowRange(0, 3));

        // U = lambda * U * norm(ERROR);

        // cout << "DB3" << endl;
        cout << "U = " << U.t() << endl;
        // cout << "Before norm P1 " << matching_result.p1 << endl;
        // cout << "Before norm P2 " << matching_result.p2 << endl;

        // Mat err = matching_result.p1 - matching_result.p2;
        // cout << "err = " << err << endl;

        // cout << "######################################################################" << endl;
        // cout << "antes: " << matching_result.p1.row(0) << " - " << matching_result.p2.row(0) << endl;
        // camera_norm(state.params, matching_result);
        // cout << "despu: " << matching_result.p1.row(0) << " - " << matching_result.p2.row(0) << endl;

        // cout camera param K
        // cout << "K = " << state.params.K << endl;

        // cout << "After norm P1 " << matching_result.p1 << endl; // << matching_result.p2 << endl;
        // cout << "After norm P2 " << matching_result.p2 << endl;
        // err = matching_result.p1 - matching_result.p2;

        // err =

        //         cout << "err = " << err << endl;
        // cout << "DB2.1" << endl;
        //         camera_norm(state.params, matching_result);

        // cout << "TAMAÑO L = " << L.size() << endl;
        // cout << "TAMAÑO Lo = " << Lo.size() << endl;
        // cout << "TAMAÑO ERROR = " << ERROR.size() << endl;
        // cout << "DET = " << det << endl;
        // exit(-1);
        // for (int i = 0; i < 10; i++)
        // {
        //         cout << "DISTANCIAS:" << distancias[i].dist2 << " - " << distancias[i].dist << " = " << distancias[i].dist2 - distancias[i].dist << endl;
        //         cout << "L = " << L.row(i) << endl;
        //         cout << "L = " << Lo.row(i) << endl;
        //         cout << "ERROR = " << ERROR.row(i) << endl;
        // }

        // cout << "LLEGA (?) 0.1" << endl;
        // cout << "LLEGA (?) 0.2" << endl;
        // exit(-1);
        // cout << "DB2.3" << endl;
        // cout << "check before: " << ERROR.at<double>(0, 0) << " " << ERROR.at<double>(0, 1) << " " << ERROR.at<double>(0, 2)
        //      << " after: " << ERROR.reshape(1, L.cols).at<double>(0, 0) << " " << ERROR.reshape(1, L.cols).at<double>(1, 0) << " " << ERROR.reshape(1, L.cols).at<double>(2, 0) << endl;
        //         cout << "ERROR = " << ERROR << endl;
        //         cout << "After p " << matching_result.p1 << endl << matching_result.p2 << endl;
        // Mat U_temp = -1.0 * lambda * Lo * ERROR.reshape(1, L.cols);
        // cout << "LLEGA (?) 1" << endl;

        // exit(-1);

        /**********Updating velocities in the axis*/
        // velocities from homography decomposition
        // cout << "L  = \n" << L <<  endl;
        state.Vx = (float)U.at<double>(1, 0);
        state.Vy = (float)U.at<double>(0, 0);
        state.Vz = (float)U.at<double>(2, 0);
        // state.Vroll  = (float) U.at<double>(3,0);
        // state.Vpitch = (float) U.at<double>(4,0);
        state.Vyaw = (float)U.at<double>(5, 0);
        cout << "Enviadas las velocidades..." << endl;
        // exit(-1);
        return 0;
}
