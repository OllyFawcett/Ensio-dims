#ifndef DIMS_TOOLS_HPP
#define DIMS_TOOLS_HPP
#endif // DIMS_TOOLS_HPP
#define _USE_MATH_DEFINES
#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_export.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <algorithm>
#include <conio.h>
#include <mutex>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <Eigen/SVD>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <fstream>
#include <numeric>
#include <boost/asio.hpp>
#include <nlohmann/json.hpp>
#include <string>
#include <unordered_map>
#include <string.h>
#include <openssl/sha.h>
#include <openssl/hmac.h>
#include <openssl/evp.h>
#include <openssl/bio.h>
#include <openssl/buffer.h>

using namespace std;
using namespace cv;
using namespace rs2;
using namespace Eigen;

using boost::asio::ip::tcp;
using json = nlohmann::json;



struct calibration_state
{

    float pitch;
    float yaw;
    float roll;
    Matrix4d Transform;
    Mat colour_image;
    Mat depth_image;
};

std::string base64_encode(const std::string& data) {
    static const char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

    std::string output;
    output.reserve(((data.size() + 2) / 3) * 4);

    for (size_t i = 0; i < data.size(); i += 3) {
        unsigned char b1 = data[i];
        unsigned char b2 = (i + 1 < data.size()) ? data[i + 1] : 0;
        unsigned char b3 = (i + 2 < data.size()) ? data[i + 2] : 0;

        unsigned char c1 = b1 >> 2;
        unsigned char c2 = ((b1 & 0x03) << 4) | (b2 >> 4);
        unsigned char c3 = ((b2 & 0x0f) << 2) | (b3 >> 6);
        unsigned char c4 = b3 & 0x3f;

        output += alphabet[c1];
        output += alphabet[c2];
        output += (i + 1 < data.size()) ? alphabet[c3] : '=';
        output += (i + 2 < data.size()) ? alphabet[c4] : '=';
    }

    return output;
}

struct dimensions
{
    float height;
    float width;
    float depth;
    float volume;
};

std::unordered_map<std::string, std::string> parse_query_string(const std::string& query_string) {
    std::unordered_map<std::string, std::string> parameters;

    std::size_t start = 0;
    while (start < query_string.length()) {
        std::size_t end = query_string.find('&', start);
        if (end == std::string::npos) {
            end = query_string.length();
        }
        std::size_t separator = query_string.find('=', start);
        if (separator != std::string::npos && separator < end) {
            std::string key = query_string.substr(start, separator - start);
            std::string value = query_string.substr(separator + 1, end - separator - 1);
            parameters[key] = value;
        }
        start = end + 1;
    }

    return parameters;
}

MatrixXd concatenateMatrices(MatrixXd& A, MatrixXd& B)
{
    if (A.cols() != B.cols())
    {
        throw std::invalid_argument("Input matrices have different numbers of columns");
    }

    Eigen::MatrixXd C(A.rows() + B.rows(), A.cols());
    C << A, B;

    return C;
}



/*Function to transform a set of points given a 3d transformation matrix.
* Input:
*   -T: 3d transformation matrix.
*   -points: points to transform.
* Output:
*   -transformedPoints: the transformed points.
*/
MatrixXd transformPoints(const Matrix4d& T, const MatrixXd& points)
{
    // Assemble the points into a matrix of 4D homogeneous points
    MatrixXd homogeneousPoints(points.rows(), 4);
    homogeneousPoints.leftCols<3>() = points;
    homogeneousPoints.col(3).setOnes();
    // Multiply the points by the transformation matrix
    MatrixXd transformedPoints = homogeneousPoints * T.transpose();
    // Return the transformed points as a matrix of 3D points
    return transformedPoints.leftCols<3>();
}

/*Function to find the 3d transformation matrix between two sets of 3d points.
* Input:
*   -points1: the first set of points.
*   -points2: the second set of points.
* Output:
*   -T: the transformation matrix.
*/
Eigen::Matrix4d findTransformation(const Eigen::MatrixXd& points1, const Eigen::MatrixXd& points2)
{
    // Check if the input matrices have the same number of rows and columns
    if (points1.rows() != points2.rows() || points1.cols() != points2.cols())
    {
        throw std::invalid_argument("Input matrices have different sizes");
    }

    // Compute the centroids of the two sets of points
    Eigen::Vector3d centroid1 = points1.colwise().mean();
    Eigen::Vector3d centroid2 = points2.colwise().mean();

    // Subtract the centroids from the points
    Eigen::MatrixXd points1_centered = points1.rowwise() - centroid1.transpose();
    Eigen::MatrixXd points2_centered = points2.rowwise() - centroid2.transpose();

    // Compute the covariance matrix
    Eigen::Matrix3d cov = points2_centered.transpose() * points1_centered;

    // Compute the SVD of the covariance matrix
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);

    // Compute the rotation matrix
    Eigen::Matrix3d rotation = (svd.matrixV() * svd.matrixU().transpose()).inverse();

    // Compute the translation vector
    Eigen::Vector3d translation = centroid2 - rotation * centroid1;

    // Assemble the transformation matrix
    Eigen::Matrix4d T;
    T.block<3, 3>(0, 0) = rotation;
    T.block<3, 1>(0, 3) = translation;
    T.row(3) << 0, 0, 0, 1;

    return T;
}


/*Function to generate the objective XYZ points on the calibration board.
* Inputs:
*	-rows: number of rows in calibration board.
*	-cols: number of columns in calibration board.
*	-dist: distance between points in calibration board.
*	-display: if true will print out the points of the calibration board.
*	-reverse: if true, order of points is reversed.
*	-height: height of the calibration board.
*/
MatrixXd generate_board(int rows, int cols, double dist, bool display, bool reverse, double height)
{
    double cur_x;
    double cur_y;
    MatrixXd objective_pts(cols * rows, 3);
    int cur = 0;
    if (reverse)
    {
        cur_x = 0;
        cur_y = (rows - 1) * dist;
    }
    else
    {
        cur_x = (cols - 1) * dist;
        cur_y = 0.0;
    }

    for (int i = 0; i < cols; i++)
    {
        if (reverse)
        {
            cur_y = (rows - 1) * dist;
        }
        else
        {
            cur_y = 0.0;
        }

        for (int j = 0; j < rows; j++)
        {
            objective_pts(cur, 0) = cur_x;
            objective_pts(cur, 1) = cur_y;
            objective_pts(cur, 2) = height;
            if (display)
            {
                cout << "[" << cur_x << "," << cur_y << ", 1.0],";
            }
            if (reverse)
            {
                cur_y -= dist;
            }
            else
            {
                cur_y += dist;
            }

            cur += 1;
        }
        if (reverse)
        {
            cur_x += dist;
        }
        else
        {
            cur_x -= dist;
        }

    }

    if (display)
    {
        cout << endl;
    }
    return(objective_pts);
}

/*Function to calibrate the camera using a checkerboard pattern.
* Input:
*	-pipe: the pipeline used to get frames from the camera.
*	-verbose: value between 0 and 5 showing how much to print to the screen.
*	-save: if true saves the calibration state to text file called calibration.txt.
*Output:
*	-output: a structure containing the transformation matrix, pitch, yaw and roll.
*/
calibration_state calibrateCamera(pipeline pipe, int verbose, bool save, string camera, float dist, float height, int rows, int cols)
{
    if (verbose == 5)
    {
        cout << "-------------------Calibrating-Camera-------------------" << endl;
    }
    int numCornersHor = rows;
    int numCornersVer = cols;
    Size board_sz = Size(numCornersHor, numCornersVer);
    vector<Point2d> corners;
    rs2::align align_to_color(RS2_STREAM_COLOR);
    colorizer c;
    float upoint[3];
    float upixel[2];
    vector<int> remove_rows;
    vector<double> ptsx;
    vector<double> ptsy;
    vector<double> ptsz;
    MatrixXd objective_pts_temp(rows * cols, 3);
    vector <double> corner_points;
    bool found = false;
    auto frames = pipe.wait_for_frames();
    frames = align_to_color.process(frames);
    auto color = frames.get_color_frame();
    auto depth = frames.get_depth_frame();
    Mat gray_image;

    //Continuously tries to find calibration board until one is found.
    while (!found)
    {
        //Aligning depth and colour frame
        frames = pipe.wait_for_frames();
        frames = align_to_color.process(frames);
        color = frames.get_color_frame();
        depth = frames.get_depth_frame();
        auto colorized_depth = c.colorize(depth);
        const int w = colorized_depth.as<rs2::video_frame>().get_width();
        const int h = colorized_depth.as<rs2::video_frame>().get_height();

        //Converting depth and colour frames to opencv images
        Mat image(Size(w, h), CV_8UC3, (void*)colorized_depth.get_data(), Mat::AUTO_STEP);
        Mat image2(Size(w, h), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
        cvtColor(image2, gray_image, cv::COLOR_BGRA2GRAY);
        //Finding corners of calibration board
        found = findChessboardCorners(gray_image, board_sz, corners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
    }

    if (verbose == 5)
    {
        cout << "Found chessboard corners" << endl;
    }

    //getting camera intrinisic
    rs2_intrinsics intr = depth.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
    //visualizing points found and getting 3d coordinates of points found
    auto colorized_depth = c.colorize(depth);
    const int w = colorized_depth.as<rs2::video_frame>().get_width();
    const int h = colorized_depth.as<rs2::video_frame>().get_height();

    //Converting depth and colour frames to opencv images
    Mat image(Size(w, h), CV_8UC3, (void*)colorized_depth.get_data(), Mat::AUTO_STEP);
    Mat image2(Size(w, h), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
    for (int i = 0; i < corners.size(); i++)
    {

        circle(image2, corners.at(i), 5, (0, 0, i * 10), 5);
        circle(image, corners.at(i), 5, (0, 0, i * 10), 5);

        upixel[0] = corners.at(i).x;
        upixel[1] = corners.at(i).y;
        if (depth.get_distance(corners.at(i).x, corners.at(i).y) != 0.0)
        {
            rs2_deproject_pixel_to_point(upoint, &intr, upixel, depth.get_distance(corners.at(i).x, corners.at(i).y));
            ptsx.push_back(upoint[0]);
            ptsy.push_back(upoint[1]);
            ptsz.push_back(upoint[2]);
            remove_rows.push_back(i);
        }

    }

    if (verbose == 5)
    {
        cout << "Found 3D coordinates of chessboard corners from camera's perspective." << endl;
    }

    //dealing with the ordering of points found on chessboard
    if (corners.at(0).x > corners.at(20).x)
    {

        objective_pts_temp = generate_board(rows, cols, dist, false, false, height);
    }
    else
    {

        objective_pts_temp = generate_board(rows, cols, dist, false, true, height);
    }

    //dealing with points with missing depth data
    MatrixXd objective_pts(ptsx.size(), 3);
    MatrixXd found_pts(ptsx.size(), 3);
    for (int i = 0; i < ptsx.size(); i++)
    {
        objective_pts(i, 0) = objective_pts_temp(remove_rows.at(i), 0);
        objective_pts(i, 1) = objective_pts_temp(remove_rows.at(i), 1);
        objective_pts(i, 2) = objective_pts_temp(remove_rows.at(i), 2);
        found_pts(i, 0) = ptsx.at(i);
        found_pts(i, 1) = ptsy.at(i);
        found_pts(i, 2) = ptsz.at(i);
    }

    //Finding transformation matrix
    Matrix4d R = findTransformation(found_pts, objective_pts);
    if (verbose == 5)
    {
        cout << "Calculated transformation matrix." << endl;
    }

    //Checking transformation
    MatrixXd validation_pts = transformPoints(R, found_pts);

    //Converting transformation matrix to angle of pitch, yaw and roll
    double pitch = atan2(-R(2, 0), sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0))) * (180.0 / M_PI);
    double yaw = atan2(R(1, 0), R(0, 0)) * (180 / M_PI);
    double roll = atan2(R(2, 1), R(2, 2)) * (180 / M_PI);
    //Finding mean-squared error or transformation
    double MSE = 0;
    for (int i = 0; i < ptsx.size(); i++) {
        Eigen::Vector3d diff = objective_pts.row(i) - validation_pts.row(i);
        MSE += diff.squaredNorm();
    }
    MSE /= found_pts.rows();
    if (verbose > 0)
    {
        cout << "Transformation matrix: " << endl;
        cout << R << endl;
        cout << "yaw:" << yaw << endl;
        cout << "pitch:" << pitch << endl;
        cout << "roll:" << roll << endl;
        std::cout << "MSE: " << MSE << std::endl;
    }
    if (verbose == 5)
    {
        cout << "--------------------------------------------------------" << endl << endl;
        cout << "Found points and validation points in python syntax:" << endl;
    }

    //printing out the found points and the transformed points
    if (verbose >= 3)
    {
        cout << "Found points:" << endl;
        for (int x = 0; x < ptsx.size(); x++)
        {
            if (x != ptsx.size() - 1)
            {
                cout << "[" << found_pts(x, 0) << "," << found_pts(x, 1) << "," << found_pts(x, 2) << "],";
            }
            else
            {
                cout << "[" << found_pts(x, 0) << "," << found_pts(x, 1) << "," << found_pts(x, 2) << "]";
            }

        }
        cout << endl << endl << "Validation points:" << endl;
        for (int x = 0; x < ptsx.size(); x++)
        {
            if (x != ptsx.size() - 1)
            {
                cout << "[" << validation_pts(x, 0) << "," << validation_pts(x, 1) << "," << validation_pts(x, 2) << "],";
            }
            else
            {
                cout << "[" << validation_pts(x, 0) << "," << validation_pts(x, 1) << "," << validation_pts(x, 2) << "]";
            }

        }

        cout << endl << "--------------------------------------------------------" << endl;
    }
    if (save)
    {
        if (camera == "right")
        {
            std::ofstream file("calibration_right.txt");
            if (file.is_open()) {
                file << R.format(IOFormat(FullPrecision, 0, " ", "\n", "", "", "", ""));
            }
            file.close();
            cout << "saved calibration" << endl;
        }
        if (camera == "left")
        {
            std::ofstream file("calibration_left.txt");
            if (file.is_open()) {
                file << R.format(IOFormat(FullPrecision, 0, " ", "\n", "", "", "", ""));
            }
            file.close();
            cout << "saved calibration" << endl;
        }

    }

    //displaying depth and color images with chessboard corners found
    if (verbose >= 3)
    {
        imshow("depth", image);
        imshow("color", image2);
    }
    calibration_state output;
    output.Transform = R;
    output.pitch = pitch;
    output.yaw = yaw;
    output.roll = roll;
    output.colour_image = image;
    output.depth_image = image2;
    return output;
}

/*Function for loading the calibration state from a text file.
* Inputs:
*	-filename: the name of the file for the transformation matrix.
* Output:
*	-output: a structure containing the transformation matrix, pitch, yaw and roll.
*/
calibration_state loadCalibrationFromFile(string filename)
{
    std::ifstream file(filename);
    MatrixXd R(4, 4);
    if (file.is_open()) {
        file >> R(0, 0) >> R(0, 1) >> R(0, 2) >> R(0, 3)
            >> R(1, 0) >> R(1, 1) >> R(1, 2) >> R(1, 3)
            >> R(2, 0) >> R(2, 1) >> R(2, 2) >> R(2, 3)
            >> R(3, 0) >> R(3, 1) >> R(3, 2) >> R(3, 3);
    }
    file.close();

    double pitch = atan2(-R(2, 0), sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0))) * (180.0 / M_PI);
    double yaw = atan2(R(1, 0), R(0, 0)) * (180 / M_PI);
    double roll = atan2(R(2, 1), R(2, 2)) * (180 / M_PI);
    calibration_state output;
    output.Transform = R;
    output.pitch = pitch;
    output.yaw = yaw;
    output.roll = roll;
    cout << "loaded calibration state" << endl;
    return output;
}

/*Function to get the distance to the closest object to the end of the forks in the y-direction.
* Inputs:
*	-points: the rotated points in the point cloud.
*	-num_pts: the number of points in the point cloud.
*	-end_of_forks: the y value for the end of the forks.
* -Output:
*	-closest_point: the distance to the closest point from the end of the forks.
*/
float distanceSensor(MatrixXd points, int num_pts, float end_of_forks)
{
    vector<float> all_pointsy;
    for (int i = 0; i < num_pts; i++)
    {
        //Checking if points are above the ground and past the forks
        if (points(i, 2) > 0.1 && points(i, 1) < end_of_forks)
        {
            all_pointsy.push_back(points(i, 1));
        }
    }
    //Finding the closest point to the end of the forks.
    float closest_point = end_of_forks - *max_element(all_pointsy.begin(), all_pointsy.end());
    return(closest_point);
}

/*Function to calculate the dimensions of a pallet.
* Inputs:
*	-points: the points in the point cloud.
*	-lg_height: height of the load guard.
*	-lg_y: the y value for the end of the load guard.
*	-lg_width: the width of the load guard.
*	-pallet_max_width: max width of pallet.
*	-pallet_max_length: max length of pallet.
*	-num_pts: number of points in point cloud.
* Output:
*	-dims: a structure containing the width, height, length and volume of pallet.
*/
dimensions calculate_dims(MatrixXd points, float lg_height, float lg_y, float lg_width, float pallet_max_width, float pallet_max_length, int num_pts)
{
    vector<float> load_guard_pointsx;
    vector<float> load_guard_pointsy;
    vector<float> load_guard_pointsz;
    vector<float> pallet_pointsx;
    vector<float> pallet_pointsy;
    vector<float> pallet_pointsz;
    vector<float> all_pointsx;
    vector<float> all_pointsy;
    vector<float> all_pointsz;
    for (int i = 0; i < num_pts; i++)
    {
        all_pointsx.push_back(points(i, 0));
        all_pointsy.push_back(points(i, 1));
        all_pointsz.push_back(points(i, 2));
        //seperating from floor (correct)
        if (points(i, 2) > 0.2)
        {
            //seperating points in distance
            if (points(i, 0) < pallet_max_width / 2 && points(i, 0) > -1 * (pallet_max_width / 2) && points(i, 1) < pallet_max_length / 2 && points(i, 1) > -1 * (pallet_max_length / 2) && points(i, 1) < lg_y + lg_width)
            {
                //adding points to load guard
                if (points(i, 1) > lg_y)
                {
                    load_guard_pointsx.push_back(points(i, 0));
                    load_guard_pointsy.push_back(points(i, 1));
                    load_guard_pointsz.push_back(points(i, 2));
                }

                //adding points to pallet
                else
                {
                    pallet_pointsx.push_back(points(i, 0));
                    pallet_pointsy.push_back(points(i, 1));
                    pallet_pointsz.push_back(points(i, 2));

                }
            }
        }
    }
    dimensions dims;
    dims.width = float(*max_element(pallet_pointsx.begin(), pallet_pointsx.end()) - *min_element(pallet_pointsx.begin(), pallet_pointsx.end()));
    dims.depth = float(*max_element(pallet_pointsy.begin(), pallet_pointsy.end()) - *min_element(pallet_pointsy.begin(), pallet_pointsy.end()));
    dims.height = lg_height - (*max_element(load_guard_pointsz.begin(), load_guard_pointsz.end()) - (*max_element(pallet_pointsz.begin(), pallet_pointsz.end())));
    dims.volume = dims.width * dims.depth * dims.height;
    cout << "Width: " << dims.width << endl;
    cout << "Depth: " << dims.depth << endl;
    cout << "Height: " << dims.height << endl;
    cout << "Volume: " << dims.volume << endl;
    return(dims);
}



