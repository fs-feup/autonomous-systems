#include <iostream>
#include <opencv2/opencv.hpp>
#include <zed/Camera.hpp>
#include <zed/Mat.hpp>

using namespace std;
using namespace cv;

// Constants for cone detection
const float CONE_HEIGHT = 0.25; // meters
const float CONE_WIDTH = 0.15; // meters

// Constants for camera calibration
const float fx = 700; // focal length in pixels
const float fy = 700;
const float cx = 640; // optical center in pixels
const float cy = 360;

int main()
{
    // Open the ZED 2 camera
    sl::Camera zed;
    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION_HD1080;
    sl::ERROR_CODE err = zed.open(init_params);
    if (err != sl::SUCCESS) {
        cout << "Error initializing the ZED camera: " << sl::errorCode2str(err) << endl;
        return -1;
    }

    // Create a window to display the output
    namedWindow("Output", WINDOW_NORMAL);

    // Start the main loop
    while (waitKey(1) != 'q') {
        // Grab a new frame from the ZED camera
        zed.grab();
        sl::Mat zed_left, zed_depth;
        zed.retrieveImage(zed_left, sl::VIEW_LEFT);
        zed.retrieveImage(zed_depth, sl::VIEW_DEPTH);

        // Convert the ZED images to OpenCV format
        Mat cv_left = Mat(zed_left.getHeight(), zed_left.getWidth(), CV_8UC4, zed_left.getPtr<sl::uchar1>(sl::MEM_GPU));
        Mat cv_depth = Mat(zed_depth.getHeight(), zed_depth.getWidth(), CV_32FC1, zed_depth.getPtr<sl::uchar1>(sl::MEM_GPU));

        // Detect cones in the current frame
        vector<Rect> cones;
        cv::cvtColor(cv_left, cv_left, COLOR_BGRA2BGR);
        cv::CascadeClassifier cone_cascade("cone_cascade.xml");
        cone_cascade.detectMultiScale(cv_left, cones, 1.1, 3, 0, Size(30, 30));

        // Calculate the distance to each cone
        for (const auto& cone : cones) {
            // Calculate the position of the cone in the image
            float x_center = cone.x + cone.width / 2;
            float y_center = cone.y + cone.height / 2;

            // Calculate the distance to the cone
            float z = cv_depth.at<float>(y_center, x_center);
            float x = (x_center - cx) * z / fx;
            float y = (y_center - cy) * z / fy;

            // Print the distance to the cone
            cout << "Cone at (" << x << ", " << y << ", " << z << ") meters" << endl;
        }

        // Display the output image with the detected cones
        for (const auto& cone : cones) {
            rectangle(cv_left, cone, Scalar(0, 255, 0), 2);
        }
        imshow("Output", cv_left);
    }

    // Clean up
    zed.close();
    destroyAllWindows();

    return 0;
}
