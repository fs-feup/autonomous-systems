#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main() {
    // Create a test image
    Mat test_image(100, 100, CV_8UC3, Scalar(0, 255, 0));

    // Display the test image in a window
    namedWindow("Test Image", WINDOW_NORMAL);
    imshow("Test Image", test_image);

    // Wait for a key press
    waitKey(0);

    // Destroy the window
    destroyWindow("Test Image");

    cout << "OpenCV version: " << CV_VERSION << endl;

    return 0;
}
