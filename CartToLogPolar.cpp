#include <opencv2/opencv.hpp>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

// Speed test
#include <chrono>

#include "RadarImageHandler.hpp"

namespace fs = std::filesystem;

int main(int argc, char **argv) {
    if (argc <= 1 || argc > 3) {
        printf("Usage: ./CartToLogPolar <path_to_cart_image_input> [path_to_output_log_polar_image]\n");
        return EXIT_FAILURE;
    }

    std::string path = std::string(argv[1]);
    
    // NOTE: data is formatted as
    assert(fs::exists(path));

    cv::Mat image;
	
	readImageFromPath(image, path);

    if (!image.data) {
        printf("No image data \n");
        return EXIT_FAILURE;
    }

    // View coarse log-polar image
    cv::Mat coarseLogPolar;

	imageCartesianToLogPolar(image, coarseLogPolar);

    if (argc == 3) {
        cv::imwrite(std::string(argv[2]), coarseLogPolar);
        printf("Log polar file written to %.100s\n", argv[2]);
    }
    else {
        cv::namedWindow("Converted Log Polar Image", cv::WINDOW_AUTOSIZE);
        cv::imshow("Converted Log Polar Image", coarseLogPolar);
        cv::waitKey(0);
        cv::destroyWindow("Converted Log Polar Image");

        cv::destroyAllWindows();
    }

    return 0;
}