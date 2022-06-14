#include <opencv2/opencv.hpp>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

// Speed test
#include <chrono>

#include "RadarImage.hpp"

void view_radar_image(RadarImage &rimage, RadarImage::ImageType t, std::string title = "Radar Image"){
    cv::Mat image = rimage.getImage(t);

    cv::namedWindow(title, cv::WINDOW_AUTOSIZE);
    cv::imshow(title, image);

    cv::waitKey(0);

    cv::destroyWindow(title);
}

int main(int argc, char **argv) {
    // NOTE: data is formatted as
    if (argc != 3) {
        printf("USAGE: ./DisplayRadarImage <setNumber> <imageNumber>\n");
        return EXIT_FAILURE;
    }

    unsigned int setNumber = atoi(argv[1]);
    unsigned int imageNumber = atoi(argv[2]);
    
    RadarImage rimage(setNumber, imageNumber);
    
    // view_radar_image(rimage, rimage.raw);
    // view_radar_image(rimage, rimage.downsampled);
    view_radar_image(rimage, rimage.coarseCart);
    view_radar_image(rimage, rimage.coarseLogPolar);
    view_radar_image(rimage, rimage.subCart);

    return 0;
}