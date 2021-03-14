#pragma once

#include <image_transport/image_transport.h>
#include <std_msgs/String.h>
#include <opencv2/core.hpp>
#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <boxes.h>
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/highgui.hpp"
#include <stdio.h>
#include <iostream>

class ImagePipeline {
    private:
        cv::Mat img;
        bool isValid;
        image_transport::Subscriber sub;

        uint8_t skyVal = 178; // RGB value of the skybox
        const uint8_t removeVal = 0; // Value used to overwrite pixels we don't care for
    public:
        ImagePipeline(ros::NodeHandle& n);
        void imageCallback(const sensor_msgs::ImageConstPtr& msg);
        int getTemplateID(Boxes& boxes);
        void loadImage(char* fileLocation); // Used for testing by loading in our own images
};
