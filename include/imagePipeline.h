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
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"

class ImagePipeline {
    private:
        cv::Mat img;
        bool isValid;
        image_transport::Subscriber sub;

        uint8_t skyVal = 178; // RGB value of the skybox
        const uint8_t removeVal = 0; // Value used to overwrite pixels we don't care for

        // Used to produce an output of the reference image in the scene based on existing matches and points
        cv::Mat drawSceneMatches(cv::Mat &scene, cv::Mat &refImage, std::vector<cv::DMatch> &matches, 
            std::vector<cv::KeyPoint> &keyPointsRef, std::vector<cv::KeyPoint> &keyPointsScene);

        // Searches for an image in a scene
        void searchInScene(cv::Mat &refImage, cv::Mat &descriptorsScene, std::vector<cv::KeyPoint> &keypointsObject,
            std::vector<cv::DMatch> &goodMatches, cv::Ptr<cv::xfeatures2d::SURF> &detector);
    public:
        ImagePipeline(ros::NodeHandle& n);
        void imageCallback(const sensor_msgs::ImageConstPtr& msg);
        int getTemplateID(Boxes& boxes);
        void loadImage(char* fileLocation); // Used for testing by loading in our own images
};
