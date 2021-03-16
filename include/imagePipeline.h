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
        float reqConfRatio   = 1.20; // Ratio between max and second to make a conclusion
        float reqConfMinimum = 0.07; // Minimum threshold to be considered conclusive

        float reqMinArea = 0.0; // Minimum area needed to be considered
        float areaConfidenceFactor = 0.00005; // Factor used to multiply area by before multiplying that to confidence

        // Confidence = (matching features / reference's features) * area * areaFactor

        int minHessian = 350;

        ImagePipeline(ros::NodeHandle& n);
        void imageCallback(const sensor_msgs::ImageConstPtr& msg);
        int getTemplateID(Boxes& boxes, bool showInternals = true);
        void loadImage(std::string fileLocation, bool printMessage = true); // Used for testing by loading in our own images
};
