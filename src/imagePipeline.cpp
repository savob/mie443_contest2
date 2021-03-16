#include <imagePipeline.h>

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"

ImagePipeline::ImagePipeline(ros::NodeHandle& n) {
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
    isValid = false;
}

void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        if(isValid) img.release();

        img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();
        isValid = true;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from %s to %s!", msg->encoding.c_str(),  IMAGE_TYPE.c_str());
        isValid = false;
    }    
}

int ImagePipeline::getTemplateID(Boxes& boxes, bool showInternals) {
    int determinedId = -1; // Default to error

    if(!isValid) {
        ROS_ERROR("INVALID IMAGE!");
        return determinedId;
    } else if(img.empty() || img.rows <= 0 || img.cols <= 0) {
        ROS_ERROR("VALID IMAGE, BUT STILL A PROBLEM EXISTS!");
        std::cout << "\timg.empty():" << img.empty() << std::endl;
        std::cout << "\timg.rows:" << img.rows << std::endl;
        std::cout << "\timg.cols:" << img.cols << std::endl;
        return determinedId;
    }

    // Preprocesing
    img = img(cv::Rect(0,0,640,420)); // Crop out the constant lip of the rover at the bottom

    // Black out coloured pixels (currently only walls)
    uint8_t* pixelPtr = (uint8_t*)img.data;
    int cn = img.channels();
    uint8_t bgrPixel[3];

    for(int i = 0; i < img.rows; i++) {
        for(int j = 0; j < img.cols; j++) {
            bgrPixel[0] = pixelPtr[i*img.cols*cn + j*cn + 0]; // B
            bgrPixel[1] = pixelPtr[i*img.cols*cn + j*cn + 1]; // G
            bgrPixel[2] = pixelPtr[i*img.cols*cn + j*cn + 2]; // R

            // Check if its greyscale (R=B=G), blank them if they aren't
            if ((bgrPixel[0] == bgrPixel[1]) && (bgrPixel[0] == bgrPixel[2])) continue;
            else {
                pixelPtr[i*img.cols*cn + j*cn + 0] = removeVal;
                pixelPtr[i*img.cols*cn + j*cn + 1] = removeVal;
                pixelPtr[i*img.cols*cn + j*cn + 2] = removeVal;
            } 
        }
    }

    // Remove sky
    // The sky is a uniform colour (in our sim 178, 178, 178) and always starts from the top until it is
    // interrupted by an object, none of which have pixels of that value along the top

    // skyVal is defined in the header, but it can be dynamically set before this is desired.

    for (int j = 0; j < img.cols; j++) {
        for (int i = 0; i < img.rows; i++) {
            // Go column by column from top to bottom until no longer in the sky

            // Check if its greyscale (R=B=G), blank them if they aren't
            if (pixelPtr[i*img.cols*cn + j*cn + 0] != skyVal) break; // Hit an object, go to next column
            else {
                pixelPtr[i*img.cols*cn + j*cn + 0] = removeVal;
                pixelPtr[i*img.cols*cn + j*cn + 1] = removeVal;
                pixelPtr[i*img.cols*cn + j*cn + 2] = removeVal;
            } 
        }
    }

    using namespace cv;
    using namespace cv::xfeatures2d;

    // Convert image from RGB to greyscale space
    cv::cvtColor(img, img, cv::COLOR_RGBA2GRAY, 0);

    //cv::imshow("Processed view. Press any key to continue.", img);

    //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
    Ptr<SURF> detector = SURF::create( minHessian ); // Defined in header
    std::vector<KeyPoint> keyPointsObject, keyPointsScene;
    Mat descriptorsScene;

    // Make features for the scene
    detector->detectAndCompute( img, noArray(), keyPointsScene, descriptorsScene );

    if (showInternals) {
        printf("\tScene has %d keypoints\n", (int) keyPointsScene.size());
    }

    float confidence[boxes.templates.size()];
    for (int tagID = 0; tagID < boxes.templates.size(); tagID++) {
        // File for current tag check
        Mat tagImageRaw = boxes.templates[tagID];
        Mat tagImage = tagImageRaw;

        GaussianBlur(tagImageRaw, tagImage, Size( 3, 3), 0, 0); // Add blur to aid feature matching
        //cv::imshow("Tag as used", tagImage);

        std::vector<DMatch> goodMatches;

        searchInScene(tagImage, descriptorsScene, keyPointsObject, goodMatches, detector);

        confidence[tagID] = (float)goodMatches.size() / (float)keyPointsObject.size();

        // Find object area in scene if there is a possibility of it being present

        float area = 0;

        if (confidence[tagID] > reqConfMinimum) {
            // Localize the object
            std::vector<Point2f> refPoints;
            std::vector<Point2f> scenePoints;
            for( size_t i = 0; i < goodMatches.size(); i++ ) {
                // Get the keypoints from the good matches
                refPoints.push_back( keyPointsObject[ goodMatches[i].queryIdx ].pt );
                scenePoints.push_back( keyPointsScene[ goodMatches[i].trainIdx ].pt );
            }

            // Determine transformation matrix of reference to scene pixels
            Mat H = findHomography(refPoints, scenePoints, RANSAC );

            // Check if there is a possible transform
            if (H.empty()) {
                // Failed to find a transform from reference to scene

                ROS_WARN("Unable to transform perspective using reference %d.", tagID + 1);
                confidence[tagID] = 0;

                // Code to try and interate until resolved
                //tagID--; 
                //continue;
            }
            else {
                // Tranform from refence to scene is possible

                // Get the corners from the reference image
                std::vector<Point2f> cornersInReference(4);
                cornersInReference[0] = Point2f(0, 0);
                cornersInReference[1] = Point2f((float)tagImage.cols, 0 );
                cornersInReference[2] = Point2f((float)tagImage.cols, (float)tagImage.rows );
                cornersInReference[3] = Point2f(0, (float)tagImage.rows );
                std::vector<Point2f> corInScene(4);

                // Apply transform to reference corners to transform into scene bounds
                cv::perspectiveTransform( cornersInReference, corInScene, H);
                
                // Calculate area of the region
                // (1/2) * {(x1y2 + x2y3 + x3y4 + x4y1) - (x2y1 + x3y2 + x4y3 + x1y4)}
                area = corInScene[0].x * corInScene[1].y + corInScene[1].x * corInScene[2].y +
                    corInScene[2].x * corInScene[3].y + corInScene[3].x * corInScene[0].y; 
                area = area - (corInScene[1].x * corInScene[0].y + corInScene[2].x * corInScene[1].y +
                    corInScene[3].x * corInScene[2].y + corInScene[0].x * corInScene[3].y);
                area = area / 2;
            }
        }

        if (area <= reqMinArea) confidence[tagID] = 0; // Nullify confidence if it isn't present
        else confidence[tagID] = confidence[tagID] * area * areaConfidenceFactor; // Add area to confidence
        // Area is added to prefer objects that are closer to rover but might have some of their features out of
        // frame, resulting in a lower "confidence" than a fully visible, but futher object in the background

        if (showInternals) {
            printf("Template %2d - Confidence %6.2f%% - KP %4d / %4d - Area %6.0f\n", 
                tagID + 1, confidence[tagID] * 100.0, (int)goodMatches.size(), (int) keyPointsObject.size(), area);
        }
    }
    
    // Find the most the ID and confidence of the two highest rated candidates
    float maxConfidence = 0.0, secondConfidence = 0.0;
    uint8_t maxID = 0, secondID = 0;

    for (uint8_t i = 0; i < boxes.templates.size(); i++) {
        float curConfidence = confidence[i];

        if (curConfidence > maxConfidence) {
            secondConfidence = maxConfidence;
            maxConfidence = curConfidence;

            secondID = maxID;
            maxID = i;
        }
        else if (curConfidence > secondConfidence) {
            secondConfidence =  curConfidence;
            secondID = i;
        }
    }
    
    // See if it is worth making a conclusion
    determinedId = 0;

    if (maxConfidence < reqConfMinimum) {
        // If there is no satisfactory option 
        ROS_INFO("Failed to find a match.");
    }
    else if ((maxConfidence > reqConfMinimum) && ((maxConfidence / secondConfidence) > reqConfRatio)) {
        determinedId = maxID + 1;

        if (showInternals) {
            ROS_INFO("Image contains %d, %.2f%% (%.2f) confidence", determinedId,
                maxConfidence * 100.0, (maxConfidence / secondConfidence));

            // Redo winning search
            std::vector<DMatch> goodMatches;
            searchInScene(boxes.templates[maxID], descriptorsScene, keyPointsObject, goodMatches, detector);

            // Show resulting matches
            Mat imgOfMatches = ImagePipeline::drawSceneMatches(img, boxes.templates[maxID], goodMatches, keyPointsObject, keyPointsScene);
            imshow("Selected match", imgOfMatches);

            cv::waitKey(250); // Wait until key pressed
        }
    }

    return determinedId;
}

void ImagePipeline::searchInScene(cv::Mat &tagImage, cv::Mat &descriptorsScene, std::vector<cv::KeyPoint> &keyPointsObject,
        std::vector<cv::DMatch> &goodMatches, cv::Ptr<cv::xfeatures2d::SURF> &detector) {
    
    using namespace cv;
    Mat descriptors_object;

    // Detect markers for the reference to find in the scene
    detector->detectAndCompute( tagImage, noArray(), keyPointsObject, descriptors_object );

    // Matching descriptor vectors with a FLANN based matcher
    // Since SURF is a floating-point descriptor NORM_L2 is used
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    std::vector< std::vector<DMatch> > knn_matches;
    matcher->knnMatch( descriptors_object, descriptorsScene, knn_matches, 2 );

    //-- Filter matches using the Lowe's ratio test
    const float ratio_thresh = 0.75f;
    for (size_t i = 0; i < knn_matches.size(); i++) {
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
            goodMatches.push_back(knn_matches[i][0]);
        }
    }
}

void ImagePipeline::loadImage(std::string fileLocation, bool printMessage) {
    // Replace image in pipeline with something else
    img = cv::imread(fileLocation, 1);
    isValid = true;

    if (printMessage) ROS_INFO("Image loaded from into video feed.\n\"%s\"", fileLocation.c_str());
}

cv::Mat ImagePipeline::drawSceneMatches(cv::Mat &scene, cv::Mat &tagImage, std::vector<cv::DMatch> &matches, 
    std::vector<cv::KeyPoint> &keyPointsRef, std::vector<cv::KeyPoint> &keyPointsScene){
    
    using namespace cv;

    //-- Draw matches
    Mat imageOfMatches; // Image with matches illustrated
    drawMatches(tagImage, keyPointsRef, scene, keyPointsScene, matches, imageOfMatches, Scalar::all(-1),
                    Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    //-- Localize the object
    std::vector<Point2f> refPoints;
    std::vector<Point2f> scenePoints;
    for( size_t i = 0; i < matches.size(); i++ ) {
        //-- Get the keypoints from the good matches
        refPoints.push_back( keyPointsRef[ matches[i].queryIdx ].pt );
        scenePoints.push_back( keyPointsScene[ matches[i].trainIdx ].pt );
    }

    Mat H = findHomography(refPoints, scenePoints, RANSAC );

    if (!H.empty()) {
        // Can preform transform from reference to scene

        float refImageCol = (float)tagImage.cols;

        //-- Get the corners from the image_1 ( the object to be "detected" )
        std::vector<Point2f> cornersInReference(4);
        cornersInReference[0] = Point2f(0, 0);
        cornersInReference[1] = Point2f( refImageCol, 0 );
        cornersInReference[2] = Point2f( refImageCol, (float)tagImage.rows );
        cornersInReference[3] = Point2f( 0, (float)tagImage.rows );
        std::vector<Point2f> corInScene(4);

        cv::perspectiveTransform( cornersInReference, corInScene, H);

        // Draw lines between the corners of the spotted object (reference) in the scene
        cv::line( imageOfMatches, corInScene[0] + Point2f(refImageCol, 0),
                corInScene[1] + Point2f(refImageCol, 0), Scalar(0, 255, 0), 4 );
        cv::line( imageOfMatches, corInScene[1] + Point2f(refImageCol, 0),
                corInScene[2] + Point2f(refImageCol, 0), Scalar( 0, 255, 0), 4 );
        cv::line( imageOfMatches, corInScene[2] + Point2f(refImageCol, 0),
                corInScene[3] + Point2f(refImageCol, 0), Scalar( 0, 255, 0), 4 );
        cv::line( imageOfMatches, corInScene[3] + Point2f(refImageCol, 0),
                corInScene[0] + Point2f(refImageCol, 0), Scalar( 0, 255, 0), 4 );
    }
    else {
        ROS_INFO("Can't draw matches");
    }
    
    return imageOfMatches;
}
