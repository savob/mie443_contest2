#include <imagePipeline.h>

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"

ImagePipeline::ImagePipeline(ros::NodeHandle& n, Boxes &boxes) {
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
    isValid = false;

    // Preprocress all tags and store in boxes so this only has to happen once
    for (int i = 0; i < boxes.templates.size(); i++) {
        tagPreprocess(boxes.templates[i]);
    }
}

void ImagePipeline::tagPreprocess(cv::Mat &tag) {
    cv::resize(tag,tag, cv::Size(500,400));               // Resize to roughly match aspect ratio on boxes
    cv::GaussianBlur(tag, tag, cv::Size( 3, 3), 0, 0);    // Add blur to aid feature matching
    //cv::imshow("Tag as used", tagImage); // Show image used in search
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

    // ============================================================
    // Preprocesing of incoming image
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

    // Convert image from RGB to greyscale space 
    // This should reduce memory usage and computation time
    cv::cvtColor(img, img, cv::COLOR_RGBA2GRAY, 0);

    // Remove sky
    // The sky is a uniform colour (in our sim 178, 178, 178) and always starts from the top until it is
    // interrupted by an object, none of which have pixels of that value along the top

    // skyVal is defined in the header, but it can be dynamically set before this is desired.
    pixelPtr = (uint8_t*)img.data; // Update the pointer before this loop
    for (int j = 0; j < img.cols; j++) {
        for (int i = 0; i < img.rows; i++) {
            // Go column by column from top to bottom until no longer in the sky

            // Check if its greyscale (R=B=G), blank them if they aren't
            if (pixelPtr[i*img.cols + j] != skyVal) break; // Hit an object, go to next column
            else {
                pixelPtr[i*img.cols + j] = removeVal;
            } 
        }
    }

    //cv::imshow("Processed view. Press any key to continue.", img); // Show result of preprocessing

    // ============================================================
    // Setup scan of image
    using namespace cv;
    using namespace cv::xfeatures2d;

    // Setup the SURF detector for features in images and associated data
    Ptr<SURF> detector = SURF::create( minHessian ); // Defined in header
    std::vector<KeyPoint> keyPointsObject, keyPointsScene;
    Mat descriptorsScene;

    // Determine features for the scene before looping through options
    detector->detectAndCompute( img, noArray(), keyPointsScene, descriptorsScene );
    if (showInternals) {
        printf("\tScene has %d keypoints\n", (int) keyPointsScene.size());
    }

    // Find the ID and confidence levels of the two highest rated candidates
    float maxConfidence = 0.0, secondConfidence = 0.0;
    uint8_t maxID = 0;
    Mat bestTag; // Stores the best matched reference tag for display purposes

    // ============================================================
    // Loop through all possible tags
    for (int tagID = 0; tagID < boxes.templates.size(); tagID++) {

        // Load reference to tag from boxes (just to help simplfy the code that follows)
        Mat &tagImage = boxes.templates[tagID];

        // See what portion of features from the reference are matched in the scene
        std::vector<DMatch> goodMatches;
        searchInScene(tagImage, descriptorsScene, keyPointsObject, goodMatches, detector);
        float confidence = (float)goodMatches.size() / (float)keyPointsObject.size();

        // ============================================================
        // Investigate futher if initial confidence is good
        float area = 0; // Area object takes up in scene (pixels)
        if (confidence > reqConfMinimum) {
            // Localize the object
            std::vector<Point2f> refPoints;
            std::vector<Point2f> scenePoints;
            for(int i = 0; i < goodMatches.size(); i++) {
                // Get the keypoints from the good matches
                refPoints.push_back(keyPointsObject[goodMatches[i].queryIdx].pt);
                scenePoints.push_back(keyPointsScene[goodMatches[i].trainIdx].pt);
            }

            // Determine transformation matrix of reference to scene pixels
            Mat H = findHomography(refPoints, scenePoints, RANSAC);

            // Check if there is a possible transform
            if (H.empty()) {
                // Failed to find a transform from reference to scene
                ROS_WARN("Unable to transform perspective using reference %d.", tagID + 1);
                confidence = 0; // It's a bad match
            }
            else {
                // Tranform from refence image to scene is possible

                // Get the corners from the reference image
                std::vector<Point2f> cornersInReference(4);
                cornersInReference[0] = Point2f(0, 0);
                cornersInReference[1] = Point2f((float)tagImage.cols, 0 );
                cornersInReference[2] = Point2f((float)tagImage.cols, (float)tagImage.rows );
                cornersInReference[3] = Point2f(0, (float)tagImage.rows );
                std::vector<Point2f> corInScene(4);

                // Apply transform to reference corners to transform into scene bounds
                cv::perspectiveTransform( cornersInReference, corInScene, H);
                
                // Check if the corners are not "tangled" before calculating area (forming a "bow" shape (invalid))
                if (checkTangledBox(corInScene) == false) {
                    // Calculate area of the region
                    // (1/2) * [(x1y2 + x2y3 + x3y4 + x4y1) - (x2y1 + x3y2 + x4y3 + x1y4)]
                    area = corInScene[0].x * corInScene[1].y + corInScene[1].x * corInScene[2].y +
                        corInScene[2].x * corInScene[3].y + corInScene[3].x * corInScene[0].y; 
                    area = area - (corInScene[1].x * corInScene[0].y + corInScene[2].x * corInScene[1].y +
                        corInScene[3].x * corInScene[2].y + corInScene[0].x * corInScene[3].y);
                    area = area / 2;
                }
            }
        }

        // ============================================================
        // Look to record this match if it's worthy

        if (area <= reqMinArea) confidence = 0; // Nullify confidence if it isn't present
        else confidence = confidence * area * areaConfidenceFactor; // Add area to confidence
        // Area is added to prefer objects that are closer to rover but might have some of their features out of
        // frame, resulting in a lower "confidence" than a fully visible, but futher object in the background

        if (showInternals) {
            printf("Template %2d - Confidence %6.2f%% - KP %4d / %4d - Area %6.0f\n", 
                tagID + 1, confidence * 100.0, (int)goodMatches.size(), (int) keyPointsObject.size(), area);
        }

        // See how this compares to previous cases
        if (confidence > maxConfidence) {
            // New best
            secondConfidence = maxConfidence;
            maxConfidence = confidence;

            // Record values needed outside the loop
            maxID = tagID;
            bestTag = tagImage.clone();
        }
        else if (confidence > secondConfidence) {
            // Record second place confidence for ratio comparison later
            secondConfidence = confidence;
        }
    }
    
    // ============================================================
    // Process the results of the scan
    determinedId = 0; // Default (inconclusive scan, but at least no error!)

    if (maxConfidence < reqConfMinimum) {
        // If there is no satisfactory option 
        ROS_INFO("Failed to find a match.");
    }
    else if ((maxConfidence > reqConfMinimum) && ((maxConfidence / secondConfidence) > reqConfRatio)) {
        determinedId = maxID + 1; // Add one to match file names and to allow 0 to be used as a fail code

        ROS_INFO("Image contains %d, %.2f%% (%.2f) confidence", determinedId,
            maxConfidence * 100.0, (maxConfidence / secondConfidence));

        if (showInternals) {
            // Redo winning search
            std::vector<DMatch> goodMatches;
            searchInScene(bestTag, descriptorsScene, keyPointsObject, goodMatches, detector);

            // Show resulting matches
            Mat imgOfMatches = ImagePipeline::drawSceneMatches(img, bestTag, goodMatches, keyPointsObject, keyPointsScene);
            imshow("Selected match", imgOfMatches);

            cv::waitKey(250); // Wait until any key is pressed or 250ms pass
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

    // Filter matches using the Lowe's ratio test
    const float ratio_thresh = 0.75;
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

    if (printMessage) ROS_INFO("Image loaded from into video feed.\n\t\"%s\"", fileLocation.c_str());
}

cv::Mat ImagePipeline::drawSceneMatches(cv::Mat &scene, cv::Mat &tagImage, std::vector<cv::DMatch> &matches, 
    std::vector<cv::KeyPoint> &keyPointsRef, std::vector<cv::KeyPoint> &keyPointsScene){
    
    using namespace cv;

    // Draw matches
    Mat imageOfMatches; // Image with matches illustrated
    drawMatches(tagImage, keyPointsRef, scene, keyPointsScene, matches, imageOfMatches, Scalar::all(-1),
                    Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    // Localize the object
    std::vector<Point2f> refPoints;
    std::vector<Point2f> scenePoints;
    for( size_t i = 0; i < matches.size(); i++ ) {
        // Get the keypoints from the good matches
        refPoints.push_back( keyPointsRef[ matches[i].queryIdx ].pt );
        scenePoints.push_back( keyPointsScene[ matches[i].trainIdx ].pt );
    }

    Mat H = findHomography(refPoints, scenePoints, RANSAC );

    if (!H.empty()) {
        // Can preform transform from reference to scene

        float refImageCol = (float)tagImage.cols;

        // Get the corners from the reference image (the object to be "detected")
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
        ROS_WARN("Can't draw matches. Corners cannot be transformed.");
    }
    
    return imageOfMatches;
}

bool ImagePipeline::checkTangledBox(std::vector<cv::Point2f> corners) {
    
    // Check that line between 1 and 4 does not cross 23
    bool tempA = checkAbove(corners[0],corners[1],corners[2]);
    bool tempB = checkAbove(corners[3],corners[1],corners[2]);
    bool result = tempA == tempB; // Store if both points fall on the same side (not tangled)

    // Check that line 12 does not cross 34
    tempA = checkAbove(corners[0],corners[3],corners[2]);
    tempB = checkAbove(corners[1],corners[3],corners[2]);
    result = result && (tempA == tempB); // Update result to ensure no sets of lines intersect

    return !result; // Return true if the system IS tangled
}

bool ImagePipeline::checkAbove(cv::Point2f test, cv::Point2f a, cv::Point2f b) {
    // Define line between a and b
    float gradient = (a.y - b.y) / (a.x - b.x);

    // Linearly extrapolate line between a and b to test point
    float dx = test.x - a.x;
    float estimateY = a.y + dx * gradient;

    // Return if the point lies above the line or not
    return test.y > estimateY;
}