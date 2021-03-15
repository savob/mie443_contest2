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

int ImagePipeline::getTemplateID(Boxes& boxes) {
    int template_id = -1;

    if(!isValid) {
        ROS_ERROR("INVALID IMAGE!");
        return template_id;
    } else if(img.empty() || img.rows <= 0 || img.cols <= 0) {
        ROS_ERROR("VALID IMAGE, BUT STILL A PROBLEM EXISTS!");
        std::cout << "\timg.empty():" << img.empty() << std::endl;
        std::cout << "\timg.rows:" << img.rows << std::endl;
        std::cout << "\timg.cols:" << img.cols << std::endl;
        return template_id;
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
    int minHessian = 400;
    Ptr<SURF> detector = SURF::create( minHessian );
    std::vector<KeyPoint> keypoints_object, keypoints_scene;
    Mat descriptors_object, descriptors_scene;

    // File for current tag check
    String tagFile = "/home/brobot/catkin_ws/src/mie443_contest2/boxes_database/tag_14.jpg";
    Mat tagImage = imread(tagFile, IMREAD_GRAYSCALE);
    //cv::imshow("Tag checked", tagImage);

    // Detect markers for the tag and the scene(img)
    detector->detectAndCompute( tagImage, noArray(), keypoints_object, descriptors_object );
    detector->detectAndCompute( img, noArray(), keypoints_scene, descriptors_scene );

    //-- Step 2: Matching descriptor vectors with a FLANN based matcher
    // Since SURF is a floating-point descriptor NORM_L2 is used
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    std::vector< std::vector<DMatch> > knn_matches;
    matcher->knnMatch( descriptors_object, descriptors_scene, knn_matches, 2 );
    //-- Filter matches using the Lowe's ratio test
    const float ratio_thresh = 0.75f;
    std::vector<DMatch> good_matches;
    for (size_t i = 0; i < knn_matches.size(); i++)
    {
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
        {
            good_matches.push_back(knn_matches[i][0]);
        }
    }

    //-- Draw matches
    Mat img_matches;
    drawMatches(tagImage, keypoints_object, img, keypoints_scene, good_matches, img_matches, Scalar::all(-1),
                    Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    //-- Localize the object
    std::vector<Point2f> obj;
    std::vector<Point2f> scene;
    for( size_t i = 0; i < good_matches.size(); i++ )
    {
        //-- Get the keypoints from the good matches
        obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
        scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
    }

    Mat H = findHomography( obj, scene, RANSAC );
    //-- Get the corners from the image_1 ( the object to be "detected" )
    std::vector<Point2f> obj_corners(4);
    obj_corners[0] = Point2f(0, 0);
    obj_corners[1] = Point2f( (float)tagImage.cols, 0 );
    obj_corners[2] = Point2f( (float)tagImage.cols, (float)tagImage.rows );
    obj_corners[3] = Point2f( 0, (float)tagImage.rows );
    std::vector<Point2f> scene_corners(4);
    perspectiveTransform( obj_corners, scene_corners, H);
    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
    line( img_matches, scene_corners[0] + Point2f((float)tagImage.cols, 0),
          scene_corners[1] + Point2f((float)tagImage.cols, 0), Scalar(0, 255, 0), 4 );
    line( img_matches, scene_corners[1] + Point2f((float)tagImage.cols, 0),
          scene_corners[2] + Point2f((float)tagImage.cols, 0), Scalar( 0, 255, 0), 4 );
    line( img_matches, scene_corners[2] + Point2f((float)tagImage.cols, 0),
          scene_corners[3] + Point2f((float)tagImage.cols, 0), Scalar( 0, 255, 0), 4 );
    line( img_matches, scene_corners[3] + Point2f((float)tagImage.cols, 0),
          scene_corners[0] + Point2f((float)tagImage.cols, 0), Scalar( 0, 255, 0), 4 );

    imshow("Good Matches & Object detection", img_matches );

    // Use: boxes.templates
    
    cv::waitKey(0); // Wait until key pressed
  
    return template_id;
}

void ImagePipeline::loadImage(char* fileLocation) {
    // Replace image in pipeline with something else
    img = cv::imread(fileLocation, 1);
    isValid = true;

    //cv::imshow("Loaded image", img);
    cv::waitKey(10);
}
