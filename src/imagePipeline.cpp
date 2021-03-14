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

    for(int i = 0; i < img.rows; i++)
    {
        for(int j = 0; j < img.cols; j++)
        {
            bgrPixel[0] = pixelPtr[i*img.cols*cn + j*cn + 0]; // B
            bgrPixel[1] = pixelPtr[i*img.cols*cn + j*cn + 1]; // G
            bgrPixel[2] = pixelPtr[i*img.cols*cn + j*cn + 2]; // R

            // Check if its greyscale (R=B=G), blank them if they aren't
            if ((bgrPixel[0] == bgrPixel[1]) && (bgrPixel[0] == bgrPixel[2])) continue;
            else {
                pixelPtr[i*img.cols*cn + j*cn + 0] = 0;
                pixelPtr[i*img.cols*cn + j*cn + 1] = 0;
                pixelPtr[i*img.cols*cn + j*cn + 2] = 0;
            } 
        }
    }

        
    // Use: boxes.templates
    cv::imshow("Processed view. Press any key to continue.", img);
    cv::waitKey(0); // Wait until key pressed
  
    return template_id;
}

void ImagePipeline::loadImage(char* fileLocation) {
    // Replace image in pipeline with something else
    img = cv::imread(fileLocation, 1);
    isValid = true;

    cv::imshow("Loaded image", img);
    cv::waitKey(10);
}
