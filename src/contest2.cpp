#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <dirent.h>         // Used for reading in the test files

int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    // Robot pose object + subscriber.
    RobotPose robotPose(0,0,0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    // Initialize box coordinates and templates
    Boxes boxes; 
    if(!boxes.load_coords() || !boxes.load_templates()) {
        ROS_FATAL("Could not load coords or templates");
        return -1;
    }
    else {
        ROS_INFO("Box coordinates loaded successfully:");

        for(int i = 0; i < boxes.coords.size(); ++i) {
            std::cout << i << "\tx: " << boxes.coords[i][0] << "\ty: " << boxes.coords[i][1] << "\tz: " 
                    << boxes.coords[i][2] << std::endl;
        }
    }

    // Initialize image object and subscriber.
    ImagePipeline imagePipeline(n);
    // Execute strategy.
    while(ros::ok()) {
        ros::spinOnce();
        /***YOUR CODE HERE***/
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi


        // Vision stuff past here, no touchy
        
        // Decide to include vision test or not (comment the #define in the image header) 
#ifdef TESTING_VISION_SAMPLES 

        ROS_INFO("\n\nRUNNING VISION TEST\n(will terminate once complete)\n");

        // Test parameters
        std::string testFileFolder = "/home/brobot/catkin_ws/src/mie443_contest2/testpics/";
        bool printInnerWorks = true;
        std::string searchTerm = "dog";
        // Leave as "" for all files in folder (not recommended since too many consecutive searches results in errors)

        // Load in all test files
        std::vector<std::string> fileNames;

        DIR *dr;
        struct dirent *en;
        dr = opendir(testFileFolder.c_str()); // Open directory
        if (dr) {
            while ((en = readdir(dr)) != NULL) {
                std::string temp = en->d_name; // Grab file names

                // Add files that end in PNG and contain search term
                if (temp.find(".png") != std::string::npos) {
                    if (temp.find(searchTerm) != std::string::npos) {
                        fileNames.push_back(temp); 
                    }
                }
            }
            closedir(dr); // Close directory
        }

        std::sort (fileNames.begin(), fileNames.end()); // Sort files alphabetically
        int result[fileNames.size()];

        // Go through each test file to ID
        for (int i = 0; i < fileNames.size(); i++) {
            std::string testFile = testFileFolder + fileNames[i];
            imagePipeline.loadImage(testFile);
            result[i] = imagePipeline.getTemplateID(boxes, printInnerWorks);
        }

        // Print result summary
        printf("\nResults of analysis, ID# and file name.\n");
        for (int i = 0; i < fileNames.size(); i++) {
            printf("ID %2d: %s\n", result[i], fileNames[i].c_str());
        }

        return 0; // Only run this all once
#else
        ROS_INFO_ONCE("\n\nNOT RUNNING VISION TEST\n\n"); // Inform user that vision test is not being run
#endif
        
        // End of vision stuff

        ros::Duration(1).sleep(); // Two second sleep per step
    }
    return 0;
}
