#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <iostream>

 

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;

    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if ( !client.call(srv) )
    {
        ROS_ERROR( "Failed to call service ball_chaser" );
    }
}

// Callback function to continuously read image data and execute drive_robot command 
void process_image_callback(const sensor_msgs::Image img)
{

    // Declaring variables
    int red_pixel, green_pixel, blue_pixel, i, row;
    int left_divider = img.step / 3;
    int right_divider = left_divider * 2;
    //Set initial ball existance
    bool ball_detection = false;
    
    // define movement speed and turning speed in radians
    float forward_move = 1.0;
    float turn_speed = 0.25;
    float stop = 0.0;

    //Turning speeds
    float turn_left = -2.0;
    float turn_right = 2.0;

    // Loop through each pixel in the image and check if there's a bright white one
    // 
    
    for (int i=0;i < img.height * img.step; ++i)
    {   
        
        int red_pixel = img.data[i];
        int green_pixel = img.data[i+1];
        int blue_pixel = img.data[i+2];

        if (i%img.step < left_divider && red_pixel == 255 && green_pixel == 255 && blue_pixel == 255)
        {
        
            ROS_INFO("Target acquired");
            
            ROS_INFO("Target is left, driving left");
            drive_robot(stop,turn_left);
            // drive_robot(turn_left,turn_speed);
        }
            
        else if (i%img.step > right_divider && red_pixel == 255 && green_pixel == 255 && blue_pixel == 255)
        {
            ROS_INFO("Target is right, driving right");
            drive_robot(stop,turn_right);
            // drive_robot(turn_right,turn_speed);
        }
        
        else if (red_pixel == 255 && green_pixel == 255 && blue_pixel == 255)
        {
            ROS_INFO("Target straight ahead, proceed forward");
            drive_robot(forward_move,stop);
        }

    // Request a stop when there's no white ball seen by the camera
        else
        {
            ROS_INFO("Target lost, standing by");
            drive_robot(stop,stop);
        }
    }
}
        // else
        // {
        // 	int mean_scan_position = scan_position_sum / white_pixel_count;
        // 	// Check if pixels are on left frame
        // 	if (mean_scan_position < img.width /3)
        // 	{
        // 		ROS_INFO("turning left, moving forward");
        // 		drive_robot(forward_move, turn_left);

        // 	}
        // 	// Check if pixels are on right frame
        // 	else if (mean_scan_position > img.width * 2/3)
        // 	{
        // 		ROS_INFO("turning right, moving forward");
        // 		drive_robot(forward_move, turn_right);
        // 	}
        // 	else
        // 	{
        // 		ROS_INFO("Target ahead, driving forward");
        // 		drive_robot(forward_move, 0.0);
        // 	}



int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
