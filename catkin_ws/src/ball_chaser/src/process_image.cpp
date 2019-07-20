#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

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

    int white_pixel = 255;
    // Declaring variables
    int i, row;

    // Dividing image into three sections, equal size, dividing it are the left and right dividers

    int left_divider = img.step / 3;
    int right_divider = left_divider * 2;
    bool ball_detection = false;
    // define movement speed and turning speed in radians
    float forward_move = 1.0;
    float turn_speed = 0.25;
    float stop = 0.0;


    float turn_left = 2.0;
    float turn_right = -2.0;

    // Loop through each pixel in the image and check if there's a bright white one

    ROS_INFO("Target acquired");
    for (int i=0; i < img.height * img.step; ++i)
    {   
        
        if (white_pixel == img.data[i])
        {
            ball_detection == true;
            
            // Identify if this pixel falls in three cases: left, middle, or right side of the image  
            // and calling the drive_bot function to pass velocities to it
            
            if (i%img.step < left_divider)
            {
                ROS_INFO("Target is left, driving left");
                drive_robot(turn_speed,turn_left);
    
            }
            
            else if (i%img.step > right_divider)
            {
                ROS_INFO("Target is right, driving right");
                drive_robot(turn_speed,turn_right);
            }
            
            else 
            {
                ROS_INFO("Target straight ahead, proceed forward");
                drive_robot(forward_move,stop);
            }
        }
    }
    // Request a stop when there's no white ball seen by the camera
    if (ball_detection == false)
    {
        ROS_INFO("Target lost, standing by");
        drive_robot(stop,stop);
    }

}


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
