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

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
    // Declaring variables
    int i, row;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

    // Dividing image into three sections, equal size, dividing it are the left and right dividers

    int left_divider = img.step / 3;
    int right_divider = img.step / 3;
    bool ball_detection = false;
    // define movement speed and turning speed in radians
    float forward_move = 0.5;
    float turn_speed = 0.5;
    float stop = 0.0;


    float turn_left = 1.0;
    float turn_right = -1.0;

    // 
    if (ball_detection = true)
    {   
        ROS_INFO("Target acquired");
        for (int i=0; i < img.height * img.step; i)
        {
            // ToDo Determine if ball is left, right or center frame
            if (white_pixel == img.data[i] && i%img.step < left_divider)
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
    else
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