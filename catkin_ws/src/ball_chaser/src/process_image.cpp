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
        
    // define movement speed and turning speed in radians
    float forward_move = 1.0;
    
    float stop = 0.0;

    //Turning speeds
    float turn_left = -0.75;
    float turn_right = 0.75;

    // Loop through each pixel in the image and check if there's a bright white one
    // 
    
    

    for (int i=0;i < img.height * img.step; i += 3)
    {   

        int red_pixel = img.data[i];
        int green_pixel = img.data[i+1];
        int blue_pixel = img.data[i+2];
        int pixel_position = i % (img.width * 3) / 3;

        // left and right divider values were determined based on observations on width outputs
        // when ball was detected
        int left_divider = 265;
        int right_divider = 533;


        if (red_pixel == 255 && green_pixel == 255 && blue_pixel == 255)
        {
            
            ROS_INFO("Found white ball");
            // std::cout<<"at position index: "<<pixel_position;
            // std::cout<<"found red: \n" << red_pixel;

            if (pixel_position >1 && pixel_position <= left_divider)
            {
                // std::cout<<"Found white ball in Left";
                ROS_INFO("Target is left, driving left");
                drive_robot(stop,turn_left);
                if (pixel_position >left_divider && pixel_position <= right_divider)
                {
                    ROS_INFO("Pixel is now centered");
                    drive_robot(stop,stop);
                }
                
            }
            else if (pixel_position > right_divider && pixel_position < img.width)
            {
                
                ROS_INFO("Target is right, driving right");
                drive_robot(stop,turn_right);

                if (pixel_position >left_divider && pixel_position <= right_divider)
                {
                    ROS_INFO("Pixel is now centered");
                    drive_robot(stop,stop);
                    ROS_INFO("position: %f", pixel_position);        
                }
                
            }
            else if (pixel_position >left_divider && pixel_position <= right_divider)
            {
                ROS_INFO("Target straight ahead, proceed forward");                                     
                drive_robot(forward_move,stop);
               
            }
        }     
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
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 2, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
