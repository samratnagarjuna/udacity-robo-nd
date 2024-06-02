#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float linear_x, float angular_z)
{
    // Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = linear_x;
    srv.request.angular_z = angular_z;
    
    if (!client.call(srv))
        ROS_ERROR("Failed to call service drive_bot");
}

uint32_t get_loc_index(uint32_t column_index, uint32_t total_columns) {
    const uint32_t width = total_columns / 3;
    if (column_index < width) return 0;
    if (column_index < width * 2) return 1;
    return 2;
}

uint32_t get_ball_loc(uint32_t (&white_pixel_map)[3]) {
    uint32_t max_index = 0;
    uint32_t max_value = white_pixel_map[0];
    for (int i = 1; i < 3; ++i) {
        if (white_pixel_map[i] > max_value) {
            max_value = white_pixel_map[i];
            max_index = i;
        }
    }
    return max_index;
}

bool is_white_pixel(const sensor_msgs::Image& img, uint32_t index) {
    constexpr uint8_t FULL_COLOR = 255;
    return img.data[index] == FULL_COLOR 
        && img.data[index + 1] == FULL_COLOR 
        && img.data[index + 2] == FULL_COLOR;
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    
    uint32_t white_pixel_map[3] = {0, 0, 0};
    bool ball_exists = false;

    // Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

    for (uint32_t r = 0; r < img.height; ++r) {
        for (uint32_t c = 0; c < img.width; ++c) {
            const uint32_t index = r * img.step + c * 3;
            if (is_white_pixel(img, index)) {
                ++white_pixel_map[get_loc_index(c, img.width)];
                ball_exists = true;
            }
        }
    }
    
    if (ball_exists) {
        uint32_t ball_loc = get_ball_loc(white_pixel_map);
        if (ball_loc == 1) {
            drive_robot(0.1f, 0.0f);
        }
        else if (ball_loc == 0) {
            drive_robot(0.0f, 0.1f);
        }
        else if (ball_loc == 2) {
            drive_robot(0.0f, -0.1f);
        }
    }
    else drive_robot(0.0f, 0.0f);
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
