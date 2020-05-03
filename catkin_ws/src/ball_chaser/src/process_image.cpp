#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "ball_chaser/DriveToTarget.h"

constexpr char kNodeName[] = "process_image";
constexpr char kServiceName[] = "/ball_chaser/command_robot";
constexpr char kSubscribeTopic[] = "/camera/rgb/image_raw";
constexpr int kSubscribeQueueSize = 10;

/// The four driving commands that we will pass to the robot
enum MovingDirection { STOP, LEFT, FORWARD, RIGHT };

class SubscribeAndPublish {
 public:
  SubscribeAndPublish() {
    // Subscribe to `kSubscribeTopic` topic to read the image data inside the `ProcessImageCallback` function
    sub_ = n_.subscribe(kSubscribeTopic, kSubscribeQueueSize, &SubscribeAndPublish::ProcessImageCallback, this);

    // Define a client that can request services
    client_ = n_.serviceClient<ball_chaser::DriveToTarget>(kServiceName);
  }

 private:
  /**
   * @brief This callback function continuously executes and reads the image data and tells the robot to drive in the
   * appropriate direction.
   *
   * @param img The image from the "/camera/rgb/image_raw" topic
   */
  void ProcessImageCallback(const sensor_msgs::Image& img) {
    const uint8_t white_pixel = 255;

    // Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    const int step_over_width = img.step / img.width;
    for (int row = 0; row < img.height; ++row) {
      for (int col = 0; col < img.step; col += step_over_width) {
        // If we see a white ball, drive the robot in its direction
        if (img.data[row * img.step + col] == white_pixel && img.data[row * img.step + col + 1] == white_pixel &&
            img.data[row * img.step + col + 2]) {
          if (col < 0.25 * img.step) {
            DriveRobot(LEFT);
            return;
          }
          if (col < 0.75 * img.step) {
            DriveRobot(FORWARD);
            return;
          }
          DriveRobot(RIGHT);
          return;
        }
      }
    }

    // We did not see a white ball, so the robot should stop
    DriveRobot(STOP);
  }

  /**
   * @brief This function calls the "command_robot" service to drive the robot in the specified direction
   *
   * @param direction The direction in which we want to drive: `STOP`, `LEFT`, `FORWARD`, or `RIGHT`
   */
  void DriveRobot(const MovingDirection& direction) {
    if (direction == direction_) {
      return;
    }

    ball_chaser::DriveToTarget srv;
    switch (direction) {
      case LEFT:
        srv.request.linear_x = 0.;
        srv.request.angular_z = 0.5;
        break;
      case FORWARD:
        srv.request.linear_x = 0.5;
        srv.request.angular_z = 0.;
        break;
      case RIGHT:
        srv.request.linear_x = 0.;
        srv.request.angular_z = -0.5;
        break;
      case STOP:
      default:
        srv.request.linear_x = 0.;
        srv.request.angular_z = 0.;
        break;
    }

    // Call the service
    if (!client_.call(srv)) {
      ROS_ERROR("Failed to call service %s", kServiceName);
    } else {
      direction_ = direction;
    }
  }

  /// ROS NodeHandle object
  ros::NodeHandle n_;

  /// Subscriber
  ros::Subscriber sub_;

  // Service client that can request services
  ros::ServiceClient client_;

  /// The direction the robot is currently moving
  MovingDirection direction_ = STOP;
};

int main(int argc, char** argv) {
  // Initialize the process_image node and create a handle to it
  ros::init(argc, argv, kNodeName);

  // Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  // Handle ROS communication events
  ros::spin();

  return 0;
}
