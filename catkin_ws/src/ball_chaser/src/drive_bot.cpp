#include "ball_chaser/DriveToTarget.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

constexpr char kNodeName[] = "drive_bot";
constexpr char kServiceName[] = "/ball_chaser/command_robot";
constexpr char kPublishTopic[] = "/cmd_vel";
constexpr char kSubscribeTopic[] = "/cmd_vel";
constexpr int kPublishQueueSize = 10;
constexpr int kSubscribeQueueSize = 1;

class SubscribeAndPublish {
 public:
  SubscribeAndPublish() {
    // Inform ROS master that we will be publishing a message of type `geometry_msgs::Twist` on the `kPublishTopic`
    // topic with a publishing queue size of `kPublishQueueSize`
    pub_ = n_.advertise<geometry_msgs::Twist>(kPublishTopic, kPublishQueueSize);

    // Topic you want to subscribe
    sub_ = n_.subscribe(kSubscribeTopic, kSubscribeQueueSize, &SubscribeAndPublish::SubscribeCallback, this);

    // Define a `kServiceName` service with a `HandleDriveRequest` callback function
    service_ = n_.advertiseService(kServiceName, &SubscribeAndPublish::HandleDriveRequest, this);
  }

  /**
   * @brief Callback function that executes whenever a `drive_bot` service is requested.
   *
   * @param req Message containing the requested linear x and angular velocities to be published to the robot wheel
   * joints
   * @param res Message feedback with the requested wheel velocities
   * @return `true`
   */
  bool HandleDriveRequest(ball_chaser::DriveToTarget::Request& req,     // NOLINT(runtime/references)
                          ball_chaser::DriveToTarget::Response& res) {  // NOLINT(runtime/references)
    geometry_msgs::Twist motor_command;

    motor_command.linear.x = req.linear_x;
    motor_command.angular.z = req.angular_z;

    // Publish the requested commands to the robot wheel joints
    pub_.publish(motor_command);

    // Fill in the response message
    res.msg_feedback = "motor_command.linear.x = " + std::to_string(motor_command.linear.x) +
                       ", motor_command.angular.z = " + std::to_string(motor_command.angular.z);
    ROS_INFO("%s", res.msg_feedback.c_str());
    return true;
  }

  /**
   * @brief Subscriber callback -- this is just a placeholder.
   *
   * @param input Unused
   */
  void SubscribeCallback(const geometry_msgs::Twist& input) {
    // This is just a placeholder
    (void)input;

    // Do something with the input and generate the output...
    // geometry_msgs::Twist output;
    // pub_.publish(output);
  }

 private:
  /// ROS NodeHandle object
  ros::NodeHandle n_;

  /// Publisher
  ros::Publisher pub_;

  /// Subscriber
  ros::Subscriber sub_;

  // Service
  ros::ServiceServer service_;
};

int main(int argc, char** argv) {
  // Initialize a ROS node
  ros::init(argc, argv, kNodeName);

  // Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}
