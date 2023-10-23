#include <ros/ros.h>

// Publish to a topic with this message type
#include <ackermann_msgs/AckermannDriveStamped.h>
// AckermannDriveStamped messages include this message type
#include <ackermann_msgs/AckermannDrive.h>

// Subscribe to a topic with this message type
#include <nav_msgs/Odometry.h>
// Subscribe to a topic with this message type
#include <sensor_msgs/LaserScan.h>

// for printing
#include <iostream>

// for RAND_MAX
#include <cstdlib>

class FollowTheGap
{
private:
    // A ROS node
    ros::NodeHandle n;

    // car parameters
    double max_speed, max_steering_angle;

    // Listen for odom messages
    ros::Subscriber odom_sub;

    // Listen for laser scan messages
    ros::Subscriber laser_sub;

    // Publish drive data
    ros::Publisher drive_pub;

    // previous desired steering angle
    double prev_angle = 0.0;

public:
    FollowTheGap()
    {
        // Initialize the node handle
        n = ros::NodeHandle("~");

        // get topic names
        std::string drive_topic, odom_topic, scan_topic;
        n.getParam("gap_drive_topic", drive_topic);
        n.getParam("odom_topic", odom_topic);
        n.getParam("scan_topic", scan_topic);

        // get car parameters
        n.getParam("max_speed", max_speed);
        n.getParam("max_steering_angle", max_steering_angle);

        // Make a publisher for drive messages
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);
        //Check to see if running
        std::cout << "Running: 2 " << std::endl;
        // Start a subscriber to listen to odom messages
        odom_sub = n.subscribe(odom_topic, 1, &FollowTheGap::odom_callback, this);
        // Start a subscriber to listen to laser scan messages
        laser_sub = n.subscribe(scan_topic, 1, &FollowTheGap::laser_callback, this);
    }

    void odom_callback(const nav_msgs::Odometry &msg)
    {
        // publishing is done in odom callback just so it's at the same rate as the sim

        //std::cout << "Running: 12 " << std::endl;
        /// SPEED CALCULATION:
        // set constant speed to be half of max speed
        //drive_msg.speed = max_speed / 2;

        // Publish the drive message
        //drive_st_msg.drive = drive_msg;

        //drive_pub.publish(drive_st_msg);
    }
    void laser_callback(const sensor_msgs::LaserScan &msg)
    {
        // initialize message to be published
        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;
        /// STEERING ANGLE CALCULATION

        // check all angles
        double best_angle = 0.00;
        double furthest_point = 0.00;
        double best_area = 0.00;
        int j = 300; // must be even

        for (size_t i = 0; i < msg.ranges.size() - j; i++)
        {
            double angle_1 = msg.angle_min + i * msg.angle_increment;
            double angle_2 = msg.angle_min + (i + j) * msg.angle_increment;
            double angle = angle_2 - angle_1;
            double area = (msg.ranges[i] * msg.ranges[i + j] * cos(angle)) / 2;
            if (area > best_area)
            {
                best_area = area;
                best_angle = (angle_1 + angle_2) / 2;
                furthest_point = msg.ranges[((i + j) / 2)];
            }
            
        }
        // Check if the furthest point is within a safe distance
        double safe_distance = 0.5;
        if (furthest_point < safe_distance)
        {
            // Publish a steering angle that will avoid the obstacle
            best_angle = -best_angle;
        }
        double ran = (rand() % 5) + 1;
        publish_to_drive(ran, best_angle);
        //std::cout<<"Speed: "<< ran;
        return;
    }

    void publish_to_drive(double desired_velocity, double desired_steer)
    {
        // This will take in a desired velocity and steering angle and make and publish an
        // AckermannDriveStamped message to the /drive topic

        // Make and publish message
        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;
        std_msgs::Header header;
        drive_msg.speed = desired_velocity;
        drive_msg.steering_angle = desired_steer;
        header.stamp = ros::Time::now();

        drive_st_msg.header = header;

        // set drive message in drive stamped message
        drive_st_msg.drive = drive_msg;

        // publish AckermannDriveStamped message to drive topic
        drive_pub.publish(drive_st_msg);
    }
}; // end of class definition

int main(int argc, char **argv)
{
    ros::init(argc, argv, "follow_the_gap");
    std::cout << "Running: 1 " << std::endl;
    FollowTheGap fg;
    ros::spin();
    return 0;
}
