#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Header.h>
#include <time.h>
#include <sstream>
#include <tf/transform_broadcaster.h>

#include "ultrasound_can_parser.h"

const char OutputTopic[] = "ultrasound";
const char OutputFrame[] = "ultrasound";

class RangeMsgPublisher {
public:
    RangeMsgPublisher(const ros::NodeHandle nh) : nh_(nh) {}

    void Init() {
        publisher_ = nh_.advertise<sensor_msgs::Range>(OutputTopic, 100);
    }

    void Start() {
        ros::Rate rate(10);
        while (ros::ok()) {
            // create msg
            sensor_msgs::Range msg;
            std_msgs::Header header;
            header.stamp = ros::Time::now();
            header.frame_id = OutputFrame;
            msg.header = header;
            msg.field_of_view = 1;
            msg.min_range = 0.1;
            msg.max_range = 2.0;
            msg.range = 1.5;
            // tf
            tf::TransformBroadcaster broadcaster;
            broadcaster.sendTransform(
                    tf::StampedTransform(
                            tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
                            ros::Time::now(), "base_link", OutputFrame
                    )
            );

            publisher_.publish(msg);
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "ultrasound");
    ros::NodeHandle nh;

    UltraSoundCanPaser* ultraSoundCanPaser = new UltraSoundCanPaser(nh);
    ultraSoundCanPaser->start();
    sleep(10);
    ultraSoundCanPaser->join();

//    delete ultraSoundCanPaser;
//    return 0;
}
