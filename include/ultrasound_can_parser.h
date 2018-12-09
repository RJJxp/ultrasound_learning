//
// Created by wurui on 12/2/18.
//

#ifndef PROJECT_ULTRASOUND_CAN_PARSER_H
#define PROJECT_ULTRASOUND_CAN_PARSER_H

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Header.h>
#include <tf/transform_broadcaster.h>
#include "pthread_base.h"

// can bus
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include "controlcan.h"

#include <ctime>
#include <cstdlib>
#include "unistd.h"
#include <iostream>

class UltraSoundCanPaser : public ThreadBase {
public:
    UltraSoundCanPaser(const ros::NodeHandle nh,
                       char *OutputTopic = "ultrasound",
                       char *FrameId = "ultrsound") : nh_(nh),
                                                      count_(0),
                                                      OutputTopic_(OutputTopic),
                                                      FrameId_(FrameId) {}

private:
    void run() override;    // why not state as virtual
                            // note by rjp

    void init();

    void receive_func(void *param);

    void sendRangeMsg(const unsigned int can_id, unsigned char const *can_data);

    void generateRangeMsg(float distance = 0.0);

    void parseCanData(const unsigned int can_id, unsigned char const *can_data, std::vector<float> &distances);

    void debugFakeMsg();

    //uint8_t getByte(const uint8_t& value, const int32_t start_pos, const int32_t length);

private:
    VCI_BOARD_INFO pInfo_;
    long count_;

    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    tf::TransformBroadcaster tfbroadcaster_;

    // ros param
    float max_range_;
    float min_range_;
    char *OutputTopic_;
    char *FrameId_;

    

};


#endif //PROJECT_ULTRASOUND_CAN_PARSER_H
