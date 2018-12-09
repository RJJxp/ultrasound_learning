//
// Created by wurui on 12/2/18.
//

#include <tf/transform_broadcaster.h>
#include "../include/ultrasound_can_parser.h"

void UltraSoundCanPaser::run() {
//    if (!isAlive()) { assert("Run: thread has died!"); }

    init();

    // debugFakeMsg();
   int m_run0=1;
   receive_func(&m_run0);

}

void UltraSoundCanPaser::init() {
    // ros publisher
    publisher_ = nh_.advertise<sensor_msgs::Range>(OutputTopic_, 100);

    std::cout << "start init ..." << std::endl; 
    // can bus
    bool can_bus = true;
    if (can_bus) {
        printf(">>this is hello !\r\n");//指示程序已运行

        //打开设备
        if (VCI_OpenDevice(VCI_USBCAN2, 0, 0) == 1) {
            printf(">>open deivce success!\n");//打开设备成功
        } else {
            printf(">>open deivce error!\n");
            exit(1);
        }

        //读取设备序列号、版本等信息。
        if (VCI_ReadBoardInfo(VCI_USBCAN2, 0, &pInfo_) == 1) {
            printf(">>Get VCI_ReadBoardInfo success!\n");

        } else {
            printf(">>Get VCI_ReadBoardInfo error!\n");
            exit(1);
        }

        //初始化参数，严格参数二次开发函数库说明书。
        VCI_INIT_CONFIG config;
        config.AccCode = 0;
        config.AccMask = 0xFFFFFFFF;
        config.Filter = 1;//接收所有帧
        config.Timing0 = 0x00;/*波特率500 Kbps  0x00  0x1C*/
        config.Timing1 = 0x1C;
        config.Mode = 0;//0:正常模式	1:only receive  2: receive and send

        // 启动can1口
        if (VCI_InitCAN(VCI_USBCAN2, 0, 0, &config) != 1) {
            printf(">>Init CAN1 error\n");
            VCI_CloseDevice(VCI_USBCAN2, 0);
        }

        if (VCI_StartCAN(VCI_USBCAN2, 0, 0) != 1) {
            printf(">>Start CAN1 error\n");
            VCI_CloseDevice(VCI_USBCAN2, 0);
        }

        ROS_INFO("init done!");
    }
}

void UltraSoundCanPaser::receive_func(void *param)  //接收线程。
{
    int reclen = 0;
    VCI_CAN_OBJ rec[3000];//接收缓存，设为3000为佳。
    int i, j;

    int *run = (int *) param;//线程启动，退出控制。
    int ind = 0;

    std::cout << " pthead receive init" << std::endl;
 
    ros::Rate rate(10);
    while (ros::ok()) {
        if(!((*run) & 0x0f)){break;}
        // std::cout << " pthead receive waiting"<<std::endl;
        //调用接收函数，如果有数据，进行数据处理显示。
        if ((reclen = VCI_Receive(VCI_USBCAN2, 0, ind, rec, 3000, 100)) > 0) {
            for (j = 0; j < reclen; j++) {
                printf("Index:%04d  ", count_);
                count_++;//序号递增
                printf("CAN%d RX ID:0x%08X", ind + 1, rec[j].ID);//ID                  !!!!!!!!!
                if (rec[j].ExternFlag == 0) printf(" Standard ");//帧格式：标准帧
                if (rec[j].ExternFlag == 1) printf(" Extend   ");//帧格式：扩展帧
                if (rec[j].RemoteFlag == 0) printf(" Data   ");//帧类型：数据帧
                if (rec[j].RemoteFlag == 1) printf(" Remote ");//帧类型：远程帧
                printf("DLC:0x%02X", rec[j].DataLen);//帧长度
                printf(" data:0x");    //数据                                           !!!!!!!!!
                for (i = 0; i < rec[j].DataLen; i++) {
                    printf(" %02X", rec[j].Data[i]);
                }
                printf(" TimeStamp:0x%08X", rec[j].TimeStamp);//时间标识。
                printf("\n");

                // send ros range msg
                //std::cout<<"send range msg \n";
                sendRangeMsg(rec[j].ID, rec[j].Data);
                //std::cout<<"send range msg done \n";
                rate.sleep();
            }
        }
        ind = !ind;//变换通道号，以便下次读取另一通道，交替读取。
    }
    printf("run thread exit\n");//退出接收线程
    return;
}

void UltraSoundCanPaser::debugFakeMsg() {
    int mode = 0;
    if (mode == 0) {
        ros::Rate rate(10);
        while (ros::ok()) {
            // create msg
            sensor_msgs::Range msg;
            std_msgs::Header header;
            header.stamp = ros::Time::now();
            header.frame_id = FrameId_;
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
                            ros::Time::now(), "base_link", FrameId_));

            publisher_.publish(msg);
            ROS_INFO(">>> msg sended");
            rate.sleep();
        }
    } else if (mode == 1) {

    }
}

void UltraSoundCanPaser::sendRangeMsg(const unsigned int can_id, unsigned char const *can_data) {
    std::vector<float> distances;

    std::cout << "parse can data ..." << std::endl;
    parseCanData(can_id, can_data, distances);
    // generate msg and send
    for (int i = 0; i < distances.size(); ++i) {
        
        generateRangeMsg(distances[i]);
    }
}

// the msg we need to send is distance
// receive the distance, create a RangeMsg in ros, set up the tf, and publish
// note by rjp
void UltraSoundCanPaser::generateRangeMsg(float distance) {
    sensor_msgs::Range msg;
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = FrameId_;
    msg.header = header;
    msg.field_of_view = 1;
    msg.min_range = min_range_;
    msg.max_range = max_range_;
    msg.range = distance;

    // tf
    tfbroadcaster_.sendTransform(
            tf::StampedTransform(
                    tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
                    ros::Time::now(), "base_link", FrameId_));

    publisher_.publish(msg);
    std::cout << "send a msg" << std::endl;
}

// translate the raw data to distances
// return distances which i thought better add a const
// note by rjp
void UltraSoundCanPaser::parseCanData(const unsigned int can_id, unsigned char const *can_data, std::vector<float> &distances){
    uint32_t frame_index_0;
    uint32_t frame_index_1;  
    double d;   // distance

    if(can_id == 0x233) {
        return ;
        // first part of id233
        frame_index_0 = *(can_data);
        frame_index_1 = *(can_data + 1);
        frame_index_1 <<= 8;
        frame_index_1 |= frame_index_0;
        d = frame_index_1;
        if(d == 505) {
            d = 0;
            std::cout<< " 0:505 error plz check\n ";   
        }
        else if(d == 808 ){
            d = 0;
            std::cout<<" 0:808 error plz check\n ";
        }
        distances.push_back(d/100.0);
        
        // second part of id233
        frame_index_0 = *(can_data + 2);
        frame_index_1 = *(can_data + 3);
        frame_index_1 <<= 8;
        frame_index_1 |= frame_index_0;
        d = frame_index_1;
        if(d == 505) {
            d = 0;
            std::cout<< " 1:505 error plz check\n ";   
        }
        else if(d == 808 ){
            d = 0;
            std::cout<<" 1:808 error plz check\n ";
        }
        distances.push_back(d/100.0);

        // third part of id233
        frame_index_0 = *(can_data + 4);
        frame_index_1 = *(can_data + 5);
        frame_index_1 <<= 8;
        frame_index_1 |= frame_index_0;
        d = frame_index_1;
        if(d == 505) {
            d = 0;
            std::cout<< " 2:505 error plz check\n ";   
        }
        else if(d == 808 ){
            d = 0;
            std::cout<<" 2:808 error plz check\n ";
        }
        distances.push_back(d/100.0);

        //fourth part of id233
        frame_index_0 = *(can_data + 6);
        frame_index_1 = *(can_data + 7);
        frame_index_1 <<= 8;
        frame_index_1 |= frame_index_0;
        d = frame_index_1;
        if(d == 505) {
            d = 0;
            std::cout<< " 3:505 error plz check\n ";   
        }
        else if(d == 808 ){
            d = 0;
            std::cout<<" 3:808 error plz check\n ";
        }
        distances.push_back(d/100.0);
    }
    else {  // id = 0x234

        // first part of id234
        frame_index_0 = *(can_data);
        frame_index_1 = *(can_data + 1);
        frame_index_1 <<= 8;
        frame_index_1 |= frame_index_0;
        d = frame_index_1;
        if(d == 505) {
            d = 0;
            std::cout<< " 0:505 error plz check\n ";   
        }
        else if(d == 808 ){
            d = 0;
            std::cout<<" 0:808 error plz check\n ";
        }

        distances.push_back(d/100.0);
        
        // second part of id234
        frame_index_0 = *(can_data + 2);
        frame_index_1 = *(can_data + 3);
        frame_index_1 <<= 8;
        frame_index_1 |= frame_index_0;
        d = frame_index_1;
        if(d == 505) {
            d = 0;
            std::cout<< " 1:505 error plz check\n ";   
        }
        else if(d == 808 ){
            d = 0;
            std::cout<<" 1:808 error plz check\n ";
        }
        distances.push_back(d/100.0);

        // no need to cared about the 3rd and 4th part of id234
    }
    
}


// uint8_t UltraSoundCanPaser::getByte(const uint8_t& value, const int32_t start_pos, const int32_t length){
//     if (start_pos > BYTE_LENGTH - 1 || start_pos < 0 || length < 1) {
//     return 0x00;
//     }
//     int32_t end_pos = std::min(start_pos + length - 1, BYTE_LENGTH - 1);
//     int32_t real_len = end_pos + 1 - start_pos;
//     uint8_t result = value >> start_pos;
//     result &= RANG_MASK_1_L[real_len - 1];
//     return result;
// }