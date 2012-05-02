#include <ros/ros.h>
#include <ros/console.h>

#include "dg14/dg14.h"

using namespace dg14;
using std::string;

DG14 *gps;

void errorMsgCallback(const std::string &msg) {
    ROS_ERROR("%s", msg.c_str());
}

void warnMsgCallback(const std::string &msg) {
    ROS_WARN("%s", msg.c_str());
}

void infoMsgCallback(const std::string &msg) {
    ROS_INFO("%s", msg.c_str());
}

void debugMsgCallback(const std::string &msg) {
    ROS_DEBUG("%s", msg.c_str());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "dg14_node");
    ros::NodeHandle n;

    std::string port;
    n.param("serial_port", port, std::string("/dev/tty.usbserial-FTFBFFQY"));

    ROS_INFO("DG14 Connecting to port %s", port.c_str());
    try {
        gps = new DG14();
        gps->warn = warnMsgCallback;
        gps->info = infoMsgCallback;
        gps->debug = debugMsgCallback;
        gps->connect(port);
    } catch(std::exception &e) {
        ROS_ERROR("Failed to connect to the DG14: %s", e.what());
        if (gps != NULL) {
            gps->disconnect();
        }
        return 0;
    }

    while(ros::ok()) {
        while(gps != NULL && gps->isConnected() && ros::ok()) {
            ros::Duration(5.0).sleep();
        }
    }



    if (gps != NULL) {
        gps->disconnect();
    }

    return 0;
}
