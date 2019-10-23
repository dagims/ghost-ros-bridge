#include <stdio.h>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "Ghost.h"

void utt_cb(std_msgs::String data)
{
    printf("Received Ghost Request: %s\n", data.data.c_str());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "GhostBridgeNode");
    ros::NodeHandle nh;
    ros::Publisher resp_pub = nh.advertise<std_msgs::String>("/ghost_response", 1);
    Ghost g = Ghost();
    g.ghost_init();
    std::string ro;
    g.loadRuleFile(ro,
    "/home/dagiopia/singnet/PKD/virtual-assistant/src/virtual-assistant/ghost/pkd/futurist.ghost");
    g.utterance("you hate humans", ro);
    usleep(10000000);
    printf("---------------%s\n", ro.c_str());
    g.ghost_shutdown();
    ros::Subscriber req_sub = nh.subscribe("/ghost_req", 1, utt_cb);
    ros::spinOnce();
return 0;
}
