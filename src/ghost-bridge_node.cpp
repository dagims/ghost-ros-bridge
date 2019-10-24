
#include <stdio.h>
#include <thread>
#include <signal.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>

#include "Ghost.h"

Ghost g;
int resp_wait_time;
ros::Publisher resp_pub;
ros::Subscriber req_sub;

void sigint_handler(int sig)
{
    ros::shutdown();
}

void utt_cb(std_msgs::String data)
{
    printf("Received Ghost Request: %s\n", data.data.c_str());
    std::string out;
    g.utterance(data.data, out);
}

void ghost_listener()
{
    std::string new_o, prev_o;
    while (ros::ok()) {
        g.getGhostResponse(new_o, resp_wait_time);
        if (new_o.compare(prev_o) != 0) {
            std_msgs::String msg;
            msg.data = new_o;
            resp_pub.publish(msg);
            prev_o = new_o;
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "GhostBridgeNode");
    ros::NodeHandle nh;
    resp_pub = nh.advertise<std_msgs::String>("/ghost_response", 1);
    req_sub = nh.subscribe("/ghost_req", 1, utt_cb);

    signal(SIGINT, sigint_handler);

    std::string ro, relex_hostname, relex_port;
    
    XmlRpc::XmlRpcValue grf_prm, gsm_prm;
    nh.getParam("ghost/rule_files", grf_prm);
    nh.getParam("guile/scheme_modules", gsm_prm);
    ROS_ASSERT(grf_prm.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(gsm_prm.getType() == XmlRpc::XmlRpcValue::TypeArray);

    ros::param::param<int>("ghost/response_wait_sec", resp_wait_time, 5);
    ros::param::param<std::string>("ghost/relex_hostname", relex_hostname, "localhost");
    ros::param::param<std::string>("ghost/relex_port", relex_port, "4444");

    g = Ghost();
    g.ghostInit();

    g.setRelexServer(relex_hostname, relex_port);
    
/*  for(size_t i = 0; i < grf_prm.size(); i++)
        g.loadRuleFile(ro, static_cast<std::string>(grf_prm[i]));
*/
    g.loadRuleFile(ro, "/home/dagiopia/singnet/PKD/virtual-assistant/src/virtual-assistant/ghost/pkd/futurist.ghost");
    usleep(10000000);
    std::thread t(ghost_listener);

    ros::spin();
    t.join();
    usleep(1000000);
    g.ghostShutdown();

return 0;
}
