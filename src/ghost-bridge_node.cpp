
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
    resp_pub = nh.advertise<std_msgs::String>("ghost/response", 1);
    req_sub = nh.subscribe("ghost/req", 1, utt_cb);

    signal(SIGINT, sigint_handler);

    std::string ro, relex_hostname, relex_port;
    std::vector<std::string> grf_prm, gsm_prm;

    nh.param("ghost/rule_files", grf_prm, std::vector<std::string>());
    nh.param("ghost/scheme_modules", gsm_prm, std::vector<std::string>());
    ros::param::param<int>("ghost/response_wait_sec", resp_wait_time, 5);
    ros::param::param<std::string>("ghost/relex_hostname", relex_hostname, "localhost");
    ros::param::param<std::string>("ghost/relex_port", relex_port, "4444");

    g = Ghost();
    g.setSchemeModules(gsm_prm);
    g.setGhostRuleFiles(grf_prm);
    g.ghostInit();
    g.ghostRun();

    // start ghost listener thread
    std::thread t(ghost_listener);

    ros::spin();
    t.join();
    usleep(1000000);
    g.ghostShutdown();

return 0;
}
