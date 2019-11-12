
#include <stdio.h>
#include <thread>
#include <mutex>
#include <chrono>
#include <signal.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include "Ghost.h"

void sigint_handler(int sig)
{
    ros::shutdown();
}

class GhostNode
{
private:
    Ghost *g;
    int resp_wait_time;
    bool lipsync_speaking;
    std::thread t;
    std::mutex mtx;
    std::regex act_r;

    ros::Publisher resp_pub;
    ros::Subscriber req_sub;
    ros::Subscriber scm_sub;

    void ghost_listener()
    {
        std::string new_o, prev_o;
        while (ros::ok()) {
            g->getGhostResponse(new_o, resp_wait_time);
            if (new_o.compare(prev_o) != 0 && new_o.size() > 0) {
                std_msgs::String msg;
                msg.data = new_o;
                resp_pub.publish(msg);
                prev_o = new_o;
            }
        }
    }

    void utt_cb(std_msgs::String _req)
    {
        std::string out;
        g->utterance(_req.data, out);
    }

    void scm_cb(std_msgs::String _req)
    {
        std::string out;
        g->scmEval(out, _req.data);
        printf("SCM EVAL OUT: <<<%s>>>\n", out.c_str());
    }

public:
    GhostNode () {}
    GhostNode(int _argc, char** _argv, AtomSpace *_as)
    {
        g = new Ghost(_as);
        ros::init(_argc, _argv, "GhostBridgeNode");
        ros::NodeHandle nh;
        resp_pub = nh.advertise<std_msgs::String>("ghost_response", 1);
        req_sub = nh.subscribe("ghost_request", 1, &GhostNode::utt_cb, this);
        scm_sub = nh.subscribe("ghost_scm_eval", 1, &GhostNode::scm_cb, this);

        std::string ro, relex_hostname, relex_port;
        std::vector<std::string> grf_prm, gsm_prm;

        nh.param("ghost/rule_files", grf_prm, std::vector<std::string>());
        nh.param("ghost/scheme_modules", gsm_prm, std::vector<std::string>());
        ros::param::param<int>("ghost/response_wait_sec", resp_wait_time, 1);
        ros::param::param<std::string>("ghost/relex_hostname", relex_hostname, "localhost");
        ros::param::param<std::string>("ghost/relex_port", relex_port, "4444");

        g->setSchemeModules(gsm_prm);
        g->setGhostRuleFiles(grf_prm);
        g->ghostInit();
        g->ghostRun();

        // start ghost listener thread
        std::thread t(&GhostNode::ghost_listener, this);

        ros::spin();
        t.join();
        usleep(1000000);
        g->ghostShutdown();

    }

    ~GhostNode()
    {
        delete g;
    }

};

int main(int argc, char** argv)
{
    signal(SIGINT, sigint_handler);
    AtomSpace *as = new AtomSpace();
    GhostNode *gs = new GhostNode(argc, argv, as);
return 0;
}
