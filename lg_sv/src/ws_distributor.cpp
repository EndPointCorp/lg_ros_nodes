#include <signal.h>
#include "uWS.h"
#include "ros/ros.h"
#include "std_srvs/Empty.h"

uWS::Hub h;

void sigIntHandler(int sig) {
    ros::shutdown();

    uWS::Group<uWS::SERVER> g = h.getDefaultGroup<uWS::SERVER>();
    g.close();
}

bool upHandler(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    return true;
}

int main(int argc, char** argv)
{
    h.onMessage([](uWS::WebSocket<uWS::SERVER> ws, char *message, size_t length, uWS::OpCode opCode) {
        uWS::Group<uWS::SERVER> g = h.getDefaultGroup<uWS::SERVER>();

        g.forEach([&ws, message, length, opCode](uWS::WebSocket<uWS::SERVER> cs) {
            if (ws.getPollHandle() != cs.getPollHandle()) {
                cs.send(message, length, opCode);
            }
        });
    });

    h.onError([](typename uWS::Group<uWS::SERVER>::errorType error) {
        ROS_ERROR_STREAM("uWS Hub had an error: " << error);
    });

    ros::init(argc, argv, "ws_broadcaster", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;

    int port;
    n.param<int>("port", port, 9091);

    h.listen(port);

    // Create the Service only after we have started listening
    // (but before running the uWS loop)
    ros::ServiceServer portService = n.advertiseService("up", upHandler);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ROS_INFO_STREAM("running on port " << port);

    signal(SIGINT, sigIntHandler);
    h.run();
}

// vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
