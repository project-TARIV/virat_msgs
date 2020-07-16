#include <ros/ros.h>
#include <virat_msgs/Polynomial.h>
#include <visualization_msgs/Marker.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>

/*
 * Publishes virat_msgs::Polynomial to a visualisation_msgs::Marker as a LINE_STRIP
 *
 * The polynomial is supposed to be in base_footprint frame
 * and is evaluated based on x values in a range
 *
 * Params:
 * For multiple names:
 *
 *  ~/names: ["global_plan"]
 *  ~/global_plan:
 *      topic: "/move_base/MPC_Local_Planner/global_plan"
 *      color: [ 0.5, 0.5, 0.0, 0.5 ]
 *      scale: 0.1
 *
 *
 * Example use in test/ in mpc_local_planner.
 *
 */


// evaluvate poly for x belong to -range, range at intervals of resolution
const double range = 4;
const double resolution = 0.1;

double polyeval(const double &x, const std::vector<double> &coeffs) {
    double ret = 0, pow = 1;
    for (const auto &coeff : coeffs) {
        ret += coeff * pow;
        pow *= x;
    }
    return ret;
}


// Converts polynomial to marker and publishes
void callback(const virat_msgs::PolynomialConstPtr &msg, ros::Publisher &pub, visualization_msgs::Marker &m,
              tf2_ros::Buffer &buffer) {

    // Something is wrong with frame... but it works. TODO
    auto trans = buffer.lookupTransform(msg->header.frame_id, "base_footprint", ros::Time(0)).transform;

    m.header = msg->header;
    m.pose.position.x = trans.translation.x;
    m.pose.position.y = trans.translation.y;
    m.pose.orientation.z = trans.rotation.z;
    m.pose.orientation.w = trans.rotation.w;

    m.points.clear();
    if (msg->coeffs.empty()) {
        pub.publish(m);
        return;
    }
    double x = -range;
    while (x <= range) {
        m.points.emplace_back();
        m.points.back().x = x;
        m.points.back().y = polyeval(x, msg->coeffs);

        x += resolution;
    }

    pub.publish(m);
}


std::string parse_params(ros::NodeHandle &nh, const std::string &name, visualization_msgs::Marker &m) {
    m.ns = name;

    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.action = visualization_msgs::Marker::MODIFY;

    std::vector<double> color;
    nh.getParam(name + "/color", color);
    color.resize(4, 0);
    m.color.r = color[0];
    m.color.g = color[1];
    m.color.b = color[2];
    m.color.a = color[3];

    m.scale.x = 0.1;
    nh.getParam(name + "/scale", m.scale.x);

    return nh.param(name + "/topic", std::string{});
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "poly_view", ros::init_options::AnonymousName);

    // For getting params
    ros::NodeHandle nh("~");

    // For subscribing to topics
    ros::NodeHandle global;

    // Get list of polynomial names
    auto names = nh.param("names", std::vector<std::string>{});

    // Store the objects that are constructed once per polynomial, but passed to the callback each time
    std::vector<std::shared_ptr<ros::Publisher>> pubs;
    std::vector<std::shared_ptr<visualization_msgs::Marker>> markers;


    std::vector<ros::Subscriber> subs;


    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    for (const auto &name : names) {
        markers.push_back(std::make_shared<visualization_msgs::Marker>());

        const auto topic = parse_params(nh, name, *markers.back());

        pubs.push_back(std::make_shared<ros::Publisher>(
                nh.advertise<visualization_msgs::Marker>(topic + "/rviz", 2)));

        subs.push_back(
                global.subscribe<virat_msgs::Polynomial>(topic, 2,
                                                         std::bind(callback,
                                                                   std::placeholders::_1, // msg
                                                                   std::ref(*pubs.back()),
                                                                   std::ref(*markers.back()),
                                                                   std::ref(buffer))));
    }

    ros::spin();
}
