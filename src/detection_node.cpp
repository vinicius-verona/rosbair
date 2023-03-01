#include <tf/transform_datatypes.h>

#include <cmath>

#include "geometry_msgs/Point.h"
#include "message_filters/subscriber.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "ros/time.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Bool.h"
#include "std_msgs/ColorRGBA.h"
#include "std_msgs/Int32.h"
#include "tf/message_filter.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "visualization_msgs/Marker.h"

#define detection_threshold 0.2  // threshold for motion detection
#define dynamic_threshold 75     // to decide if a cluster is static or dynamic

// threshold for clustering
#define cluster_threshold 0.2

// used for detection of leg
#define leg_size_min 0.05
#define leg_size_max 0.25
#define legs_distance_min 0.1
#define legs_distance_max 0.7

// used for uncertainty of leg
#define uncertainty_min_leg 0.5
#define uncertainty_max_leg 1
#define uncertainty_inc_leg 0.05

using namespace std;

class detection_node {
   private:
    ros::NodeHandle n;

    ros::Subscriber sub_scan;
    ros::Subscriber sub_robot_moving;

    ros::Publisher pub_detection_node;
    ros::Publisher pub_detection_marker;

    // to store, process and display both laserdata
    bool init_laser;
    int nb_beams;
    float range_min, range_max;
    float angle_min, angle_max, angle_inc;
    float r[1000], theta[1000];
    geometry_msgs::Point current_scan[1000];

    // to perform detection of motion
    bool init_robot;
    float background[1000];
    bool stored_background;  // true = valid
    bool dynamic[1000];
    bool current_robot_moving;
    bool previous_robot_moving;

    // to perform clustering
    int nb_cluster;            // number of cluster
    int cluster[1000];         // to store for each hit, the cluster it belongs to
    float cluster_size[1000];  // to store the size of each cluster
    geometry_msgs::Point
        cluster_middle[1000];   // to store the middle of each cluster
    int cluster_dynamic[1000];  // to store the percentage of the cluster that
                                // is dynamic

    // to perform detection of legs and to store them
    int nb_legs_detected;
    geometry_msgs::Point leg_detected[1000];
    int leg_cluster[1000];   // to store the cluster corresponding to a leg
    bool leg_dynamic[1000];  // to know if a leg is dynamic or not

    // to perform detection of a moving person and store it
    int nb_persons_detected;
    geometry_msgs::Point person_detected[1000];
    bool person_dynamic[1000];
    geometry_msgs::Point
        moving_person_detected;  // to store the coordinates of the moving
                                 // person that we have detected

    // GRAPHICAL DISPLAY
    int nb_pts;
    geometry_msgs::Point display[1000];
    std_msgs::ColorRGBA colors[1000];

   public:
    detection_node() {
        sub_scan = n.subscribe("scan", 1, &detection_node::scanCallback, this);
        sub_robot_moving = n.subscribe(
            "robot_moving", 1, &detection_node::robot_movingCallback, this);

        // communication with action
        pub_detection_node = n.advertise<geometry_msgs::Point>(
            "goal_to_reach",
            1);  // Preparing a topic to publish the goal to reach.
        pub_detection_marker = n.advertise<visualization_msgs::Marker>(
            "detection_marker",
            1);  // Preparing a topic to publish our results. This will be used
                 // by the visualization tool rviz

        init_laser = false;
        init_robot = false;
        previous_robot_moving = true;

        ros::Rate r(10);

        while (ros::ok()) {
            ros::spinOnce();
            update();
            r.sleep();
        }
    }

    // UPDATE
    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
    void update() {
        // we wait for new data of the laser and of the robot_moving_node to
        // perform laser processing
        if (init_laser && init_robot) {
            ROS_INFO("\n");
            ROS_INFO("New data of laser received");
            ROS_INFO("New data of robot_moving received");

            nb_pts = 0;
            if (!current_robot_moving) {
                // if the robot is not moving then we can perform moving person
                // detection DO NOT FORGET to store the background but when ???
                ROS_INFO("robot is not moving");

                if (!stored_background) {
                    store_background();
                    stored_background = true;
                } else {
                    detect_motion();
                }
            } else {
                // IMPOSSIBLE TO DETECT MOTIONS because the base is moving
                // what is the value of dynamic table for each hit of the laser
                // ?
                ROS_INFO("robot is moving");

                // clear background
                stored_background = false;
            }
            previous_robot_moving = current_robot_moving;

            // we search for moving person in 4 steps
            perform_clustering();  // to perform clustering
            detect_legs();         // to detect moving legs using cluster
            detect_persons();      // to detect moving_person using moving legs
                                   // detected
            detect_a_moving_person();

            // graphical display of the results
            populateMarkerTopic();

        } else if (!init_robot) {
            ROS_WARN("waiting for robot_moving_node");
            ROS_WARN("launch: rosrun follow_me robot_moving_node");
        }

    }  // update

    // DETECT MOTION FOR BOTH LASER
    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
    void store_background() {
        // store all the hits of the laser in the background table

        ROS_INFO("storing background");

        for (int loop = 0; loop < nb_beams; loop++) {
            background[loop] = r[loop];
        }

        ROS_INFO("background stored");

    }  // store_background

    void reset_motion() {
        // for each hit, compare the current range with the background to detect
        // motion

        ROS_INFO("reset motion");
        for (int loop = 0; loop < nb_beams; loop++) dynamic[loop] = false;

        ROS_INFO("reset_motion done");

    }  // reset_motion

    void detect_motion() {
        ROS_INFO("detecting motion");

        for (int loop = 0; loop < nb_beams; loop++) {
            // loop over all the hits
            //  if the difference between ( the background and the current value
            //  r ) is higher than "detection_threshold"
            float diff = abs(background[loop] - r[loop]);

            if (diff > detection_threshold) {
                dynamic[loop] = true;  // the current hit is dynamic
                ROS_INFO("motion detected");
            } else {
                dynamic[loop] = false;
            }

            if (dynamic[loop]) {
                ROS_INFO("hit[%i](%f, %f) is dynamic", loop,
                         current_scan[loop].x, current_scan[loop].y);

                // display in blue of hits that are dynamic
                display[nb_pts] = current_scan[loop];

                colors[nb_pts].r = 0;
                colors[nb_pts].g = 0;
                colors[nb_pts].b = 1;
                colors[nb_pts].a = 1.0;

                nb_pts++;
            }
        }

    }  // detect_motion

    // CLUSTERING
    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
    void perform_clustering() {
        // store in the table cluster, the cluster of each hit of the laser
        // if the distance between the previous hit and the current one is
        // higher than "cluster_threshold" then we end the current cluster with
        // the previous hit and start a new cluster with the current hit else
        // the current hit belongs to the current cluster

        ROS_INFO("performing clustering");

        nb_cluster = 0;  // to count the number of cluster

        int start = 0;  // to store the index of the start of current cluster
        int end;        // to store the index of the end of the current cluster

        // graphical display of the start of the current cluster in green
        display[nb_pts] = current_scan[start];

        colors[nb_pts].r = 0;
        colors[nb_pts].g = 1;
        colors[nb_pts].b = 0;
        colors[nb_pts].a = 1.0;
        nb_pts++;

        float middle_x = 0;
        float middle_y = 0;

        // TODO - compare
        for (int loop = 1; loop < nb_beams; loop++) {  // loop over all the hits
            // if euclidian DISTANCE between the previous hit and the current
            // one is higher than "cluster_threshold"
            float points_dist =
                distancePoints(current_scan[loop], current_scan[loop - 1]);
            if (points_dist > cluster_threshold || loop == nb_beams - 1) {
                // we end the current cluster
                end = loop;

                // graphical display of the end of the current cluster in red
                display[nb_pts] = current_scan[end];

                colors[nb_pts].r = 1;
                colors[nb_pts].g = 0;
                colors[nb_pts].b = 0;
                colors[nb_pts].a = 1.0;
                nb_pts++;

                // compute the middle of the cluster
                middle_x = 0;
                middle_y = 0;

                int points_dynamic = 0;

                for (int j = start; j <= end; j++) {
                    middle_x += current_scan[j].x;
                    middle_y += current_scan[j].y;
                    if (dynamic[j]) {
                        points_dynamic++;
                    }
                }

                middle_x /= (end - start + 1);
                middle_y /= (end - start + 1);

                // TODO - compare
                // the previous hit is the end of the current cluster
                geometry_msgs::Point middle = {};
                middle.x = middle_x;
                middle.y = middle_y;

                // textual display
                // ROS_INFO(
                //     "cluster[%i] (%f, %f): hit[%i](%f, %f) -> hit[%i](%f,
                //     %f)", nb_cluster, middle.x, middle.y, start,
                //     current_scan[start].x, current_scan[start].y, end,
                //     current_scan[end].x, current_scan[end].y);

                // graphical display of the middle of the current cluster in
                // blue
                display[nb_pts] = middle;

                colors[nb_pts].r = 1;
                colors[nb_pts].g = 0.8;
                colors[nb_pts].b = 0.8;
                colors[nb_pts].a = 1.0;
                nb_pts++;

                // the current hit is the start of the next cluster
                // we start the next cluster
                start = loop;

                // graphical display of the start of the current cluster in
                // green
                display[nb_pts] = current_scan[start];

                colors[nb_pts].r = 0;
                colors[nb_pts].g = 1;
                colors[nb_pts].b = 0;
                colors[nb_pts].a = 1.0;
                nb_pts++;

                // increase cluster number
                cluster_size[nb_cluster] = end - start;
                cluster_middle[nb_cluster] = middle;
                cluster_dynamic[nb_cluster] =
                    100 * (int)((double)(points_dynamic) /
                                (double)(end - start));  //  % of dynamic points
                                                         //  within a cluster
                nb_cluster++;
            }
            cluster[loop] = nb_cluster;
        }
        // Dont forget to update and display the last cluster
        // ROS_INFO

        ROS_INFO("clustering performed");

    }  // perform_clustering

    // DETECTION OF A MOVING PERSON
    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
    void detect_legs() {
        // a leg is a cluster:
        // - with a size higher than "leg_size_min";
        // - with a size lower than "leg_size_max;
        // if more than "dynamic_threshold"% of its hits are dynamic the leg is
        // considered to be dynamic

        ROS_INFO("detecting legs");
        nb_legs_detected = 0;

        nb_pts = 0;
        for (int loop = 0; loop < nb_cluster; loop++) {
            int curr_cluster_size = cluster_size[loop];
            float min_x, max_x, min_y, max_y;
            for (int i = 0; i < curr_cluster_size; i++) {
                int index = 0;  // WARNING TODO
                float x = current_scan[index].x;
                float y = current_scan[index].y;
                if (x < min_x) min_x = x;
                if (x > max_x) max_x = x;
                if (y < min_y) min_y = y;
                if (y > max_y) max_y = y;
            }

            float size_x = max_x - min_x;
            float size_y = max_y - min_y;

            // loop over all the clusters
            //      if the size of the current cluster is higher than
            //      "leg_size_min" and
            //  lower than "leg_size_max" then the current cluster is a leg
            if (size_x > leg_size_min && size_x < leg_size_max &&
                size_y > leg_size_min && size_y < leg_size_max) {
                // we update:
                // - the leg_detected table to store the middle of the moving
                // leg;
                // - the leg_cluster to store the cluster corresponding to a
                // leg;
                // - the leg_dynamic to know if the leg is dynamic or not.
                leg_detected[nb_legs_detected] = cluster_middle[loop];
                leg_cluster[nb_legs_detected] = loop;
                leg_dynamic[nb_legs_detected] = (cluster_dynamic[loop] >= 75);

                if (leg_dynamic[nb_legs_detected]) {
                    // ROS_INFO("moving leg found: %i -> cluster = %i, (%f, %f),
                    // size: %f, dynamic: %i", nb_legs_detected, loop,
                    // leg_detected[nb_legs_detected].x,
                    // leg_detected[nb_legs_detected].y,
                    // cluster_size[loop],
                    // cluster_dynamic[loop]);

                    for (int loop2 = 0; loop2 < nb_beams; loop2++) {
                        if (cluster[loop2] == loop) {
                            // dynamic legs are yellow
                            display[nb_pts] = current_scan[loop2];

                            colors[nb_pts].r = 1;
                            colors[nb_pts].g = 1;
                            colors[nb_pts].b = 0;
                            colors[nb_pts].a = 1.0;

                            nb_pts++;
                        }
                    }
                } else {
                    // ROS_INFO("static leg found: %i -> cluster = %i, (%f, %f),
                    // size: %f, dynamic: %i", nb_legs_detected, loop,
                    // leg_detected[nb_legs_detected].x,
                    // leg_detected[nb_legs_detected].y,
                    // cluster_size[loop],
                    // cluster_dynamic[loop]);
                    for (int loop2 = 0; loop2 < nb_beams; loop2++) {
                        if (cluster[loop2] == loop) {
                            // static legs are white
                            display[nb_pts] = current_scan[loop2];

                            colors[nb_pts].r = 1;
                            colors[nb_pts].g = 1;
                            colors[nb_pts].b = 1;
                            colors[nb_pts].a = 1.0;

                            nb_pts++;
                        }
                    }
                }

                nb_legs_detected++;
            }

            nb_pts++;  // actually should be here - otherwise "Points should not be empty in rviz"
        }

        if (nb_legs_detected) {
            ROS_INFO("%d legs have been detected.\n", nb_legs_detected);
        }

        ROS_INFO("detecting legs done");

    }  // detect_legs

    void detect_persons() {
        // a person has two legs located at less than "legs_distance_max" one
        // from the other a moving person (ie, person_dynamic array) has 2 legs
        // that are dynamic

        ROS_INFO("detecting persons");
        nb_persons_detected = 0;

        // Iterate through all the legs and compare the distance between each
        // If the distance is lower than "legs_distance_max", we have a person.
        for (int loop_leg1 = 0; loop_leg1 < nb_legs_detected; loop_leg1++) {
            for (int loop_leg2 = loop_leg1 + 1; loop_leg2 < nb_legs_detected; loop_leg2++) {
                if (distancePoints(cluster_middle[loop_leg1], cluster_middle[loop_leg2]) < legs_distance_max) {
                    // Increment the number of people detected
                    nb_persons_detected++;

                    // Compute the middle of the person
                    float middle_x = (cluster_middle[loop_leg1].x +
                                      cluster_middle[loop_leg2].x) /
                                     2;
                    float middle_y = (cluster_middle[loop_leg1].y +
                                      cluster_middle[loop_leg2].y) /
                                     2;

                    geometry_msgs::Point middle = {};
                    middle.x = middle_x;
                    middle.y = middle_y;

                    person_detected[nb_persons_detected] = middle;

                    // if person is moving, then update the person_dynamic table
                    if (leg_dynamic[loop_leg1] && leg_dynamic[loop_leg2]) {
                        person_dynamic[nb_persons_detected] = true;
                    } else {
                        person_dynamic[nb_persons_detected] = false;
                    }

                    if (person_dynamic[nb_persons_detected]) {
                        ROS_INFO("moving person detected: leg[%i]+leg[%i] -> (%f,%f)", loop_leg1, loop_leg2, person_detected[nb_persons_detected].x,
                                 person_detected[nb_persons_detected].y);
                        // a moving person detected is green
                        display[nb_pts] = person_detected[nb_persons_detected];

                        colors[nb_pts].r = 0;
                        colors[nb_pts].g = 1;
                        colors[nb_pts].b = 0;
                        colors[nb_pts].a = 1.0;

                        nb_pts++;
                    } else {
                        ROS_INFO("static person detected: leg[%i]+leg[%i] -> (%f,%f)", loop_leg1, loop_leg2, person_detected[nb_persons_detected].x,
                                 person_detected[nb_persons_detected].y);
                        // a static person detected is red
                        display[nb_pts] = person_detected[nb_persons_detected];

                        colors[nb_pts].r = 1;
                        colors[nb_pts].g = 0;
                        colors[nb_pts].b = 0;
                        colors[nb_pts].a = 1.0;

                        nb_pts++;
                    }

                    nb_persons_detected++;
                }
            }
        }

        if (nb_persons_detected) {
            ROS_INFO("%d persons have been detected.\n", nb_persons_detected);
        }

        ROS_INFO("persons detected");

    }  // detect_persons

    void detect_a_moving_person() {
        ROS_INFO("detecting a moving person");

        for (int loop = 0; loop < nb_persons_detected; loop++) {
            if (person_dynamic[loop]) {
                // we update moving_person_tracked and publish it
                /*    moving_person_detected = ... */
                moving_person_detected = person_detected[loop];
                pub_detection_node.publish(moving_person_detected);
            }
        }

        ROS_INFO("detecting a moving person done");

    }  // detect_moving_person

    // CALLBACKS
    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        init_laser = true;

        // store the important data related to laserscanner
        range_min = scan->range_min;
        range_max = scan->range_max;
        angle_min = scan->angle_min;
        angle_max = scan->angle_max;
        angle_inc = scan->angle_increment;
        nb_beams = ((-1 * angle_min) + angle_max) / angle_inc;

        // store the range and the coordinates in cartesian framework of each
        // hit
        float beam_angle = angle_min;
        for (int loop = 0; loop < nb_beams; loop++, beam_angle += angle_inc) {
            if ((scan->ranges[loop] < range_max) &&
                (scan->ranges[loop] > range_min))
                r[loop] = scan->ranges[loop];
            else
                r[loop] = range_max;
            theta[loop] = beam_angle;

            // transform the scan in cartesian framewrok
            current_scan[loop].x = r[loop] * cos(beam_angle);
            current_scan[loop].y = r[loop] * sin(beam_angle);
            current_scan[loop].z = 0.0;
            // ROS_INFO("laser[%i]: (%f, %f) -> (%f, %f)", loop, range[loop],
            // beam_angle*180/M_PI, current_scan[loop].x, current_scan[loop].y);
        }

    }  // scanCallback

    void robot_movingCallback(const std_msgs::Bool::ConstPtr& state) {
        init_robot = true;
        current_robot_moving = state->data;

    }  // robot_movingCallback

    // Distance between two points
    float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {
        return sqrt(pow((pa.x - pb.x), 2.0) + pow((pa.y - pb.y), 2.0));
    }

    // Draw the field of view and other references
    void populateMarkerReference() {
        visualization_msgs::Marker references;

        references.header.frame_id = "laser";
        references.header.stamp = ros::Time::now();
        references.ns = "one_person_detector_tracker";
        references.id = 1;
        references.type = visualization_msgs::Marker::LINE_STRIP;
        references.action = visualization_msgs::Marker::ADD;
        references.pose.orientation.w = 1;

        references.scale.x = 0.02;

        references.color.r = 1.0f;
        references.color.g = 1.0f;
        references.color.b = 1.0f;
        references.color.a = 1.0;
        geometry_msgs::Point v;

        v.x = 0.02 * cos(-2.356194);
        v.y = 0.02 * sin(-2.356194);
        v.z = 0.0;
        references.points.push_back(v);

        v.x = 5.6 * cos(-2.356194);
        v.y = 5.6 * sin(-2.356194);
        v.z = 0.0;
        references.points.push_back(v);

        float beam_angle = -2.356194 + 0.006136;
        // first and last beam are already included
        for (int i = 0; i < 723; i++, beam_angle += 0.006136) {
            v.x = 5.6 * cos(beam_angle);
            v.y = 5.6 * sin(beam_angle);
            v.z = 0.0;
            references.points.push_back(v);
        }

        v.x = 5.6 * cos(2.092350);
        v.y = 5.6 * sin(2.092350);
        v.z = 0.0;
        references.points.push_back(v);

        v.x = 0.02 * cos(2.092350);
        v.y = 0.02 * sin(2.092350);
        v.z = 0.0;
        references.points.push_back(v);

        pub_detection_marker.publish(references);
    }

    void populateMarkerTopic() {
        visualization_msgs::Marker marker;

        marker.header.frame_id = "laser";
        marker.header.stamp = ros::Time::now();
        marker.ns = "one_person_detector_tracker";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.orientation.w = 1;

        marker.scale.x = 0.05;
        marker.scale.y = 0.05;

        marker.color.a = 1.0;

        // ROS_INFO("%i points to display", nb_pts);
        for (int loop = 0; loop < nb_pts; loop++) {
            geometry_msgs::Point p;
            std_msgs::ColorRGBA c;

            p.x = display[loop].x;
            p.y = display[loop].y;
            p.z = display[loop].z;

            c.r = colors[loop].r;
            c.g = colors[loop].g;
            c.b = colors[loop].b;
            c.a = colors[loop].a;

            // ROS_INFO("(%f, %f, %f) with rgba (%f, %f, %f, %f)", p.x, p.y,
            // p.z, c.r, c.g, c.b, c.a);
            marker.points.push_back(p);
            marker.colors.push_back(c);
        }

        pub_detection_marker.publish(marker);
        populateMarkerReference();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "detection_node");

    ROS_INFO("waiting for activation of detection");
    detection_node bsObject;

    ros::spin();

    return 0;
}
