#include <tf/transform_datatypes.h>

#include <cmath>
#include <fstream>
#include <csignal>
#include <stdio.h>
#include <iostream>

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

#define detection_threshold 0.2 // threshold for motion detection -
#define dynamic_threshold 75    // to decide if a cluster is static or dynamic

// threshold for clustering
#define cluster_threshold 0.2

// used for detection of leg
#define leg_size_min 0.05
#define leg_size_max 0.25 // Note: for the rectangle sizes, it helped a bit when we changed from 25 to 50cm
#define legs_distance_min 0.1
#define legs_distance_max 0.7

// used for uncertainty of leg
#define uncertainty_min_leg 0.5
#define uncertainty_max_leg 1
#define uncertainty_inc_leg 0.05

using namespace std;

// When outputting CSV logs, silence standard progress messages
// #define LOG_CSV 1 // When commented, does not generate the CSV logs and enable the ROS logs

#ifdef LOG_CSV
// undefine ROS_INFO
#undef ROS_INFO
// define ROS_INFO to be silent
#define ROS_INFO(...) \
    do                \
    {                 \
    } while (0);

#undef ROS_WARN
// define ROS_WARN to be silent
#define ROS_WARN(...) \
    do                \
    {                 \
    } while (0);
#endif

std::vector<int> num_persons_detected;
std::vector<int> num_moving_detected;
std::vector<int> num_legs_detected;  // unused
int t = 0;  // time tick
int k = -1; // lines written into csv, -1 is intentional

class detection_node
{
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
    bool stored_background; // true = valid
    bool dynamic[1000];
    bool current_robot_moving;
    bool previous_robot_moving;

    /*
    the middle of a person is calculated as the mean of the legs clusters
    but we do not keep track of the leg cluster centers explicitly,

    */
    // to perform clustering
    int nb_cluster;           // number of cluster
    int cluster[1000];        // to store for each hit, the cluster it belongs to
    float cluster_size[1000]; // to store the size of each cluster
    int cluster_count[1000];  // to store the number of points in each cluster

    geometry_msgs::Point
        cluster_middle[1000];  // to store the middle of each cluster
    int cluster_dynamic[1000]; // to store the percentage of the cluster that
                               // is dynamic

    // to perform detection of legs and to store them
    int nb_legs_detected;
    geometry_msgs::Point leg_detected[1000];
    int leg_cluster[1000]; // to store the cluster corresponding to a leg
    int leg_cluster_center_idx;
    bool leg_dynamic[1000]; // to know if a leg is dynamic or not

    // to perform detection of a moving person and store it
    int nb_persons_detected;
    geometry_msgs::Point person_detected[1000];
    bool person_dynamic[1000];
    geometry_msgs::Point
        moving_person_detected; // to store the coordinates of the moving
                                // person that we have detected

    // GRAPHICAL DISPLAY
    int nb_pts;
    geometry_msgs::Point display[1000];
    std_msgs::ColorRGBA colors[1000];

public:
    detection_node()
    {
        sub_scan = n.subscribe("scan", 1, &detection_node::scanCallback, this);
        sub_robot_moving = n.subscribe(
            "robot_moving", 1, &detection_node::robot_movingCallback, this);

        // communication with action
        pub_detection_node = n.advertise<geometry_msgs::Point>(
            "goal_to_reach",
            1); // Preparing a topic to publish the goal to reach.
        pub_detection_marker = n.advertise<visualization_msgs::Marker>(
            "detection_marker",
            1); // Preparing a topic to publish our results. This will be used
                // by the visualization tool rviz

        init_laser = false;
        init_robot = false;
        previous_robot_moving = true;

        ros::Rate r(10);

        while (ros::ok())
        {
            ros::spinOnce();
            update();
            r.sleep();
        }
    }

    // UPDATE
    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
    void update()
    {
        // we wait for new data of the laser and of the robot_moving_node to
        // perform laser processing
        if (init_laser && init_robot)
        {
            // ROS_INFO("\n");
            // ROS_INFO("New data of laser received");
            // ROS_INFO("New data of robot_moving received");

            nb_pts = 0;
            if (!current_robot_moving)
            {
                // if the robot is not moving then we can perform moving person
                // detection DO NOT FORGET to store the background but when ???
                // ROS_INFO("robot is not moving");

                if (!stored_background)
                {
                    store_background();
                    stored_background = true;
                }
                else
                {
                    detect_motion();
                    // we search for moving person in 4 steps
                    perform_clustering(); // to perform clustering
                    detect_legs();        // to detect moving legs using cluster
                    detect_persons();     // to detect moving_person using moving legs
                    detect_a_moving_person();
                }
            }
            else
            {
                // IMPOSSIBLE TO DETECT MOTIONS because the base is moving
                // what is the value of dynamic table for each hit of the laser
                // ?
                ROS_INFO("robot is moving");

                // clear background
                stored_background = false;
            }
            previous_robot_moving = current_robot_moving;

            // graphical display of the results
            populateMarkerTopic();
            t++; // increase time tick
        }
        else if (!init_robot)
        {
            ROS_WARN("waiting for robot_moving_node");
            ROS_WARN("launch: rosrun follow_me robot_moving_node");
        }
    } // update

    // DETECT MOTION FOR BOTH LASER
    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
    void store_background()
    {
        // store all the hits of the laser in the background table

        ROS_INFO("storing background");

        for (int loop = 0; loop < nb_beams; loop++)
        {
            background[loop] = r[loop];
        }

        ROS_INFO("background stored");

    } // store_background

    void reset_motion()
    {
        // for each hit, compare the current range with the background to detect
        // motion

        ROS_INFO("reset motion");
        for (int loop = 0; loop < nb_beams; loop++)
            dynamic[loop] = false;

        ROS_INFO("reset_motion done");

    } // reset_motion

    void detect_motion()
    {
        // ROS_INFO("detecting motion");
        // ROS_INFO("detecting moving points: %f, %f, %f", r[360], background[360], abs(background[360] - r[360]));
        for (int loop = 0; loop < nb_beams; loop++)
        {
            // loop over all the hits
            //  if the difference between ( the background and the current value
            //  r ) is higher than "detection_threshold"
            float diff = fabs(background[loop] - r[loop]);

            if (diff > detection_threshold * (1 + 0.2 * fabs(background[loop]))) // remember change from assignment here, 20cm at 0m, 40cm at 5m
            {
                dynamic[loop] = true; // the current hit is dynamic
                // ROS_INFO("motion detected");
            }
            else
            {
                dynamic[loop] = false;
            }

            // WARNING COMMENTED OUT LOOP PRINTING BASIC HITS
            // These are displayed in BLUE
            // if (dynamic[loop])
            // {
            //     // ROS_INFO("hit[%i](%f, %f) is dynamic", loop,
            //     //          current_scan[loop].x, current_scan[loop].y);

            //     // display in blue of hits that are dynamic
            //     display[nb_pts] = current_scan[loop];

            //     colors[nb_pts].r = 0;
            //     colors[nb_pts].g = 0;
            //     colors[nb_pts].b = 1;
            //     colors[nb_pts].a = 1.0;

            //     nb_pts++;
            // }
        }

    } // detect_motion

    // CLUSTERING
    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
    void perform_clustering()
    {
        // store in the table cluster, the cluster of each hit of the laser
        // if the distance between the previous hit and the current one is
        // higher than "cluster_threshold" then we end the current cluster with
        // the previous hit and start a new cluster with the current hit else
        // the current hit belongs to the current cluster

        // ROS_INFO("performing clustering");

        nb_cluster = 0; // to count the number of cluster

        int start = 0; // to store the index of the start of current cluster
        int end;       // to store the index of the end of the current cluster

        // graphical display of the start of the current cluster in green
        if (false)
        {
            display[nb_pts] = current_scan[start];

            colors[nb_pts].r = 0;
            colors[nb_pts].g = 1;
            colors[nb_pts].b = 0;
            colors[nb_pts].a = 1.0;
            nb_pts++;
        }

        float middle_x = 0;
        float middle_y = 0;

        for (int loop = 1; loop < nb_beams; loop++)
        { // loop over all the hits
            // if euclidian DISTANCE between the previous hit and the current
            // one is higher than "cluster_threshold"
            float points_dist = distancePoints(current_scan[loop], current_scan[loop - 1]);

            if (points_dist > cluster_threshold || loop == nb_beams - 1)
            {
                /////////////////// LAST CHANGES MADE ///////////////////
                // last cluster -> end the current cluster with the current hit
                // end the current cluster
                if (loop == nb_beams - 1)
                {
                    end = loop;
                }
                else
                {
                    end = loop - 1;
                }
                // The size of the cluster is the euclidean distance between the start point and end point
                cluster_size[nb_cluster] = distancePoints(current_scan[start], current_scan[end]);
                /////////////////////////////////////////////////////////

                // graphical display of the end of the current cluster in red
                if (false)
                {
                    display[nb_pts] = current_scan[end];

                    colors[nb_pts].r = 1;
                    colors[nb_pts].g = 0;
                    colors[nb_pts].b = 0;
                    colors[nb_pts].a = 1.0;
                    nb_pts++; // XYZXYZ
                }

                // compute the middle of the cluster
                middle_x = 0;
                middle_y = 0;

                int points_dynamic = 0;

                for (int j = start; j <= end; j++)
                {
                    middle_x += current_scan[j].x;
                    middle_y += current_scan[j].y;
                    if (dynamic[j])
                    {
                        points_dynamic++;
                    }
                }
                // ROS_INFO("dynamic points: %i/%i", points_dynamic, end - start + 1);
                int current_cluster_count = end - start + 1;

                middle_x /= current_cluster_count;
                middle_y /= current_cluster_count;

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
                // These are displayed in red pink almost white
                if (false)
                {
                    display[nb_pts] = middle;

                    colors[nb_pts].r = 1;
                    colors[nb_pts].g = 0.6;
                    colors[nb_pts].b = 0.6;
                    colors[nb_pts].a = 1.0;
                    nb_pts++;
                }

                cluster_middle[nb_cluster] = middle;
                cluster_dynamic[nb_cluster] =
                    100 * (int)((double)(points_dynamic) /
                                (double)(abs(end - start) + 1)); //  % of dynamic points
                                                                 //  within a cluster

                // the current hit is the start of the next cluster
                // we start the next cluster
                start = loop;

                // graphical display of the start of the current cluster in
                // These are displayed in green
                if (false)
                {
                    display[nb_pts] = current_scan[start];

                    colors[nb_pts].r = 0;
                    colors[nb_pts].g = 1;
                    colors[nb_pts].b = 0;
                    colors[nb_pts].a = 1.0;
                    nb_pts++;
                }

                cluster_count[nb_cluster] = current_cluster_count;

                // ROS_INFO("start: (%f, %f), end: (%f, %f), distance: %f", current_scan[start].x, current_scan[start].y, current_scan[end].x, current_scan[end].y, cluster_size[nb_cluster]);

                // increase cluster number
                nb_cluster++;
            }

            cluster[loop] = nb_cluster;
        }

    } // perform_clustering

    // DETECTION OF A MOVING PERSON
    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
    void detect_legs()
    {
        // a leg is a cluster:
        // - with a size higher than "leg_size_min";
        // - with a size lower than "leg_size_max;
        // if more than "dynamic_threshold"% of its hits are dynamic the leg is
        // considered to be dynamic

        // ROS_INFO("detecting legs");
        nb_legs_detected = 0;

        nb_pts = 0;
        int sum_cluster_counts = 0;
        for (int loop = 0; loop < nb_cluster; loop++)
        {
            // loop over all the clusters
            //      if the size of the current cluster is higher than
            //      "leg_size_min" and
            //  lower than "leg_size_max" then the current cluster is a leg

            // Note: old way of calculating leg size
            // if (size_x > leg_size_min && size_x < leg_size_max &&
            //     size_y > leg_size_min && size_y < leg_size_max &&
            //     (size_x + size_y > 0.10)) // NOTE: own condition

            if (cluster_size[loop] < leg_size_max && cluster_size[loop] > leg_size_min)
            {
                // we update:
                // - the leg_detected table to store the middle of the moving
                // leg;
                // - the leg_cluster to store the cluster corresponding to a
                // leg;
                // - the leg_dynamic to know if the leg is dynamic or not.
                leg_detected[nb_legs_detected] = cluster_middle[loop];
                leg_cluster[nb_legs_detected] = loop;
                leg_dynamic[nb_legs_detected] = (cluster_dynamic[loop] >= 75);

                // leg cluster will contain the ID of the cluster
                // in cluster_middle[ID] there is the center of this leg cluster

                if (leg_dynamic[nb_legs_detected])
                {
                    // ROS_INFO("moving leg found: %i -> cluster = %i, (%f, %f),
                    // size: %f, dynamic: %i", nb_legs_detected, loop,
                    // leg_detected[nb_legs_detected].x,
                    // leg_detected[nb_legs_detected].y,
                    // cluster_size[loop],
                    // cluster_dynamic[loop]);

                    for (int loop2 = 0; loop2 < nb_beams; loop2++)
                    {
                        if (cluster[loop2] == loop)
                        {
                            // dynamic legs are yellow
                            display[nb_pts] = current_scan[loop2];

                            colors[nb_pts].r = 1;
                            colors[nb_pts].g = 1;
                            colors[nb_pts].b = 0;
                            colors[nb_pts].a = 1.0;

                            nb_pts++;
                        }
                    }
                }
                else
                {
                    // ROS_INFO("static leg found: %i -> cluster = %i, (%f, %f),
                    // size: %f, dynamic: %i", nb_legs_detected, loop,
                    // leg_detected[nb_legs_detected].x,
                    // leg_detected[nb_legs_detected].y,
                    // cluster_size[loop],
                    // cluster_dynamic[loop]);
                    for (int loop2 = 0; loop2 < nb_beams; loop2++)
                    {
                        if (cluster[loop2] == loop)
                        {
                            // static legs are white
                            // display[nb_pts] = current_scan[loop2];

                            // colors[nb_pts].r = 1;
                            // colors[nb_pts].g = 1;
                            // colors[nb_pts].b = 1;
                            // colors[nb_pts].a = 1.0;

                            // nb_pts++;
                        }
                    }
                }

                nb_legs_detected++;
            }

            nb_pts++; // actually should be here - otherwise "Points should not be empty in rviz"
        }

        if (nb_legs_detected)
        {
            ROS_INFO("%d legs have been detected.\n", nb_legs_detected);
        }
        num_legs_detected.push_back(nb_legs_detected);

        // ROS_INFO("detecting legs done");

    } // detect_legs

    void detect_persons()
    {
        int nb_moving_persons = 0;
        // a person has two legs located at less than "legs_distance_max" one
        // from the other a moving person (ie, person_dynamic array) has 2 legs
        // that are dynamic

        // ROS_INFO("detecting persons");
        nb_persons_detected = 0;

        // Iterate through all the legs and compare the distance between each
        // If the distance is lower than "legs_distance_max", we have a person.
        std::set<int> legs_used;
        for (int loop_leg1 = 0; loop_leg1 < nb_legs_detected; loop_leg1++)
        {
            auto leg1_middle = cluster_middle[leg_cluster[loop_leg1]]; // == leg_detected[loop_leg1] // hopefully
            if (legs_used.find(loop_leg1) != legs_used.end())
            {
                continue;
            }
            for (int loop_leg2 = loop_leg1 + 1; loop_leg2 < nb_legs_detected; loop_leg2++)
            {
                if (legs_used.find(loop_leg2) != legs_used.end())
                {
                    continue;
                }
                auto leg2_middle = cluster_middle[leg_cluster[loop_leg2]]; // == leg_detected[loop_leg2] // hopefully
                float dist = distancePoints(leg1_middle, leg2_middle);
                // ROS_INFO("distance between legs %d and %d: %f", loop_leg1, loop_leg2, dist);
                if (dist < legs_distance_max && dist > legs_distance_min)
                {
                    // legs_used.insert(loop_leg1);  // assumption: not necessary, since we increase
                    //                                  loop_leg1 variable when a person is found.
                    legs_used.insert(loop_leg2);

                    // Compute the middle of the person
                    float middle_x = (leg1_middle.x +
                                      leg2_middle.x) /
                                     2;
                    float middle_y = (leg1_middle.y +
                                      leg2_middle.y) /
                                     2;
                    float middle_z = (leg1_middle.z +
                                      leg2_middle.z) /
                                     2;

                    geometry_msgs::Point middle = {};
                    middle.x = middle_x;
                    middle.y = middle_y;
                    middle.z = middle_z;

                    person_detected[nb_persons_detected] = middle;

                    // if person is moving, then update the person_dynamic table
                    if (leg_dynamic[loop_leg1] && leg_dynamic[loop_leg2])
                    {
                        person_dynamic[nb_persons_detected] = true;
                    }
                    else
                    {
                        person_dynamic[nb_persons_detected] = false;
                    }

                    if (person_dynamic[nb_persons_detected])
                    {
                        nb_moving_persons++;
                        ROS_INFO("moving person detected: leg[%i]+leg[%i] -> (%f,%f)", loop_leg1, loop_leg2, person_detected[nb_persons_detected].x,
                                 person_detected[nb_persons_detected].y);

                        ROS_INFO("^made up of [%f, %f], [%f, %f]", leg1_middle.x, leg1_middle.y, leg2_middle.x, leg2_middle.y);
                        // a moving person detected is green
                        display[nb_pts] = person_detected[nb_persons_detected];

                        colors[nb_pts].r = 0.2;
                        colors[nb_pts].g = 0.8;
                        colors[nb_pts].b = 0.2;
                        colors[nb_pts].a = 1.0;

                        nb_pts++;
                    }
                    else
                    {
                        ROS_INFO("static person detected: leg[%i]+leg[%i] -> (%f,%f)", loop_leg1, loop_leg2, person_detected[nb_persons_detected].x,
                                 person_detected[nb_persons_detected].y);
                        // a static person detected is red
                        display[nb_pts] = person_detected[nb_persons_detected];

                        colors[nb_pts].r = 0.8;
                        colors[nb_pts].g = 0.2;
                        colors[nb_pts].b = 0.2;
                        colors[nb_pts].a = 1.0;

                        nb_pts++;
                    }

                    nb_persons_detected++;

                    // prevent finding the same leg twice
                    break;
                }
            }
        }

        if (nb_persons_detected)
        {
            ROS_INFO("%d/%d moving/persons detected.\n", nb_moving_persons, nb_persons_detected);
        }

        num_persons_detected.push_back(nb_persons_detected);
        num_moving_detected.push_back(nb_moving_persons);

        // ROS_INFO("persons detected");

    } // detect_persons

    void detect_a_moving_person()
    {
        // ROS_INFO("detecting a moving person");
        int idx_closest = -1;
        float distance_closest = 10000000;
        geometry_msgs::Point robot_position;
        robot_position.x = 0;
        robot_position.y = 0;
        robot_position.z = 0;

        for (int loop = 0; loop < nb_persons_detected; loop++)
        {
            if (person_dynamic[loop])
            {
                // we update moving_person_tracked and publish it
                moving_person_detected = person_detected[loop];
                // pub_detection_node.publish(moving_person_detected);  // Note: commented out

                float distance = distancePoints(moving_person_detected, robot_position);
                if (distance < distance_closest)
                {
                    distance_closest = distance;
                    idx_closest = loop;
                }
            }
        }

        if (idx_closest != -1)
        {
            geometry_msgs::Point person_closest = person_detected[idx_closest];
            ROS_INFO("closest person detected: (%f,%f)", person_closest.x, person_closest.y);
            pub_detection_node.publish(person_closest);

            // display the closest moving person
            display[nb_pts] = person_closest;
            // These are displayed in BLUE
            colors[nb_pts].r = 0;
            colors[nb_pts].g = 0;
            colors[nb_pts].b = 1;
            colors[nb_pts].a = 1.0;

            nb_pts++;
            // display 0, 0

            display[nb_pts] = robot_position;
            // These are displayed in BLUE DARK GREY
            colors[nb_pts].r = 0.2;
            colors[nb_pts].g = 0.2;
            colors[nb_pts].b = 0.5;
            colors[nb_pts].a = 1.0;

            nb_pts++;
        }

        // ROS_INFO("detecting a moving person done");

    } // detect_moving_person

    // CALLBACKS
    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
    {
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
        for (int loop = 0; loop < nb_beams; loop++, beam_angle += angle_inc)
        {
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

    } // scanCallback

    void robot_movingCallback(const std_msgs::Bool::ConstPtr &state)
    {
        init_robot = true;
        current_robot_moving = state->data;

    } // robot_movingCallback

    // Distance between two points
    float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb)
    {
        return sqrt(pow((pa.x - pb.x), 2.0) + pow((pa.y - pb.y), 2.0));
    }

    // Draw the field of view and other references
    void populateMarkerReference()
    {
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
        for (int i = 0; i < 723; i++, beam_angle += 0.006136)
        {
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

    void populateMarkerTopic()
    {
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
        for (int loop = 0; loop < nb_pts; loop++)
        {
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

#ifdef LOG_CSV
void signalHandler(int signum)
{
    cout << "Interrupt signal (" << signum << ") received.\n";

    // Note: not terminating
    if (signum == 2)
    {
        // cleanup and close up stuff here
        std::string str = "";
        str += "time,num_persons_detected,num_moving_detected\n";
        for (int i = 0; i < num_persons_detected.size(); i++)
        {
            str += std::to_string(i) + "," + std::to_string(num_persons_detected.at(i))+ "," + std::to_string(num_moving_detected.at(i)) + "\n";
        }
        // std::cout << str << std::endl;
        std::cout << num_persons_detected.size() << " entries in vector" << std::endl;

        // write everything to a file
        std::ofstream myfile("persons_detected.csv"); // , std::ios_base::app
        myfile << str << std::endl;
        myfile.close();

        // erase whole vector
        num_persons_detected.clear();

        // Note: not terminating
    }
}
#endif // LOG_CSV

int main(int argc, char **argv)
{
#ifdef LOG_CSV
    signal(SIGINT, signalHandler);
    ros::init(argc, argv, "detection_node", ros::init_options::NoSigintHandler);
#endif // LOG_CSV
    ros::init(argc, argv, "detection_node");

    ROS_INFO("waiting for activation of detection");
    detection_node bsObject;

    ros::spin();

    return 0;
}