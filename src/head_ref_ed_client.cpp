#include <actionlib/client/action_client.h>
#include "head_ref/HeadReferenceAction.h"

#include <ed/SimpleQuery.h>

#include <geolib/datatypes.h>
#include <geolib/ros/tf_conversions.h>
#include <geolib/ros/msg_conversions.h>

#include <tf/transform_listener.h>

struct Comp
{
    bool operator()(const ed::EntityInfo& a, const ed::EntityInfo& b)
    {
        geo::Vector3 a_cp, b_cp, a_diff, b_diff;
        geo::convert(a.center_point, a_cp);
        geo::convert(b.center_point, b_cp);

        a_diff = a_cp - p; a_diff.x = 0;
        b_diff = b_cp - p; b_diff.x = 0;

        return ( a_diff.length2() < b_diff.length2() );
    }

    geo::Vector3 p;

};

typedef actionlib::ActionClient<head_ref::HeadReferenceAction> HeadReferenceActionClient;

tf::TransformListener* tf_listener;
bool getRobotPoseMap(const ros::Time& time_stamp, geo::Pose3D& robot_pose)
{
    try {
        tf::StampedTransform t_robot_pose;
        tf_listener->lookupTransform("/map", "/amigo/base_link", time_stamp, t_robot_pose);
        geo::convert(t_robot_pose, robot_pose);
    } catch(tf::TransformException& ex) {
        return false;
    }
    return true;
}

bool inFrontOf(const geo::Vector3& p, const geo::Pose3D& pose)
{
    // Transform point
    geo::Vector3 p_transformed = pose.inverse() * p;

    // Check if it is in front (x-axis defined as front)
    return (p_transformed.x > 0 && fabs(p_transformed.y) < p_transformed.x);
}

int main(int argc, char** argv){

    ros::init(argc, argv, "ed_entity_checker");

    ros::NodeHandle nh;

    ros::ServiceClient ed_client = nh.serviceClient<ed::SimpleQuery>("/ed/simple_query");
    actionlib::ActionClient<head_ref::HeadReferenceAction> ac("HeadReference");
    tf_listener = new tf::TransformListener(nh);

    HeadReferenceActionClient::GoalHandle gh;

    ros::Rate r(1);
    while (ros::ok()) {

        //! Request the pose of the robot in the map frame
        geo::Pose3D robot_pose;
        if (getRobotPoseMap(ros::Time(0), robot_pose))
        {
            //! Request and response
            ed::SimpleQueryRequest req;
            ed::SimpleQueryResponse resp;

            //! Fill the request
            geo::convert(robot_pose.getOrigin(), req.center_point);
            req.radius = 3;
            req.type = "unknown";

            if ( ed_client.call(req, resp) )
            {
                if (resp.entities.size() == 0) {
//                    std::cout << "No entities in response" << std::endl;
                    continue;
                }

                // Sort the response
                Comp comp;
                comp.p = robot_pose.getOrigin();
                std::sort(resp.entities.begin(), resp.entities.end(), comp);

                bool found = false;
                for (std::vector<ed::EntityInfo>::const_iterator it = resp.entities.begin(); it != resp.entities.end(); ++it)
                {
                    const ed::EntityInfo& e = *it;

                    geo::Vector3 cp;
                    geo::convert(e.center_point, cp);

                    if (!e.has_shape && inFrontOf(cp, robot_pose))
                    {
//                        std::cout << "View target: " << e.id << std::endl;

                        // send a goal to the action
                        head_ref::HeadReferenceGoal goal;
                        goal.target_point.header.frame_id = "/map";
                        goal.target_point.header.stamp = ros::Time::now();
                        goal.target_point.point = e.center_point;
                        goal.goal_type = head_ref::HeadReferenceGoal::LOOKAT;

                        goal.priority = 100;

                        gh = ac.sendGoal(goal);
                        found = true;
                        break; // Only closest one
                    }
                }
                if (!found) {
//                    std::cout << "No target, cancelling all goals" << std::endl;
                    ac.cancelAllGoals();
                }
            }
        }

        r.sleep();
    }

    // Cancel the goal when loop is interupted with key press for example
    gh.cancel();

    // Delete tf listener
    delete tf_listener;

    return 0;
}
