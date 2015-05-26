#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <urdf_model/model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include "trajectory_verifier/CheckTrajectoryValidity.h"
#include "trajectory_verifier/BatchCheckTrajectoryValidity.h"

class TrajectoryVerifier
{
protected:

    std::shared_ptr<planning_scene::PlanningScene> planning_scene_ptr_;
    ros::NodeHandle nh_;
    ros::ServiceClient planning_scene_client_;
    ros::ServiceServer trajectory_validity_server_;
    ros::ServiceServer batch_trajectory_validity_server_;

public:

    TrajectoryVerifier(ros::NodeHandle& nh, const std::string& planning_scene_service, const std::string& trajectory_validity_service, const std::string& batch_trajectory_validity_service) : nh_(nh)
    {
        robot_model_loader::RobotModelLoader loader;
        planning_scene_ptr_.reset();
        planning_scene_ptr_ = std::shared_ptr<planning_scene::PlanningScene>(new planning_scene::PlanningScene(loader.getModel()));
        planning_scene_client_ = nh.serviceClient<moveit_msgs::GetPlanningScene>(planning_scene_service);
        trajectory_validity_server_ = nh.advertiseService(trajectory_validity_service, &TrajectoryVerifier::CheckTrajectoryValidityCB, this);
        batch_trajectory_validity_server_ = nh.advertiseService(batch_trajectory_validity_service, &TrajectoryVerifier::BatchCheckTrajectoryValidityCB, this);
    }

    void Loop()
    {
        ros::Rate spin_rate(10.0);
        while (ros::ok())
        {
            ros::spinOnce();
            spin_rate.sleep();
        }
    }

    bool UpdateInternalPlanningScene()
    {
        try
        {
            // Update the planning scene
            moveit_msgs::GetPlanningSceneRequest ps_req;
            ps_req.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES
                                         | moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY
                                         | moveit_msgs::PlanningSceneComponents::OCTOMAP
                                         | moveit_msgs::PlanningSceneComponents::ROBOT_STATE
                                         | moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS
                                         | moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX
                                         | moveit_msgs::PlanningSceneComponents::SCENE_SETTINGS
                                         | moveit_msgs::PlanningSceneComponents::TRANSFORMS
                                         | moveit_msgs::PlanningSceneComponents::OBJECT_COLORS
                                         | moveit_msgs::PlanningSceneComponents::LINK_PADDING_AND_SCALING;
            moveit_msgs::GetPlanningSceneResponse ps_res;
            planning_scene_client_.call(ps_req, ps_res);
            moveit_msgs::PlanningScene& planning_scene_state = ps_res.scene;
            planning_scene_ptr_->usePlanningSceneMsg(planning_scene_state);
            return true;
        }
        catch (...)
        {
            ROS_ERROR("Failed updating planning scene");
            return false;
        }
    }


    void PrintCollisions(const collision_detection::CollisionResult& col_res) const
    {
        const collision_detection::CollisionResult::ContactMap& contacts = col_res.contacts;
        collision_detection::CollisionResult::ContactMap::const_iterator contacts_itr;
        for (contacts_itr = contacts.begin(); contacts_itr != contacts.end(); ++contacts_itr)
        {
            ROS_WARN("Contact detected between %s and %s", contacts_itr->first.first.c_str(), contacts_itr->first.second.c_str());
            for (size_t idx = 0; idx < contacts_itr->second.size(); idx++)
            {
                ROS_WARN("Details - contact between %s and %s at position (%f,%f,%f) with penetration %f", contacts_itr->second[idx].body_name_1.c_str(), contacts_itr->second[idx].body_name_2.c_str(), contacts_itr->second[idx].pos.x(), contacts_itr->second[idx].pos.y(), contacts_itr->second[idx].pos.z(), contacts_itr->second[idx].depth);
            }
        }
    }

    std::vector<double> Interpolate(const std::vector<double>& p1, const std::vector<double>& p2, const double fraction) const
    {
        if (p1.size() != p2.size())
        {
            throw std::invalid_argument("Cannot interpolated between different-sized vectors");
        }
        else
        {
            std::vector<double> interpolated(p1.size());
            for (size_t idx = 0; idx < interpolated.size(); idx++)
            {
                const double p1_val = p1[idx];
                const double p2_val = p2[idx];
                interpolated[idx] = p1_val + ((p2_val - p1_val) * fraction);
            }
            return interpolated;
        }
    }

    double Interpolate(const double v1, const double v2, const double fraction) const
    {
        double interped = v1 + ((v2 - v1) * fraction);
        return interped;
    }

    trajectory_msgs::JointTrajectoryPoint Interpolate(const trajectory_msgs::JointTrajectoryPoint& p1, const trajectory_msgs::JointTrajectoryPoint& p2, const double fraction) const
    {
        // Interpolate the joint angles between the two points provided
        trajectory_msgs::JointTrajectoryPoint interpolated_point;
        interpolated_point.positions = Interpolate(p1.positions, p2.positions, fraction);
        interpolated_point.velocities = Interpolate(p1.velocities, p2.velocities, fraction);
        interpolated_point.accelerations = Interpolate(p1.accelerations, p2.accelerations, fraction);
        interpolated_point.effort = Interpolate(p1.effort, p2.effort, fraction);
        interpolated_point.time_from_start = ros::Duration(Interpolate(p1.time_from_start.toSec(), p2.time_from_start.toSec(), fraction));
        return interpolated_point;
    }

    std::pair<trajectory_msgs::JointTrajectory, bool> InterpolateCompleteTrajectory(const trajectory_msgs::JointTrajectory& original_traj) const
    {
        trajectory_msgs::JointTrajectory interpolated_trajectory;
        interpolated_trajectory.joint_names = original_traj.joint_names;
        if (original_traj.points.size() < 2)
        {
            ROS_WARN("Trajectory has fewer than 2 points, not interpolating for finer collision checks");
            interpolated_trajectory.points = original_traj.points;
            return std::pair<trajectory_msgs::JointTrajectory, bool>(interpolated_trajectory, true);
        }
        else
        {
            // Add the start point
            interpolated_trajectory.points.push_back(original_traj.points[0]);
            for (size_t idx = 1; idx < original_traj.points.size(); idx++)
            {
                // Grab the current and previous points
                const trajectory_msgs::JointTrajectoryPoint& prev_point = original_traj.points[idx - 1];
                const trajectory_msgs::JointTrajectoryPoint& cur_point = original_traj.points[idx];
                // Add interpolated points between them
                interpolated_trajectory.points.push_back(Interpolate(prev_point, cur_point, 0.25));
                interpolated_trajectory.points.push_back(Interpolate(prev_point, cur_point, 0.5));
                interpolated_trajectory.points.push_back(Interpolate(prev_point, cur_point, 0.75));
                interpolated_trajectory.points.push_back(Interpolate(prev_point, cur_point, 1.0));
            }
            return std::pair<trajectory_msgs::JointTrajectory, bool>(interpolated_trajectory, true);
        }
    }

    trajectory_verifier::CheckTrajectoryValidityResult CheckTrajectoryValidity(const trajectory_verifier::CheckTrajectoryValidityQuery& query)
    {
        trajectory_verifier::CheckTrajectoryValidityResult result;
        result.status = trajectory_verifier::CheckTrajectoryValidityResult::SUCCESS;
        // Safety check the trajectory first
        if (query.initial_state.name.size() != query.initial_state.position.size())
        {
            ROS_ERROR("Initial state is invalid");
            result.status |= trajectory_verifier::CheckTrajectoryValidityResult::INVALID_PARAMETERS;
            return result;
        }
        std::pair<trajectory_msgs::JointTrajectory, bool> interpolated_trajectory_query = InterpolateCompleteTrajectory(query.trajectory);
        if (!interpolated_trajectory_query.second)
        {
            ROS_ERROR("Interpolation of complete trajectory failed");
            result.status |= trajectory_verifier::CheckTrajectoryValidityResult::INVALID_PARAMETERS;
            return result;
        }
        // Verify the interpolated trajectory
        else
        {
            collision_detection::CollisionRequest col_req;
            col_req.contacts = true;
            collision_detection::CollisionResult col_res;
            robot_state::RobotState& robot_state = planning_scene_ptr_->getCurrentStateNonConst();
            // Set the initial state
            for (size_t jdx = 0; jdx < query.initial_state.name.size(); jdx++)
            {
                robot_state.setJointPositions(query.initial_state.name[jdx], &(query.initial_state.position[jdx]));
            }
            // Loop through the trajectory
            const std::vector<std::string>& trajectory_joint_names = interpolated_trajectory_query.first.joint_names;
            for (size_t idx = 0; idx < interpolated_trajectory_query.first.points.size(); idx++)
            {
                const trajectory_msgs::JointTrajectoryPoint& current_point = interpolated_trajectory_query.first.points[idx];
                if (current_point.positions.size() == trajectory_joint_names.size())
                {
                    // Update the robot state
                    for (size_t jdx = 0; jdx < trajectory_joint_names.size(); jdx++)
                    {
                        robot_state.setJointPositions(trajectory_joint_names[jdx], &(current_point.positions[jdx]));
                    }
                    // Check for collisions (if check_type > 0, we also check environemnt collisions
                    if (query.check_type > 0)
                    {
                        col_res.clear();
                        planning_scene_ptr_->checkCollision(col_req, col_res);
                        if (col_res.collision)
                        {
                            PrintCollisions(col_res);
                            ROS_WARN("Trajectory invalid due to environment collision at state %zu of %zu", idx + 1, query.trajectory.points.size());
                            result.status |= trajectory_verifier::CheckTrajectoryValidityResult::ENVIRONMENT_COLLISION;
                            break;
                        }
                    }
                    else
                    {
                        col_res.clear();
                        planning_scene_ptr_->checkSelfCollision(col_req, col_res);
                        if (col_res.collision)
                        {
                            PrintCollisions(col_res);
                            ROS_WARN("Trajectory invalid due to self collision at state %zu of %zu", idx + 1, query.trajectory.points.size());
                            result.status |= trajectory_verifier::CheckTrajectoryValidityResult::SELF_COLLISION;
                            break;
                        }
                    }
                }
                else
                {
                    result.status |= trajectory_verifier::CheckTrajectoryValidityResult::INVALID_PARAMETERS;
                    break;
                }
            }
            // Return the result
            return result;
        }
    }

    bool CheckTrajectoryValidityCB(trajectory_verifier::CheckTrajectoryValidity::Request& req, trajectory_verifier::CheckTrajectoryValidity::Response& res)
    {
        ROS_INFO("Processing trajectory validity service...");
        ROS_INFO("Updating planning scene...");
        bool got_planning_scene = UpdateInternalPlanningScene();
        if (!got_planning_scene)
        {
            res.result.status = trajectory_verifier::CheckTrajectoryValidityResult::PLANNING_SCENE_UPDATE_FAILED;
        }
        ROS_INFO("Checking trajectory validity...");
        res.result = CheckTrajectoryValidity(req.query);
        ROS_INFO("...done");
        return true;
    }

    bool BatchCheckTrajectoryValidityCB(trajectory_verifier::BatchCheckTrajectoryValidity::Request& req, trajectory_verifier::BatchCheckTrajectoryValidity::Response& res)
    {
        ROS_INFO("Processing batch trajectory validity service...");
        ROS_INFO("Updating planning scene...");
        bool got_planning_scene = UpdateInternalPlanningScene();
        ROS_INFO("Checking trajectory validity for %zu trajectories", req.queries.size());
        res.results.resize(req.queries.size());
        for (size_t idx = 0; idx < req.queries.size(); idx++)
        {
            if (!got_planning_scene)
            {
                res.results[idx].status = trajectory_verifier::CheckTrajectoryValidityResult::PLANNING_SCENE_UPDATE_FAILED;
            }
            else
            {
                res.results[idx] = CheckTrajectoryValidity(req.queries[idx]);
            }
        }
        ROS_INFO("...done");
        return true;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_verifier");
    ROS_INFO("Starting trajectory verifier...");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string planning_scene_service;
    std::string trajectory_validity_service;
    std::string batch_trajectory_validity_service;
    nhp.param(std::string("planning_scene_service"), planning_scene_service, std::string("get_planning_scene"));
    nhp.param(std::string("trajectory_validity_service"), trajectory_validity_service, std::string("check_trajectory_validity"));
    nhp.param(std::string("batch_trajectory_validity_service"), batch_trajectory_validity_service, std::string("batch_check_trajectory_validity"));
    TrajectoryVerifier verifier(nh, planning_scene_service, trajectory_validity_service, batch_trajectory_validity_service);
    ROS_INFO("...startup complete");
    verifier.Loop();
    return 0;
}
