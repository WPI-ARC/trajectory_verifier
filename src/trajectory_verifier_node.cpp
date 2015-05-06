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
                                         | moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS
                                         | moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
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

    trajectory_verifier::CheckTrajectoryValidityResult CheckTrajectoryValidity(const trajectory_verifier::CheckTrajectoryValidityQuery& query)
    {
        trajectory_verifier::CheckTrajectoryValidityResult result;
        result.status = trajectory_verifier::CheckTrajectoryValidityResult::SUCCESS;
        // Safety check the trajectory first
        if (query.initial_state.name.size() != query.initial_state.position.size())
        {
            ROS_ERROR("Initial state is invalid");
            result.status |= trajectory_verifier::CheckTrajectoryValidityResult::INVALID_PARAMETERS;
        }
        else
        {
            collision_detection::CollisionRequest col_req;
            collision_detection::CollisionResult col_res;
            robot_state::RobotState& robot_state = planning_scene_ptr_->getCurrentStateNonConst();
            // Set the initial state
            for (size_t jdx = 0; jdx < query.initial_state.name.size(); jdx++)
            {
                robot_state.setJointPositions(query.initial_state.name[jdx], &(query.initial_state.position[jdx]));
            }
            // Loop through the trajectory
            const std::vector<std::string>& trajectory_joint_names = query.trajectory.joint_names;
            for (size_t idx = 0; idx < query.trajectory.points.size(); idx++)
            {
                const trajectory_msgs::JointTrajectoryPoint& current_point = query.trajectory.points[idx];
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
        }
        // Return the result
        return result;
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
