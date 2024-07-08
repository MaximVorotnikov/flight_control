#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/bool.hpp"

#include "drone_msgs/msg/goal.hpp"
#include "drone_msgs/msg/flight_control_mode.hpp"
#include "drone_msgs/msg/mission.hpp"
#include "drone_msgs/msg/autopilot_command.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/msg/state.hpp"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// #include "ros_timers.hpp"
#include "euler_angles_lib/euler_angles.hpp"

using drone_msgs::msg::Goal, drone_msgs::msg::FlightControlMode, drone_msgs::msg::Mission, drone_msgs::msg::MissionSegment;

class AutopilotOperative
{
    public:
    drone_msgs::msg::GoalTrajectory trajectory;
    int points_in_current_trajectory;
    int current_point_in_trajectory;

    MissionSegment current_mission_segment;
    Mission current_mission;
    Goal current_goal;
    FlightControlMode mode;

    AutopilotOperative(){}
};


class Autopilot : public rclcpp::Node
{
    public:    
    rclcpp::Publisher<drone_msgs::msg::Goal>::SharedPtr goal_pub;
    rclcpp::Publisher<drone_msgs::msg::Goal>::SharedPtr mode_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mission_status_pub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
    rclcpp::Subscription<drone_msgs::msg::Mission>::SharedPtr mission_sub;
    rclcpp::Subscription<drone_msgs::msg::AutopilotCommand>::SharedPtr command_sub;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr interrupt_sub;

    
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client = std::make_shared<rclcpp::Client<mavros_msgs::srv::CommandBool>>();
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_px4_mode_client = std::make_shared<rclcpp::Client<mavros_msgs::srv::SetMode>>();

    std::shared_ptr<rclcpp::Client<mavros_msgs::srv::CommandBool>::FutureAndRequestId> arming_client_response_future;
    std::shared_ptr<rclcpp::Client<mavros_msgs::srv::SetMode>::FutureAndRequestId> set_px4_mode_client_response_future;
    // std::shared_ptr<ros::ServiceClient> arming_client;
    // std::shared_ptr<ros::ServiceClient> set_px4_mode_client;


    Autopilot()
    : Node("flight_control")
    {  
        enable_course_quantization = false;
        enable_pose_quantization = false;
        course_quantization_factor = M_PI_2;
        pose_quantization_factor = 0.25;

        goal_pub = this->create_publisher<drone_msgs::msg::Goal> ("/goal_pose", rclcpp::QoS(rclcpp::KeepLast{10}).best_effort().durability_volatile());
        mode_pub = this->create_publisher<drone_msgs::msg::Goal> ("/autopilot/mode_updates", rclcpp::QoS(rclcpp::KeepLast{10}).best_effort().durability_volatile());    
        mission_status_pub = this->create_publisher<std_msgs::msg::Bool> ("/autopilot/mission_status", rclcpp::QoS(rclcpp::KeepLast{10}).best_effort().durability_volatile());
        pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("/mavros/local_position/pose", 10, std::bind(&Autopilot::pose_cb, this, std::placeholders::_1));
        interrupt_sub = this->create_subscription<std_msgs::msg::Bool>("/autopilot/interrupt_mission", 10, std::bind(&Autopilot::interrupt_cb, this, std::placeholders::_1));
        mission_sub = this->create_subscription<drone_msgs::msg::Mission>("/autopilot/mission", 10, std::bind(&Autopilot::mission_cb, this, std::placeholders::_1));
        command_sub = this->create_subscription<drone_msgs::msg::AutopilotCommand>("/autopilot/commands", 10, std::bind(&Autopilot::command_cb, this, std::placeholders::_1));
        state_sub = this->create_subscription<mavros_msgs::msg::State>("/mavros/state", 10, std::bind(&Autopilot::mavros_state_cb, this, std::placeholders::_1));

        arming_client = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
        
        set_px4_mode_client = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

        // arming_client = std::make_shared<ros::ServiceClient>(nh.serviceClient<mavros_msgs::msg::CommandBool>("/mavros/cmd/arming"));
        // set_px4_mode_client = std::make_shared<ros::ServiceClient>(nh.serviceClient<mavros_msgs::msg::SetMode>("/mavros/set_mode"));

    auto timer_callback = [this]() -> void
    {
        if (curr_cmd.command == curr_cmd.UNDEFINED)
        {
            point_timer.timer_tick();
            _move_to_current_goal();
            
            std_msgs::msg::Bool status; 
            status.data = mission_status;
            mission_status_pub->publish(status);
        }
        else
        {
            if (_activate_command(curr_cmd))
            {
                curr_cmd = drone_msgs::msg::AutopilotCommand();
                curr_cmd.command = curr_cmd.UNDEFINED;
            }
        }
    };

    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), timer_callback);

    }


    void set_current_state(mavros_msgs::msg::State state)
    {
        current_state = state;
    }


    void activate_command(drone_msgs::msg::AutopilotCommand cmd)
    {
        // _activate_command(cmd);
        curr_cmd = cmd;
    }


    drone_msgs::msg::GoalTrajectory operative_trajectory()
    {
        return operative.trajectory;
    }


    void operative_trajectory_erase_waypoint(int idx)
    {
        operative.trajectory.waypoints.erase(operative.trajectory.waypoints.begin() + idx);
    }


    int points_in_current_trajectory()
    {
        return operative.points_in_current_trajectory;
    }


    int current_point_in_trajectory()
    {
        return current_point_in_trajectory();
    }


    FlightControlMode current_mode()
    {
        return operative.mode;
    }


    MissionSegment current_mission_segment()
    {
        return operative.current_mission_segment;
    }


    //  current_completion_condition()
    // {
    //     return operative.current_mission_segment.completion_condition;
    // } 


    Mission current_mission()
    {
        return operative.current_mission;
    }


    void set_current_mission(Mission mission)
    {
        mission_status = false;
        __current_mission = mission;
        operative.current_mission = mission;
        if (!operative.current_mission.segments.empty())
        {
            _set_current_mission_segment(operative.current_mission.segments.at(0));
        }
    }


    Goal current_goal()
    {
        return _quantize_goal_pose(_quantize_goal_course(operative.current_goal));
    }


    void set_current_goal(drone_msgs::msg::Goal new_goal)
    {
        operative.current_goal = _convert_goal_coordinate_system(new_goal);
        // point_timer.set_off();
    }


    void set_current_pose(geometry_msgs::msg::PoseStamped pose)
    {
        current_pose = pose;
    }


    geometry_msgs::msg::PoseStamped get_current_pose()
    {
        return current_pose;
    }


    void set_interrupt(bool interrupt_in)
    {
        interrupt = interrupt_in;
    }

    private:
    rclcpp::TimerBase::SharedPtr timer_;
    Mission __current_mission; // This object is meant to be an immutable mission that is only changed when a new mission arrives and is provided for reference
    /**
     * @brief This object is meant to provide easy access to modifiable objects that are needed to provide autopilot services.
     * 
     * @warning you should be aware that this object is meant to be constantly changing and the contained mission parameters might differ from the actual ones. To find information about the mission parameters refer to  __current_mission object
     */
    AutopilotOperative operative; 
    geometry_msgs::msg::PoseStamped current_pose;
    RosTimer point_timer;
    bool enable_course_quantization;
    bool enable_pose_quantization;
    double course_quantization_factor;
    double pose_quantization_factor;
    bool mission_status = false;
    mavros_msgs::msg::State current_state;
    drone_msgs::msg::AutopilotCommand curr_cmd;


    bool interrupt; // Variable to interrupt current mission and do something else and then continue the mission. 


    drone_msgs::msg::Goal _convert_goal_coordinate_system(drone_msgs::msg::Goal goal)
    {
        if (goal.pose.coordinates_type == drone_msgs::msg::DronePose::LOCAL)
        {
            drone_msgs::msg::Goal new_goal;
            new_goal = goal;

            EulerAngles angles;
            angles.get_RPY_from_msg_quaternion(current_pose.pose.orientation);

            new_goal.pose.point.x = goal.pose.point.x * cos(angles.yaw()) - goal.pose.point.y * sin(angles.yaw());
            new_goal.pose.point.y = goal.pose.point.x * sin(angles.yaw()) + goal.pose.point.y * cos(angles.yaw());


            new_goal.pose.point.x = new_goal.pose.point.x + current_pose.pose.position.x;
            new_goal.pose.point.y = new_goal.pose.point.y + current_pose.pose.position.y;
            new_goal.pose.point.z = goal.pose.point.z + current_pose.pose.position.z;
            new_goal.pose.course = angles.yaw() + goal.pose.course;

            return new_goal;
        }
        else if (goal.pose.coordinates_type == drone_msgs::msg::DronePose::LOCAL_XYC)
        {
            drone_msgs::msg::Goal new_goal;
            new_goal = goal;

            EulerAngles angles;
            angles.get_RPY_from_msg_quaternion(current_pose.pose.orientation);

            new_goal.pose.point.x = goal.pose.point.x * cos(angles.yaw()) - goal.pose.point.y * sin(angles.yaw());
            new_goal.pose.point.y = goal.pose.point.x * sin(angles.yaw()) + goal.pose.point.y * cos(angles.yaw());


            new_goal.pose.point.x = new_goal.pose.point.x + current_pose.pose.position.x;
            new_goal.pose.point.y = new_goal.pose.point.y + current_pose.pose.position.y;
            new_goal.pose.point.z = goal.pose.point.z + 0;
            new_goal.pose.course = angles.yaw() + goal.pose.course;

            return new_goal;
        }

        return goal;
    }


    drone_msgs::msg::Goal _quantize_goal_course(drone_msgs::msg::Goal goal)
    {
        if (enable_course_quantization)
        {
            EulerAngles angles;
            goal.pose.course = angles.quantize_angle_by(course_quantization_factor, goal.pose.course);
        }
        return goal;
    }


    drone_msgs::msg::Goal _quantize_goal_pose(drone_msgs::msg::Goal goal)
    {
        if (enable_pose_quantization)
        {
            goal.pose.point.x = _quantize_double_by(pose_quantization_factor, goal.pose.point.x);
            goal.pose.point.y = _quantize_double_by(pose_quantization_factor, goal.pose.point.y);
            goal.pose.point.z = _quantize_double_by(pose_quantization_factor, goal.pose.point.z);
        }

        return goal;
    }


    double _quantize_double_by(double quant_factor, double number)
    {
        double division_res = number / quant_factor;
        // ROS_INFO_STREAM("\nangle: " << angle << "\nradians: " << quant_factor << "\ndivision_res * quant_factor: " << round(division_res) * quant_factor << "\ndivision_res raw: " << division_res << "\ndivision_res rounded: " << round(division_res));
        division_res = round(division_res);
        return division_res * quant_factor;
    }


    void _move_to_current_goal()
    {
        if (!interrupt)
        {
            if (operative.trajectory.waypoints.size())
            {
                // std::cout << "_check_completion_condition() = " << _check_completion_condition() << "\n"; 
                RCLCPP_INFO_STREAM(this->get_logger(),"_check_completion_condition(): " << _check_completion_condition());
                if(_check_completion_condition())
                {
                    _start_new_mission_segment();
                }
                _update_goal_point();

                // goal_pub->publish(current_goal());
            }
        }
    }


    void _set_current_mission_segment(MissionSegment& mission_segment)
    {
        operative.current_mission_segment = mission_segment;
        operative.trajectory = mission_segment.trajectory;
        operative.mode = mission_segment.mode;
        set_current_goal(operative_trajectory().waypoints.at(0));
        goal_pub->publish(current_goal());
    }


    void _start_new_mission_segment()
    {
        if (operative.current_mission.segments.size() > 1)
        {
            operative.current_mission.segments.erase(operative.current_mission.segments.begin());
            _set_current_mission_segment(std::ref(operative.current_mission.segments.at(0)));
            // goal_pub->publish(current_goal());
        }
        else
        {
            mission_status = true;
            RCLCPP_ERROR_STREAM(this->get_logger(),"Autopilot: autopilot is trying to switch to a new segments when there are none.");
        }
    }


    void _update_goal_point()
    {
        std::cout << "_point_is_reached()" << _point_is_reached() << "\n";
            if (_point_is_reached() && operative_trajectory().waypoints.size() > 1)
            {
                operative_trajectory_erase_waypoint(0);
                point_timer.set_off();
                set_current_goal(operative_trajectory().waypoints.at(0));
                goal_pub->publish(current_goal());
            }
    }


    bool _point_is_reached()
    {
        if (current_goal().completion_policy == drone_msgs::msg::Goal::POLICY_DISTANCE_REACHED)
        {
            std::cout << "current_goal().completion_policy == drone_msgs::msg::Goal::POLICY_DISTANCE_REACHED\n";
            double dx = current_goal().pose.point.x - current_pose.pose.position.x;
            double dy = current_goal().pose.point.y - current_pose.pose.position.y;
            double dz = current_goal().pose.point.z - current_pose.pose.position.z;

            double dr = sqrt(dx*dx + dy*dy + dz*dz);

            bool goal_coords_reached = dr < current_goal().policy_distance_threshold;
            std::cout << "dr: " << dr << "\n";
            std::cout << "current_goal().policy_distance_threshold: " << current_goal().policy_distance_threshold << "\n";
            std::cout << "goal_coords_reached: " << goal_coords_reached << "\n";
            return goal_coords_reached;
        }


        else if (current_goal().completion_policy == drone_msgs::msg::Goal::POLICY_POSITION_REACHED)
        {
            double dx = current_goal().pose.point.x - current_pose.pose.position.x;
            double dy = current_goal().pose.point.y - current_pose.pose.position.y;
            double dz = current_goal().pose.point.z - current_pose.pose.position.z;

            double dr = sqrt(dx*dx + dy*dy + dz*dz);
            bool goal_coords_reached = dr < current_goal().policy_distance_threshold;


            EulerAngles pose_orientation;
            pose_orientation.get_RPY_from_msg_quaternion(current_pose.pose.orientation);
            

            double dyaw = abs(pose_orientation.normalize_angle(current_goal().pose.course - pose_orientation.yaw()));
            bool goal_yaw_reached = dyaw < current_goal().policy_orientation_threshold;

            RCLCPP_INFO_STREAM(this->get_logger(),"\ndr: " << dr << "\ndyaw: " << dyaw);

            return goal_coords_reached && goal_yaw_reached;
        }


        else if (current_goal().completion_policy == drone_msgs::msg::Goal::POLICY_TIMER)
        {
            // std::cout << "current_goal().completion_policy == drone_msgs::msg::Goal::POLICY_TIMER\n";
            if (!point_timer.started() && !point_timer.finished())
            {
                std::cout << "!point_timer.started()\n";
                double dx = current_goal().pose.point.x - current_pose.pose.position.x;
                double dy = current_goal().pose.point.y - current_pose.pose.position.y;
                double dz = current_goal().pose.point.z - current_pose.pose.position.z;

                double dr = sqrt(dx*dx + dy*dy + dz*dz);
                bool goal_coords_reached = dr < current_goal().policy_distance_threshold;

                std::cout << "dr: " << dr << "\n";
                std::cout << "current_goal().policy_distance_threshold: " << current_goal().policy_distance_threshold << "\n";
                std::cout << "goal_coords_reached: " << goal_coords_reached << "\n";

                if (goal_coords_reached)
                {
                    point_timer.set_duration(current_goal().policy_timer_time);
                    point_timer.start_timer();
                }
                return false;
            }
            else if (point_timer.started() && point_timer.finished())
            {
                // point_timer.set_off();
                return true;
            }
            else if (point_timer.started() && !point_timer.finished())
            {
                std::cout << "point_timer.started()\n";
                std::cout << "Remaining: " << point_timer.remaining_time() << "\n"; 
                // if (point_timer.finished())
                // {
                //     std::cout << "point_timer.finished()\n";
                //     return point_timer.finished();
                // }
                return false;
            }
        }

        std::cerr << "Autopilot: no policy for point completion\n";
        return false;
    }


    bool _check_completion_condition()
    {
        // std::cout << "_check_completion_condition()\n"; 
        if (current_mission_segment().completion_condition == drone_msgs::msg::MissionSegment::CUSTOM_CONDITION_TOPIC)
        {
            // std::cout << "drone_msgs::msg::MissionSegment::CUSTOM_CONDITION_TOPIC\n"; 
            int ya_hz_che_tut_delat;
        }


        else if (current_mission_segment().completion_condition == drone_msgs::msg::MissionSegment::LAST_POINT_ARRIVAL)
        {
            // std::cout << "ne_msgs::MissionSegment::LAST_POINT_ARRIVAL\n"; 
            // bool curr_goal_is_final_trajectory_point = current_goal() == _convert_goal_coordinate_system(current_mission_segment().trajectory.waypoints.at(current_mission_segment().trajectory.waypoints.size() - 1));
            // bool curr_goal_is_final_trajectory_point = true;
            bool curr_goal_is_final_trajectory_point = false;

            if (operative_trajectory().waypoints.size() == 1)
            {
                curr_goal_is_final_trajectory_point = true;
            }

            if (curr_goal_is_final_trajectory_point)
            {
                // EulerAngles pose_orientation;
                // pose_orientation.get_RPY_from_msg_quaternion(current_pose.pose.orientation);

                // double dyaw = abs(current_goal().pose.course - pose_orientation.yaw);
                // // bool goal_yaw_achieved = dyaw < 0.1;
                
                // // if (goal_yaw_achieved)
                // // {
                    // double dx = current_goal().pose.point.x - current_pose.pose.position.x;
                    // double dy = current_goal().pose.point.y - current_pose.pose.position.y;
                    // double dz = current_goal().pose.point.z - current_pose.pose.position.z;

                    // double dr = sqrt(dx*dx + dy*dy + dz*dz);

                    // bool destination_reached = dr < current_goal().policy_distance_threshold;
                    return _point_is_reached();
                // }
            }
        }

        return false;
    }


    bool _activate_command(drone_msgs::msg::AutopilotCommand cmd)
    {
        if (cmd.command == cmd.ARM_AND_OFFBOARD)
        {
            ros::Rate rate(1.0);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            mavros_msgs::srv::SetMode::Request::SharedPtr offb_set_mode;
            offb_set_mode->custom_mode = "OFFBOARD";

            mavros_msgs::srv::CommandBool::Request::SharedPtr arm_cmd;
            arm_cmd->value = true;

            rclcpp::Time last_request = rclcpp::Clock(RCL_ROS_TIME).now();

            mavros_msgs::srv::CommandBool::Response arm_cmd_response;
            arm_cmd_response.result = true ;
            mavros_msgs::srv::SetMode::Response offb_set_mode_response;
            offb_set_mode_response.mode_sent = "OFFBOARD";
//////////////////////////////////////////////////////////////////////////
            arming_client_response_future = std::make_shared<rclcpp::Client<mavros_msgs::srv::CommandBool>::FutureAndRequestId>(this->arming_client->async_send_request(std::make_shared<mavros_msgs::srv::CommandBool::Request>(arm_cmd)));

            while (arming_client_response_future->wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) 
            RCLCPP_WARN_STREAM(this->get_logger(), "Waiting call back arm...");

            std::shared_ptr<mavros_msgs::srv::CommandBool::Response> response_arm = arming_client_response_future->get();
/////////////////////////////////////////////////////////////////////////
            set_px4_mode_client_response_future = std::make_shared<rclcpp::Client<mavros_msgs::srv::SetMode>::FutureAndRequestId>(this->set_px4_mode_client->async_send_request(std::make_shared<mavros_msgs::srv::SetMode::Request>(offb_set_mode)));

            while (set_px4_mode_client_response_future->wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) 
            RCLCPP_WARN_STREAM(this->get_logger(), "Waiting call back mode...");

            std::shared_ptr<mavros_msgs::srv::SetMode::Response> response_mode = set_px4_mode_client_response_future->get();
//////////////////////////////////////////////////////////////////////////




            if( current_state.mode != "OFFBOARD")// && (rclcpp::Clock(RCL_ROS_TIME).now() - last_request > ros::Duration(1.0)))
            {
                if( response_arm && arm_cmd_response.success)
                {
                    RCLCPP_INFO_STREAM(this->get_logger(),"Autopilot: Vehicle armed");
                }
                if(response_mode && offb_set_mode_response.mode_sent)
                {
                    RCLCPP_INFO_STREAM(this->get_logger(),"Autopilot: Offboard enabled");
                }
                last_request = rclcpp::Clock(RCL_ROS_TIME).now();
            }
            else if ( current_state.mode != "OFFBOARD" && (rclcpp::Clock(RCL_ROS_TIME).now() - last_request > rclcpp::Duration(std::chrono::milliseconds(1000))))
            {
                if(!current_state.armed)// && (rclcpp::Clock(RCL_ROS_TIME).now() - last_request > ros::Duration(1.0)))
                {
                    RCLCPP_INFO_STREAM(this->get_logger(),"Autopilot: Vehicle armed");
                }
                if(response_mode && offb_set_mode_response.mode_sent)
                {
                    RCLCPP_INFO_STREAM(this->get_logger(),"Autopilot: Offboard enabled");
                }
                last_request = rclcpp::Clock(RCL_ROS_TIME).now();
            } 
            if (current_pose.pose.position.z > 0.5)
            {
                return true;
            }
            return false;
        }
        else
        {
            RCLCPP_WARN_STREAM(this->get_logger(),"Autopilot: The recieved command is not implemented yet!");
            return false;
        }
    }

    void command_cb(const drone_msgs::msg::AutopilotCommand::ConstPtr& command)
    {
        activate_command(*command);
    }


    void mavros_state_cb(const mavros_msgs::msg::State::ConstPtr& state)
    {
        set_current_state(*state);
    }


    void pose_cb(const geometry_msgs::msg::PoseStamped::ConstPtr& pose)
    {
        set_current_pose(*pose);
    }


    void interrupt_cb(const std_msgs::msg::Bool::ConstPtr& interrupt)
    {
        set_interrupt(interrupt->data);
    }


    void mission_cb(const drone_msgs::msg::Mission::ConstPtr& mission)
    {
        set_current_mission(*mission);
        RCLCPP_INFO_STREAM(this->get_logger(),"AUTOPILOT: Got mission.");
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Autopilot>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}










/**
* EXAMPLE MISSIONS:
        // drone_msgs::msg::Mission init_mission;
        
        // drone_msgs::msg::MissionSegment segment;
        // segment.completion_condition = drone_msgs::msg::MissionSegment::LAST_POINT_ARRIVAL;
        // segment.mode.mode = drone_msgs::msg::FlightControlMode::GO_ALONG_TRAJECTORY;

        // drone_msgs::msg::Goal goal;
        // goal.completion_policy = drone_msgs::msg::Goal::POLICY_DISTANCE_REACHED;
        // goal.policy_distance_threshold = 0.15;
        // goal.pose.point.x = 0;
        // goal.pose.point.y = 0;
        // goal.pose.point.z = 1.5;
        // goal.pose.course = 0;

        // segment.trajectory.waypoints.push_back(goal);
        
        // init_mission.segments.push_back(segment);

        // drone_msgs::msg::MissionSegment segment2;
        // segment2.completion_condition = drone_msgs::msg::MissionSegment::LAST_POINT_ARRIVAL;
        // segment2.mode.mode = drone_msgs::msg::FlightControlMode::GO_ALONG_TRAJECTORY;

        // goal = drone_msgs::msg::Goal();
        // goal.completion_policy = drone_msgs::msg::Goal::POLICY_TIMER;
        // goal.policy_distance_threshold = 0.15;
        // goal.policy_timer_time = 5;
        // goal.pose.point.x = 2;
        // goal.pose.point.y = 0;
        // goal.pose.point.z = 1.5;
        // goal.pose.course = 0;

        // segment2.trajectory.waypoints.push_back(goal);

        // goal = drone_msgs::msg::Goal();

        // goal.completion_policy = drone_msgs::msg::Goal::POLICY_POSITION_REACHED;
        // goal.policy_distance_threshold = 0.15;
        // goal.policy_orientation_threshold = 0.087;
        // goal.pose.point.x = 2;
        // goal.pose.point.y = -1;
        // goal.pose.point.z = 1;
        // goal.pose.course = -1.57;

        // segment2.trajectory.waypoints.push_back(goal);

        // init_mission.segments.push_back(segment2);

        // drone_msgs::msg::MissionSegment segment3;
        // segment3.completion_condition = drone_msgs::msg::MissionSegment::LAST_POINT_ARRIVAL;
        // segment3.mode.mode = drone_msgs::msg::FlightControlMode::GO_ALONG_TRAJECTORY;

        // goal = drone_msgs::msg::Goal();
        // goal.completion_policy = drone_msgs::msg::Goal::POLICY_DISTANCE_REACHED;
        // goal.policy_distance_threshold = 0.15;
        // goal.pose.point.x = 2;
        // goal.pose.point.y = 0;
        // goal.pose.point.z = 1;
        // goal.pose.course = 0;

        // segment3.trajectory.waypoints.push_back(goal);

        // goal = drone_msgs::msg::Goal();
        // goal.completion_policy = drone_msgs::msg::Goal::POLICY_DISTANCE_REACHED;
        // goal.policy_distance_threshold = 0.15;
        // goal.pose.point.x = 0;
        // goal.pose.point.y = 0;
        // goal.pose.point.z = 1;
        // goal.pose.course = 0;

        // segment3.trajectory.waypoints.push_back(goal);

        // goal.policy_distance_threshold = 0.15;
        // goal.pose.point.x = 0;
        // goal.pose.point.y = 0;
        // goal.pose.point.z = 0.0;
        // goal.pose.course = 0;

        // segment3.trajectory.waypoints.push_back(goal);
        // init_mission.segments.push_back(segment3);

        // set_current_mission(init_mission);
*/