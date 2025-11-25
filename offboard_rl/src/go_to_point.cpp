#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <offboard_rl/utils.h>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

//Added
//This structure just contains the pose to achieve and the time in which it should be achieved
//It is meant for storing the user points specifications
struct PointTimes {
    Eigen::Vector4d pose;    // x, y, z, yaw
    double time_to_next; // Time (seconds) to travel to the NEXT point
};

//Added
//This is the structure that contains the structured waypoint (the one that will actually be part of the trajectory)
struct Waypoint {
    // The Target State (Where we want to be)
    Eigen::Vector3d position;
    double yaw;
    Eigen::Vector3d velocity;      

    // The Timing
    double time_to_next; // How many seconds to travel to the NEXT point
};

class GoToPoint : public rclcpp::Node
{
	public:
	GoToPoint() : Node("go_to_point")
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		local_position_subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position",
		qos, std::bind(&GoToPoint::vehicle_local_position_callback, this, std::placeholders::_1));
		attitude_subscription_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>("/fmu/out/vehicle_attitude",
		qos, std::bind(&GoToPoint::vehicle_attitude_callback, this, std::placeholders::_1));
		offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

		timer_offboard_ = this->create_wall_timer(100ms, std::bind(&GoToPoint::activate_offboard, this));
		timer_trajectory_publish_ = this->create_wall_timer(20ms, std::bind(&GoToPoint::publish_trajectory_setpoint_new, this));

		keyboard_thread = std::thread(&GoToPoint::keyboard_listener2, this);
	}

	private:
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_subscription_;
	rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_subscription_;
	rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;

	rclcpp::TimerBase::SharedPtr timer_offboard_;
	rclcpp::TimerBase::SharedPtr timer_trajectory_publish_;

	std::thread keyboard_thread;

	bool set_point_received{false};
	bool offboard_active{false};
	bool trajectory_computed{false};
	Eigen::Vector<double, 6> x; //coefficients of the trajectory polynomial

	double T, t{0.0};
	Eigen::Vector4d pos_i, pos_f;
	VehicleLocalPosition current_position_{};
	VehicleAttitude current_attitude_{};
	double offboard_counter{0};

	//Added vector of waypoints (we consider also initial and final point)
	std::vector<Waypoint> path_;
	//Added
	int current_segment_idx{0}; // Tracks which waypoint we are flying FROM

	void vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
	{
		current_position_ = *msg;
	}

	void vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
	{
		current_attitude_ = *msg;
	}

	void keyboard_listener()
	{
		// Set x, y, z, yaw setpoints
		while (rclcpp::ok() && !set_point_received)
		{
			std::cout << "Enter setpoints as x y z yaw (meters, meters, meters, radiants) & time to complete path (seconds): ";
			std::string line;
			std::getline(std::cin, line);
			std::istringstream iss(line);
			if (!(iss >> pos_f(0) >> pos_f(1) >> pos_f(2) >> pos_f(3) >> T)) {
				std::cout << "Invalid input. Please enter FIVE numeric values." << std::endl;
				continue;
			}
			else {
				pos_f(2) = -pos_f(2); // PX4 uses NED frame
				set_point_received = true;
				std::cout << "Setpoints received: x=" << pos_f(0) << ", y=" << pos_f(1) << ", z=" << pos_f(2) << ", yaw=" << pos_f(3) << ", T=" << T << std::endl;
				std::cout << "Activating offboard mode" << std::endl;
				std::cout << "----------------------------------------" << std::endl;
			}
		}
	}

	void activate_offboard()
	{
		if (set_point_received)
		{
			if(offboard_counter == 10) {
				// Change to Offboard mode after 1 second of sending offboard messages
				VehicleCommand msg{};
				msg.param1 = 1;
				msg.param2 = 6;
				msg.command = VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
				msg.target_system = 1;
				msg.target_component = 1;
				msg.source_system = 1;
				msg.source_component = 1;
				msg.from_external = true;
				msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
				vehicle_command_publisher_->publish(msg);

				// Arm the vehicle
				msg.param1 = 1.0;
				msg.param2 = 0.0;
				msg.command = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
				msg.target_system = 1;
				msg.target_component = 1;
				msg.source_system = 1;
				msg.source_component = 1;
				msg.from_external = true;
				msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
				vehicle_command_publisher_->publish(msg);

				// Set initial position
				pos_i(0) = current_position_.x;
				pos_i(1) = current_position_.y;
				pos_i(2) = current_position_.z;

				auto rpy = utilities::quatToRpy( Vector4d( current_attitude_.q[0], current_attitude_.q[1], current_attitude_.q[2], current_attitude_.q[3] ) );
				pos_i(3) = rpy[2];

				offboard_active = true;
			}

			OffboardControlMode msg{};
			msg.position = true;
			msg.velocity = false;
			msg.acceleration = false;
			msg.attitude = false;
			msg.body_rate = false;
			msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
			offboard_control_mode_publisher_->publish(msg);	

			if (offboard_counter < 11) offboard_counter++;
		}
	}

	void publish_trajectory_setpoint()
	{
		if (!set_point_received || !offboard_active || t > T) {
			return;
		}

		// std::cout << "Publishing trajectory setpoint to x=" << pos_f(0) << ", y=" << pos_f(1) << ", z=" << pos_f(2) << ", yaw=" << pos_f(3) << std::endl;
		// TrajectorySetpoint msg{};
		// msg.position = {float(pos_f(0)), float(pos_f(1)), float(pos_f(2))};
		// msg.yaw = float(pos_f(3)); // [-PI:PI]
		// msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		// trajectory_setpoint_publisher_->publish(msg);

		double dt = 1/50.0; // 20 ms
		TrajectorySetpoint msg{compute_trajectory_setpoint(t)};
		trajectory_setpoint_publisher_->publish(msg);
		t += dt;
	}

	TrajectorySetpoint compute_trajectory_setpoint(double t)
	{
		Vector4d e = pos_f - pos_i;
		e(3) = utilities::angleError(pos_f(3), pos_i(3));
		double s_f = e.norm();

		if (!trajectory_computed)
		{
			Eigen::VectorXd b(6);
			Eigen::Matrix<double, 6, 6> A;

			b << 0.0, 0.0, 0.0, s_f, 0.0, 0.0;
			A << 0, 0, 0, 0, 0, 1,
             0, 0, 0, 0, 1, 0,
             0, 0, 0, 1, 0, 0,
             pow(T,5), pow(T,4), pow(T,3), pow(T,2), T, 1,
             5*pow(T,4), 4*pow(T,3), 3*pow(T,2), 2*T, 1, 0,
             20*pow(T,3), 12*pow(T,2), 6*T, 1, 0, 0;

			x = A.inverse() * b;
			trajectory_computed = true;
		}

		double s, s_d, s_dd;
		Eigen::Vector4d ref_traj_pos, ref_traj_vel, ref_traj_acc;

		s   = x(0) * std::pow(t, 5.0)
			+ x(1) * std::pow(t, 4.0)
			+ x(2) * std::pow(t, 3.0)
			+ x(3) * std::pow(t, 2.0)
			+ x(4) * t
			+ x(5);

		s_d = 5.0  * x(0) * std::pow(t, 4.0)
			+ 4.0  * x(1) * std::pow(t, 3.0)
			+ 3.0  * x(2) * std::pow(t, 2.0)
			+ 2.0  * x(3) * t
			+        x(4);

		s_dd = 20.0 * x(0) * std::pow(t, 3.0)
			+ 12.0 * x(1) * std::pow(t, 2.0)
			+  6.0 * x(2) * t
			+        x(3);

		ref_traj_pos = pos_i + s*e/s_f;
    	ref_traj_vel = s_d*e/s_f;
        ref_traj_acc = s_dd*e/s_f;

		TrajectorySetpoint msg{};
		msg.position = {float(ref_traj_pos(0)), float(ref_traj_pos(1)), float(ref_traj_pos(2))};
		msg.velocity = {float(ref_traj_vel(0)), float(ref_traj_vel(1)), float(ref_traj_vel(2))};
		msg.acceleration = {float(ref_traj_acc(0)), float(ref_traj_acc(1)), float(ref_traj_acc(2))};
		msg.yaw = float(ref_traj_pos(3)); // [-PI:PI]
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

		return msg;
	}

	//Added function to initialize a vector of points with timings 
	// (hard coded for simplicity, but might be done with file etc...)
	void keyboard_listener2()
    {
        // Wait for ROS to be fully ready
        while (!rclcpp::ok()) return;

        std::cout << "\n==============================================\n";
        std::cout << "   OFFBOARD TRAJECTORY GENERATOR\n";
        std::cout << "==============================================\n";
        std::cout << "   Mission: Standard 7-Waypoint Assignment\n";
        std::cout << "   Press [ENTER] to compute path and fly...\n";
        std::cout << "==============================================\n";

        // Wait for the user to press Enter
        std::string line;
        std::getline(std::cin, line);

        // Define some Waypoints directly here
        // Format: { {x, y, z, yaw}, duration_to_next_point }
        // NOTE: We use NED coordinates.
        std::vector<PointTimes> mission_points = {
            // 1. Takeoff to 10m height 
            { {0.0, 0.0, -10.0, 0.0},  10.0 },  

            // 2. Move Forward 2m 
            { {2.0, 0.0, -10.0, 0.0},  10.0 },  

            // 3. Move Left 2m & Turn Yaw 90 deg 
            { {2.0, 2.0, -10.0, 1.57}, 12.0 },  

			// 3.5 added just for fun 
            { {1.0, 2.0, -10.0, 1.57}, 12.0 }, 

            // 4. Move Backward 2m & Turn Yaw 180 deg 
            { {0.0, 2.0, -10.0, 3.14}, 10.0 },  

			// 4.5 added just for fun 
            { {1.0, 1.0, -10.0, 1.57}, 12.0 }, 

            // 5. Move Right 2m (Return to Home X,Y) 
            { {0.0, 0.0, -10.0, 0.0},  10.0 },  

            // 6. Go Up to 5m height 
            { {0.0, 0.0, -15.0, 0.0},  10.0 },  

            // 7. Go Down to 2m height (End) - Duration 0 stops the logic
            { {0.0, 0.0, -3.0, 0.0},  0.0 }   
        };

        std::cout << "Loaded " << mission_points.size() << " waypoints.\n";
        std::cout << "Computing smooth trajectory with start-point interpolation...\n";

        // Compute the path (This adds the drone's current position as the start!)
        compute_path(mission_points);

        // Trigger the flight loop
        set_point_received = true;

        std::cout << "Trajectory computed! Switching to Offboard Mode.\n";
        std::cout << "==============================================\n";
    }

	//Added function to compute the actual position velocity and accelerations for each coordinate and each curve 
	Eigen::Vector3d compute_trajectory_state(
		double t,       // Current time in segment
		double T,       // Total duration of segment
		double start_p, double start_v, // Start Position & Velocity
		double end_p,   double end_v    // End Position & Velocity
	) 
	{
		//Safety (Don't go past T)
		if (t > T) t = T;
		if (t < 0) t = 0;

		//Define Matrix A 
		Eigen::Matrix<double, 6, 6> A;
		A << 0, 0, 0, 0, 0, 1,
			0, 0, 0, 0, 1, 0,
			0, 0, 0, 2, 0, 0,
			pow(T,5),   pow(T,4),   pow(T,3),   pow(T,2), T, 1,
			5*pow(T,4), 4*pow(T,3), 3*pow(T,2), 2*T,      1, 0,
			20*pow(T,3), 12*pow(T,2), 6*T,      2,        0, 0;

		//Define Vector b (Boundary Conditions)
		// We assume 0 acceleration at the waypoints for simplicity
		Eigen::Vector<double, 6> b;
		b << start_p, start_v, 0.0,  // Start: Pos, Vel, Acc
			end_p,   end_v,   0.0;  // End:   Pos, Vel, Acc

		//Solve for Coefficients (c0...c5)
		Eigen::Vector<double, 6> x = A.fullPivLu().solve(b);

		//Evaluate at time t
		double p = x(0)*pow(t,5) + x(1)*pow(t,4) + x(2)*pow(t,3) + x(3)*pow(t,2) + x(4)*t + x(5);
		double v = 5*x(0)*pow(t,4) + 4*x(1)*pow(t,3) + 3*x(2)*pow(t,2) + 2*x(3)*t + x(4);
		double a = 20*x(0)*pow(t,3) + 12*x(1)*pow(t,2) + 6*x(2)*t + 2*x(3);

		return Eigen::Vector3d(p, v, a);
	}

	//Added function to compute a path from the specified points and timings
	void compute_path(const std::vector<PointTimes>& user_inputs) {
		path_.clear();

		
		//Get Current Yaw from Quaternion
		Eigen::Vector4d q(current_attitude_.q[0], current_attitude_.q[1], current_attitude_.q[2], current_attitude_.q[3]);
		auto rpy = utilities::quatToRpy(q);
		double current_yaw = rpy(2);

		//Create the Start position (Where the drone is NOW)
		PointTimes start_pos;
		start_pos.pose = {current_position_.x, current_position_.y, current_position_.z, current_yaw};
		
		//Compute time to fly to the first user point (Safety check for empty input)
		if (!user_inputs.empty()) {
			double dist = (user_inputs[0].pose.head(3) - start_pos.pose.head(3)).norm();
			start_pos.time_to_next = std::max(2.0, dist / 1.0); // Min 2 seconds, or 1 m/s speed
		} else {
			start_pos.time_to_next = 1.0;
		}

		// Create the full list: [Start pos] + [User Inputs]
		std::vector<PointTimes> all_points;
		all_points.push_back(start_pos);
		all_points.insert(all_points.end(), user_inputs.begin(), user_inputs.end());

		int n = all_points.size(); 
		
		for (int i = 0; i < n; i++) {
			Waypoint wp;
			
			// Copy Position & Yaw
			wp.position = all_points[i].pose.head(3);
			wp.yaw      = all_points[i].pose(3);

			// Copy Duration
			wp.time_to_next = (i < n - 1) ? all_points[i].time_to_next : 0.0;

			// Compute Velocity 
			if (i == 0) {
				wp.velocity = Eigen::Vector3d::Zero(); // Start (Current Pos)
			} 
			else if (i == n - 1) {
				wp.velocity = Eigen::Vector3d::Zero(); // End
			} 
			else {
				Eigen::Vector3d prev = all_points[i-1].pose.head(3);
				Eigen::Vector3d next = all_points[i+1].pose.head(3);
				Eigen::Vector3d dir = next - prev;

				double avg_dist = (dir).norm();
				double avg_time = all_points[i-1].time_to_next + all_points[i].time_to_next;
				
				// Avoid division by zero
				double est_speed = (avg_time > 0) ? avg_dist / (avg_time / 2.0) : 0.0; 

				if (dir.norm() > 0.01) {
					wp.velocity = dir.normalized() * est_speed;
				} else {
					wp.velocity = Eigen::Vector3d::Zero();
				}
			}

			path_.push_back(wp);
		}

		// Reset execution state
		current_segment_idx = 0;
		t = 0;
	}


	//Added function to publish the trajectory
	void publish_trajectory_setpoint_new()
    {
        // Basic Checks: Do we have a path? Are we armed?
        if (!offboard_active || !set_point_received || path_.empty()) {
            return;
        }

        // Check if mission is complete
        if (current_segment_idx >= path_.size() - 1) {
            return;
        }

        // Get Current Segment Data
        Waypoint wp_start = path_[current_segment_idx];
        Waypoint wp_end   = path_[current_segment_idx + 1];
        double T_segment = wp_start.time_to_next;

        // Handle Time Switching (Next Segment)
        if (t > T_segment) {
            t = 0.0;
            current_segment_idx++;
            return; 
        }

        // Compute Trajectory for X, Y, Z
        auto x_state = compute_trajectory_state(t, T_segment, wp_start.position.x(), wp_start.velocity.x(), wp_end.position.x(), wp_end.velocity.x());
        auto y_state = compute_trajectory_state(t, T_segment, wp_start.position.y(), wp_start.velocity.y(), wp_end.position.y(), wp_end.velocity.y());
        auto z_state = compute_trajectory_state(t, T_segment, wp_start.position.z(), wp_start.velocity.z(), wp_end.position.z(), wp_end.velocity.z());

        // Compute Trajectory for Yaw
        double yaw_start = wp_start.yaw;
        double yaw_end = wp_end.yaw;
        // Calculate shortest angular distance 
        double yaw_diff = utilities::angleError(yaw_end, yaw_start); 
        
        // We assume 0 yaw velocity at waypoints
        auto yaw_state = compute_trajectory_state(t, T_segment, yaw_start, 0.0, yaw_start + yaw_diff, 0.0);


        // Create and Publish Message
        TrajectorySetpoint msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        // Position
        msg.position = { (float)x_state(0), (float)y_state(0), (float)z_state(0) };
        // Velocity
        msg.velocity = { (float)x_state(1), (float)y_state(1), (float)z_state(1) };
        // Acceleration
        msg.acceleration = { (float)x_state(2), (float)y_state(2), (float)z_state(2) };
        // Yaw
        msg.yaw = (float)yaw_state(0);
        msg.yawspeed = (float)yaw_state(1);

        trajectory_setpoint_publisher_->publish(msg);

        // Increment Time
        double dt = 0.02; // 50Hz
        t += dt;
    }
};

int main(int argc, char *argv[])
{
	std::cout << "Starting vehicle_local_position listener node..." << std::endl;
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<GoToPoint>());
	rclcpp::shutdown();
	return 0;
}