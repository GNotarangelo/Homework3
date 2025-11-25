#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
//Added for landing detection
#include <px4_msgs/msg/vehicle_land_detected.hpp> // this can be found in the px4_msgs/msg folder and the are translated 
// in hpp thanks to the compiler

using namespace std::chrono_literals;

class ForceLand : public rclcpp::Node
{
	public:
	ForceLand() : Node("force_land"), need_land(false), has_landed(true)//added default value for the new private bool var
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position",
		qos, std::bind(&ForceLand::height_callback, this, std::placeholders::_1));
		publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
		
		//Added subscription to vehicle_land_detected topic to acknowledge if and when the landing takes place
		land_ack_sub_ = this->create_subscription<px4_msgs::msg::VehicleLandDetected>("/fmu/out/vehicle_land_detected",
		qos, std::bind(&ForceLand::land_ack_callback, this, std::placeholders::_1));

		timer_ = this->create_wall_timer(10ms, std::bind(&ForceLand::activate_switch, this));
	}

	private:
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr subscription_;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr publisher_;
	//Added private pointer for the second subscriber
	rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_ack_sub_;

	rclcpp::TimerBase::SharedPtr timer_;

	bool need_land;
	//Added flag for remembering if the vehicle has landed or not (it's the reset condition)
	bool has_landed;//by default it's true (we assume to start from the ground
	// and it is good anyway for getting the first execution // When the landing is forced the first time 
	// (actually in the height callback) it is set to false//When landing is achieved it becomes again true

	//Modified: only modifies the need_land flag if the height is surpassed and the landing was already attempted but not ack
	void height_callback(const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) 
	{
		float z_ = -msg->z;
		std::cout << "Current drone height: " << z_ << " meters" <<  std::endl;
		if(z_ > 20 && has_landed)
		{
			need_land = true;
			has_landed = false;
		}

		return;
	}

	
	void activate_switch()
	{
		if(need_land)
		{
			std::cout << "Drone height exceeded 20 meters threshold, Landing forced" << std::endl;
			auto command = px4_msgs::msg::VehicleCommand();
			command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;
			this->publisher_->publish(command);
			need_land = false;
		}
	}
	//Added: if the drone lands the flag is set to true
	void land_ack_callback(const px4_msgs::msg::VehicleLandDetected::UniquePtr msg){
		if(msg->landed==true){
			has_landed=true;
		}
	}
};

int main(int argc, char *argv[])
{
	std::cout << "Starting vehicle_local_position listener node..." << std::endl;
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ForceLand>());
	rclcpp::shutdown();
	return 0;
}
