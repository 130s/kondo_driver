/**
 * Kondo ICS motor driver
 */
#include <boost/shared_ptr.hpp>
#include <math.h>
#include "ros/ros.h"
#include "controller_manager/controller_manager.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/actuator_command_interface.h"
#include "hardware_interface/actuator_state_interface.h"
#include "hardware_interface/robot_hw.h"
extern "C" {
#include "kondo_driver/ics.h"
}
#include "kondo_driver/setPower.h"

/* Maximum motor num (32 is maximum on spec. sheet) */
const int MAX_MOTOR_NUM = 12;
const int MAX_PULSE = 11500;
const int MIN_PULSE = 3500;
const int CNT_PULSE = 7500;
const double RADIAN_PER_PULSE = 270.0*M_PI/(MAX_PULSE-MIN_PULSE)/180.0;

double pulse_to_radian (double pulse)
{
    return (pulse - CNT_PULSE)*RADIAN_PER_PULSE;
}

int radian_to_pulse (double radian)
{
    return CNT_PULSE + radian/RADIAN_PER_PULSE;
}

class KondoMotor {
private:
    bool motor_power;
    ros::ServiceServer power_service;
    int id;
    ICSData* ics;
    int speed;
    int stretch;
public:
    double cmd, pos, vel, eff;
    std::string joint_name;
    bool set_power (kondo_driver::setPower::Request &req, kondo_driver::setPower::Response &res) {
	ROS_INFO("id %d, request: %d", this->id, req.request);
	motor_power = req.request;
	res.result = req.request;
	return true;
    }
    KondoMotor (ICSData* ics, ros::NodeHandle nh, std::string actuator_name, hardware_interface::JointStateInterface& state_interface, hardware_interface::PositionJointInterface& pos_interface) {
	cmd=0, pos=0;
	motor_power = true;
	this->ics = ics;
	if (nh.getParam(actuator_name+std::string("/id"), id)) {
	    ROS_INFO("id: %d", id);
	}
	if (nh.getParam(actuator_name+std::string("/joint"), joint_name)) {
	    ROS_INFO("joint: %s", joint_name.c_str());
	}
	if (nh.getParam(actuator_name+std::string("/speed"), speed)) {
	    ROS_INFO("speed: %d", speed);
	    set_speed(speed);
	}
	if (nh.getParam(actuator_name+std::string("/stretch"), stretch)) {
	    ROS_INFO("stretch: %d", stretch);
	    set_stretch(stretch);
	}
	hardware_interface::JointStateHandle state_handle(joint_name, &pos, &vel, &eff);
	state_interface.registerHandle(state_handle);
	hardware_interface::JointHandle pos_handle(state_interface.getHandle(joint_name), &cmd);
	pos_interface.registerHandle(pos_handle);
	power_service = nh.advertiseService(actuator_name+std::string("/set_power"), &KondoMotor::set_power, this);
    }
    void update (void) {
	int pulse_cmd = 0;
	if (motor_power == true) {
	    pulse_cmd = radian_to_pulse(cmd);
	}
	int pulse_ret = ics_pos(ics, id, pulse_cmd);
	pos = pulse_to_radian (pulse_ret);
    }
    // Set speed parameter
    void set_speed (unsigned char speed) {
	this->speed = ics_set_stretch(ics, id, speed);
	ROS_INFO("%s: %d", __func__, this->speed);
    }
    // Set strech parameter
    void set_stretch (unsigned char stretch) {
	this->stretch = ics_set_stretch(ics, id, stretch);
	ROS_INFO("%s: %d", __func__, this->stretch);
    }
};

class KondoDriver : public hardware_interface::RobotHW
{
  private:
    // Hardware interface
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;
    // ICS hardware resource
    ICSData ics;
    // Vector of motors
    std::vector<boost::shared_ptr<KondoMotor> > actuator_vector;
  public:
    KondoDriver (int num, char** actuators) {
	ros::NodeHandle nh("~");
	// Initiallize ICS interface
	if (ics_init(&ics) < 0) {
	    ROS_ERROR ("Could not init ICS: %s\n", ics.error);
	}
	// Load atuators
	for (int i=0; i<num; i++) {
	    boost::shared_ptr<KondoMotor> actuator(new KondoMotor(&ics, nh, std::string(actuators[i]), jnt_state_interface, jnt_pos_interface));
	    actuator_vector.push_back(actuator);
	}
	registerInterface(&jnt_state_interface);
	registerInterface(&jnt_pos_interface);
    }
    // Read & write function
    void update () {
	for (int i=0; i<actuator_vector.size(); i++) {
	    actuator_vector[i]->update();
	}
    }
    ~KondoDriver () {
	ics_close (&ics);
    }
    ros::Time getTime() const {return ros::Time::now();}
    ros::Duration getPeriod() const {return ros::Duration(0.01);}

#if 0
    void listMotors(void) {
	for (int i=0; i<actuator_vector.size(); i++) {
	    actuator_vector[i]->update();
	}
    }
#endif
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kondo_driver");
    ros::NodeHandle nh;

    // Create hardware interface 
    KondoDriver robot(argc-1, &argv[1]);
    // Connect to controller manager
    controller_manager::ControllerManager cm(&robot, nh);

    ros::Rate rate(1.0 / robot.getPeriod().toSec());
    ros::AsyncSpinner spinner(1);
    spinner.start();

    while(ros::ok()){
	cm.update(robot.getTime(), robot.getPeriod());
	robot.update();
	rate.sleep();
    }
    spinner.stop();

    return 0;
}


#if 0

#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/transmission_interface.h>

int main(int argc, char** argv)
{
  using namespace transmission_interface;

  // Raw data
  double a_pos;
  double j_pos;

  // Transmission
  SimpleTransmission trans(10.0); // 10x reducer

  // Wrap raw data
  ActuatorData a_data;
  a_data.position.push_back(&a_pos);

  JointData j_data;
  j_data.position.push_back(&j_pos);

  // Transmission interface
  ActuatorToJointPositionInterface act_to_jnt_pos;
  act_to_jnt_pos.registerHandle(ActuatorToJointPositionHandle("trans", &trans, a_data, j_data));

  // Propagate actuator position to joint space
  act_to_jnt_pos.propagate();
}
#endif
