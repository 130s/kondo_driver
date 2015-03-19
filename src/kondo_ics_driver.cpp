/**
 * Kondo ICS motor driver
 */
#include <iostream>
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
public:
    double cmd, pos, vel, eff;
    std::string joint_name;
    bool set_power (kondo_driver::setPower::Request &req, kondo_driver::setPower::Response &res) {
	ROS_INFO("id %d, request: %d", this->id, req.request);
	motor_power = req.request;
	res.result = req.request;
	return true;
    }
    KondoMotor (ros::NodeHandle nh, std::string actuator_name, hardware_interface::JointStateInterface& state_interface, hardware_interface::PositionJointInterface& pos_interface) {
	cmd=0, pos=0;
	if (nh.getParam(std::string("/kondo_driver/")+actuator_name+std::string("/id"), id)) {
	    ROS_INFO("id: %d", id);
	}
	if (nh.getParam(std::string("/kondo_driver/")+actuator_name+std::string("/joint"), joint_name)) {
	    ROS_INFO("joint: %s", joint_name.c_str());
	}
	hardware_interface::JointStateHandle state_handle(joint_name, &pos, &vel, &eff);
	state_interface.registerHandle(state_handle);
	hardware_interface::JointHandle pos_handle(state_interface.getHandle(joint_name), &cmd);
	pos_interface.registerHandle(pos_handle);
	power_service = nh.advertiseService(actuator_name+std::string("/set_power"), &KondoMotor::set_power, this);
    }
    void update (void) {
#if 0
	int pulse_cmd = 0;
	if (motor_power == true) {
	    pulse_cmd = radian_to_pulse(cmd);
	}
	//int pulse_ret = ics_pos(&ics, i, pulse_cmd);
	int pulse_ret = 0;
	//pos = pulse_to_radian (pulse_ret);
	//pos = cmd;
	//ROS_INFO("pulse[%d]:cmd, ret: %d, %d\n", id, pulse_cmd, pulse_ret);
#endif
	ROS_INFO("pulse[%d]:cmd, pos: %f, %f\n", id, cmd, pos);
    }
};

class KondoICSHW : public hardware_interface::RobotHW
{
  private:
    ICSData ics; // ICS hardware resource
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;
    std::vector<KondoMotor> actuator_vector;
  public:
    KondoICSHW (ros::NodeHandle nh, int num, char** actuators) {
	if (ics_init(&ics) < 0) {
	    ROS_ERROR ("Could not init: %s\n", ics.error);
	    // exit (0);
	}
	//listMotors();
	// Load parameters
	for (int i=0; i<num; i++) {
	    std::string actuator_name = std::string("/kondo_driver/")+std::string(actuators[i]);
	    if (!nh.hasParam(actuator_name)) {
		ROS_WARN("No parameter for actuator %s", actuator_name.c_str());
	    }
	    KondoMotor* actuator = new KondoMotor(nh, std::string(actuators[i]), jnt_state_interface, jnt_pos_interface);
	    actuator_vector.push_back(*actuator);

	    ROS_INFO("joint_name2: %s", actuator->joint_name.c_str());
	    hardware_interface::JointStateHandle state_handle(actuator->joint_name, &actuator->pos, &actuator->vel, &actuator->eff);
	    jnt_state_interface.registerHandle(state_handle);
	}
	registerInterface(&jnt_state_interface);
	for (int i=0; i<num; i++) {
	    std::string actuator_name = std::string("/kondo_driver/")+std::string(actuators[i]);
	    if (!nh.hasParam(actuator_name)) {
		ROS_WARN("No parameter for actuator %s", actuator_name.c_str());
	    }
	    KondoMotor* actuator = new KondoMotor(nh, std::string(actuators[i]), jnt_state_interface, jnt_pos_interface);
	    actuator_vector.push_back(*actuator);

	    ROS_INFO("joint_name2: %s", actuator->joint_name.c_str());
	    hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(actuator->joint_name), &actuator->cmd);
	    jnt_pos_interface.registerHandle(pos_handle);
	}
	registerInterface(&jnt_pos_interface);
#if 0
	// Setup actuator
	std::string driver_name = std::string("/kondo_driver/")+actuator_name;
	ROS_INFO("actuator: %s", actuator_name.c_str());
	int id;
	if (nh.getParam(driver_name+sftd::string("/id"), id)) {
	    ROS_INFO("id: %d", id);
	}
	std::string joint_name;
	if (nh.getParam(driver_name+std::string("/joint"), joint_name)) {
	    ROS_INFO("joint: %s", joint_name.c_str());
	}
	// Read parameters
	for (int i=0; i<MAX_MOTOR_NUM; i++) {
	    std::string motor_str, joint_str;
	    motor_str = motor_name[i];
	    if (nh.hasParam(std::string("/kondo_ics_driver/")+motor_str)) {
		ROS_INFO("found:  %s", motor_str.c_str());
		if (nh.getParam(std::string("/kondo_ics_driver/")+motor_str+std::string("/joint"), joint_str)) {
		    ROS_INFO("mapping motor %s to joint: %s", motor_str.c_str(), joint_str.c_str());
		    joint_name[i] = joint_str;
		}
	    }
	}
	for (int i=0; i<MAX_MOTOR_NUM; i++) {
	    if (joint_name[i] != "none") {
		// Connect and register the joint state interface
		hardware_interface::JointStateHandle state_handle (joint_name[i], &pos[i], &vel[i], &eff[i]);
		jnt_state_interface.registerHandle(state_handle);
	    }
	}
	registerInterface(&jnt_state_interface);
	// Connect and register the joint position interface
	for (int i=0; i<MAX_MOTOR_NUM; i++) {
	    if (joint_name[i] != "none") {
		hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(joint_name[i]), &cmd[i]);
		jnt_pos_interface.registerHandle(pos_handle);
	    }
	}
	registerInterface(&jnt_pos_interface);
#endif
    }
    void update (void) {
	for (int i=0; i<actuator_vector.size(); i++) {
	    actuator_vector[i].update();
	}
    }
    ~KondoICSHW () {
	//listMotors();
	ics_close (&ics);
    }
    ros::Time getTime() const {return ros::Time::now();}
    ros::Duration getPeriod() const {return ros::Duration(0.01);}

#if 0
    void listMotors(void) {
	for (int i=0; i<MAX_MOTOR_NUM; i++) {
	    int ret = ics_pos(&ics, i, 0);
	    if (ret > 0) {
		char buff[32];
		sprintf (buff, "motor_%d", i);
		motor_name[i] = std::string(buff);
	    } else {
		motor_name[i] = std::string("none");
	    }
	}
	for (int i=0; i<MAX_MOTOR_NUM; i++) {
	    if (motor_name[i] != "none") {
		ROS_INFO("motor[%d]=%s", i, motor_name[i].c_str());
	    }
	}
    }
#endif
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kondo_ics_driver");
    ros::NodeHandle nh;

    KondoICSHW robot(nh, argc-1, &argv[1]);
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
