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
#include "ics.h"
}

/* Maximum motor num (32 is maximum on spec. sheet) */
const int MAX_MOTOR_NUM = 12;

class KondoICSHW : public hardware_interface::RobotHW
{
  private:
    ICSData ics; // ICS hardware resource
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;
    double cmd[MAX_MOTOR_NUM];
    double pos[MAX_MOTOR_NUM];
    double vel[MAX_MOTOR_NUM];
    double eff[MAX_MOTOR_NUM];
    std::vector<std::string> joint_name;
    std::vector<std::string> motor_name;
    
  public:
    KondoICSHW () : joint_name(MAX_MOTOR_NUM, "none"), motor_name(MAX_MOTOR_NUM, "none") { 
	pos[0] = pos[1] = 0;
	vel[0] = vel[1] = 0;

	if (ics_init(&ics) < 0) {
	    ROS_ERROR ("Could not init: %s\n", ics.error);
	    exit (0);
	}
	listMotors();
	// Read parameters
	ros::NodeHandle nh;
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
    }
    void update (void) {
	for (int i=0; i<MAX_MOTOR_NUM; i++) {
	    if (motor_name[i] != "none") {
		int pulse_cmd = 8192.0 + cmd[i]*8192.0*180.0/M_PI/135.0;
		int pulse_ret = ics_pos(&ics, i, pulse_cmd);
		pos[i] = M_PI*135.0*(pulse_ret - 8192.0) / 8192.0 / 180.0;
		// ROS_INFO("pulse[%d]:cmd, ret: %d, %d\n", i, pulse_cmd, pulse_ret);
	    }
	}
    }
    ~KondoICSHW () {
	listMotors();
	ics_close (&ics);
    }

    ros::Time getTime() const {return ros::Time::now();}
    ros::Duration getPeriod() const {return ros::Duration(0.01);}

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
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kondo_ics_driver");
    ros::NodeHandle nh;

    KondoICSHW robot;
    controller_manager::ControllerManager cm(&robot, nh);

    // ros::ServiceServer service = nh.advertiseService("set_power", set_power);

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
bool set_power(::Request  &req,
         beginner_tutorials::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}


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
