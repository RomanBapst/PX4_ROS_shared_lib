/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file fw_att_control_main.c
 * Implementation of a generic attitude controller based on classic orthogonal PIDs.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <cmath>
#include <math.h>
#include <time.h>


#include <ros/ros.h>

#include <math/Vector.hpp>
#include <math/Matrix.hpp>
#include <math/Limits.hpp>
#include <math/Quaternion.hpp>
#include <geo/geo.h>

#include <mavros/ManualControl.h>
#include <mavros/AttitudeRates.h>
#include <mavros/AttitudeSetpoint.h>
#include <mavros/AttitudeRateSetpoint.h>
#include <mavros/Attitude.h>
#include <mavros/Airspeed.h>
#include <mavros/Actuator.h>

#include <manual_control_setpoint.h>
#include <vehicle_attitude.h>
#include <vehicle_attitude_setpoint.h>
#include <vehicle_rates_setpoint.h>
#include <vehicle_control_mode.h>
#include <actuator_armed.h>
#include <actuator_controls.h>
#include <airspeed.h>
#include <vehicle_global_position.h>

#include <attitude_fw/ecl_pitch_controller.h>
#include <attitude_fw/ecl_roll_controller.h>
#include <attitude_fw/ecl_yaw_controller.h>

/**
 * Fixedwing attitude control app start / stop handling function
 *
 * @ingroup apps
 */

class FixedwingAttitudeControl
{
public:
	/**
	 * Constructor
	 */
	FixedwingAttitudeControl();

	/**
	 * Destructor, also kills the sensors task.
	 */
	~FixedwingAttitudeControl(){};

	/**
	 * Start the sensors task.
	 *
	 * @return	OK on success.
	 */
	int		start();

	/**
	 * Task status
	 *
	 * @return	true if the mainloop is running
	 */
	bool		task_running() { return _task_running; }

private:

	bool		_task_should_exit;		/**< if true, sensor task should exit */
	bool		_task_running;			/**< if true, task is running in its mainloop */
	int		_control_task;			/**< task handle for sensor task */

	int		_att_sub;			/**< vehicle attitude subscription */
	int		_accel_sub;			/**< accelerometer subscription */
	int		_att_sp_sub;			/**< vehicle attitude setpoint */
	int		_attitude_sub;			/**< raw rc channels data subscription */
	int		_airspeed_sub;			/**< airspeed subscription */
	int		_vcontrol_mode_sub;		/**< vehicle status subscription */
	int 		_params_sub;			/**< notification of parameter updates */
	int 		_manual_sub;			/**< notification of manual control updates */
	int		_global_pos_sub;		/**< global position subscription */

	ros::NodeHandle n;
	ros::Subscriber attitude_sub;
	ros::Subscriber manual_control_sub;
	ros::Subscriber airspeed_sub;


	ros::Publisher _actuators_virtual_5_pub;
	ros::Publisher _attitude_sp_pub;
	ros::Publisher _rate_sp_pub;

	double time_last;	//used to measure elapsed time



	struct vehicle_attitude_s			_att;			/**< vehicle attitude */
	struct vehicle_attitude_setpoint_s		_att_sp;		/**< vehicle attitude setpoint */
	struct manual_control_setpoint_s		_manual;		/**< r/c channel data */
	struct airspeed_s				_airspeed;		/**< airspeed */
	struct vehicle_control_mode_s			_vcontrol_mode;		/**< vehicle control mode */
	struct actuator_controls_s			_actuators;		/**< actuator control inputs */
	struct actuator_controls_s			_actuators_airframe;	/**< actuator control inputs */
	struct vehicle_global_position_s		_global_pos;		/**< global position */



	bool		_setpoint_valid;		/**< flag if the position control setpoint is valid */
	bool		_debug;				/**< if set to true, print debug output */

	struct {
		float tconst;
		float p_p;
		float p_d;
		float p_i;
		float p_ff;
		float p_rmax_pos;
		float p_rmax_neg;
		float p_integrator_max;
		float p_roll_feedforward;
		float r_p;
		float r_d;
		float r_i;
		float r_ff;
		float r_integrator_max;
		float r_rmax;
		float y_p;
		float y_i;
		float y_d;
		float y_ff;
		float y_roll_feedforward;
		float y_integrator_max;
		float y_coordinated_min_speed;
		float y_rmax;

		float airspeed_min;
		float airspeed_trim;
		float airspeed_max;

		float trim_roll;
		float trim_pitch;
		float trim_yaw;
		float rollsp_offset_deg;			/**< Roll Setpoint Offset in deg */
		float pitchsp_offset_deg;			/**< Pitch Setpoint Offset in deg */
		float rollsp_offset_rad;			/**< Roll Setpoint Offset in rad */
		float pitchsp_offset_rad;			/**< Pitch Setpoint Offset in rad */
		float man_roll_max;						/**< Max Roll in rad */
		float man_pitch_max;					/**< Max Pitch in rad */

	}		_parameters;			/**< local copies of interesting parameters */



	ECL_RollController				_roll_ctrl;
	ECL_PitchController				_pitch_ctrl;
	ECL_YawController				_yaw_ctrl;




	void		task_main();
	double		get_dt();
	uint64_t	elapsed_time(uint64_t &time_ref);

	void 		attitude_cb(mavros::Attitude msg);
	void 		manual_control_cb(mavros::ManualControl msg);
	void 		airspeed_cb(mavros::Airspeed msg);
	void		fill_attitude_sp_msg(mavros::AttitudeSetpoint &msg,struct vehicle_attitude_setpoint_s &att_sp);
	void		fill_attitude_rate_sp_msg(mavros::AttitudeRateSetpoint &msg, struct vehicle_rates_setpoint_s &rates_sp);
	void 		fill_actuator_msg(mavros::Actuator &msg,struct actuator_controls_s &actuators);

};



FixedwingAttitudeControl::FixedwingAttitudeControl() :

	/* states */
	_setpoint_valid(false),
	_debug(false)
{

	//setup all subscribers
	attitude_sub 		= n.subscribe("mavros/imu/Attitude",1000,&FixedwingAttitudeControl::attitude_cb,this);
	manual_control_sub 	= n.subscribe("mavros/ManualSetpoint",1000,&FixedwingAttitudeControl::manual_control_cb,this);
	airspeed_sub 		= n.subscribe("mavros/Airspeed",1000,&FixedwingAttitudeControl::airspeed_cb,this);

	//setup all publishers
	_actuators_virtual_5_pub 	= n.advertise<mavros::Actuator>("FixedwingAttitudeControl/actuators_virtual_5",10);
	_attitude_sp_pub 			= n.advertise<mavros::AttitudeSetpoint>("FixedwingAttitudeControl/attitude_setpoint",10);
	_rate_sp_pub 				= n.advertise<mavros::AttitudeRateSetpoint>("FixedwingAttitudeControl/attitude_rate_setpoint",10);

	//setup time variable, this is used by the class-method 'get_dt()'
	time_last = ros::Time::now().toSec();

	/* safely initialize structs */
	_att = {};
	_att_sp = {};
	_manual = {};
	_airspeed = {};
	_vcontrol_mode = {};
	_actuators = {};
	_actuators_airframe = {};
	_global_pos = {};

	_vcontrol_mode.flag_armed 						= true;
	_vcontrol_mode.flag_control_altitude_enabled 	= false;
	_vcontrol_mode.flag_control_attitude_enabled 	= true;
	_vcontrol_mode.flag_control_auto_enabled		= false;
	_vcontrol_mode.flag_control_climb_rate_enabled 	= false;
	_vcontrol_mode.flag_control_manual_enabled 		= true;
	_vcontrol_mode.flag_control_offboard_enabled 	= false;
	_vcontrol_mode.flag_control_position_enabled 	= false;
	_vcontrol_mode.flag_control_rates_enabled 		= true;
	_vcontrol_mode.flag_control_termination_enabled = false;
	_vcontrol_mode.flag_control_velocity_enabled	= false;
	_vcontrol_mode.flag_external_manual_override_ok	= false;
	_vcontrol_mode.flag_system_hil_enabled			= false;

	//initialize parameters struct ...
	memset(&_parameters,0,sizeof(_parameters));
	_parameters.tconst = 0.1;	//400 Hz
	_parameters.p_p = 0.3;
	_parameters.p_d = 0;
	_parameters.p_i = 0.01;
	_parameters.p_ff = 0.2;
	_parameters.p_rmax_pos = 0;
	_parameters.p_rmax_neg = 0;
	_parameters.p_integrator_max = 1;
	_parameters.p_roll_feedforward = 0.3;
	_parameters.r_p = 1;
	_parameters.r_d = 0;
	_parameters.r_i = 0.01;
	_parameters.r_ff = 0.3;
	_parameters.r_integrator_max = 1;
	_parameters.r_rmax = 0.0;
	_parameters.y_p = 0.5;
	_parameters.y_i = 0;
	_parameters.y_d = 0;
	_parameters.y_ff = 0.3;
	_parameters.y_roll_feedforward = 0.3;
	_parameters.y_integrator_max = 1;
	_parameters.y_coordinated_min_speed = 0;
	_parameters.y_rmax = 0.3;

	_parameters.airspeed_min = 5;
	_parameters.airspeed_trim = 10;
	_parameters.airspeed_max = 20;

	_parameters.trim_roll = 0;
	_parameters.trim_pitch = 0;
	_parameters.trim_yaw = 0;
	_parameters.rollsp_offset_deg = 0;			/**< Roll Setpoint Offset in deg */
	_parameters.pitchsp_offset_deg = 0;			/**< Pitch Setpoint Offset in deg */
	_parameters.rollsp_offset_rad = 0;			/**< Roll Setpoint Offset in rad */
	_parameters.pitchsp_offset_rad = 0;
	_parameters.man_pitch_max = 0.6;
	_parameters.man_roll_max = 0.6;

	/* pitch control parameters */
	_pitch_ctrl.set_time_constant(_parameters.tconst);
	_pitch_ctrl.set_k_p(_parameters.p_p);
	_pitch_ctrl.set_k_i(_parameters.p_i);
	_pitch_ctrl.set_k_ff(_parameters.p_ff);
	_pitch_ctrl.set_integrator_max(_parameters.p_integrator_max);
	_pitch_ctrl.set_max_rate_pos(math::radians(_parameters.p_rmax_pos));
	_pitch_ctrl.set_max_rate_neg(math::radians(_parameters.p_rmax_neg));
	_pitch_ctrl.set_roll_ff(_parameters.p_roll_feedforward);

	/* roll control parameters */
	_roll_ctrl.set_time_constant(_parameters.tconst);
	_roll_ctrl.set_k_p(_parameters.r_p);
	_roll_ctrl.set_k_i(_parameters.r_i);
	_roll_ctrl.set_k_ff(_parameters.r_ff);
	_roll_ctrl.set_integrator_max(_parameters.r_integrator_max);
	_roll_ctrl.set_max_rate(math::radians(_parameters.r_rmax));

	/* yaw control parameters */
	_yaw_ctrl.set_k_p(_parameters.y_p);
	_yaw_ctrl.set_k_i(_parameters.y_i);
	_yaw_ctrl.set_k_ff(_parameters.y_ff);
	_yaw_ctrl.set_integrator_max(_parameters.y_integrator_max);
	_yaw_ctrl.set_coordinated_min_speed(_parameters.y_coordinated_min_speed);
	_yaw_ctrl.set_max_rate(math::radians(_parameters.y_rmax));
}



double FixedwingAttitudeControl::get_dt()
{
	double helper = time_last;
	time_last 	  = ros::Time::now().toSec();
	return ros::Time::now().toSec() - helper;

}

uint64_t FixedwingAttitudeControl::elapsed_time(uint64_t &time_ref)
{
	return (ros::Time::now().toNSec() - time_ref) / 1e3;	//maybe should guard against negative values
}

//this function is automatically called if a mavlink attitude message arrives from the PIXHAWK
//this function triggers the main-task function, which calculates the attitude rates setpoint
void FixedwingAttitudeControl::attitude_cb(const mavros::Attitude msg)
{
	_att.timestamp = ros::Time::now().toNSec() / 1e3;
	_att.roll 		= msg.roll;
	_att.pitch 		= msg.pitch;
	_att.yaw 		= msg.yaw;
	_att.rollspeed 	= msg.rollspeed;
	_att.pitchspeed = msg.pitchspeed;
	_att.yawspeed 	= msg.yawspeed;

//	//calculate Rotation matrix
	math::Matrix<3,3> R;
	R.from_euler(_att.roll,_att.pitch,_att.yaw);

	_att.R[0][0] = R(0,0);
	_att.R[0][1] = R(0,1);
	_att.R[0][2] = R(0,2);
	_att.R[1][0] = R(1,0);
	_att.R[1][1] = R(1,1);
	_att.R[1][2] = R(1,2);
	_att.R[2][0] = R(2,0);
	_att.R[2][1] = R(2,1);
	_att.R[2][2] = R(2,2);
	_att.R_valid = true;


task_main(); //run the main task


}

void FixedwingAttitudeControl::manual_control_cb(const mavros::ManualControl msg)
{
	_manual.timestamp = ros::Time::now().toNSec() / 1e3;
	_manual.x = (float)msg.x/1000.f;
	_manual.y = (float)msg.y/1000.f;
	_manual.z = (float)msg.z/1000.f;
	_manual.r = (float)msg.r/1000.f;
	//still have to fill out all other field, like timestamp...
}



void FixedwingAttitudeControl::airspeed_cb(mavros::Airspeed msg)
{
	_airspeed.timestamp = ros::Time::now().toNSec() / 1e3;
	_airspeed.true_airspeed_m_s 		= msg.true_airspeed_m_s;
	_airspeed.indicated_airspeed_m_s 	= msg.indicated_airspeed_m_s;
	_airspeed.air_temperature_celsius 	= msg.air_temperature_celsius;
}

void FixedwingAttitudeControl::fill_attitude_sp_msg(mavros::AttitudeSetpoint &msg,struct vehicle_attitude_setpoint_s &att_sp)
{
	msg.roll = att_sp.roll_body;
	msg.pitch = att_sp.pitch_body;
	msg.yaw  = att_sp.yaw_body;
	msg.thrust = att_sp.thrust;

}

void FixedwingAttitudeControl::fill_attitude_rate_sp_msg(mavros::AttitudeRateSetpoint &msg, struct vehicle_rates_setpoint_s &rates_sp)
{
	msg.rollrate = rates_sp.roll;
	msg.pitchrate = rates_sp.pitch;
	msg.yawrate  = rates_sp.yaw;

}

void FixedwingAttitudeControl::fill_actuator_msg(mavros::Actuator &msg,struct actuator_controls_s &actuators)
{
	msg.roll = actuators.control[0];
	msg.pitch = actuators.control[1];
	msg.yaw = actuators.control[2];
	msg.thrust = actuators.control[3];

}

void
FixedwingAttitudeControl::task_main()
{
	float dt = get_dt();
	//protect against too small or too large dt
	if (dt < 0.002f)
	{
		dt = 0.002f;
	}
	else if (dt > 0.02f)
	{
		dt = 0.02f;
	}

	/* lock integrator until control is started */
	bool lock_integrator;

	if (_vcontrol_mode.flag_control_attitude_enabled) {
		lock_integrator = false;

	} else {
		lock_integrator = true;
	}

	/* Simple handling of failsafe: deploy parachute if failsafe is on */
	if (_vcontrol_mode.flag_control_termination_enabled) {
		_actuators_airframe.control[1] = 1.0f;

	} else {
		_actuators_airframe.control[1] = 0.0f;
	}

	/* decide if in stabilized or full manual control */

	if (_vcontrol_mode.flag_control_attitude_enabled) {

			/* scale around tuning airspeed */

			float airspeed;

			/* if airspeed is not updating, we assume the normal average speed */
			if (bool nonfinite = !std::isfinite(_airspeed.true_airspeed_m_s) ||
			    elapsed_time(_airspeed.timestamp) > 1e6) {
				airspeed = _parameters.airspeed_trim;

			} else {
				/* prevent numerical drama by requiring 0.5 m/s minimal speed */
				airspeed = math::max(0.5f, _airspeed.true_airspeed_m_s);
			}

			/*
			 * For scaling our actuators using anything less than the min (close to stall)
			 * speed doesn't make any sense - its the strongest reasonable deflection we
			 * want to do in flight and its the baseline a human pilot would choose.
			 *
			 * Forcing the scaling to this value allows reasonable handheld tests.
			 */

			float airspeed_scaling = _parameters.airspeed_trim / ((airspeed < _parameters.airspeed_min) ? _parameters.airspeed_min : airspeed);

			float roll_sp = _parameters.rollsp_offset_rad;
			float pitch_sp = _parameters.pitchsp_offset_rad;
			float throttle_sp = 0.0f;

			if (_vcontrol_mode.flag_control_velocity_enabled || _vcontrol_mode.flag_control_position_enabled) {
				/* read in attitude setpoint from attitude setpoint uorb topic */
				roll_sp = _att_sp.roll_body + _parameters.rollsp_offset_rad;
				pitch_sp = _att_sp.pitch_body + _parameters.pitchsp_offset_rad;
				throttle_sp = _att_sp.thrust;

				/* reset integrals where needed */
				if (_att_sp.roll_reset_integral) {
					_roll_ctrl.reset_integrator();
				}
				if (_att_sp.pitch_reset_integral) {
					_pitch_ctrl.reset_integrator();
				}
				if (_att_sp.yaw_reset_integral) {
					_yaw_ctrl.reset_integrator();
				}
			} else {
				/*
				 * Scale down roll and pitch as the setpoints are radians
				 * and a typical remote can only do around 45 degrees, the mapping is
				 * -1..+1 to -man_roll_max rad..+man_roll_max rad (equivalent for pitch)
				 *
				 * With this mapping the stick angle is a 1:1 representation of
				 * the commanded attitude.
				 *
				 * The trim gets subtracted here from the manual setpoint to get
				 * the intended attitude setpoint. Later, after the rate control step the
				 * trim is added again to the control signal.
				 */
				roll_sp = (_manual.y * _parameters.man_roll_max - _parameters.trim_roll)
					+ _parameters.rollsp_offset_rad;
				pitch_sp = -(_manual.x * _parameters.man_pitch_max - _parameters.trim_pitch)
				+ _parameters.pitchsp_offset_rad;
				throttle_sp = _manual.z;
				_actuators.control[4] = _manual.flaps;

				/*
				 * in manual mode no external source should / does emit attitude setpoints.
				 * emit the manual setpoint here to allow attitude controller tuning
				 * in attitude control mode.
				 */
				struct vehicle_attitude_setpoint_s att_sp;
				//att_sp.timestamp = hrt_absolute_time();
				att_sp.roll_body = roll_sp;
				att_sp.pitch_body = pitch_sp;
				att_sp.yaw_body = 0.0f - _parameters.trim_yaw;
				att_sp.thrust = throttle_sp;

				//publish attitude setpoint
				mavros::AttitudeSetpoint message;
				fill_attitude_sp_msg(message,att_sp);
				_attitude_sp_pub.publish(message);
			}

			/* Prepare speed_body_u and speed_body_w */
			float speed_body_u = 0.0f;
			float speed_body_v = 0.0f;
			float speed_body_w = 0.0f;
			if(_att.R_valid) 	{
				speed_body_u = _att.R[0][0] * _global_pos.vel_n + _att.R[1][0] * _global_pos.vel_e + _att.R[2][0] * _global_pos.vel_d;
				speed_body_v = _att.R[0][1] * _global_pos.vel_n + _att.R[1][1] * _global_pos.vel_e + _att.R[2][1] * _global_pos.vel_d;
				speed_body_w = _att.R[0][2] * _global_pos.vel_n + _att.R[1][2] * _global_pos.vel_e + _att.R[2][2] * _global_pos.vel_d;
			} else	{
				ROS_INFO("Warning: Did not get a valid rotation matrix!");
			}

			/* Run attitude controllers */
			if (std::isfinite(roll_sp) && std::isfinite(pitch_sp)) {
				_roll_ctrl.control_attitude(roll_sp, _att.roll);
				_pitch_ctrl.control_attitude(pitch_sp, _att.roll, _att.pitch, airspeed);
				_yaw_ctrl.control_attitude(_att.roll, _att.pitch,
						speed_body_u, speed_body_v, speed_body_w,
						_roll_ctrl.get_desired_rate(), _pitch_ctrl.get_desired_rate()); //runs last, because is depending on output of roll and pitch attitude

				/* Run attitude RATE controllers which need the desired attitudes from above, add trim */
				float roll_u = _roll_ctrl.control_bodyrate(_att.pitch,
						_att.rollspeed, _att.yawspeed,
						_yaw_ctrl.get_desired_rate(),
						_parameters.airspeed_min, _parameters.airspeed_max, airspeed, airspeed_scaling, lock_integrator);
				_actuators.control[0] = (std::isfinite(roll_u)) ? roll_u + _parameters.trim_roll : _parameters.trim_roll;
				if (!std::isfinite(roll_u)) {
					_roll_ctrl.reset_integrator();
					ROS_INFO("Warning: Roll output of roll-rate controller not finite!");
				}

				float pitch_u = _pitch_ctrl.control_bodyrate(_att.roll, _att.pitch,
						_att.pitchspeed, _att.yawspeed,
						_yaw_ctrl.get_desired_rate(),
						_parameters.airspeed_min, _parameters.airspeed_max, airspeed, airspeed_scaling, lock_integrator);
				_actuators.control[1] = (std::isfinite(pitch_u)) ? pitch_u + _parameters.trim_pitch : _parameters.trim_pitch;
				ROS_INFO("Pitch output: %.5f",pitch_u);
				if (!std::isfinite(pitch_u)) {
					_pitch_ctrl.reset_integrator();
					ROS_INFO("Warning: Pitch output of pitch-rate controller not finite!");
				}

				float yaw_u = _yaw_ctrl.control_bodyrate(_att.roll, _att.pitch,
						_att.pitchspeed, _att.yawspeed,
						_pitch_ctrl.get_desired_rate(),
						_parameters.airspeed_min, _parameters.airspeed_max, airspeed, airspeed_scaling, lock_integrator);
				_actuators.control[2] = (std::isfinite(yaw_u)) ? yaw_u + _parameters.trim_yaw : _parameters.trim_yaw;
				if (!std::isfinite(yaw_u)) {
					_yaw_ctrl.reset_integrator();
					ROS_INFO("Warning: Yaw output of yaw-rate controller not finite!");
				}

				/* throttle passed through */
				_actuators.control[3] = (std::isfinite(throttle_sp)) ? throttle_sp : 0.0f;
				if (!std::isfinite(throttle_sp)) {
					ROS_INFO("Warning: Throttle setpoint not finite!");
				}
			} else {
				ROS_INFO("Warning: Either roll or pitch setpoint not finite!");
			}

			/*
			 * Lazily publish the rate setpoint (for analysis, the actuators are published below)
			 * only once available
			 */
			vehicle_rates_setpoint_s rates_sp;
			rates_sp.roll = _roll_ctrl.get_desired_rate();
			rates_sp.pitch = _pitch_ctrl.get_desired_rate();
			rates_sp.yaw = _yaw_ctrl.get_desired_rate();

			//publish attitude setpoint
			mavros::AttitudeRateSetpoint message;
			fill_attitude_rate_sp_msg(message,rates_sp);
			_rate_sp_pub.publish(message);

			} else {
				/* manual/direct control */
				_actuators.control[0] = _manual.y;
				_actuators.control[1] = -_manual.x;
				_actuators.control[2] = _manual.r;
				_actuators.control[3] = _manual.z;
				_actuators.control[4] = _manual.flaps;
			}

			_actuators.control[5] = _manual.aux1;
			_actuators.control[6] = _manual.aux2;
			_actuators.control[7] = _manual.aux3;

			//publish the actuator setpoints
			mavros::Actuator message;
			fill_actuator_msg(message,_actuators);
			_actuators_virtual_5_pub.publish(message);


		}



int main(int argc, char *argv[])
{
	//intialize node
	ros::init(argc,argv,"fw_att_controller");

	//create instance of the node
	FixedwingAttitudeControl AttController;

	ros::spin();

	return 0;
}
