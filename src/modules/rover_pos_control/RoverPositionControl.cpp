/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 *
 * This module is a modification of the fixed wing module and it is designed for ground rovers.
 * It has been developed starting from the fw module, simplified and improved with dedicated items.
 *
 * All the acknowledgments and credits for the fw wing app are reported in those files.
 *
 * @author Marco Zorzi <mzorzi@student.ethz.ch>
 */


#include "RoverPositionControl.hpp"
#include <lib/ecl/geo/geo.h>

#define ACTUATOR_PUBLISH_PERIOD_MS 4

using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;

/**
 * L1 control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int rover_pos_control_main(int argc, char *argv[]);

float RoverPositionControl::convert_angle(float angle)
{
	int del = (angle + pi) / (pi * 2);
	float normalized_angle = 0;
	if (angle + pi < 0) {
		normalized_angle = angle + (abs(del) + 1) * pi * 2;
	} else {
		normalized_angle = angle - del * pi * 2;
	}
	return normalized_angle;
}

RoverPositionControl::RoverPositionControl() :
	ModuleParams(nullptr),
	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "rover position control")) // TODO : do we even need these perf counters
{
}

RoverPositionControl::~RoverPositionControl()
{
	perf_free(_loop_perf);
}

void RoverPositionControl::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();

		_gnd_control.set_l1_damping(_param_l1_damping.get());
		_gnd_control.set_l1_period(_param_l1_period.get());
		_gnd_control.set_l1_roll_limit(math::radians(0.0f));

		pid_init(&_speed_ctrl, PID_MODE_DERIVATIV_CALC, 0.01f);
		pid_set_parameters(&_speed_ctrl,
				   _param_speed_p.get(),
				   _param_speed_d.get(),
				   _param_speed_i.get(),
				   _param_speed_imax.get(),
				   _param_gndspeed_max.get());
	}
}

void
RoverPositionControl::vehicle_control_mode_poll()
{
	bool updated;
	orb_check(_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
	}
}

void
RoverPositionControl::manual_control_setpoint_poll()
{
	bool manual_updated;
	orb_check(_manual_control_sub, &manual_updated);

	if (manual_updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sub, &_manual);
	}
}

void
RoverPositionControl::position_setpoint_triplet_poll()
{
	bool pos_sp_triplet_updated;
	orb_check(_pos_sp_triplet_sub, &pos_sp_triplet_updated);

	if (pos_sp_triplet_updated) {
		orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);
	}
}

void
RoverPositionControl::vehicle_attitude_poll()
{
	bool att_updated;
	orb_check(_vehicle_attitude_sub, &att_updated);

	if (att_updated) {
		orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &_vehicle_att);
	}
}

bool
RoverPositionControl::rover_control_position(const matrix::Vector2f &current_position, const matrix::Vector2f &setpoint, float observer_angle)
{
	_inst.x = current_position(0);
	_inst.y = current_position(1);

	_inst.angle = convert_angle(observer_angle);

	// position regulator
	_inst.e_x = setpoint(0) - _inst.x;
	_inst.e_y = setpoint(1) - _inst.y;
	_inst.e_dst = sqrt(_inst.e_x * _inst.e_x + _inst.e_y * _inst.e_y);
	_debug_msg.control_e_dst = _inst.e_dst;
	_debug_msg.control_e_x = _inst.e_x;
	_debug_msg.control_e_y = _inst.e_y;
	_debug_msg.control_setpoint_x = setpoint(0);
	_debug_msg.control_setpoint_y = setpoint(1);
	_debug_msg.control_real_yaw = _inst.angle;


	if (_inst.e_dst > dst_radius) {
		if (!(_inst.e_x > -0.000001f && _inst.e_x < 0.000001f) && !(_inst.e_y > -0.000001f && _inst.e_y < 0.000001f)) {
			_inst.angle_s = atan2f(_inst.e_y, _inst.e_x);
		}
		_debug_msg.control_angle_est = _inst.angle_s;

		/*if (_inst.e_x < 0) {
			if (_inst.angle_s > 0)
				_inst.angle_s -= pi;
			else
				_inst.angle_s += pi;
		}*/

		if (_inst.angle_s - _inst.angle > pi)
			_inst.angle += 2 * pi;
		else if (_inst.e_ang < -pi) {
			_inst.angle -= 2 * pi;
		}

		_inst.e_ang = _inst.angle_s - _inst.angle;
		_inst.e_dist_dir = _inst.e_dst;
		_inst.e_ang_dir = _inst.e_ang;

		_debug_msg.control_second_est = _inst.e_ang;



		/*_inst.e_ang = _inst.angle_s - _inst.angle;
		if (abs(_inst.e_ang) > 1.570796327) {
			_inst.e_dist_dir = -_inst.e_dst;
			if (_inst.e_ang > 0)
				_inst.e_ang_dir = _inst.e_ang - pi;
			else
				_inst.e_ang_dir = _inst.e_ang + pi;
		} else {
			_inst.e_dist_dir = _inst.e_dst;
			_inst.e_ang_dir = _inst.e_ang;
		}*/

		/*_inst.v_i_value = _inst.v_i_value + _inst.e_dist_dir * kv_i * _dt;
		_inst.w_i_value = _inst.w_i_value + _inst.e_ang_dir * kw_i * _dt;
		_inst.v_set_ul = _inst.e_dist_dir * kv_p + _inst.v_i_value;
		_inst.w_set_ul = _inst.e_ang_dir * kw_p + _inst.w_i_value;*/

		_inst.v_set_ul = _inst.e_dist_dir * kv_p;
		_inst.w_set_ul = _inst.e_ang_dir * kw_p;

	} else {
		_inst.v_set_ul = 0;
		_inst.w_set_ul = 0;
	}

	_debug_msg.control_w_set_ul = _inst.w_set_ul;
	_debug_msg.control_v_set_ul = _inst.v_set_ul;

	if (math::abs_t(_inst.v_set_ul) > v_lim) {
		if (_inst.v_set_ul > 0)
			_inst.v_set = v_lim;
		else
			_inst.v_set = -v_lim;
	} else {
		_inst.v_set = _inst.v_set_ul;
	}

	if (math::abs_t(_inst.w_set_ul) > w_lim) {
		if (_inst.w_set_ul > 0)
			_inst.w_set = w_lim;
		else
			_inst.w_set = -w_lim;
	} else {
		_inst.w_set = _inst.w_set_ul;
	}

	_inst.vl = _inst.v_set - _inst.w_set * L / 2;
	_inst.vr = _inst.v_set + _inst.w_set * L / 2;

	_debug_msg.control_v = _inst.v_set;
	_debug_msg.control_w = _inst.w_set;

	float mission_throttle = _inst.v_set;
	//mission_throttle = math::constrain(mission_throttle, _param_throttle_min.get(), _param_throttle_max.get());

	_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = mission_throttle;
	_act_controls.control[actuator_controls_s::INDEX_YAW] = -_inst.w_set;

	_debug_msg.control_throttle = mission_throttle;
	float w_set = _inst.w_set;
	float vl = _inst.vl;
	float vr = _inst.vr;

	PX4_WARN("rover control info: %f, %f, %f", (double)w_set, (double)vl, (double)vr);
	_debug_msg.control_w = w_set;
	_debug_msg.control_wl = vl;
	_debug_msg.control_wr = vr;

	return true;
}

void
RoverPositionControl::observer(const matrix::Vector2f convert_local_position, const matrix::Vector2f actuator_outputs)
{
	//angle observer

	//PX4_INFO("lat, lon, convert_x, convert_y: %f, %f, %f, %f", (double)_lat, (double)_lon, (double)_x, (double)_y);
	_debug_msg.gps_x = convert_local_position(0);
	_debug_msg.gps_y = convert_local_position(1);

	//float control_h = 0;

	_v = (xn[3] + xn[4])*R/2;
	_w = (xn[3] - xn[4])*R/L;
	observer_speed2d(0) = _v * sinf(h_conv);
	observer_speed2d(1) = _v * cosf(h_conv);
	observer_speed3d(0) = observer_speed2d(0);
	observer_speed3d(1) = observer_speed2d(1);
	observer_speed3d(2) = 0.0f;
	_debug_msg.v = _v;
	_debug_msg.w = _w;
	_debug_msg.v_x = observer_speed2d(0);
	_debug_msg.v_y = observer_speed2d(1);

	if (freq != 0) {
		if (freq != (int)(1 / _dt)) {
			if (freq_count > 70) {
				_dt = 1.0f / freq;
				freq_count = 0;
			} else {
				freq_count += 1;
			}
		} else {
			freq_count = 0;
		}
	} else {
		_dt = 0.1;
	}

	_debug_msg.dt_debug = _dt;

	xn_cur[0] = xn[0] + cosf(xn[2]) * _v * _dt;
	xn_cur[1] = xn[1] + sinf(xn[2]) * _v * _dt;
	xn_cur[2] = xn[2] + (_w * _dt) / 3.5f;
	xn_cur[3] = ((actuator_outputs(1)-1500) / ke - xn[3]) / tm * _dt + xn[3];
	xn_cur[4] = ((actuator_outputs(0)-1500) / ke - xn[4]) / tm * _dt + xn[4];

	if (_gps_pos.timestamp != _gps_prev.timestamp) {
		dz[0] = convert_local_position(0) - xn[0];
		dz[1] = convert_local_position(1) - xn[1];
		d_z[0] = (convert_local_position(0) - _prev_local_pos(0));
		d_z[1] = (convert_local_position(1) - _prev_local_pos(1));

		_v = (xn[3] + xn[4]) / 2;
		if ((double)_v > - 0.0001 && (double)_v < 0.0001) {
			_v = 1;
		}
		if (abs(_v) < 10) {
			_v = math::sign(_v);
		}

		d_zf[0] = cosf(xn[2]);
		d_zf[1] = sinf(xn[2]);

		xn_cur[0] = xn_cur[0] + k * dz[0];
		xn_cur[1] = xn_cur[1] + k * dz[1];
		//xn_cur[2] = xn_cur[2] + kh * (d_z[1] * d_zf[0] - d_z[0] * d_zf[1]) / _v;

		filtered_cor[0] = filtered_cor[1];
		filtered_cor[1] = filtered_cor[2];
		filtered_cor[2] = filtered_cor[3];
		filtered_cor[3] = d_z[1] * d_zf[0] - d_z[0] * d_zf[1];
		_debug_msg.w_cor_filtered = (filtered_cor[0] + filtered_cor[1]+ filtered_cor[2] + filtered_cor[3]) / 4;

		xn_cur[2] = xn_cur[2] + kh * (d_z[1] * d_zf[0] - d_z[0] * d_zf[1]);

		_debug_msg.v_cor = _v;
		_debug_msg.x_cor = dz[0];
		_debug_msg.y_cor = dz[1];
		_debug_msg.dzx = d_z[0];
		_debug_msg.dzy = d_z[1];
		_debug_msg.d_zx = d_zf[0];
		_debug_msg.d_zy = d_zf[1];
		_debug_msg.prev_x = _prev_local_pos(0);
		_debug_msg.prev_y = _prev_local_pos(1);

		_debug_msg.w_cor_v = (d_z[1] * d_zf[0] - d_z[0] * d_zf[1]);

		_prev_local_pos = convert_local_position;
	}

	for (int i=0; i<5; i++) {
		xn[i] = xn_cur[i];
	}

	_gps_prev = _gps_pos;

	_debug_msg.timestamp = hrt_absolute_time();
	_debug_msg.x = xn_cur[0];
	_debug_msg.y = xn_cur[1];

	int del = (xn_cur[2] + pi) / (pi * 2);
	if (xn_cur[2] + pi < 0) {
		h_conv = xn_cur[2] + (abs(del) + 1) * pi * 2;
	} else {
		h_conv = xn_cur[2] - del * pi * 2;
	}

	_debug_msg.real_h = xn_cur[2];
	_debug_msg.h = h_conv;
	_debug_msg.vl = xn_cur[3];
	_debug_msg.vr = xn_cur[4];
}


bool
RoverPositionControl::control_position(const matrix::Vector2f &current_position,
				       const matrix::Vector3f &ground_speed, const position_setpoint_triplet_s &pos_sp_triplet)
{
	float dt = 0.01; // Using non zero value to a avoid division by zero

	if (_control_position_last_called > 0) {
		dt = hrt_elapsed_time(&_control_position_last_called) * 1e-6f;
	}

	_control_position_last_called = hrt_absolute_time();

	bool setpoint = true;

	if ((_control_mode.flag_control_auto_enabled ||
	     _control_mode.flag_control_offboard_enabled) && pos_sp_triplet.current.valid) {
		/* AUTONOMOUS FLIGHT */

		_control_mode_current = UGV_POSCTRL_MODE_AUTO;

		/* get circle mode */
		//bool was_circle_mode = _gnd_control.circle_mode();

		/* current waypoint (the one currently heading for) */
		matrix::Vector2f curr_wp((float)pos_sp_triplet.current.lat, (float)pos_sp_triplet.current.lon);

		/* previous waypoint */
		matrix::Vector2f prev_wp = curr_wp;

		if (pos_sp_triplet.previous.valid) {
			prev_wp(0) = (float)pos_sp_triplet.previous.lat;
			prev_wp(1) = (float)pos_sp_triplet.previous.lon;
		}

		matrix::Vector2f ground_speed_2d(ground_speed);

		float mission_throttle = _param_throttle_cruise.get();

		/* Just control the throttle */
		if (_param_speed_control_mode.get() == 1) {
			/* control the speed in closed loop */

			float mission_target_speed = _param_gndspeed_trim.get();

			if (PX4_ISFINITE(_pos_sp_triplet.current.cruising_speed) &&
			    _pos_sp_triplet.current.cruising_speed > 0.1f) {
				mission_target_speed = _pos_sp_triplet.current.cruising_speed;
			}

			// Velocity in body frame
			const Dcmf R_to_body(Quatf(_vehicle_att.q).inversed());
			const Vector3f vel = R_to_body * Vector3f(observer_speed3d(0), observer_speed3d(1), observer_speed3d(2));

			const float x_vel = vel(0);
			const float x_acc = _vehicle_acceleration_sub.get().xyz[0];

			// Compute airspeed control out and just scale it as a constant
			mission_throttle = _param_throttle_speed_scaler.get()
					   * pid_calculate(&_speed_ctrl, mission_target_speed, x_vel, x_acc, dt);

			// Constrain throttle between min and max
			mission_throttle = math::constrain(mission_throttle, _param_throttle_min.get(), _param_throttle_max.get());

		} else {
			/* Just control throttle in open loop */
			if (PX4_ISFINITE(_pos_sp_triplet.current.cruising_throttle) &&
			    _pos_sp_triplet.current.cruising_throttle > 0.01f) {

				mission_throttle = _pos_sp_triplet.current.cruising_throttle;
			}
		}

		float dist = get_distance_to_next_waypoint(_global_pos.lat, _global_pos.lon,
				pos_sp_triplet.current.lat, pos_sp_triplet.current.lon);

		bool should_idle = true;

		if (pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LOITER) {
			// Because of noise in measurements, if the rover was always trying to reach an exact point, it would
			// move around when it should be parked. So, I try to get the rover within loiter_radius/2, but then
			// once I reach that point, I don't move until I'm outside of loiter_radius.
			// TODO: Find out if there's a better measurement to use than loiter_radius.
			if (dist > pos_sp_triplet.current.loiter_radius) {
				_waypoint_reached = false;

			} else if (dist <= pos_sp_triplet.current.loiter_radius / 2) {
				_waypoint_reached = true;
			}

			should_idle = _waypoint_reached;

		} else if (pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_POSITION ||
			   pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF ||
			   pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {
			should_idle = false;
		}

		if (should_idle) {
			_act_controls.control[actuator_controls_s::INDEX_YAW] = 0.0f;
			_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;

		} else {
			/* waypoint is a plain navigation waypoint or the takeoff waypoint, does not matter */
			_gnd_control.navigate_waypoints(prev_wp, curr_wp, current_position, observer_speed2d);

			_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = mission_throttle;

			float desired_r = observer_speed2d.norm_squared() / math::abs_t(_gnd_control.nav_lateral_acceleration_demand());
			float desired_theta = atan2f(desired_r, _param_wheel_base.get());
			float control_effort = (desired_theta / _param_max_turn_angle.get()) * math::sign(
						_gnd_control.nav_lateral_acceleration_demand());
			control_effort = math::constrain(control_effort, -1.0f, 1.0f);
			_act_controls.control[actuator_controls_s::INDEX_YAW] = control_effort;

			_debug_msg.desired_r = desired_r * math::sign(_gnd_control.nav_lateral_acceleration_demand());
			_debug_msg.desired_theta = desired_theta;
			_debug_msg.control_effort = control_effort;

		}

	} else {
		_control_mode_current = UGV_POSCTRL_MODE_OTHER;
		setpoint = false;
	}

	return setpoint;
}

void
RoverPositionControl::run()
{
	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_manual_control_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	_vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	_actuator_outputs_sub = orb_subscribe(ORB_ID(actuator_outputs));
	_gps_pos_sub = orb_subscribe(ORB_ID(vehicle_gps_position));

	/* rate limit control mode updates to 5Hz */
	orb_set_interval(_control_mode_sub, 200);

	/* rate limit position updates to 50 Hz */
	orb_set_interval(_global_pos_sub, 20);
	orb_set_interval(_local_pos_sub, 20);

	parameters_update(true);

	/* wakeup source(s) */
	px4_pollfd_struct_t fds[3];

	/* Setup of loop */
	fds[0].fd = _global_pos_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _manual_control_sub;
	fds[1].events = POLLIN;
	fds[2].fd = _sensor_combined_sub;
	fds[2].events = POLLIN;

	while (!should_exit()) {
		/* wait for up to 500ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 250);

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		/* check vehicle control mode for changes to publication state */
		vehicle_control_mode_poll();
		//manual_control_setpoint_poll();

		_vehicle_acceleration_sub.update();

		/* update parameters from storage */
		parameters_update();

		bool manual_mode = _control_mode.flag_control_manual_enabled;


		/* only run controller if position changed */
		if (fds[0].revents & POLLIN) {
			perf_begin(_loop_perf);
			if ((int)hrt_absolute_time() > _time + 1000000) {
				_time = hrt_absolute_time();
				_debug_msg.tik_frequansy = iter;
				freq = iter;
				iter = 0;
			} else {
				iter += 1;
			}

			/* load local copies */
			orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);
			orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);

			position_setpoint_triplet_poll();
			vehicle_attitude_poll();

			//Convert Local setpoints to global setpoints
			if (_control_mode.flag_control_offboard_enabled) {
				if (!globallocalconverter_initialized()) {
					globallocalconverter_init(_local_pos.ref_lat, _local_pos.ref_lon,
								  _local_pos.ref_alt, _local_pos.ref_timestamp);

				} else {
					globallocalconverter_toglobal(_pos_sp_triplet.current.x, _pos_sp_triplet.current.y, _pos_sp_triplet.current.z,
								      &_pos_sp_triplet.current.lat, &_pos_sp_triplet.current.lon, &_pos_sp_triplet.current.alt);

				}
			}
			_debug_msg.sp_lat = _pos_sp_triplet.current.lat;
			_debug_msg.sp_lon = _pos_sp_triplet.current.lon;
			//PX4_INFO("global setpoint: %f %f", (double)_pos_sp_triplet.current.lat, (double)_pos_sp_triplet.current.lon);

			// update the reset counters in any case
			_pos_reset_counter = _global_pos.lat_lon_reset_counter;

			matrix::Vector3f ground_speed(_global_pos.vel_n, _global_pos.vel_e,  _global_pos.vel_d);
			//matrix::Vector2f current_position((float)_global_pos.lat, (float)_global_pos.lon);

			orb_copy(ORB_ID(actuator_outputs), _actuator_outputs_sub, &_actuator_outputs);
			orb_copy(ORB_ID(vehicle_gps_position), _gps_pos_sub, &_gps_pos);

			double _lat = (double)_gps_pos.lat / 10000000;
			double _lon = (double)_gps_pos.lon / 10000000;
			double _alt = (double)_gps_pos.lon;

			if (!globallocalconverter_initialized()) {
				globallocalconverter_init(_local_pos.ref_lat, _local_pos.ref_lon,
								_local_pos.ref_alt, _local_pos.ref_timestamp);
			} else {
				globallocalconverter_tolocal(_lat, _lon, _alt, &_cur_local_pos.x, &_cur_local_pos.y, &_cur_local_pos.z);
			}

			/*if (PX4_ISFINITE(convert_start_angle)) {
				convert_start_angle = _local_pos.yaw;
			}
			converted_x = cos(convert_start_angle) * (double)_pos_sp_triplet.current.x - sin(convert_start_angle) * (double)_pos_sp_triplet.current.y;
			converted_y = sin(convert_start_angle) * (double)_pos_sp_triplet.current.x + cos(convert_start_angle) * (double)_pos_sp_triplet.current.y;*/
			if (yaw_offset > -0.000001f && yaw_offset < 0.000001f) {
				// reversed map
				yaw_offset = _local_pos.yaw;
			}
			if (x_offset > -0.000001f && x_offset < 0.000001f) {
				x_offset = _cur_local_pos.x;
			}
			if (y_offset > -0.000001f && y_offset < 0.000001f) {
				y_offset = _cur_local_pos.y;
			}
			_debug_msg.control_x_offset = x_offset;
			_debug_msg.control_y_offset = y_offset;
			_debug_msg.control_yaw_offset = yaw_offset;

			matrix::Vector2f current_position(((float)_cur_local_pos.x - x_offset) * 1.63f , ((float)_cur_local_pos.y - y_offset) * 1.63f);
			matrix::Vector2f _current_setpoint(_pos_sp_triplet.current.x, _pos_sp_triplet.current.y);
			matrix::Vector2f wheels_control((float)_actuator_outputs.output[0], (float)_actuator_outputs.output[1]);

			observer(current_position, wheels_control);

			// This if statement depends upon short-circuiting: If !manual_mode, then control_position(...)
			// should not be called.
			// It doesn't really matter if it is called, it will just be bad for performance.
			if (!manual_mode && rover_control_position(current_position, _current_setpoint, -(_local_pos.yaw - yaw_offset))) {

				/* XXX check if radius makes sense here */
				float turn_distance = _param_l1_distance.get(); //_gnd_control.switch_distance(100.0f);

				// publish status
				position_controller_status_s pos_ctrl_status = {};

				pos_ctrl_status.nav_roll = 0.0f;
				pos_ctrl_status.nav_pitch = 0.0f;
				pos_ctrl_status.nav_bearing = _gnd_control.nav_bearing();

				pos_ctrl_status.target_bearing = _gnd_control.target_bearing();
				pos_ctrl_status.xtrack_error = _gnd_control.crosstrack_error();

				pos_ctrl_status.wp_dist = get_distance_to_next_waypoint(_global_pos.lat, _global_pos.lon,
							  _pos_sp_triplet.current.lat, _pos_sp_triplet.current.lon);

				pos_ctrl_status.acceptance_radius = turn_distance;
				pos_ctrl_status.yaw_acceptance = NAN;

				pos_ctrl_status.timestamp = hrt_absolute_time();

				if (_pos_ctrl_status_pub != nullptr) {
					orb_publish(ORB_ID(position_controller_status), _pos_ctrl_status_pub, &pos_ctrl_status);

				} else {
					_pos_ctrl_status_pub = orb_advertise(ORB_ID(position_controller_status), &pos_ctrl_status);
				}
			}

			if (_debug_msg_pub != nullptr) {
				orb_publish(ORB_ID(debug_msg), _debug_msg_pub, &_debug_msg);
			} else {
				_debug_msg_pub = orb_advertise(ORB_ID(debug_msg), &_debug_msg);
			}

			perf_end(_loop_perf);
		}

		if (fds[1].revents & POLLIN) {
			perf_begin(_loop_perf);
			if ((int)hrt_absolute_time() > _time + 1000000) {
				PX4_INFO("time per second: %d", iter);
				_time = hrt_absolute_time();
				_debug_msg.tik_frequansy = iter;
				freq = iter;
				iter = 0;
			} else {
				iter += 1;
			}

			// This should be copied even if not in manual mode. Otherwise, the poll(...) call will keep
			// returning immediately and this loop will eat up resources.
			orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sub, &_manual);
			orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);

			orb_copy(ORB_ID(actuator_outputs), _actuator_outputs_sub, &_actuator_outputs);
			orb_copy(ORB_ID(vehicle_gps_position), _gps_pos_sub, &_gps_pos);

			double _lat = (double)_gps_pos.lat / 10000000;
			double _lon = (double)_gps_pos.lon / 10000000;
			double _alt = (double)_gps_pos.lon;

			if (!globallocalconverter_initialized()) {
				globallocalconverter_init(_local_pos.ref_lat, _local_pos.ref_lon,
								_local_pos.ref_alt, _local_pos.ref_timestamp);
			} else {
				globallocalconverter_tolocal(_lat, _lon, _alt, &_cur_local_pos.x, &_cur_local_pos.y, &_cur_local_pos.z);
			}

			matrix::Vector2f current_position((float)_cur_local_pos.x * 1.63f, (float)_cur_local_pos.y * 1.63f);
			matrix::Vector2f _current_setpoint((float)_pos_sp_triplet.current.x, (float)_pos_sp_triplet.current.y);
			matrix::Vector2f wheels_control((float)_actuator_outputs.output[0], (float)_actuator_outputs.output[1]);

			//observer(current_position, wheels_control);

			if (manual_mode) {
				/* manual/direct control */
				//PX4_INFO("Manual mode!");
				_act_controls.control[actuator_controls_s::INDEX_ROLL] = _manual.y;
				_act_controls.control[actuator_controls_s::INDEX_PITCH] = -_manual.x;
				_act_controls.control[actuator_controls_s::INDEX_YAW] = _manual.r; //TODO: Readd yaw scale param
				_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = _manual.z;
			}

			if (_debug_msg_pub != nullptr) {
				orb_publish(ORB_ID(debug_msg), _debug_msg_pub, &_debug_msg);
			} else {
				_debug_msg_pub = orb_advertise(ORB_ID(debug_msg), &_debug_msg);
			}

		}

		if (fds[2].revents & POLLIN) {

			orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub, &_sensor_combined);

			//orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &_vehicle_att);
			_act_controls.timestamp = hrt_absolute_time();

			if (_actuator_controls_pub != nullptr) {
				//PX4_INFO("Publishing actuator from pos control");
				orb_publish(ORB_ID(actuator_controls_0), _actuator_controls_pub, &_act_controls);

			} else {

				_actuator_controls_pub = orb_advertise(ORB_ID(actuator_controls_0), &_act_controls);
			}
		}

	}

	orb_unsubscribe(_control_mode_sub);
	orb_unsubscribe(_global_pos_sub);
	orb_unsubscribe(_local_pos_sub);
	orb_unsubscribe(_manual_control_sub);
	orb_unsubscribe(_pos_sp_triplet_sub);
	orb_unsubscribe(_vehicle_attitude_sub);
	orb_unsubscribe(_sensor_combined_sub);

	warnx("exiting.\n");
}

int RoverPositionControl::task_spawn(int argc, char *argv[])
{
	/* start the task */
	_task_id = px4_task_spawn_cmd("rover_pos_ctrl",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_POSITION_CONTROL,
				      1700,
				      (px4_main_t)&RoverPositionControl::run_trampoline,
				      nullptr);

	if (_task_id < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

RoverPositionControl *RoverPositionControl::instantiate(int argc, char *argv[])
{

	if (argc > 0) {
		PX4_WARN("Command 'start' takes no arguments.");
		return nullptr;
	}

	RoverPositionControl *instance = new RoverPositionControl();

	if (instance == nullptr) {
		PX4_ERR("Failed to instantiate RoverPositionControl object");
	}

	return instance;
}

int RoverPositionControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int RoverPositionControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Controls the position of a ground rover using an L1 controller.

Publishes `actuator_controls_0` messages at a constant 250Hz.

### Implementation
Currently, this implementation supports only a few modes:

 * Full manual: Throttle and yaw controls are passed directly through to the actuators
 * Auto mission: The rover runs missions
 * Loiter: The rover will navigate to within the loiter radius, then stop the motors

### Examples
CLI usage example:
$ rover_pos_control start
$ rover_pos_control status
$ rover_pos_control stop

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rover_pos_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start")
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int rover_pos_control_main(int argc, char *argv[])
{
	return RoverPositionControl::main(argc, argv);
}
