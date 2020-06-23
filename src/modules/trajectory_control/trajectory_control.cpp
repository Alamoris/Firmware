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


#include <px4_config.h>
#include <px4_tasks.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <time.h>
#include <sys/ioctl.h>
#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>

#include <px4_getopt.h>

#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationQueued.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/position_controller_status.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/position_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/trajectory_point.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>
#include <mathlib/mathlib.h>
#include <platforms/px4_module.h>


extern "C" __EXPORT int trajectory_control_main(int argc, char *argv[]);

//#define FILE_PATH "spiral_anim_v.txt"

class TrajectoryControl : public ModuleBase<TrajectoryControl>
{
public:
	TrajectoryControl();

	int		start();

	bool		publish_next_point();

	char		file_name[51];

	int	vehicle_type;

private:
	bool		_task_should_exit;		/**< if true, task should exit */
	int		_main_task;			/**< handle for task */

	orb_advert_t	_mavlink_log_pub;

	uORB::Publication<position_setpoint_triplet_s>		position_setpoint_triplet_pub{ORB_ID(position_setpoint_triplet)};
	uORB::Publication<offboard_control_mode_s>		offboard_control_mode_pub{ORB_ID(offboard_control_mode)};

	uORB::PublicationQueued<vehicle_command_s> 		_vehicle_command_pub{ORB_ID(vehicle_command)};

	uORB::Subscription	vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription	_vehicle_local_pos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription	local_position_setpoint_sub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::Subscription	_position_controller_status_sub{ORB_ID(position_controller_status)};

	struct	offboard_control_mode_s			offboard_control;

	struct	vehicle_local_position_setpoint_s	local_position_setpoint;
	struct	vehicle_status_s			_status;
	struct	vehicle_local_position_s		_vehicle_local_pos;
	struct	position_controller_status_s		_controller_status;

	struct trajectory_point_s			_logger_point;

	uORB::Publication<trajectory_point_s>		trajectory_point_pub{ORB_ID(trajectory_point)};

	void		task_main();
	void		offboard_and_arm();
	void		publish_offboard_control();
	void		set_offboard_mode();
	void		arm_copter();
	void		publish_sp_triplet(position_setpoint_triplet_s _pos_sp);

	static int	task_main_trampoline(int argc, char *argv[]);
};

namespace trajectory_control
{
TrajectoryControl	*g_trajectory_control;
}

TrajectoryControl::TrajectoryControl() :
	file_name{"rover_circle.txt"},
	//file_name{"/fs/microsd/trajectory/rover_min_square.txt"},
	vehicle_type(0),
	_task_should_exit(false),
	_main_task(-1),
	_mavlink_log_pub(nullptr),
	offboard_control{},
	local_position_setpoint{},
	_status{},
	_vehicle_local_pos{},
	_controller_status{},
	_logger_point{}
{
}

int
TrajectoryControl::start()
{
	/* start the task */
	_main_task = px4_task_spawn_cmd("trajectory_control",
					SCHED_DEFAULT,
					SCHED_PRIORITY_DEFAULT + 15,
					4000,
					(px4_main_t)&TrajectoryControl::task_main_trampoline,
					nullptr);

	if (_main_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}


void
TrajectoryControl::task_main()
{
	mavlink_log_info(&_mavlink_log_pub, "[trajectory_control] started");
	PX4_INFO("Start trajectory module");

	FILE *trajectory_file = fopen(file_name, "r");
	if (!trajectory_file) {
		PX4_ERR("Couldn't jpen file, shutdown");
		PX4_INFO("File name: %s", file_name);
		_task_should_exit = true;
	}

	int send_arm_message = 1;
	int send_offboard_message = 1;

	//float vel_p_pid_x = 1.8;
	//float vel_p_pid_y = 1.8; /*x, y. Params for speed < 1.5m/s*/

	float vel_p_pid_x = 2;
	float vel_p_pid_y = 2;

	bool takeoff = 1;
	float takeoff_z = 1.0;
	float z_threshold = 0.2;
	float up_time = 5000000;
	float current_z_sp = 0;

	int time_delta_control = 70000; /*us. Params for speed < 1.5m/s*/
	int pid_time = 4000;
	int circle_buffer_size = ((time_delta_control / 5) / pid_time) + 1;
	float circle_buffer[circle_buffer_size][2] = {{0.0}, {0.0}};
	int circle_index = 0;

	int setpoint_buffer_size = 5;
	position_setpoint_triplet_s setpoint_buffer[setpoint_buffer_size] = {0};
	int setpoint_read_index = 0;
	int setpoint_write_index = 0;

	int publish_zeros = 1;

	char read_str[50];
	float x, y, z, vx, vy, vz;
	offboard_control.ignore_thrust = 0;
	offboard_control.ignore_attitude = 0;
	offboard_control.ignore_bodyrate_x = 1;
	offboard_control.ignore_bodyrate_y = 1;
	offboard_control.ignore_bodyrate_z = 1;
	offboard_control.ignore_position = 1;
	offboard_control.ignore_velocity = 0;
	offboard_control.ignore_acceleration_force = 1;
	offboard_control.ignore_alt_hold = 0;

	int start_time = hrt_absolute_time();
	int resend = hrt_absolute_time();

	int break_timer = 0;
	int break_timeout = 5000000;
	int break_prev_time = hrt_absolute_time();

	int prev_time = 0;
	int cur_time = 0;

	position_setpoint_triplet_s zero_sp{};
	/*Initialize setpoint*/
	zero_sp.current.vx = 0;
	zero_sp.current.vy = 0;
	zero_sp.current.z = 0;
	zero_sp.current.type = position_setpoint_s::SETPOINT_TYPE_IDLE;
	publish_sp_triplet(zero_sp);

	while (!_task_should_exit) {
		if (feof(trajectory_file)) {
			warnx("exiting by feof.");

			_task_should_exit = true;
		}

		if ((setpoint_write_index + 1) % setpoint_buffer_size != setpoint_read_index) {
			position_setpoint_triplet_s my_sp{};
			if (fgets(read_str, 50, trajectory_file)) {
				if (sscanf(read_str, "%f %f %f %f %f %f", &x, &y, &z, &vx, &vy, &vz) == 6) {
					//PX4_INFO("read -- x:%f, vx: %f, vy: %f, vz: %f", (double)x, (double)vx, (double)vy, (double)vz);
					_vehicle_local_pos_sub.copy(&_vehicle_local_pos);

					circle_index = (circle_index + 1) % circle_buffer_size;
					my_sp.current.x = x;
					my_sp.current.y = y;
					my_sp.current.z = z - takeoff_z;
					my_sp.current.vx = vx + vel_p_pid_x * circle_buffer[circle_index][0];
					my_sp.current.vy = vy + vel_p_pid_y * circle_buffer[circle_index][1];

					my_sp.current.cruising_speed = vx;

					if (setpoint_write_index - 1 < 0) {
						my_sp.previous.x = setpoint_buffer[setpoint_buffer_size - 1].current.x;
						my_sp.previous.y = setpoint_buffer[setpoint_buffer_size - 1].current.y;
						my_sp.previous.z = setpoint_buffer[setpoint_buffer_size - 1].current.z;
					} else {
						my_sp.previous.x = setpoint_buffer[setpoint_write_index - 1].current.x;
						my_sp.previous.y = setpoint_buffer[setpoint_write_index - 1].current.y;
						my_sp.previous.z = setpoint_buffer[setpoint_write_index - 1].current.z;
					}

					circle_buffer[circle_index][0] = x - _vehicle_local_pos.x;
					circle_buffer[circle_index][1] = y - _vehicle_local_pos.y;

					//Publish debug trajectory point
					_logger_point.timestamp = hrt_absolute_time();
					_logger_point.read_x = x;
					_logger_point.read_y = y;
					_logger_point.read_z = z;
					_logger_point.read_vx = vx;
					_logger_point.read_vy = vy;

					_logger_point.x_err = x - _vehicle_local_pos.x;
					_logger_point.y_err = y - _vehicle_local_pos.y;
					trajectory_point_pub.publish(_logger_point);
				}
			}
			setpoint_buffer[setpoint_write_index] = my_sp;

			//PX4_INFO("Prev setpoint write index: %d, read: %d", (int)setpoint_write_index, (int)setpoint_read_index);
			setpoint_write_index = (setpoint_write_index + 1) % setpoint_buffer_size;
			continue;
		}

		if (vehicle_status_sub.updated()) {
			vehicle_status_sub.copy(&_status);
		}

		//PX4_WARN("arm:%d, state:%d, time:%d", (int)_status.arming_state, (int)_status.nav_state, (int)_status.timestamp);
		publish_offboard_control();

		if (send_offboard_message) {
			if (resend + 500000 < (int)hrt_absolute_time()) {
				PX4_INFO("Send offboard command");
				resend = hrt_absolute_time();
				set_offboard_mode();
			}

			if (_status.nav_state != vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
				PX4_WARN("Waiting for offboard, state:%d", (int)_status.nav_state);
				continue;
			} else {
				send_offboard_message = 0;
				resend = hrt_absolute_time();
			}
		}

		if (send_arm_message) {
			if (resend + 500000 < (int)hrt_absolute_time()) {
				PX4_INFO("Send arm command");
				//resend = hrt_absolute_time();
				arm_copter();
			}

			if (_status.arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
				PX4_WARN("Waiting for system activation. arm:%d", (int)_status.arming_state);
				start_time = hrt_absolute_time();

				continue;
			}
			send_arm_message = 0;

		}

		if (publish_zeros) {
			position_setpoint_triplet_s my_sp{};
			my_sp.current.x = 0;
			my_sp.current.y = 0;
			my_sp.current.z = 0;
			my_sp.current.vx = 0;
			my_sp.current.vy = 0;
			my_sp.current.type = position_setpoint_s::SETPOINT_TYPE_IDLE;
			publish_sp_triplet(my_sp);
			usleep(3);
		}


		if (local_position_setpoint_sub.updated() || _position_controller_status_sub.updated()) {
			publish_zeros = 0;
			break_timer = 0;
			break_prev_time = 0;

			position_setpoint_triplet_s my_sp{};

			if (vehicle_type == 0) {
				local_position_setpoint_sub.copy(&local_position_setpoint);
				cur_time = local_position_setpoint.timestamp;

			} else if (vehicle_type == 1) {
				_position_controller_status_sub.copy(&_controller_status);
				cur_time = _controller_status.timestamp;
			}

			if (prev_time != cur_time) {

				prev_time = cur_time;

				// Copter takeoff
				_vehicle_local_pos_sub.copy(&_vehicle_local_pos);
				if (takeoff == 1 && vehicle_type == 0) {
					if ((_vehicle_local_pos.z > -takeoff_z + z_threshold) || (_vehicle_local_pos.z < -takeoff_z - z_threshold)) {
						PX4_INFO("Try takeoff");

						if (_vehicle_local_pos.timestamp < start_time + up_time) {
							current_z_sp = -((takeoff_z / up_time) * (hrt_absolute_time() - start_time));
						} else {
							current_z_sp = -takeoff_z;
						}

						my_sp.current.vx = 0;
						my_sp.current.vy = 0;
						my_sp.current.z = current_z_sp;
						publish_sp_triplet(my_sp);
					} else {
						takeoff = 0;
					}
					continue;
				}

				PX4_INFO("Publish setpoint");
				publish_sp_triplet(setpoint_buffer[setpoint_read_index]);
				//PX4_INFO("Publish setpoint. timestamp:%d", (int)hrt_absolute_time());

				setpoint_read_index = (setpoint_read_index + 1) % setpoint_buffer_size;
			}
		} else {
			if (break_timer > break_timeout) {
				PX4_WARN("No local position, exit");
				_task_should_exit = true;
			} else {
				//PX4_INFO("Waiting local position setpoint");
				if (break_prev_time == 0) {
					break_prev_time = hrt_absolute_time();
				}
				break_timer += (int)hrt_absolute_time() - break_prev_time;
				break_prev_time = hrt_absolute_time();
			}
		}
	}

	warnx("exiting.");


	fclose(trajectory_file);
	_main_task = -1;
}

/*More info in commander.cpp 3615-3653*/
void
TrajectoryControl::publish_offboard_control()
{
	offboard_control.timestamp = hrt_absolute_time();

	offboard_control_mode_pub.publish(offboard_control);
}

void
TrajectoryControl::publish_sp_triplet(position_setpoint_triplet_s _pos_sp)
{
	if (_pos_sp.current.type != position_setpoint_s::SETPOINT_TYPE_IDLE) {
		_pos_sp.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
	}
	_pos_sp.timestamp = hrt_absolute_time();
	_pos_sp.current.timestamp = hrt_absolute_time();
	_pos_sp.current.velocity_frame = position_setpoint_s::VELOCITY_FRAME_LOCAL_NED;
	_pos_sp.current.valid = true;
	_pos_sp.next.valid = false;
	_pos_sp.previous.valid = false;

	_pos_sp.current.position_valid = false;
	_pos_sp.current.alt_valid = true;
	_pos_sp.current.velocity_valid = true;
	_pos_sp.current.acceleration_valid = false;
	_pos_sp.current.yaw_valid = false;
	_pos_sp.current.yawspeed_valid = false;

	position_setpoint_triplet_pub.publish(_pos_sp);
}

void
TrajectoryControl::set_offboard_mode()
{
	struct vehicle_command_s _command{
		.timestamp = hrt_absolute_time(),
		.param5 = 0.0f,
		.param6 = 0.0f,
		.param1 = 1, /*basemode pass*/
		.param2 = 6, /*publish offboard command*/
		.param3 = 0.0f,
		.param4 = 0.0f,
		.param7 = 0.0f,
		.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE,
		.target_system = 1,
		.target_component = 0,
		.source_system = _status.system_id,
		.source_component = _status.component_id,
		.confirmation = true,
		.from_external = false
	};

	_vehicle_command_pub.publish(_command);
}


void
TrajectoryControl::arm_copter()
{
	struct vehicle_command_s _command{
		.timestamp = hrt_absolute_time(),
		.param5 = 0.0f,
		.param6 = 0.0f,
		.param1 = 1,
		.param2 = 0.0f,
		.param3 = 0.0f,
		.param4 = 0.0f,
		.param7 = 0.0f,
		.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM,
		.target_system = _status.system_id,
		.target_component = _status.component_id,
		.source_system = _status.system_id,
		.source_component = _status.component_id,
		.confirmation = true,
		.from_external = false
	};

	_vehicle_command_pub.publish(_command);
}

int
TrajectoryControl::task_main_trampoline(int argc, char *argv[])
{
	trajectory_control::g_trajectory_control->task_main();
	return 0;
}

void
check_file(const char *path)
{
	PX4_INFO("Check file for read");
	struct position_setpoint_triplet_s	next_pos_sp;
	char file_path[50];
	char str[50];
	float x, y, z, vx, vy, vz;

	int time_delta_control = 70000; /*us. Params for speed < 1.5m/s*/
	int pid_time = 4000;
	int circle_buffer_size = ((time_delta_control / 5) / pid_time) + 1;
	float circle_buffer[circle_buffer_size][2] = {{0.0}, {0.0}};
	int circle_index = 0;

	int setpoint_buffer_size = 5;
	position_setpoint_triplet_s setpoint_buffer[setpoint_buffer_size];
	int setpoint_read_index = 0;
	int setpoint_write_index = 0;

	strncpy(file_path, path, sizeof(file_path));
	file_path[sizeof(file_path) - 1] = '\0';
	PX4_INFO("Try open file");
	FILE *trajectory_file = fopen(file_path, "r");

	if (trajectory_file) {
		PX4_INFO("File opened, start reading");
		PX4_INFO("Read 5 mesages from file");
		for (int i = 0; i < 10; i++) {
			if (fgets(str, 50, trajectory_file)) {
				if (sscanf(str, "%f %f %f %f %f %f", &x, &y, &z, &vx, &vy, &vz) == 6) {
					PX4_INFO("Read message- x:%f, y:%f, z:%f, vx:%f, vy:%f, vz:%f", (double)x, (double)y, (double)z, (double)vx, (double)vy, (double)vz);
					circle_index = (circle_index + 1) % circle_buffer_size;

					next_pos_sp.current.x = x;
					next_pos_sp.current.y = y;
					next_pos_sp.current.z = z - 1;
					next_pos_sp.current.vx = vx + 1 * circle_buffer[circle_index][0];
					next_pos_sp.current.vy = vy + 1 * circle_buffer[circle_index][1];
					circle_buffer[circle_index][0] = x;
					circle_buffer[circle_index][1] = y;
					//PX4_INFO("read -- vx: %f, vy: %f, vz: %f", (double)next_pos_sp.current.vx, (double)next_pos_sp.current.vy, (double)next_pos_sp.current.vz);
				}
			}

			PX4_INFO("Prev setpoint write index: %d, read: %d", (int)setpoint_write_index, (int)setpoint_read_index);
			setpoint_write_index = (setpoint_write_index + 1) % setpoint_buffer_size;

			setpoint_buffer[setpoint_write_index] = next_pos_sp;
			//PX4_INFO("Publish setpoint. timestamp:%d", (int)hrt_absolute_time());

			setpoint_read_index = (setpoint_read_index + 1) % setpoint_buffer_size;
		}
	} else {
		PX4_INFO("Error trying to open file");
	}
}

void check_local_position()
{
	struct vehicle_local_position_s vehicle_local_position;
	uORB::Subscription	_vehicle_local_pos_sub{ORB_ID(vehicle_local_position)};
	_vehicle_local_pos_sub.copy(&vehicle_local_position);
	PX4_INFO("Local position. x:%f, y:%f, z:%f, vx:%f, vy:%f, vz:%f", (double)vehicle_local_position.x, (double)vehicle_local_position.y, (double)vehicle_local_position.z, (double)vehicle_local_position.vx, (double)vehicle_local_position.vy, (double)vehicle_local_position.vz);
}

int trajectory_control_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	if (!strcmp(argv[1], "start")) {

		if (trajectory_control::g_trajectory_control != nullptr) {
			return 0;
		}

		trajectory_control::g_trajectory_control = new TrajectoryControl;
		while ((ch = px4_getopt(argc, argv, "f:v:", &myoptind, &myoptarg)) != EOF) {
			switch (ch) {
			case 'f':
				strncpy(trajectory_control::g_trajectory_control->file_name, myoptarg, 50);
				/* enforce null termination */
				trajectory_control::g_trajectory_control->file_name[50] = '\0';
				break;

			case 'v':
				// TODO: change vehicle_type to enum

				if (myoptarg[0] == 'r') {
					// Check rover vehicle type
					trajectory_control::g_trajectory_control->vehicle_type = 1;
					PX4_INFO("Enable rover vehicle type");
				} else {
					// Check copter vehicle type
					trajectory_control::g_trajectory_control->vehicle_type = 0;
					PX4_INFO("Enable copter vehicle type");
				}

				break;
			}
		}

		if (trajectory_control::g_trajectory_control == nullptr) {
			return 0;
		}

		if (OK != trajectory_control::g_trajectory_control->start()) {
			delete trajectory_control::g_trajectory_control;
			trajectory_control::g_trajectory_control = nullptr;
			return 0;
		}

		return 0;
	}

	if (trajectory_control::g_trajectory_control == nullptr) {
		PX4_INFO("Module not running");
	}

	if (!strcmp(argv[1], "stop")) {
		delete trajectory_control::g_trajectory_control;
		trajectory_control::g_trajectory_control = nullptr;

	} else if (!strcmp(argv[1], "help")) {
		PX4_INFO("Enter 'start' to enable trajectory control module");

	} else if (!strcmp(argv[1], "check")) {
		check_local_position();
	} else if (!strcmp(argv[1], "try")) {
		while ((ch = px4_getopt(argc, argv, "f:", &myoptind, &myoptarg)) != EOF) {
			switch (ch) {
			case 'f':
				check_file(myoptarg);
				break;
			}
		}
	} else {
		PX4_ERR("usage: trajectory_control {start|stop|status|help}");
		return 0;
	}
	return 0;
}
