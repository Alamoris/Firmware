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
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <sys/ioctl.h>
#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_status.h>
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


extern "C" __EXPORT int trajectory_control_main(int argc, char *argv[]);

#define FILE_PATH "tmp/spiral_anim_v.txt"

class TrajectoryControl
{
public:
	TrajectoryControl();

	int		start();

	bool		publish_next_point();
private:
	bool		_task_should_exit;		/**< if true, task should exit */
	int		_main_task;			/**< handle for task */

	orb_advert_t	_mavlink_log_pub;

	int		position_setpoint_sub;
	orb_advert_t	next_point_pub;

	struct	trajectory_point_s		next_trajectory_setpoint;
	int		trajectory_point_sub;

	orb_advert_t	_vehicle_command_pub;
	int		_vehicle_command_sub;
	int		vehicle_status_sub;
	struct	vehicle_status_s		_status;

	orb_advert_t offboard_control_mode_pub;
	struct	offboard_control_mode_s		offboard_control;

	orb_advert_t position_setpoint_triplet_pub;
	struct position_setpoint_triplet_s	next_pos_sp;

	int		_vehicle_local_pos_sub;
	struct	vehicle_local_position_s	_vehicle_local_pos;


	//int task_start_time;

	void		task_main();
	void		offboard_and_arm();
	void		publish_offboard_control();
	void		set_offboard_mode();
	void		arm_copter();
	void		publish_sp_triplet(position_setpoint_triplet_s _pos_sp);

	bool		flight_start;

	static int	task_main_trampoline(int argc, char *argv[]);
};

namespace trajectory_control
{
TrajectoryControl	*g_trajectory_control;
}

TrajectoryControl::TrajectoryControl() :
	_task_should_exit(false),
	_main_task(-1),
	_mavlink_log_pub(nullptr),
	position_setpoint_sub(-1),
	next_point_pub(nullptr),
	next_trajectory_setpoint{},
	trajectory_point_sub(-1),
	_vehicle_command_pub(nullptr),
	_vehicle_command_sub(-1),
	vehicle_status_sub(-1),
	_status{},
	offboard_control_mode_pub(nullptr),
	offboard_control{},
	position_setpoint_triplet_pub(nullptr),
	next_pos_sp{},
	_vehicle_local_pos_sub(-1),
	_vehicle_local_pos{}
	//task_start_time(hrt_absolute_time())
{
	flight_start = false;
}

int
TrajectoryControl::start()
{
	/* start the task */
	_main_task = px4_task_spawn_cmd("trajectory_control",
					SCHED_DEFAULT,
					SCHED_PRIORITY_DEFAULT + 15,
					1500,
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

	//bool updated;

	position_setpoint_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	trajectory_point_sub = orb_subscribe(ORB_ID(trajectory_point));
	_vehicle_command_sub = orb_subscribe(ORB_ID(vehicle_command));
	vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_vehicle_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));

	FILE *trajectory_file = fopen(FILE_PATH, "r");

	float vel_p_pid[2] = {1.8, 1.8}; /*x, y. Params for speed < 1.5m/s*/

	bool takeoff = 1;
	float takeoff_z = 1.5;
	float z_threshold = 0.2;
	float up_time = 5000000; /*us*/
	float current_z_sp = 0;

	int time_delta_control = 70000; /*us. Params for speed < 1.5m/s*/
	int pid_time = 4000;
	int circle_buffer_size = ((time_delta_control / 5) / pid_time) + 1;
	float circle_buffer[circle_buffer_size][2] = {{},{}};
	int circle_index = 0;

	int setpoint_buffer_size = 5;
	position_setpoint_triplet_s setpoint_buffer[setpoint_buffer_size];
	int setpoint_read_index = 0;
	int setpoint_write_index = 0;


	char str[50];
	float x, y, z, vx, vy, vz;
	offboard_control.ignore_thrust = 0;
	offboard_control.ignore_attitude = 0;
	offboard_control.ignore_bodyrate_x = 1;
	offboard_control.ignore_bodyrate_y = 1;
	offboard_control.ignore_bodyrate_z = 1;
	offboard_control.ignore_position = 0;
	offboard_control.ignore_velocity = 0;
	offboard_control.ignore_acceleration_force = 1;
	offboard_control.ignore_alt_hold = 0;

	/*next_trajectory_setpoint.timestamp = hrt_absolute_time();
	next_trajectory_setpoint.valid = true;


	if (next_point_pub != nullptr) {
		orb_publish(ORB_ID(trajectory_point), next_point_pub, &next_trajectory_setpoint);
	} else {
		next_point_pub = orb_advertise(ORB_ID(trajectory_point), &next_trajectory_setpoint);
	}*/

	int start_time = hrt_absolute_time();

	orb_copy(ORB_ID(trajectory_point) ,trajectory_point_sub, &next_trajectory_setpoint);
	int prev_time;
	prev_time = next_trajectory_setpoint.time;

	while (!_task_should_exit) {
		PX4_INFO("Check feof");
		if (feof(trajectory_file)) {
			warnx("exiting by feof.");

			_task_should_exit = true;
		}

		if ((setpoint_write_index + 1) % setpoint_buffer_size != setpoint_read_index) {
			PX4_INFO("Try read");
			if (fgets(str, 50, trajectory_file)) {
				if (sscanf(str, "%f %f %f %f %f %f", &x, &y, &z, &vx, &vy, &vz) == 6) {
					circle_index = (circle_index + 1) % circle_buffer_size;
					next_pos_sp.current.x = x;
					next_pos_sp.current.y = y;
					next_pos_sp.current.z = z - takeoff_z;
					next_pos_sp.current.vx = vx + vel_p_pid[0] * circle_buffer[circle_index][0];
					next_pos_sp.current.vy = vy + vel_p_pid[1] * circle_buffer[circle_index][1];

					circle_buffer[circle_index][0] = x - _vehicle_local_pos.x;
					circle_buffer[circle_index][1] = y - _vehicle_local_pos.y;
					//PX4_INFO("read -- vx: %f, vy: %f, vz: %f", (double)next_pos_sp.current.vx, (double)next_pos_sp.current.vy, (double)next_pos_sp.current.vz);
				}
			}
			setpoint_buffer[setpoint_write_index] = next_pos_sp;

			PX4_INFO("Prev setpoint write index: %d, read: %d", (int)setpoint_write_index, (int)setpoint_read_index);
			setpoint_write_index = (setpoint_write_index + 1) % setpoint_buffer_size;
			continue;
		}

		publish_offboard_control();

		orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &_status);

		/* Armed and switch to offbooard mode */
		if (_status.nav_state != vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
			PX4_INFO("Copter in mode: %d. Try set OFFBOARD", (int)_status.nav_state);
			set_offboard_mode();
			continue;
		}

		if (_status.arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
			/* TODO: publish vehicle command to arm copter */
			PX4_INFO("copter not armed, state mode: %d", (int)_status.arming_state);
			arm_copter();
			start_time = hrt_absolute_time();
			continue;
		}

		orb_copy(ORB_ID(trajectory_point), trajectory_point_sub, &next_trajectory_setpoint);
		int cur_time = next_trajectory_setpoint.time;

		//PX4_INFO("old_time-%d, current_time-%d", (int)prev_time, (int)next_trajectory_setpoint.time);
		if (prev_time != cur_time) {
			//PX4_INFO("Read next string with new_loc_pos take");

			prev_time = next_trajectory_setpoint.time;

			// Copter takeoff
			orb_copy(ORB_ID(vehicle_local_position), _vehicle_local_pos_sub, &_vehicle_local_pos);
			if (takeoff == 1) {
				if ((_vehicle_local_pos.z > -takeoff_z + z_threshold) || (_vehicle_local_pos.z < -takeoff_z - z_threshold)) {
					PX4_INFO("Try takeoff");

					if (_vehicle_local_pos.timestamp < start_time + up_time) {
						current_z_sp = -((takeoff_z / up_time) * (hrt_absolute_time() - start_time));
					} else {
						current_z_sp = -takeoff_z;
					}

					next_pos_sp.current.vx = 0;
					next_pos_sp.current.vy = 0;
					next_pos_sp.current.z = current_z_sp;
					publish_sp_triplet(next_pos_sp);
				} else {
					takeoff = 0;
				}
				continue;
			}

			publish_sp_triplet(setpoint_buffer[setpoint_read_index]);
			PX4_INFO("Publish setpoint. timestamp:%d", (int)hrt_absolute_time());

			setpoint_read_index = (setpoint_read_index + 1) % setpoint_buffer_size;

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

	if (offboard_control_mode_pub != nullptr) {
		orb_publish(ORB_ID(offboard_control_mode), offboard_control_mode_pub, &offboard_control);
	} else {
		offboard_control_mode_pub = orb_advertise(ORB_ID(offboard_control_mode), &offboard_control);
	}
}

void
TrajectoryControl::publish_sp_triplet(position_setpoint_triplet_s _pos_sp)
{
	_pos_sp.current.timestamp = hrt_absolute_time();
	_pos_sp.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
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

	if (position_setpoint_triplet_pub != nullptr) {
		orb_publish(ORB_ID(position_setpoint_triplet), position_setpoint_triplet_pub, &_pos_sp);
	} else {
		position_setpoint_triplet_pub = orb_advertise(ORB_ID(position_setpoint_triplet), &_pos_sp);
	}
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
		.target_system = _status.system_id,
		.target_component = 0,
		.source_system = _status.system_id,
		.source_component = _status.component_id,
		.confirmation = false,
		.from_external = false
	};
	if (_vehicle_command_pub != nullptr) {
		/* to arm watch commander.cpp 784+ and px4io.cpp 736*/
		orb_publish(ORB_ID(vehicle_command), _vehicle_command_pub, &_command);
	} else {
		_vehicle_command_pub = orb_advertise(ORB_ID(vehicle_command), &_command);
	}
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
		.confirmation = false,
		.from_external = false
	};

	if (_vehicle_command_pub != nullptr) {
		/* to arm watch commander.cpp 784+ and px4io.cpp 736*/
		orb_publish(ORB_ID(vehicle_command), _vehicle_command_pub, &_command);
	} else {
		_vehicle_command_pub = orb_advertise(ORB_ID(vehicle_command), &_command);
	}
}

int
TrajectoryControl::task_main_trampoline(int argc, char *argv[])
{
	trajectory_control::g_trajectory_control->task_main();
	return 0;
}

static void usage()
{
	errx(1, "usage: trajectory_control {start|stop|status}");
}

int trajectory_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
	}

	if (!strcmp(argv[1], "start")) {

		if (trajectory_control::g_trajectory_control != nullptr) {
			errx(1, "already running");
		}

		trajectory_control::g_trajectory_control = new TrajectoryControl;

		if (trajectory_control::g_trajectory_control == nullptr) {
			errx(1, "alloc failed");
		}

		if (OK != trajectory_control::g_trajectory_control->start()) {
			delete trajectory_control::g_trajectory_control;
			trajectory_control::g_trajectory_control = nullptr;
			err(1, "start failed");
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

	} /*else if (!strcmp(argv[1], "status")) {
		trajectory_control::g_trajectory_control->status();

	} */else {
		usage();
	}

	return 0;
}
