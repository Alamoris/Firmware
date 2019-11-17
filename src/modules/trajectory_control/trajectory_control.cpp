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
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/wind_estimate.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_global_position.h>
#include <parameters/param.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>
#include <lib/ecl/geo/geo.h>
#include <dataman/dataman.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

using matrix::wrap_pi;


extern "C" __EXPORT int trajectory_control_main(int argc, char *argv[]);

class BottleDrop
{
public:
	/**
	 * Constructor
	 */
	BottleDrop();

	/**
	 * Destructor, also kills task.
	 */
	~BottleDrop();

	/**
	 * Start the task.
	 *
	 * @return		OK on success.
	 */
	int		start();

	/**
	 * Display status.
	 */
	void		status();

	void		open_bay();
	void		close_bay();
	void		drop();
	void		lock_release();

private:
	bool		_task_should_exit;		/**< if true, task should exit */
	int		_main_task;			/**< handle for task */
	orb_advert_t	_mavlink_log_pub;

	int		_command_sub;
	int		_wind_estimate_sub;
	struct vehicle_command_s	_command;
	struct vehicle_global_position_s _global_pos;
	map_projection_reference_s ref;

	orb_advert_t	_actuator_pub;
	struct actuator_controls_s _actuators;

	bool		_drop_approval;
	hrt_abstime	_doors_opened;
	hrt_abstime	_drop_time;

	float		_alt_clearance;

	struct position_s {
		double lat;	///< degrees
		double lon;	///< degrees
		float alt;	///< m
	} _target_position, _drop_position;

	enum DROP_STATE {
		DROP_STATE_INIT = 0,
		DROP_STATE_TARGET_VALID,
		DROP_STATE_TARGET_SET,
		DROP_STATE_BAY_OPEN,
		DROP_STATE_DROPPED,
		DROP_STATE_BAY_CLOSED
	} _drop_state;

	struct mission_s	_onboard_mission;
	orb_advert_t		_onboard_mission_pub;

	void		task_main();

	void		handle_command(struct vehicle_command_s *cmd);

	void		answer_command(struct vehicle_command_s *cmd, unsigned result);

	/**
	 * Set the actuators
	 */
	int		actuators_publish();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static int	task_main_trampoline(int argc, char *argv[]);
};

namespace trajectory_control
{
BottleDrop	*g_trajectory_control;
}

BottleDrop::BottleDrop() :

	_task_should_exit(false),
	_main_task(-1),
	_mavlink_log_pub(nullptr),
	_command_sub(-1),
	_wind_estimate_sub(-1),
	_command {},
	_global_pos {},
	ref {},
	_actuator_pub(nullptr),
	_actuators {},
	_drop_approval(false),
	_doors_opened(0),
	_drop_time(0),
	_alt_clearance(70.0f),
	_target_position {},
	_drop_position {},
	_drop_state(DROP_STATE_INIT),
	_onboard_mission {},
	_onboard_mission_pub(nullptr)
{
	_onboard_mission.dataman_id = DM_KEY_WAYPOINTS_ONBOARD;
}

int
BottleDrop::start()
{
	ASSERT(_main_task == -1);

	/* start the task */
	_main_task = px4_task_spawn_cmd("trajectory_control",
					SCHED_DEFAULT,
					SCHED_PRIORITY_DEFAULT + 15,
					1500,
					(px4_main_t)&BottleDrop::task_main_trampoline,
					nullptr);

	if (_main_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}


void
BottleDrop::task_main()
{

	mavlink_log_info(&_mavlink_log_pub, "[trajectory_control] started");

	while (!_task_should_exit) {

	}

	warnx("exiting.");

	_main_task = -1;
	_exit(0);
}

int
BottleDrop::task_main_trampoline(int argc, char *argv[])
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

		trajectory_control::g_trajectory_control = new BottleDrop;

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
		errx(1, "not running");
	}

	if (!strcmp(argv[1], "stop")) {
		delete trajectory_control::g_trajectory_control;
		trajectory_control::g_trajectory_control = nullptr;

	} else if (!strcmp(argv[1], "status")) {
		trajectory_control::g_trajectory_control->status();

	} else if (!strcmp(argv[1], "drop")) {
		trajectory_control::g_trajectory_control->drop();

	} else if (!strcmp(argv[1], "open")) {
		trajectory_control::g_trajectory_control->open_bay();

	} else if (!strcmp(argv[1], "close")) {
		trajectory_control::g_trajectory_control->close_bay();

	} else if (!strcmp(argv[1], "lock")) {
		trajectory_control::g_trajectory_control->lock_release();

	} else {
		usage();
	}

	return 0;
}
