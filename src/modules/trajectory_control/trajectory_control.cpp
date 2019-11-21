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
#include <uORB/topics/position_setpoint_triplet.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>
#include <mathlib/mathlib.h>


extern "C" __EXPORT int trajectory_control_main(int argc, char *argv[]);

#define FILE_PATH "rootfs/fs/microsd/spiral_anim.txt"

class TrajectoryControl
{
public:
	TrajectoryControl();

	int		start();
	
	bool		update_buffer();
private:
	bool		_task_should_exit;		/**< if true, task should exit */
	int		_main_task;			/**< handle for task */
	orb_advert_t	_mavlink_log_pub;



	struct position_setpoint_triplet_s		_pos_sp_triplet;
	struct trajectory_s {
		int numb;
		double x;
		double y;
		double z;
	} current_trajectory_setpoint;

	void		task_main();

	static int	task_main_trampoline(int argc, char *argv[]);
};

namespace trajectory_control
{
TrajectoryControl	*g_trajectory_control;
}

TrajectoryControl::TrajectoryControl() :
	_task_should_exit(false),
	_main_task(-1),
	_mavlink_log_pub(nullptr)
{
	//init = 0.0f;
}

int
TrajectoryControl::start()
{
	ASSERT(_main_task == -1);

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

	// trajectory_s sp_buffer [20];
	// bool update_buffer = True;
	FILE *trajectory_file = fopen(FILE_PATH, "r");

	int numb;
	float x, y, z;
	
	while (!_task_should_exit) {
		if(feof(trajectory_file))
		{
			warnx("Trajectory final");

			_main_task = -1;
			fclose(trajectory_file);
			_exit(0);
		}

		if(fscanf(trajectory_file, "%d %f %f %f", &numb, &x, &y, &z) == 4)
		{
			current_trajectory_setpoint.num = numb;
			current_trajectory_setpoint.x = x;
			current_trajectory_setpoint.y = y;
			current_trajectory_setpoint.z = z;
			PX4_INFO("num: %d, x: %f, y: %f, z: %f", (int)numb, (double)x, (double)y, (double)z);
		}
	}

	warnx("exiting.");


	fclose(trajectory_file);
	_main_task = -1;
	_exit(0);
}

/*
void
TrajectoryControl::update_buffer()
{
	
}*/

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
