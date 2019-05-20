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
 * @file precland.cpp
 *
 * Helper class to do precision landing with a landing target
 *
 * @author Nicolas de Palezieux (Sunflower Labs) <ndepal@gmail.com>
 */

#include "precland.h"
#include "navigator.h"

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <fcntl.h>

#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>

#include <uORB/uORB.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>

#define SEC2USEC 1000000.0f

#define PLD_STATUS_MAVLINK(_level, _text, ...)	if (_param_info.get()) mavlink_vasprintf(_level, _navigator->get_mavlink_log_pub(), _text, ##__VA_ARGS__);

PrecLand::PrecLand(Navigator *navigator) :
	MissionBlock(navigator),
	ModuleParams(navigator)
{
}

void
PrecLand::on_activation()
{
	// We need to subscribe here and not in the constructor because constructor is called before the navigator task is spawned
	if (_target_pose_sub < 0) {
		_target_pose_sub = orb_subscribe(ORB_ID(landing_target_pose));
	}

	_state = PrecLandState::Start;
	_search_cnt = 0;
	_last_slewrate_time = 0;

	// Strict precland is enabled
	if (_param_strict.get())
	{
		// Strict precland in funnel enabled
		if (_param_funnel_top_rad.get() > 0)
		{
			// Check if the funnel geometry is correct
			if (_param_funnel_top_rad.get() > _param_hacc_rad.get() && 
				_param_funnel_le_alt.get() >= _param_final_approach_alt.get() &&
				_param_funnel_le_alt.get() < _param_search_alt.get())
			{
				/* Calculate funnel linear part slope from two points
				* 	k = (r_t - r_h) / (a_s - a_b), where:
				*		r_t - a top funnel linear part radius,
				*		r_h - a bottom funnel linear part radius (a horizontal acceptance radius)
				*		a_s - a top funnel height (a search altitude),
				*		a_b - a bottom funnel linear part height. 
				*/
				_strict_funnel_k = (_param_funnel_top_rad.get() - _param_hacc_rad.get()) /
					(_param_search_alt.get() - _param_funnel_le_alt.get());

				/* Calculate funnel linear part offset from the top point
				* 	r_o = r_t + k * a_s, where:
				*		r_t - a top funnel radius,
				*		k - a funnel linear part slope,
				*		a_s - a top funnel height (a search altitude)
				*/
				_strict_funnel_r_o = _param_funnel_top_rad.get() - _strict_funnel_k * _param_search_alt.get();

				PX4_INFO("Strict precland in a funnel");
				PLD_STATUS_MAVLINK(_MSG_PRIO_WARNING, "PLD: Strict precland: FUNNEL");
				// PX4_INFO("r(a) = a * %f + %f", (double)_strict_funnel_k, (double)_strict_funnel_r_o);
			}
			// Funnel parameters integrity check has failed
			else
			{
				// Switch to the cylinder mode by setting linear funnel part slope to zero
				_strict_funnel_k = 0;

				PX4_ERR("Invalid strict precland funnel parameters");
				PLD_STATUS_MAVLINK(_MSG_PRIO_WARNING, "PLD: Invalid strict precland funnel parameters");
			}
		}
		else
		{
			// Switch to the cylinder mode by setting linear funnel part slope to zero
			_strict_funnel_k = 0;

			PX4_INFO("Strict precland in a cylinder");
			PLD_STATUS_MAVLINK(_MSG_PRIO_WARNING, "PLD: Strict precland: CYLINDER");
		}

		_strict_stop = false;
	}
	// Strict precland is disabled
	else
	{
		PX4_INFO("Regular precland");
		PLD_STATUS_MAVLINK(_MSG_PRIO_WARNING, "PLD: Regular precland");
	}

	vehicle_local_position_s *vehicle_local_position = _navigator->get_local_position();

	if (!map_projection_initialized(&_map_ref)) {
		map_projection_init(&_map_ref, vehicle_local_position->ref_lat, vehicle_local_position->ref_lon);
	}

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	pos_sp_triplet->next.valid = false;

	// Check that the current position setpoint is valid, otherwise land at current position
	if (!pos_sp_triplet->current.valid) {
		PX4_WARN("Resetting landing position to current position");
		PLD_STATUS_MAVLINK(_MSG_PRIO_WARNING, "PLD: Resetting landing position to current");
		pos_sp_triplet->current.lat = _navigator->get_global_position()->lat;
		pos_sp_triplet->current.lon = _navigator->get_global_position()->lon;
		pos_sp_triplet->current.alt = _navigator->get_global_position()->alt;
		pos_sp_triplet->current.valid = true;
	}

	_sp_pev = matrix::Vector2f(0, 0);
	_sp_pev_prev = matrix::Vector2f(0, 0);
	_last_slewrate_time = 0;

	switch_to_state_start();

}

void
PrecLand::on_active()
{
	// get new target measurement
	orb_check(_target_pose_sub, &_target_pose_updated);

	if (_target_pose_updated) {
		orb_copy(ORB_ID(landing_target_pose), _target_pose_sub, &_target_pose);
		_target_pose_valid = true;
	}

	if ((hrt_elapsed_time(&_target_pose.timestamp) / 1e6f) > _param_timeout.get()) {
		_target_pose_valid = false;
	}

	// stop if we are landed
	if (_navigator->get_land_detected()->landed) {
		switch_to_state_done();
	}

	switch (_state) {
	case PrecLandState::Start:
		run_state_start();
		break;

	case PrecLandState::HorizontalApproach:
		run_state_horizontal_approach();
		break;

	case PrecLandState::DescendAboveTarget:
		run_state_descend_above_target();
		break;

	case PrecLandState::FinalApproach:
		run_state_final_approach();
		break;

	case PrecLandState::Search:
		run_state_search();
		break;

	case PrecLandState::ActiveSearchReset:
		run_state_asearch_reset();
		break;

	case PrecLandState::ActiveSearchStart:
		run_state_asearch_start();
		break;

	case PrecLandState::ActiveSearchNewCircle:
		run_state_asearch_new_circle();
		break;

	case PrecLandState::ActiveSearch:
		run_state_asearch();
		break;

	case PrecLandState::ActiveSearchReturn:
		run_state_asearch_return();
		break;

	case PrecLandState::Fallback:
		run_state_fallback();
		break;

	case PrecLandState::Done:
		// nothing to do
		break;

	default:
		// unknown state
		break;
	}

}

void
PrecLand::run_state_start()
{
	// check if target visible and go to horizontal approach
	if (switch_to_state_horizontal_approach()) {
		return;
	}

	if (_mode == PrecLandMode::Opportunistic) {
		// could not see the target immediately, so just fall back to normal landing
		if (!switch_to_state_fallback()) {
			PX4_ERR("Can't switch to search or fallback landing");
			PLD_STATUS_MAVLINK(_MSG_PRIO_ERROR, "PLD: Can't switch to search or fallback landing");
		}
	}

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	float dist = get_distance_to_next_waypoint(pos_sp_triplet->current.lat, pos_sp_triplet->current.lon,
			_navigator->get_global_position()->lat, _navigator->get_global_position()->lon);

	// check if we've reached the start point
	if (dist < _navigator->get_acceptance_radius()) {
		if (!_point_reached_time) {
			_point_reached_time = hrt_absolute_time();
		}

		// if we don't see the target after 1 second, search for it
		if (_param_search_timeout.get() > 0) {

			if (hrt_absolute_time() - _point_reached_time > 2000000) {
				if (!switch_to_state_search()) {
					if (!switch_to_state_fallback()) {
						PX4_ERR("Can't switch to search or fallback landing");
						PLD_STATUS_MAVLINK(_MSG_PRIO_ERROR, "PLD: Can't switch to search or fallback landing");
					}
				}
			}

		} else {
			if (!switch_to_state_fallback()) {
				PX4_ERR("Can't switch to search or fallback landing");
				PLD_STATUS_MAVLINK(_MSG_PRIO_ERROR, "PLD: Can't switch to search or fallback landing");
			}
		}
	}
}

void
PrecLand::run_state_horizontal_approach()
{
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	// check if target visible, if not go to start
	if (!check_state_conditions(PrecLandState::HorizontalApproach)) {
		PX4_WARN("Lost landing target while landing (horizontal approach).");
		PLD_STATUS_MAVLINK(_MSG_PRIO_WARNING, "PLD: Target lost in the horizontal approach");

		// Stay at current position for searching for the landing target
		pos_sp_triplet->current.lat = _navigator->get_global_position()->lat;
		pos_sp_triplet->current.lon = _navigator->get_global_position()->lon;
		pos_sp_triplet->current.alt = _navigator->get_global_position()->alt;

		if (!switch_to_state_start()) {
			if (!switch_to_state_fallback()) {
				PX4_ERR("Can't switch to fallback landing");
				PLD_STATUS_MAVLINK(_MSG_PRIO_ERROR, "PLD: Can't switch to fallback landing");
			}
		}

		return;
	}

	if (check_state_conditions(PrecLandState::DescendAboveTarget)) {
		if (!_point_reached_time) {
			_point_reached_time = hrt_absolute_time();
		}

		if (hrt_absolute_time() - _point_reached_time > 2000000) {
			// if close enough for descent above target go to descend above target
			if (switch_to_state_descend_above_target()) {
				return;
			}
		}

	}

	if (hrt_absolute_time() - _state_start_time > _param_state_timeout.get()*SEC2USEC) {
		PX4_ERR("Precision landing took too long during horizontal approach phase");
		PLD_STATUS_MAVLINK(_MSG_PRIO_ERROR, "PLD: Too long in the horizontal approach phase");

		if (switch_to_state_fallback()) {
			return;
		}

		PX4_ERR("Can't switch to fallback landing");
		PLD_STATUS_MAVLINK(_MSG_PRIO_ERROR, "PLD: Can't switch to fallback landing");
	}

	float x = _target_pose.x_abs;
	float y = _target_pose.y_abs;

	slewrate(x, y);

	// XXX need to transform to GPS coords because mc_pos_control only looks at that
	double lat, lon;
	map_projection_reproject(&_map_ref, x, y, &lat, &lon);

	pos_sp_triplet->current.lat = lat;
	pos_sp_triplet->current.lon = lon;
	pos_sp_triplet->current.alt = _approach_alt;
	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;

	_navigator->set_position_setpoint_triplet_updated();
}

void
PrecLand::run_state_descend_above_target()
{
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	// check if target visible
	if (!check_state_conditions(PrecLandState::DescendAboveTarget)) {
		if (!switch_to_state_final_approach()) {
			PX4_WARN("Lost landing target while landing (descending).");
			PLD_STATUS_MAVLINK(_MSG_PRIO_WARNING, "PLD: Lost landing target while descending");

			// Stay at current position for searching for the target
			pos_sp_triplet->current.lat = _navigator->get_global_position()->lat;
			pos_sp_triplet->current.lon = _navigator->get_global_position()->lon;
			pos_sp_triplet->current.alt = _navigator->get_global_position()->alt;

			if (!switch_to_state_start()) {
				if (!switch_to_state_fallback()) {
					PX4_ERR("Can't switch to fallback landing");
					PLD_STATUS_MAVLINK(_MSG_PRIO_ERROR, "PLD: Can't switch to fallback landing");
				}
			}
		}

		return;
	}

	// If a strict precland is requested (horizontal offset is important)
	// HINT: This branch may try to switch to horizontal approach while target is not updated.
	// It will cause multiple errors output after unsuccessfull switch_to_state_horizontal_approach
	// calls. To avoid such behaviour _target_pose_updated flag check has been added.
	if (_param_strict.get() && _target_pose_updated)
	{
		// Get the current local position
		vehicle_local_position_s *vehicle_local_position = _navigator->get_local_position();

		// Target is still visible, but we are close to the final approach altitude
		if (check_state_conditions(PrecLandState::FinalApproach)) {
			// Strict precland is not locked yet
			if (!_strict_stop) {
				PX4_WARN("Final approach altitude has been reached, strict precland is disabled");
				PLD_STATUS_MAVLINK(_MSG_PRIO_WARNING, "PLD: Strict precland is disabled");

				// Block the strict precland to avoid horizontal approach close to the surface
				_strict_stop = true;
			}
		}
		// Strict precland is not blocked, we are out of the horizontal acceptance radius and over the final approach altitude
		else if (!_strict_stop && !check_hacc_rad(vehicle_local_position)) {
			PX4_WARN("Out of the horizontal acceptance radius (strict precland)");
			PLD_STATUS_MAVLINK(_MSG_PRIO_WARNING, "PLD: Out of the horizontal acceptance radius");

			// Correct the vehicle position using the horizontal approach step
			if (!switch_to_state_horizontal_approach()) {
				// Landing target position has been lost
				PX4_ERR("Can't switch to horizontal approach");
				PLD_STATUS_MAVLINK(_MSG_PRIO_ERROR, "PLD: Can't switch to horizontal approach");

				// No need to do anything else.
				// Condition check will handle the case on the next iteration.
			}

			return;
		}
	}

	// XXX need to transform to GPS coords because mc_pos_control only looks at that
	double lat, lon;
	map_projection_reproject(&_map_ref, _target_pose.x_abs, _target_pose.y_abs, &lat, &lon);

	pos_sp_triplet->current.lat = lat;
	pos_sp_triplet->current.lon = lon;

	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_LAND;

	_navigator->set_position_setpoint_triplet_updated();
}

void
PrecLand::run_state_final_approach()
{
	// nothing to do, will land
}

void
PrecLand::run_state_search()
{
	// check if we can see the target
	if (check_state_conditions(PrecLandState::HorizontalApproach)) {
		if (!_target_acquired_time) {
			// target just became visible. Stop climbing, but give it some margin so we don't stop too apruptly
			_target_acquired_time = hrt_absolute_time();
			position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
			float new_alt = _navigator->get_global_position()->alt + 1.0f;
			pos_sp_triplet->current.alt = new_alt < pos_sp_triplet->current.alt ? new_alt : pos_sp_triplet->current.alt;
			_navigator->set_position_setpoint_triplet_updated();
		}

	}

	// stay at that height for a second to allow the vehicle to settle
	if (_target_acquired_time && (hrt_absolute_time() - _target_acquired_time) > 1000000) {
		// try to switch to horizontal approach
		if (switch_to_state_horizontal_approach()) {
			return;
		}
	}

	// check if search timed out and go to fallback
	if (hrt_absolute_time() - _state_start_time > _param_search_timeout.get()*SEC2USEC) {
		PX4_WARN("Search timed out");
		PLD_STATUS_MAVLINK(_MSG_PRIO_WARNING, "PLD: Search timed out");

		if (_param_asearch_enabled.get()) {
			// Active search is enabled, reset an active search (reset the attempts counter)
			if (!switch_to_state_asearch_reset()) {
				PX4_ERR("Can't reset an active search");
				PLD_STATUS_MAVLINK(_MSG_PRIO_ERROR, "PLD: Can't reset an active search");
			}
			else
				// Failed to start an active search, falling back
				return;
		}
		
		// Start a fall back
		if (!switch_to_state_fallback()) {
			PX4_ERR("Can't switch to fallback landing");
			PLD_STATUS_MAVLINK(_MSG_PRIO_ERROR, "PLD: Can't switch to fallback landing");
		}
	}
}

void
PrecLand::run_state_asearch_reset()
{
	// The active search counter has been reset during the switch procedure, start a new active search
	if (!switch_to_state_asearch_start()) {
		PX4_ERR("Can't start an active search");
		PLD_STATUS_MAVLINK(_MSG_PRIO_ERROR, "PLD: Can't start an active search");

		// Start a fall back
		if (!switch_to_state_fallback()) {
			PX4_ERR("Can't switch to a fallback");
			PLD_STATUS_MAVLINK(_MSG_PRIO_ERROR, "PLD: Can't switch to a fallback");
		}
	}
}

void
PrecLand::run_state_asearch_start()
{
	// Initialisation has been completed during the switch procedure, start a new circle
	if (!switch_to_state_asearch_new_circle()) {
		PX4_ERR("Can't start a first circle");
		PLD_STATUS_MAVLINK(_MSG_PRIO_ERROR, "PLD: Can't start a new circle");

		// Start a fall back
		if (!switch_to_state_fallback()) {
			PX4_ERR("Can't switch to a fallback");
			PLD_STATUS_MAVLINK(_MSG_PRIO_ERROR, "PLD: Can't switch to a fallback");
		}
	}
}

void
PrecLand::run_state_asearch_new_circle()
{
	// Check if the target is visible, start the horizontal approach
	if (check_state_conditions(PrecLandState::HorizontalApproach) && !switch_to_state_horizontal_approach())
		return;

	// We have reach a start position for the new circle
	if (check_state_conditions(PrecLandState::ActiveSearch)) {
		// Start moving around the circle
		if (!switch_to_state_asearch()) {
			PX4_ERR("Can't switch to an active search");
			PLD_STATUS_MAVLINK(_MSG_PRIO_ERROR, "PLD: Can't switch to active search");

			// Start a fallback
			if (!switch_to_state_fallback()) {
				PX4_ERR("Can't switch to a fallback");
				PLD_STATUS_MAVLINK(_MSG_PRIO_ERROR, "PLD: Can't switch to a fallback");
			}
		}
	}
}

void
PrecLand::run_state_asearch()
{
	// Check if the target is visible, start the horizontal approach
	if (check_state_conditions(PrecLandState::HorizontalApproach) && !switch_to_state_horizontal_approach())
		return;

	// Full circle
	if (_asearch_phi >= 2 * (float)M_PI) {
		PX4_INFO("Active search circle finished");
		PLD_STATUS_MAVLINK(_MSG_PRIO_WARNING, "PLD: Active search circle finished");

		// Increase the circle radius
		_asearch_radius += _param_asearch_cc_step.get();

		// Start a new circle
		if (!switch_to_state_asearch_new_circle()) {
			PX4_WARN("Final circle radius reached");
			PLD_STATUS_MAVLINK(_MSG_PRIO_WARNING, "PLD: Final circle radius reached");

			// Return to the starting position
			if (!switch_to_state_asearch_return()) {
				PX4_ERR("Can't start a return state");
				PLD_STATUS_MAVLINK(_MSG_PRIO_ERROR, "PLD: Can't start a return state");

				// Fall back
				if (!switch_to_state_fallback()) {
					PX4_ERR("Can't switch to a fallback");
					PLD_STATUS_MAVLINK(_MSG_PRIO_ERROR, "PLD: Can't switch to a fallback");
				}
			}
		}
	}
	// Target coordinates are invalid or the target setpoint is reached
	else if (isnan(_asearch_target_x) || check_state_conditions(PrecLandState::ActiveSearch))
	{
		position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

		// Get a new coordinates from a polar formula: r = R(phi)
		_asearch_target_x = _asearch_radius * cos(_asearch_phi);
		_asearch_target_y = _asearch_radius * sin(_asearch_phi);

		// Convert local NED coordinates to the global coordinates
		map_projection_reproject(&_asearch_ref, _asearch_target_x, _asearch_target_y, &pos_sp_triplet->current.lat,
			&pos_sp_triplet->current.lon);

		// Set a new setpoint
		_navigator->set_position_setpoint_triplet_updated();

		// Move around the circle
		_asearch_phi += _asearch_phi_step;
	}
}

void
PrecLand::run_state_asearch_return()
{
	// Check if the target is visible, start the horizontal approach
	if (check_state_conditions(PrecLandState::HorizontalApproach) && !switch_to_state_horizontal_approach())
		return;

	// We have returned to the start position
	if (check_state_conditions(PrecLandState::ActiveSearchReturn)) {
		// Increase the active search attempts counter
		_asearch_cnt++;

		// Check if we exceeded the search attempts
		if (!check_state_conditions(PrecLandState::ActiveSearchStart)) {
			PX4_ERR("Too many active search attempts");
			PLD_STATUS_MAVLINK(_MSG_PRIO_ERROR, "PLD: Too many active search attempts");
		}
		else
		{
			// Restart the active search
			if (switch_to_state_asearch_start())
				return;
			else {
				PX4_ERR("Failed to start an active search");
				PLD_STATUS_MAVLINK(_MSG_PRIO_ERROR, "PLD: Failed to start an active search");
			}	
		}

		// Start a fall back
		if (!switch_to_state_fallback()) {
			PX4_ERR("Can't switch to a fallback");
			PLD_STATUS_MAVLINK(_MSG_PRIO_ERROR, "PLD: Can't switch to a fallback");
		}
	}
}

void
PrecLand::run_state_fallback()
{
	// nothing to do, we should be in the other module right now
}

bool
PrecLand::switch_to_state_start()
{
	if (check_state_conditions(PrecLandState::Start)) {
		position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
		pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
		_navigator->set_position_setpoint_triplet_updated();
		_search_cnt++;

		_point_reached_time = 0;

		_state = PrecLandState::Start;
		_state_start_time = hrt_absolute_time();

		PX4_WARN("Precland start state");
		PLD_STATUS_MAVLINK(_MSG_PRIO_WARNING, "PLD: Precland start state");
		return true;
	}

	return false;
}

bool
PrecLand::switch_to_state_horizontal_approach()
{
	if (check_state_conditions(PrecLandState::HorizontalApproach)) {
		_approach_alt = _navigator->get_global_position()->alt;

		_point_reached_time = 0;

		_state = PrecLandState::HorizontalApproach;
		_state_start_time = hrt_absolute_time();

		PX4_WARN("Precland horizontal approach");
		PLD_STATUS_MAVLINK(_MSG_PRIO_WARNING, "PLD: Precland horizontal approach");
		return true;
	}

	return false;
}

bool
PrecLand::switch_to_state_descend_above_target()
{
	if (check_state_conditions(PrecLandState::DescendAboveTarget)) {
		_state = PrecLandState::DescendAboveTarget;
		_state_start_time = hrt_absolute_time();

		PX4_WARN("Precland descend above target");
		PLD_STATUS_MAVLINK(_MSG_PRIO_WARNING, "PLD: Precland descend above target");
		return true;
	}

	return false;
}

bool
PrecLand::switch_to_state_final_approach()
{
	if (check_state_conditions(PrecLandState::FinalApproach)) {
		_state = PrecLandState::FinalApproach;
		_state_start_time = hrt_absolute_time();

		PX4_WARN("Precland final approach");
		PLD_STATUS_MAVLINK(_MSG_PRIO_WARNING, "PLD: Precland final approach");
		return true;
	}

	return false;
}

bool
PrecLand::switch_to_state_search()
{
	PX4_INFO("Climbing to search altitude");
	PLD_STATUS_MAVLINK(_MSG_PRIO_WARNING, "PLD: Climbing to search altitude");
	vehicle_local_position_s *vehicle_local_position = _navigator->get_local_position();

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	pos_sp_triplet->current.alt = vehicle_local_position->ref_alt + _param_search_alt.get();
	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
	_navigator->set_position_setpoint_triplet_updated();

	_target_acquired_time = 0;

	_state = PrecLandState::Search;
	_state_start_time = hrt_absolute_time();
	return true;
}

bool
PrecLand::switch_to_state_asearch_reset()
{
	// Check if it's possible to reset an active search from the current state
	if (!check_state_conditions(PrecLandState::ActiveSearchReset))
		return false;

	PX4_INFO("Starting an active search reset");
	PLD_STATUS_MAVLINK(_MSG_PRIO_WARNING, "PLD: Starting an active search reset");

	// Reset the active search attempts counter
	_asearch_cnt = 0;

	_state = PrecLandState::ActiveSearchReset;
	_state_start_time = hrt_absolute_time();
	return true;
}

bool
PrecLand::switch_to_state_asearch_start()
{
	if (_param_asearch_cc_step.get() > _param_asearch_final_radius.get()) {
		PX4_ERR("Concentric circles radius step is too hight");
		PLD_STATUS_MAVLINK(_MSG_PRIO_ERROR, "PLD: Concentric circles radius step is too hight");

		return false;
	}

	if (_param_asearch_cc_step.get() < _param_asearch_acc_rad.get()) {
		PX4_ERR("Circle radius step is less than an acceptance radius");
		PLD_STATUS_MAVLINK(_MSG_PRIO_ERROR, "PLD: Too low circle radius step");

		return false;
	}

	if (_param_asearch_setpoint_step.get() < _param_asearch_acc_rad.get()) {
		PX4_ERR("Setpoint step is less than an acceptance radius");
		PLD_STATUS_MAVLINK(_MSG_PRIO_ERROR, "PLD: Too low setpoint step");

		return false;
	}

	// Init the active search attempts counter
	_asearch_cnt = 1;
	// First circle radius
	_asearch_radius = _param_asearch_cc_step.get();

	PX4_INFO("Starting an active search");
	PLD_STATUS_MAVLINK(_MSG_PRIO_WARNING, "PLD: Starting an active search");

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	// Init the global coordinates reference by the current coordinates
	map_projection_init(&_asearch_ref, pos_sp_triplet->current.lat, pos_sp_triplet->current.lon);

	_state = PrecLandState::ActiveSearchStart;
	_state_start_time = hrt_absolute_time();
	return true;
}

bool
PrecLand::switch_to_state_asearch_new_circle()
{
	// Check if it's possible to start a new from the current state
	if (!check_state_conditions(PrecLandState::ActiveSearchNewCircle))
		return false;

	PX4_INFO("Starting a new active search circle");
	PLD_STATUS_MAVLINK(_MSG_PRIO_WARNING, "PLD: Starting a new active search circle");

	// Calculate a new polar coordinates step as a horde between two set setpoints: phi = 2 * asin(m / (2 * R))
	_asearch_phi_step = 2 * asin(_param_asearch_setpoint_step.get() / (2 * _asearch_radius));

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	// Set an starting circle point
	_asearch_target_x = _asearch_radius;
	_asearch_target_y = 0;

	// Convert local NED coordinates to the global coordinates
	map_projection_reproject(&_asearch_ref, _asearch_radius, 0, &pos_sp_triplet->current.lat, &pos_sp_triplet->current.lon);
	
	// Set a new setpoint
	_navigator->set_position_setpoint_triplet_updated();

	_state = PrecLandState::ActiveSearchNewCircle;
	_state_start_time = hrt_absolute_time();
	return true;
}

bool
PrecLand::switch_to_state_asearch()
{
	PX4_INFO("Active search");
	PLD_STATUS_MAVLINK(_MSG_PRIO_WARNING, "PLD: Active search");

	// Invalidate a target coordinates (to force a setpoint update)
	_asearch_target_x = NAN;
	// Set an initial polar coordinate
	_asearch_phi = 0;

	_state = PrecLandState::ActiveSearch;
	_state_start_time = hrt_absolute_time();
	return true;
}

bool
PrecLand::switch_to_state_asearch_return()
{
	PX4_INFO("Return to the active search start position");
	PLD_STATUS_MAVLINK(_MSG_PRIO_WARNING, "PLD: Return to the active search start position");

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	// Get the initial global coordinates from the reference
	map_projection_reproject(&_asearch_ref, 0, 0, &pos_sp_triplet->current.lat, &pos_sp_triplet->current.lon);

	// Reset the target coordinates
	_asearch_target_x = 0;
	_asearch_target_y = 0;
	
	// Set a new setpoint
	_navigator->set_position_setpoint_triplet_updated();

	_state = PrecLandState::ActiveSearchReturn;
	_state_start_time = hrt_absolute_time();
	return true;
}

bool
PrecLand::switch_to_state_fallback()
{
	_state = PrecLandState::Fallback;
	_state_start_time = hrt_absolute_time();

	vehicle_command_s vcmd = {};

	switch(_param_fallback_action.get())
	{
	case PrecLand::PLD_FOAC_LAND:
		vcmd.command = vehicle_command_s::VEHICLE_CMD_NAV_LAND;

		PX4_WARN("Falling back to normal land");
		PLD_STATUS_MAVLINK(_MSG_PRIO_WARNING, "PLD: Falling back to normal land");
		break;
	case PrecLand::PLD_FOAC_RTL:
		vcmd.command = vehicle_command_s::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH;

		PX4_WARN("Falling back to RTL");
		PLD_STATUS_MAVLINK(_MSG_PRIO_WARNING, "PLD: Falling back to RTL");
		break;
	default:
		// Falling back to land if the action is unknown
		vcmd.command = vehicle_command_s::VEHICLE_CMD_NAV_LAND;

		PX4_WARN("Unknown fallback action!");
		PLD_STATUS_MAVLINK(_MSG_PRIO_WARNING, "PLD: Unknown fallback action!");
		break;
	}

	_navigator->publish_vehicle_cmd(&vcmd);

	return true;
}

bool
PrecLand::switch_to_state_done()
{
	_state = PrecLandState::Done;
	_state_start_time = hrt_absolute_time();
	return true;
}

bool PrecLand::check_hacc_rad(vehicle_local_position_s *vehicle_local_position)
{
	// Strict precland is enabled and it's a strict precland in a funnel
	if (_param_strict.get() && (_strict_funnel_k > 0))
	{
		/* Calculate radius using a funnel linear part function and then limit output value
		* to the funnel top radius from the top and to the horizontal acceptance radius from
		* the bottom forming a funnel.
		*
		* 	r(a) = a * k + r_o, where:
		* 		a - a vehicle altitude,
		*		k - a funnel linear part slope,
		*		r_o - a funnel linear part offset.
		*	
		*/
		float funnel_rad = math::max(math::min((_target_pose.z_abs - vehicle_local_position->z) *
			_strict_funnel_k + _strict_funnel_r_o, _param_funnel_top_rad.get()), _param_hacc_rad.get());
		
		// PX4_WARN("FUNNEL: %f m, ALT: %f m", (double)funnel_rad, (double)fabsf(_target_pose.z_abs - vehicle_local_position->z));
		
		// Check if the vehicle is inside the funnel
		return fabsf(_target_pose.x_abs - vehicle_local_position->x) < funnel_rad
			    	&& fabsf(_target_pose.y_abs - vehicle_local_position->y) < funnel_rad;	
	}
	else
		// Check if the vehicle is inside the horizontal acceptance radius
		return fabsf(_target_pose.x_abs - vehicle_local_position->x) < _param_hacc_rad.get()
			    	&& fabsf(_target_pose.y_abs - vehicle_local_position->y) < _param_hacc_rad.get();			
}

bool PrecLand::check_setpoint_reached(struct map_projection_reference_s *ref, float target_x, float target_y, float acc_rad)
{
	vehicle_global_position_s *vehicle_global_position = _navigator->get_global_position();

	float cur_x, cur_y;

	map_projection_project(ref, vehicle_global_position->lat, vehicle_global_position->lon,
		&cur_x, &cur_y);

	return (fabs(target_x - cur_x) < acc_rad) && (fabs(target_y - cur_y) < acc_rad);
}

bool PrecLand::check_state_conditions(PrecLandState state)
{
	vehicle_local_position_s *vehicle_local_position = _navigator->get_local_position();

	switch (state) {
	case PrecLandState::Start:
		return _search_cnt <= _param_max_searches.get();

	case PrecLandState::HorizontalApproach:

		// if we're already in this state, only want to make it invalid if we reached the target but can't see it anymore
		if (_state == PrecLandState::HorizontalApproach) {
			if (check_hacc_rad(vehicle_local_position)) {
				// we've reached the position where we last saw the target. If we don't see it now, we need to do something
				return _target_pose_valid && _target_pose.abs_pos_valid;
			} else {
				// We've seen the target sometime during horizontal approach.
				// Even if we don't see it as we're moving towards it, continue approaching last known location
				return true;
			}
		}

		// If we're trying to switch to this state, the target needs to be visible
		return _target_pose_updated && _target_pose_valid && _target_pose.abs_pos_valid;

	case PrecLandState::DescendAboveTarget:

		// If we're already in this state, only leave it if target becomes unusable, don't care about horizontal offset to target
		if (_state == PrecLandState::DescendAboveTarget) {
			// if we're close to the ground, we're more critical of target timeouts so we quickly go into descend
			if (check_state_conditions(PrecLandState::FinalApproach)) {
				return hrt_absolute_time() - _target_pose.timestamp < 500000; // 0.5s

			} else {
				return _target_pose_valid && _target_pose.abs_pos_valid;
			}

		} else {
			// if not already in this state, need to be above target to enter it
			return _target_pose_updated && _target_pose.abs_pos_valid
			       && check_hacc_rad(vehicle_local_position);
		}

	case PrecLandState::FinalApproach:
		return _target_pose_valid && _target_pose.abs_pos_valid
		       && (_target_pose.z_abs - vehicle_local_position->z) < _param_final_approach_alt.get();

	case PrecLandState::Search:
		return true;

	case PrecLandState::ActiveSearchReset:
		return true;

	case PrecLandState::ActiveSearchStart:
		return _asearch_cnt <= _param_max_asearches.get();

	case PrecLandState::ActiveSearchNewCircle:
		if (_state == PrecLandState::ActiveSearchStart)
			return _asearch_radius >= _param_asearch_setpoint_step.get();
		else if (_state == PrecLandState::ActiveSearch)
			return _asearch_radius <= _param_asearch_final_radius.get();
		else
			return false;

	case PrecLandState::ActiveSearch:
		if ((_state == PrecLandState::ActiveSearchNewCircle) || (_state == PrecLandState::ActiveSearch))
			return check_setpoint_reached(&_asearch_ref, _asearch_target_x, _asearch_target_y,
				_param_asearch_acc_rad.get());
		else
			return false;

	case PrecLandState::ActiveSearchReturn:
		if (_state == PrecLandState::ActiveSearchReturn)
			return check_setpoint_reached(&_asearch_ref, _asearch_target_x, _asearch_target_y,
				_param_asearch_acc_rad.get());
		else
			return false;
		
	case PrecLandState::Fallback:
		return true;

	default:
		return false;
	}
}

void PrecLand::slewrate(float &sp_x, float &sp_y)
{
	matrix::Vector2f sp_curr(sp_x, sp_y);
	uint64_t now = hrt_absolute_time();

	float dt = (now - _last_slewrate_time);

	if (dt < 1) {
		// bad dt, can't divide by it
		return;
	}

	dt /= SEC2USEC;

	if (!_last_slewrate_time) {
		// running the first time since switching to precland

		// assume dt will be about 50000us
		dt = 50000 / SEC2USEC;

		// set a best guess for previous setpoints for smooth transition
		map_projection_project(&_map_ref, _navigator->get_position_setpoint_triplet()->current.lat,
				       _navigator->get_position_setpoint_triplet()->current.lon, &_sp_pev(0), &_sp_pev(1));
		_sp_pev_prev(0) = _sp_pev(0) - _navigator->get_local_position()->vx * dt;
		_sp_pev_prev(1) = _sp_pev(1) - _navigator->get_local_position()->vy * dt;
	}

	_last_slewrate_time = now;

	// limit the setpoint speed to the maximum cruise speed
	matrix::Vector2f sp_vel = (sp_curr - _sp_pev) / dt; // velocity of the setpoints

	if (sp_vel.length() > _param_xy_vel_cruise.get()) {
		sp_vel = sp_vel.normalized() * _param_xy_vel_cruise.get();
		sp_curr = _sp_pev + sp_vel * dt;
	}

	// limit the setpoint acceleration to the maximum acceleration
	matrix::Vector2f sp_acc = (sp_curr - _sp_pev * 2 + _sp_pev_prev) / (dt * dt); // acceleration of the setpoints

	if (sp_acc.length() > _param_acceleration_hor.get()) {
		sp_acc = sp_acc.normalized() * _param_acceleration_hor.get();
		sp_curr = _sp_pev * 2 - _sp_pev_prev + sp_acc * (dt * dt);
	}

	// limit the setpoint speed such that we can stop at the setpoint given the maximum acceleration/deceleration
	float max_spd = sqrtf(_param_acceleration_hor.get() * ((matrix::Vector2f)(_sp_pev - matrix::Vector2f(sp_x,
			      sp_y))).length());
	sp_vel = (sp_curr - _sp_pev) / dt; // velocity of the setpoints

	if (sp_vel.length() > max_spd) {
		sp_vel = sp_vel.normalized() * max_spd;
		sp_curr = _sp_pev + sp_vel * dt;
	}

	_sp_pev_prev = _sp_pev;
	_sp_pev = sp_curr;

	sp_x = sp_curr(0);
	sp_y = sp_curr(1);
}
