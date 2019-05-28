/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file followme.cpp
 *
 * Helper class to track and follow a given position
 *
 * @author Jimmy Johnson <catch22@fastmail.net>
 */

#include "follow_target.h"

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <fcntl.h>

#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/follow_target.h>
#include <lib/ecl/geo/geo.h>
#include <lib/mathlib/math/Limits.hpp>
#include "navigator.h"

using matrix::wrap_pi;

constexpr float FollowTarget::_follow_position_matricies[4][9];
#define TESTANGLE 0.0f
FollowTarget::FollowTarget(Navigator *navigator) :
	MissionBlock(navigator),
	ModuleParams(navigator)
{
	_current_vel.zero();
	_step_vel.zero();
	_est_target_vel.zero();
	_target_distance.zero();
	_target_position_offset.zero();
	_target_position_delta.zero();
}

void FollowTarget::on_inactive()
{
	reset_target_validity();
}

void FollowTarget::on_activation()
{
	_follow_offset = _param_tracking_dist.get() < 1.0F ? 1.0F : _param_tracking_dist.get();

	_responsiveness = math::constrain((float) _param_tracking_resp.get(), .1F, 1.0F);

	_follow_target_position = _param_tracking_side.get();

	if ((_follow_target_position > FOLLOW_FROM_LEFT) || (_follow_target_position < FOLLOW_FROM_RIGHT)) {
		_follow_target_position = FOLLOW_FROM_BEHIND;
	}

	_rot_matrix = (_follow_position_matricies[_follow_target_position]);

	if (_follow_target_sub < 0) {
		_follow_target_sub = orb_subscribe(ORB_ID(follow_target));
	}
#ifdef FORMATION
    _last_time = hrt_absolute_time();
    ///订阅其他飞机位置信息        ***zjm
    for(int i = 0; i < 4; i++)
    {
        if(_formation_pos_sub[i] < 0)
            _formation_pos_sub[i]= orb_subscribe_multi(ORB_ID(formation_position), i);
        memset(&_formation_positions[i], 0, sizeof(_formation_positions[i]));
    }///
    #ifdef VIRTUALTEST
        _virtual_target_movement.zero();
        _prev_time = hrt_absolute_time();
        _follow_offset = _param_tracking_dist.get();
    #endif
#endif
}

void FollowTarget::on_active()
{
#ifndef FORMATION
    struct map_projection_reference_s target_ref;
    follow_target_s target_motion_with_offset = {};
    uint64_t current_time = hrt_absolute_time();
    bool _radius_entered = false;
    bool _radius_exited = false;
    bool updated = false;
    float dt_ms = 0;
	orb_check(_follow_target_sub, &updated);

	if (updated) {
		follow_target_s target_motion;

		_target_updates++;

		// save last known motion topic

		_previous_target_motion = _current_target_motion;

		orb_copy(ORB_ID(follow_target), _follow_target_sub, &target_motion);

		if (_current_target_motion.timestamp == 0) {
			_current_target_motion = target_motion;
		}

		_current_target_motion.timestamp = target_motion.timestamp;
		_current_target_motion.lat = (_current_target_motion.lat * (double)_responsiveness) + target_motion.lat * (double)(
						     1 - _responsiveness);
		_current_target_motion.lon = (_current_target_motion.lon * (double)_responsiveness) + target_motion.lon * (double)(
						     1 - _responsiveness);

	} else if (((current_time - _current_target_motion.timestamp) / 1000) > TARGET_TIMEOUT_MS && target_velocity_valid()) {
		reset_target_validity();
	}

	// update distance to target

	if (target_position_valid()) {

		// get distance to target

		map_projection_init(&target_ref, _navigator->get_global_position()->lat, _navigator->get_global_position()->lon);
		map_projection_project(&target_ref, _current_target_motion.lat, _current_target_motion.lon, &_target_distance(0),
				       &_target_distance(1));

	}

	// update target velocity

	if (target_velocity_valid() && updated) {

		dt_ms = ((_current_target_motion.timestamp - _previous_target_motion.timestamp) / 1000);

		// ignore a small dt
		if (dt_ms > 10.0F) {
			// get last gps known reference for target
			map_projection_init(&target_ref, _previous_target_motion.lat, _previous_target_motion.lon);

			// calculate distance the target has moved
			map_projection_project(&target_ref, _current_target_motion.lat, _current_target_motion.lon,
					       &(_target_position_delta(0)), &(_target_position_delta(1)));

			// update the average velocity of the target based on the position
			_est_target_vel = _target_position_delta / (dt_ms / 1000.0f);

			// if the target is moving add an offset and rotation
			if (_est_target_vel.length() > .5F) {
				_target_position_offset = _rot_matrix * _est_target_vel.normalized() * _follow_offset;
			}

			// are we within the target acceptance radius?
			// give a buffer to exit/enter the radius to give the velocity controller
			// a chance to catch up

			_radius_exited = ((_target_position_offset + _target_distance).length() > (float) TARGET_ACCEPTANCE_RADIUS_M * 1.5f);
			_radius_entered = ((_target_position_offset + _target_distance).length() < (float) TARGET_ACCEPTANCE_RADIUS_M);

			// to keep the velocity increase/decrease smooth
			// calculate how many velocity increments/decrements
			// it will take to reach the targets velocity
			// with the given amount of steps also add a feed forward input that adjusts the
			// velocity as the position gap increases since
			// just traveling at the exact velocity of the target will not
			// get any closer or farther from the target

			_step_vel = (_est_target_vel - _current_vel) + (_target_position_offset + _target_distance) * FF_K;
			_step_vel /= (dt_ms / 1000.0F * (float) INTERPOLATION_PNTS);
			_step_time_in_ms = (dt_ms / (float) INTERPOLATION_PNTS);

			// if we are less than 1 meter from the target don't worry about trying to yaw
			// lock the yaw until we are at a distance that makes sense

			if ((_target_distance).length() > 1.0F) {

				// yaw rate smoothing

				// this really needs to control the yaw rate directly in the attitude pid controller
				// but seems to work ok for now since the yaw rate cannot be controlled directly in auto mode

				_yaw_angle = get_bearing_to_next_waypoint(_navigator->get_global_position()->lat,
						_navigator->get_global_position()->lon,
						_current_target_motion.lat,
						_current_target_motion.lon);

				_yaw_rate = wrap_pi((_yaw_angle - _navigator->get_global_position()->yaw) / (dt_ms / 1000.0f));

			} else {
				_yaw_angle = _yaw_rate = NAN;
			}
		}

//		warnx(" _step_vel x %3.6f y %3.6f cur vel %3.6f %3.6f tar vel %3.6f %3.6f dist = %3.6f (%3.6f) mode = %d yaw rate = %3.6f",
//				(double) _step_vel(0),
//				(double) _step_vel(1),
//				(double) _current_vel(0),
//				(double) _current_vel(1),
//				(double) _est_target_vel(0),
//				(double) _est_target_vel(1),
//				(double) (_target_distance).length(),
//				(double) (_target_position_offset + _target_distance).length(),
//				_follow_target_state,
//				(double) _yaw_rate);
	}

	if (target_position_valid()) {

		// get the target position using the calculated offset

		map_projection_init(&target_ref,  _current_target_motion.lat, _current_target_motion.lon);
		map_projection_reproject(&target_ref, _target_position_offset(0), _target_position_offset(1),
					 &target_motion_with_offset.lat, &target_motion_with_offset.lon);
	}

	// clamp yaw rate smoothing if we are with in
	// 3 degrees of facing target

	if (PX4_ISFINITE(_yaw_rate)) {
		if (fabsf(fabsf(_yaw_angle) - fabsf(_navigator->get_global_position()->yaw)) < math::radians(3.0F)) {
			_yaw_rate = NAN;
		}
	}
#else
    struct map_projection_reference_s target_ref;
    follow_target_s target_motion_with_offset = {};
    uint64_t current_time = hrt_absolute_time();
    bool _radius_entered = false;
    bool _radius_exited = false;
    float dt_ms = 0;
    if(_time_updated)
    {
        _last_time = hrt_absolute_time();
        _time_updated = false;
    }

    //    PX4_INFO("In on_active current_time:%llu", current_time);
    ///更新编队飞机位置，用下标表示相应飞机编号，下标0个为起降跟踪的无人船编号，下标LEANDER_ID=1为领飞无人机编号mavlink_system.sysid为此飞机编号
    formation_position_s formation_pos;
    bool pos_updated[4] = {false, false, false, false};
    for(int i = 0; i < 4; i++)
    {
        bool updated = false;
        orb_check(_formation_pos_sub[i], &updated);
        if(updated)
        {
            orb_copy(ORB_ID(formation_position), _formation_pos_sub[i], &formation_pos);
            _formation_positions[formation_pos.sysid].timestamp = formation_pos.timestamp;
            _formation_positions[formation_pos.sysid].lon = formation_pos.lon;
            _formation_positions[formation_pos.sysid].lat = formation_pos.lat;
            _formation_positions[formation_pos.sysid].alt = formation_pos.alt;
            _formation_positions[formation_pos.sysid].vel_n = formation_pos.vel_n;
            _formation_positions[formation_pos.sysid].vel_e = formation_pos.vel_e;
            _formation_positions[formation_pos.sysid].vel_d = formation_pos.vel_d;
            _formation_positions[formation_pos.sysid].yaw = formation_pos.yaw;
            _formation_positions[formation_pos.sysid].sysid = formation_pos.sysid;
            pos_updated[formation_pos.sysid] = true;

            PX4_INFO("i:%d,receive sysid:%d pos, current_time:%llu, leader_updated:%d",
                     i, formation_pos.sysid, current_time, pos_updated[formation_pos.sysid]);
        }
    }

#ifdef VIRTUALTEST
    ///使用home点作为起始点，产生向北以vel_sp速度前进的期望点
    matrix::Vector3f vel_sp(1.0f, 0.0f, 0.0f);
    if(_prev_time > 0.0f && _navigator->get_home_position()->valid_hpos && (hrt_elapsed_time(&_prev_time) > 2e5))   //已经赋值，经过一次循环
    {
        _virtual_target_movement += vel_sp * (current_time - _prev_time) * 1e-6;
        map_projection_init(&target_ref,  _navigator->get_home_position()->lat, _navigator->get_home_position()->lon);
        map_projection_reproject(&target_ref, _virtual_target_movement(0), _virtual_target_movement(1),
                     &_virtual_target.lat, &_virtual_target.lon);
        _virtual_target.alt = _navigator->get_home_position()->alt;
        _virtual_target.vx = vel_sp(0);
        _virtual_target.vy = vel_sp(1);
        _virtual_target.vz = vel_sp(2);
        _virtual_target.timestamp = current_time;
        /* lazily publish the position setpoint triplet only once available */
        if (_follow_target_pub != nullptr) {
            orb_publish(ORB_ID(follow_target), _follow_target_pub, &_virtual_target);

        } else {
            _follow_target_pub = orb_advertise(ORB_ID(follow_target), &_virtual_target);
        }
        pos_updated[LEADER_ID] = true;
        PX4_INFO("deltaMs:%d,vx_sp:%.2f,x:%.2f,step_vel:%.1f,current_vel(0):%.1f,lat:%.7f",
                 (int)((current_time - _prev_time)*1e-3), (double)vel_sp(0),
                 (double)_virtual_target_movement(0), (double)_step_vel(0), (double)_current_vel(0),_virtual_target.lat);
        _prev_time = current_time;
    }

#endif

    if(pos_updated[LEADER_ID])
    {
        _target_updates++;
        // get distance to target
        _previous_target_motion = _current_target_motion;
        _current_target_motion.lon = _formation_positions[LEADER_ID].lon;
        _current_target_motion.lat = _formation_positions[LEADER_ID].lat;
        _current_target_motion.alt = _formation_positions[LEADER_ID].alt;
        _current_target_motion.vx = _formation_positions[LEADER_ID].vel_n;
        _current_target_motion.vy = _formation_positions[LEADER_ID].vel_e;
        _current_target_motion.vz = _formation_positions[LEADER_ID].vel_d;
        _current_target_motion.timestamp = _formation_positions[LEADER_ID].timestamp;
#ifdef VIRTUALTEST
        _current_target_motion = _virtual_target;
#endif

    }else if (((current_time - _current_target_motion.timestamp) / 1000) > TARGET_TIMEOUT_MS && target_velocity_valid()) {
//        reset_target_validity();
    }
    // update distance to target

    if (target_position_valid()) {

        // get distance to target
        map_projection_init(&target_ref, _navigator->get_global_position()->lat, _navigator->get_global_position()->lon);
        map_projection_project(&target_ref, _current_target_motion.lat, _current_target_motion.lon, &_target_distance(0),
                               &_target_distance(1));
    }

    if (target_velocity_valid() && pos_updated[LEADER_ID])//因为第一次更新前一点时间戳为0，导致dt_ms很大，_step_vel无穷大
    {
        dt_ms = ((_current_target_motion.timestamp - _previous_target_motion.timestamp) / 1000);
        PX4_INFO("target_velocity_valid():%d,pos_updated[LEADER_ID]:%d,dt_ms:%.1f, _current_target_motion.timestamp:%llu",
                 target_velocity_valid(), pos_updated[LEADER_ID],(double)dt_ms, _current_target_motion.timestamp);
        if(dt_ms > 10.0f)
        {
            _est_target_vel(0) = _current_target_motion.vx;
            _est_target_vel(1) = _current_target_motion.vy;

            // if the target is moving add an offset and rotation
            //        if (_est_target_vel.length() > .5F) {
            //            _target_position_offset = _rot_matrix * _est_target_vel.normalized() * _follow_offset;
            //        }
            //        else
            {
                //目标移动速度小,则认为目标没有移动,队形以目标的航向为基准
                float yaw = _formation_positions[LEADER_ID].yaw;//target yaw
                matrix::Vector3f targ_yaw_vector(cos(yaw),sin(yaw), 0);//
                float rotate_angle = (float)pow(-1.0f, (double)(mavlink_system.sysid - 1)) * (float)(5 * M_PI / 6);//三角队形，偶数编号在左侧，奇数在右侧
                matrix::Euler<float> euler( 0,0,rotate_angle);
                matrix::Dcmf rot_matrix(euler);

                //三角形编队_follow_offset * (mavlink_system.sysid / 2)：使用飞机编号计算跟踪距离
                _target_position_offset = rot_matrix * targ_yaw_vector * _follow_offset * (mavlink_system.sysid / 2);
            }

            // are we within the target acceptance radius?
            // give a buffer to exit/enter the radius to give the velocity controller
            // a chance to catch up

            _radius_exited = ((_target_position_offset + _target_distance).length() > (float) TARGET_ACCEPTANCE_RADIUS_M * 1.5f);
            _radius_entered = ((_target_position_offset + _target_distance).length() < (float) TARGET_ACCEPTANCE_RADIUS_M);

            // to keep the velocity increase/decrease smooth
            // calculate how many velocity increments/decrements
            // it will take to reach the targets velocity
            // with the given amount of steps also add a feed forward input that adjusts the
            // velocity as the position gap increases since
            // just traveling at the exact velocity of the target will not
            // get any closer or farther from the target

            _step_vel = (_est_target_vel - _current_vel) + (_target_position_offset + _target_distance) * FF_K;
            _interpolation_points = dt_ms / 50;//跟踪线程执行周期50ms,得到两次更新时间间隔需要插值个数
            _interpolation_points = _interpolation_points < 1 ? 1 : _interpolation_points;
            _step_vel /= (/*dt_ms / 1000.0F **/ (float)INTERPOLATION_PNTS);// _interpolation_points);
            _step_time_in_ms = (dt_ms / (float)INTERPOLATION_PNTS);
            PX4_INFO("_step_vel(0):%.1f,_interpolation_points:%d,dt_ms:%.3f",
                     (double)_step_vel(0), _interpolation_points, (double)dt_ms);

//            // if we are less than 1 meter from the target don't worry about trying to yaw
//            // lock the yaw until we are at a distance that makes sense

//            if ((_target_distance + _target_position_offset).length() > 1.0F) {

//                // yaw rate smoothing

//                // this really needs to control the yaw rate directly in the attitude pid controller
//                // but seems to work ok for now since the yaw rate cannot be controlled directly in auto mode

//                _yaw_angle = get_bearing_to_next_waypoint(_navigator->get_global_position()->lat,
//                                                          _navigator->get_global_position()->lon,
//                                                          target_motion_with_offset.lat,
//                                                          target_motion_with_offset.lon);
//                //测试:使用领航机的航向角作为偏置方向的标准,测试通过后清屏蔽此行代码
//                _yaw_angle = _formation_positions[LEADER_ID].yaw;
//                _yaw_rate = wrap_pi((_yaw_angle - _navigator->get_global_position()->yaw) / (dt_ms / 1000.0f));

//            } else {
//                _yaw_angle = _yaw_rate = NAN;
//            }

        }


//        // clamp yaw rate smoothing if we are with in
//        // 3 degrees of facing target

//        if (PX4_ISFINITE(_yaw_rate)) {
//            if (fabsf(fabsf(_yaw_angle) - fabsf(_navigator->get_global_position()->yaw)) < math::radians(3.0F)) {
//                _yaw_rate = NAN;
//            }
//        }
    }

    if (target_position_valid()) {
        // get the target position using the calculated offset
        map_projection_init(&target_ref,  _current_target_motion.lat, _current_target_motion.lon);
        map_projection_reproject(&target_ref, _target_position_offset(0), _target_position_offset(1),
                                 &target_motion_with_offset.lat, &target_motion_with_offset.lon);
        target_motion_with_offset.alt = _current_target_motion.alt;

        // if we are less than 1 meter from the target don't worry about trying to yaw
        // lock the yaw until we are at a distance that makes sense

        if ((_target_distance + _target_position_offset).length() > 1.0F) {

            // yaw rate smoothing

            // this really needs to control the yaw rate directly in the attitude pid controller
            // but seems to work ok for now since the yaw rate cannot be controlled directly in auto mode

            _yaw_angle = get_bearing_to_next_waypoint(_navigator->get_global_position()->lat,
                                                      _navigator->get_global_position()->lon,
                                                      target_motion_with_offset.lat,
                                                      target_motion_with_offset.lon);
            //测试:使用领航机的航向角作为偏置方向的标准,测试通过后清屏蔽此行代码
            _yaw_angle = _formation_positions[LEADER_ID].yaw;
            _yaw_rate = wrap_pi((_yaw_angle - _navigator->get_global_position()->yaw) / (dt_ms / 1000.0f));

        } else {
            _yaw_angle = _yaw_rate = NAN;
            _yaw_angle = _formation_positions[LEADER_ID].yaw;
        }
        // clamp yaw rate smoothing if we are with in
        // 3 degrees of facing target
        if (PX4_ISFINITE(_yaw_rate)) {
            if (fabsf(fabsf(_yaw_angle) - fabsf(_navigator->get_global_position()->yaw)) < math::radians(3.0F)) {
                _yaw_rate = NAN;
            }
        }
        if(hrt_elapsed_time(&_last_time) > 1e6) //1s
        {
            mavlink_log_info(_navigator->get_mavlink_log_pub(), "vel:%.1f,%.2f,%d,dist:%.1f,vel:%.1f,"
                                                                "dalt:%.1f,sta:%d",
                             (double)_current_vel(1), (double)_step_vel(1),_interpolation_points,
                             (double)(_target_position_offset + _target_distance).length(),
                             (double)_est_target_vel(1),
                             (double)(_current_target_motion.alt - _navigator->get_global_position()->alt),
                             _follow_target_state);
            _time_updated = true;
        }
        // update state machine
        switch (_follow_target_state) {

        case TRACK_POSITION: {

                if (_radius_entered == true) {
                    _follow_target_state = TRACK_VELOCITY;

                } else if (target_velocity_valid()) {
                    set_follow_target_item(&_mission_item, _param_min_alt.get(), target_motion_with_offset, _yaw_angle);
                    // keep the current velocity updated with the target velocity for when it's needed
                    _current_vel = _est_target_vel;

                    update_position_sp(true, true, _yaw_rate);

                } else {
                    _follow_target_state = SET_WAIT_FOR_TARGET_POSITION;
                }

                break;
            }

        case TRACK_VELOCITY: {

                if (_radius_exited == true) {
                    _follow_target_state = TRACK_POSITION;

                } else if (target_velocity_valid()) {

                    if ((float)(current_time - _last_update_time) / 1000.0f >= _step_time_in_ms) {
                        _current_vel += _step_vel;
                        _last_update_time = current_time;
                    }

                    set_follow_target_item(&_mission_item, _param_min_alt.get(), target_motion_with_offset, _yaw_angle);

                    update_position_sp(true, false, _yaw_rate);

                } else {
                    _follow_target_state = SET_WAIT_FOR_TARGET_POSITION;
                }

                break;
            }

        case SET_WAIT_FOR_TARGET_POSITION: {

                // Climb to the minimum altitude
                // and wait until a position is received

                follow_target_s target = {};

                // for now set the target at the minimum height above the uav

                target.lat = _navigator->get_global_position()->lat;
                target.lon = _navigator->get_global_position()->lon;
    //			set_follow_target_item(&_mission_item, _param_min_alt.get(), target, _yaw_angle);

                //设置期望高度为当前高度(高度偏移min_clearance量为0)
                target.alt = _navigator->get_global_position()->alt;
                set_follow_target_item(&_mission_item, 0, target, _yaw_angle);

                update_position_sp(false, false, _yaw_rate);

                _follow_target_state = WAIT_FOR_TARGET_POSITION;
            }

        /* FALLTHROUGH */

        case WAIT_FOR_TARGET_POSITION: {

                if (is_mission_item_reached() && target_velocity_valid()) {
                    _target_position_offset(0) = _follow_offset;
                    _follow_target_state = TRACK_POSITION;
                }

                break;
            }
        }
    }
//    warnx(" _step_vel x %3.6f y %3.6f cur vel %3.6f %3.6f tar vel %3.6f %3.6f dist = %3.6f (%3.6f) mode = %d yaw rate = %3.6f",
//          (double) _step_vel(0),
//          (double) _step_vel(1),
//          (double) _current_vel(0),
//          (double) _current_vel(1),
//          (double) _est_target_vel(0),
//          (double) _est_target_vel(1),
//          (double) (_target_distance).length(),
//          (double) (_target_position_offset + _target_distance).length(),
//          _follow_target_state,
//          (double) _yaw_rate);


    ///
#endif
//	// update state machine
//	switch (_follow_target_state) {

//	case TRACK_POSITION: {

//			if (_radius_entered == true) {
//				_follow_target_state = TRACK_VELOCITY;

//			} else if (target_velocity_valid()) {
//                set_follow_target_item(&_mission_item, _param_min_alt.get(), target_motion_with_offset, _yaw_angle);
//				// keep the current velocity updated with the target velocity for when it's needed
//				_current_vel = _est_target_vel;

//				update_position_sp(true, true, _yaw_rate);

//			} else {
//				_follow_target_state = SET_WAIT_FOR_TARGET_POSITION;
//			}

//			break;
//		}

//	case TRACK_VELOCITY: {

//			if (_radius_exited == true) {
//				_follow_target_state = TRACK_POSITION;

//			} else if (target_velocity_valid()) {

//				if ((float)(current_time - _last_update_time) / 1000.0f >= _step_time_in_ms) {
//                    _current_vel += _step_vel;
//					_last_update_time = current_time;
//				}

//                set_follow_target_item(&_mission_item, _param_min_alt.get(), target_motion_with_offset, _yaw_angle);

//				update_position_sp(true, false, _yaw_rate);

//			} else {
//				_follow_target_state = SET_WAIT_FOR_TARGET_POSITION;
//			}

//			break;
//		}

//	case SET_WAIT_FOR_TARGET_POSITION: {

//			// Climb to the minimum altitude
//			// and wait until a position is received

//			follow_target_s target = {};

//			// for now set the target at the minimum height above the uav

//			target.lat = _navigator->get_global_position()->lat;
//			target.lon = _navigator->get_global_position()->lon;
////			set_follow_target_item(&_mission_item, _param_min_alt.get(), target, _yaw_angle);

//            //设置期望高度为当前高度(高度偏移min_clearance量为0)
//            target.alt = _navigator->get_global_position()->alt;
//            set_follow_target_item(&_mission_item, 0, target, _yaw_angle);

//            update_position_sp(false, false, _yaw_rate);

//			_follow_target_state = WAIT_FOR_TARGET_POSITION;
//		}

//	/* FALLTHROUGH */

//	case WAIT_FOR_TARGET_POSITION: {

//			if (is_mission_item_reached() && target_velocity_valid()) {
//				_target_position_offset(0) = _follow_offset;
//				_follow_target_state = TRACK_POSITION;
//			}

//			break;
//		}
//	}
}

void FollowTarget::update_position_sp(bool use_velocity, bool use_position, float yaw_rate)
{
	// convert mission item to current setpoint

	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	// activate line following in pos control if position is valid

	pos_sp_triplet->previous.valid = use_position;
	pos_sp_triplet->previous = pos_sp_triplet->current;
	mission_apply_limitation(_mission_item);
	mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET;
	pos_sp_triplet->current.position_valid = use_position;
	pos_sp_triplet->current.velocity_valid = use_velocity;
	pos_sp_triplet->current.vx = _current_vel(0);
	pos_sp_triplet->current.vy = _current_vel(1);
	pos_sp_triplet->next.valid = false;
	pos_sp_triplet->current.yawspeed_valid = PX4_ISFINITE(yaw_rate);
	pos_sp_triplet->current.yawspeed = yaw_rate;
	_navigator->set_position_setpoint_triplet_updated();
}

void FollowTarget::reset_target_validity()
{
	_yaw_rate = NAN;
	_previous_target_motion = {};
	_current_target_motion = {};
	_target_updates = 0;
	_current_vel.zero();
	_step_vel.zero();
	_est_target_vel.zero();
	_target_distance.zero();
	_target_position_offset.zero();
	reset_mission_item_reached();
	_follow_target_state = SET_WAIT_FOR_TARGET_POSITION;
#ifdef VIRTUALTEST
    _virtual_target_movement.zero();
#endif
}

bool FollowTarget::target_velocity_valid()
{
	// need at least 2 continuous data points for velocity estimate
	return (_target_updates >= 2);
}

bool FollowTarget::target_position_valid()
{
	// need at least 1 continuous data points for position estimate
	return (_target_updates >= 1);
}

void
FollowTarget::set_follow_target_item(struct mission_item_s *item, float min_clearance, follow_target_s &target,
				     float yaw)
{
	if (_navigator->get_land_detected()->landed) {
		/* landed, don't takeoff, but switch to IDLE mode */
		item->nav_cmd = NAV_CMD_IDLE;

	} else {

		item->nav_cmd = NAV_CMD_DO_FOLLOW_REPOSITION;

		/* use current target position */
		item->lat = target.lat;
		item->lon = target.lon;
#ifndef FORMATION
		item->altitude = _navigator->get_home_position()->alt;
		if (min_clearance > 8.0f) {
			item->altitude += min_clearance;

		} else {
			item->altitude += 8.0f; // if min clearance is bad set it to 8.0 meters (well above the average height of a person)
		}
#else
        item->altitude = target.alt + min_clearance;
#endif
	}
	item->altitude_is_relative = false;
	item->yaw = yaw;
	item->loiter_radius = _navigator->get_loiter_radius();
	item->acceptance_radius = _navigator->get_acceptance_radius();
	item->time_inside = 0.0f;
	item->autocontinue = false;
	item->origin = ORIGIN_ONBOARD;
}
