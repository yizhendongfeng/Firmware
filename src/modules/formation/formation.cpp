/****************************************************************************
 *
 *   Copyright (c) 2013 - 2017 PX4 Development Team. All rights reserved.
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
 * @file Formation.hpp
 * Formation controller
 * 根据不同队形和领航飞机位置速度来计算当前飞机的期望位置，并发送给位置控制模块
 * @author 张纪敏 <869159813@qq.com>
 */


#include "formation.hpp"
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_module_params.h>
#include <px4_tasks.h>
#include <px4_posix.h>

#include <lib/ecl/geo/geo.h>

#include <drivers/drv_hrt.h>

#include <float.h>
#include <string.h>



extern "C" __EXPORT int formation_main(int argc, char *argv[]);
using matrix::wrap_pi;

namespace formation_control
{
Formation	*g_formation;
}


Formation::Formation()
{
    _current_vel.zero();
    _step_vel.zero();
    _target_vel.zero();
    _target_distance.zero();
    _target_position_offset.zero();
    _target_position_delta.zero();
}

Formation::~Formation()
{
    if (_control_task != -1) {

        /* task wakes up every 100ms or so at the longest */
        _task_should_exit = true;

        /* wait for a second for the task to quit at our request */
        unsigned i = 0;

        do {
            /* wait 20ms */
            usleep(20000);

            /* if we have given up, kill it */
            if (++i > 50) {
                px4_task_delete(_control_task);
                break;
            }
        } while (_control_task != -1);
    }
}


void
Formation::task_main()
{
    if(_current_vehicle_pos_pub < 0)
        _current_vehicle_pos_pub = orb_subscribe(ORB_ID(vehicle_global_position));
    memset(&_current_global_position, 0, sizeof(_current_global_position));
    //订阅其他飞机位置信息        ***zjm
    for(int i = 0; i < 4; i++)
    {
        if(_formation_pos_sub[i] < 0)
            _formation_pos_sub[i]= orb_subscribe_multi(ORB_ID(formation_position), i);
        memset(&_formation_positions[i], 0, sizeof(_formation_positions[i]));
    }

//    int formation_sub = orb_subscribe_multi(ORB_ID(formation_position), 0);
//    vehicle_global_position_s global_pos_test = {};
    while (!_task_should_exit)
    {
        usleep(10000);
        bool _radius_entered = false;
        bool _radius_exited = false;
        float dt_ms = 0;
        struct map_projection_reference_s target_ref;
        follow_target_s target_motion_with_offset = {};
        //        uint64_t current_time = hrt_absolute_time();


//        //更新本机位置
//        bool current_vehicle_pos_updated = false;
//        orb_check(_current_vehicle_pos_pub, &current_vehicle_pos_updated);
//        if(current_vehicle_pos_updated)
//        {
//            orb_copy(ORB_ID(vehicle_global_position), _current_vehicle_pos_pub, &_current_global_position);
//            _formation_positions[current_sysid].timestamp = _current_global_position.timestamp;
//            _formation_positions[current_sysid].lon = _current_global_position.lon;
//            _formation_positions[current_sysid].lat = _current_global_position.lat;
//            _formation_positions[current_sysid].alt = _current_global_position.alt;
//            _formation_positions[current_sysid].vel_n = _current_global_position.vel_n;
//            _formation_positions[current_sysid].vel_e = _current_global_position.vel_e;
//            _formation_positions[current_sysid].vel_d = _current_global_position.vel_d;
//            _formation_positions[current_sysid].sysid = _current_global_position.sysid;
//            PX4_INFO("current_sysid：%d,_current_global_position.alt:%.3f", current_sysid, (double)_current_global_position.alt);
//        }

        ///更新目标位置
        formation_position_s global_pos;
        bool pos_updated[4] = {false, false, false, false};
        for(int i = 0; i < 4; i++)
        {
            orb_check(_formation_pos_sub[i], &pos_updated[i]);
            if(pos_updated[i])
            {
                orb_copy(ORB_ID(formation_position), _formation_pos_sub[i], &global_pos);
                //                memcpy(&global_pos, &_formation_positions[global_pos.sysid], sizeof(global_pos));
                _formation_positions[i].timestamp = global_pos.timestamp;
                _formation_positions[i].lon = global_pos.lon;
                _formation_positions[i].lat = global_pos.lat;
                _formation_positions[i].alt = global_pos.alt;
                _formation_positions[i].vel_n = global_pos.vel_n;
                _formation_positions[i].vel_e = global_pos.vel_e;
                _formation_positions[i].vel_d = global_pos.vel_d;
                _formation_positions[i].sysid = global_pos.sysid;
                PX4_INFO("i:%d,receive sysid:%d pos", i, global_pos.sysid);
            }
        }

        //*领飞飞机编号为leader_sysid，本机编号为mavlink_system.sysid*//
        //根据不同队形计算本机期望位置

        // update distance to target

        if (_formation_positions[mavlink_system.sysid].timestamp > 0) {    //已收到本机位置信息
            // get distance to target
            map_projection_init(&target_ref, _formation_positions[mavlink_system.sysid].lat, _formation_positions[mavlink_system.sysid].lon);
            map_projection_project(&target_ref, _formation_positions[leader_sysid].lat, _formation_positions[leader_sysid].lon,
                                   &_target_distance(0), &_target_distance(1));
        }

        // update target velocity
        if (pos_updated[leader_sysid]) {


            // 使用原有速度变量接收目标速度
            _target_vel(0) = _formation_positions[leader_sysid].vel_n;
            _target_vel(1) = _formation_positions[leader_sysid].vel_e;
            _target_vel(2) = _formation_positions[leader_sysid].vel_d;
            // if the target is moving add an offset and rotation
            if (_target_vel.length() > .5F) {
                _target_position_offset = _rot_matrix * _target_vel.normalized() * _follow_offset;
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

            _step_vel = (_target_vel - _current_vel) + (_target_position_offset + _target_distance) * FF_K;
            _step_vel /= (dt_ms / 1000.0F * (float) INTERPOLATION_PNTS);
            _step_time_in_ms = (dt_ms / (float) INTERPOLATION_PNTS);

            // if we are less than 1 meter from the target don't worry about trying to yaw
            // lock the yaw until we are at a distance that makes sense

            if ((_target_distance).length() > 1.0F) {

                // yaw rate smoothing

                // this really needs to control the yaw rate directly in the attitude pid controller
                // but seems to work ok for now since the yaw rate cannot be controlled directly in auto mode

                _yaw_angle = get_bearing_to_next_waypoint(_formation_positions[mavlink_system.sysid].lat,
                        _formation_positions[mavlink_system.sysid].lon,
                        _current_target_motion.lat,
                        _current_target_motion.lon);

                _yaw_rate = wrap_pi((_yaw_angle - _formation_positions[mavlink_system.sysid].yaw) / (dt_ms / 1000.0f));

            } else {
                _yaw_angle = _yaw_rate = NAN;
            }


            //		warnx(" _step_vel x %3.6f y %3.6f cur vel %3.6f %3.6f tar vel %3.6f %3.6f dist = %3.6f (%3.6f) mode = %d yaw rate = %3.6f",
            //				(double) _step_vel(0),
            //				(double) _step_vel(1),
            //				(double) _current_vel(0),
            //				(double) _current_vel(1),
            //				(double) _target_vel(0),
            //				(double) _target_vel(1),
            //				(double) (_target_distance).length(),
            //				(double) (_target_position_offset + _target_distance).length(),
            //				_follow_target_state,
            //				(double) _yaw_rate);
        }

        if (pos_updated[leader_sysid]) {

            // get the target position using the calculated offset

            map_projection_init(&target_ref,  _current_target_motion.lat, _current_target_motion.lon);
            map_projection_reproject(&target_ref, _target_position_offset(0), _target_position_offset(1),
                                     &target_motion_with_offset.lat, &target_motion_with_offset.lon);
        }

        // clamp yaw rate smoothing if we are with in
        // 3 degrees of facing target

        //        if (PX4_ISFINITE(_yaw_rate)) {
        //            if (fabsf(fabsf(_yaw_angle) - fabsf(_navigator->get_global_position()->yaw)) < math::radians(3.0F)) {
        //                _yaw_rate = NAN;
        //            }
        //        }
        ///


        if (_radius_entered == true) {}
        else if (_radius_exited == true) {}
    }

}


int
Formation::start()
{
    /* start the task */
    _control_task = px4_task_spawn_cmd("formation",
                                       SCHED_DEFAULT,
                                       SCHED_PRIORITY_DEFAULT,
                                       1900,
                                       (px4_main_t)&Formation::task_main_trampoline,
                                       nullptr);

    if (_control_task < 0) {
        warn("task start failed");
        return -errno;
    }

    return OK;
}
void
Formation::task_main_trampoline(int argc, char *argv[])
{
    formation_control::g_formation->task_main();
}
int formation_main(int argc, char *argv[])
{
    if (argc < 2) {
        warnx("usage: formation {start|stop|status}");
        return 1;
    }

    if (!strcmp(argv[1], "start")) {

        if (formation_control::g_formation != nullptr) {
            warnx("already running");
            return 1;
        }

        formation_control::g_formation = new Formation;

        if (formation_control::g_formation == nullptr) {
            warnx("alloc failed");
            return 1;
        }

        if (OK != formation_control::g_formation->start()) {
            delete formation_control::g_formation;
            formation_control::g_formation = nullptr;
            warnx("start failed");
            return 1;
        }

        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        if (formation_control::g_formation == nullptr) {
            warnx("not running");
            return 1;
        }

        delete formation_control::g_formation;
        formation_control::g_formation = nullptr;
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (formation_control::g_formation) {
            warnx("running");
            return 0;

        } else {
            warnx("not running");
            return 1;
        }
    }

    warnx("unrecognized command");
    return 1;
}
