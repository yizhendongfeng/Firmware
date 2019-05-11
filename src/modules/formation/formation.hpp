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
 * @file Formation.hpp
 * @author 张纪敏 <869159813@qq.com>
 */

#pragma once
//#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

#include <v2.0/mavlink_types.h>

#include <uORB/uORB.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/follow_target.h>
#include <uORB/topics/formation_position.h>

extern const mavlink_system_t mavlink_system;

class Formation
{
public:
    Formation();

    ~Formation();
    /**
     * Start task.
     *
     * @return		OK on success.
     */
    int		start();

private:
    static constexpr int TARGET_TIMEOUT_MS = 2500;
    static constexpr int TARGET_ACCEPTANCE_RADIUS_M = 5;
    static constexpr int INTERPOLATION_PNTS = 20;
    static constexpr float FF_K = .25F;
    static constexpr float OFFSET_M = 8;
    static const uint8_t leader_sysid{1};
    const uint8_t current_sysid = mavlink_system.sysid;

    int		_control_task{-1};			/**< task handle for task */
    bool	_task_should_exit{false};		/**< if true, task should exit */
    int		_formation_pos_sub[4] = {-1, -1, -1, -1};
    int     _current_vehicle_pos_pub = -1;
    float _follow_offset{OFFSET_M};

    formation_position_s _formation_positions[4];   //一条船，三架飞机，使用下标来获取船和其他飞机位置
    vehicle_global_position_s _current_global_position{};
    follow_target_s _current_target_motion{};
    follow_target_s _previous_target_motion{};

    matrix::Vector3f _current_vel;
    matrix::Vector3f _target_position_offset;
    matrix::Vector3f _target_position_delta;
    matrix::Vector3f _target_vel;
    matrix::Vector3f _target_distance;
    matrix::Vector3f _step_vel;
    matrix::Dcmf _rot_matrix;

    float _yaw_rate{0.0f};
    float _responsiveness{0.0f};
    float _yaw_angle{0.0f};
    float _step_time_in_ms{0.0f};
private:
    /**
     * Shim for calling task_main from task_create.
     */
    static void	task_main_trampoline(int argc, char *argv[]);

    /**
     * Main sensor collection task.
     */
    void		task_main();
};
