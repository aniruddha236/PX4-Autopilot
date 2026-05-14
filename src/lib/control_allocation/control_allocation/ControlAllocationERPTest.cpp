/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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

#include <gtest/gtest.h>

#include "ControlAllocationERP.hpp"

using namespace matrix;

namespace
{
constexpr int NUM_MOTORS = 8;
constexpr float BETA = 0.54f;
constexpr float CLUB = 0.70710678118f; // sqrt(2) / 2
constexpr float THRUST_MAX = 14.52f;
constexpr float TORQUE_MAX = 0.2016f;
constexpr float COMMAND_MIN = 0.0f;
constexpr float COMMAND_MAX = 1.f;
constexpr float COMMAND_TRIM = 0.75f;
}

class ControlAllocationERPTestGenericOctocopter : public ::testing::Test
{
public:
	ControlAllocationERP _control_allocation;

	void SetUp() override
	{
		ControlAllocation::ActuatorVector actuator_min;
		ControlAllocation::ActuatorVector actuator_max;
		ControlAllocation::ActuatorVector actuator_trim;
		ControlAllocation::ActuatorVector linearization_point;
		Matrix<float, ControlAllocation::NUM_AXES, ControlAllocation::NUM_ACTUATORS> effectiveness;

		actuator_min.setAll(0.f);
		actuator_max.setAll(0.f);
		actuator_trim.setAll(0.f);
		linearization_point.setAll(0.f);
		effectiveness.setAll(0.f);

		const float roll[NUM_MOTORS] = {
			-CLUB, -1.f, -CLUB, 0.f, CLUB, 1.f, CLUB, 0.f
		};

		const float pitch[NUM_MOTORS] = {
			CLUB, 0.f, -CLUB, -1.f, -CLUB, 0.f, CLUB, 1.f
		};

		const float direction[NUM_MOTORS] = {
			-1.f, 1.f, -1.f, 1.f, -1.f, 1.f, -1.f, 1.f
		};

		for (int i = 0; i < NUM_MOTORS; ++i) {
			effectiveness(ControlAllocation::ROLL, i) = roll[i] * THRUST_MAX * BETA;
			effectiveness(ControlAllocation::PITCH, i) = pitch[i] * THRUST_MAX * BETA;
			effectiveness(ControlAllocation::YAW, i) = direction[i] * TORQUE_MAX;
			effectiveness(ControlAllocation::THRUST_Z, i) = -THRUST_MAX;

			actuator_min(i) = COMMAND_MIN;
			actuator_max(i) = COMMAND_MAX;
			linearization_point(i) = COMMAND_TRIM;
		}

		_control_allocation.setEffectivenessMatrix(effectiveness, actuator_trim, linearization_point,
				NUM_MOTORS, false);
		_control_allocation.setActuatorMin(actuator_min);
		_control_allocation.setActuatorMax(actuator_max);
	}

	void allocate(const float roll, const float pitch, const float yaw, const float thrust_z)
	{
		Vector<float, ControlAllocation::NUM_AXES> control_setpoint;
		control_setpoint(ControlAllocation::ROLL) = roll;
		control_setpoint(ControlAllocation::PITCH) = pitch;
		control_setpoint(ControlAllocation::YAW) = yaw;
		control_setpoint(ControlAllocation::THRUST_Z) = thrust_z;

		_control_allocation.setControlSetpoint(control_setpoint);
		_control_allocation.allocate();
	}

	void expectMotorOutputsWithinBounds() const
	{
		const ControlAllocation::ActuatorVector &actuator_sp = _control_allocation.getActuatorSetpoint();

		for (int i = 0; i < NUM_MOTORS; ++i) {
			EXPECT_GE(actuator_sp(i), COMMAND_MIN - 1e-4f);
			EXPECT_LE(actuator_sp(i), COMMAND_MAX + 1e-4f);
		}
	}
};

TEST_F(ControlAllocationERPTestGenericOctocopter, ZeroIncrementReturnsTrim)
{
	allocate(0.f, 0.f, 0.f, -6.f * THRUST_MAX);

	const ControlAllocation::ActuatorVector &actuator_sp = _control_allocation.getActuatorSetpoint();

	for (int i = 0; i < NUM_MOTORS; ++i) {
		EXPECT_NEAR(actuator_sp(i), COMMAND_TRIM, 1e-4f);
	}
}

TEST_F(ControlAllocationERPTestGenericOctocopter, FeasibleCollectiveThrustIsDistributedEqually)
{
	constexpr float requested_thrust_z = -8.f;
	constexpr float expected_increment = -requested_thrust_z / (NUM_MOTORS * THRUST_MAX);

	allocate(0.f, 0.f, 0.f, -6.f * THRUST_MAX + requested_thrust_z);

	const ControlAllocation::ActuatorVector &actuator_sp = _control_allocation.getActuatorSetpoint();

	for (int i = 0; i < NUM_MOTORS; ++i) {
		EXPECT_NEAR(actuator_sp(i), COMMAND_TRIM + expected_increment, 1e-4f);
	}
}

TEST_F(ControlAllocationERPTestGenericOctocopter, InfeasibleMixedDemandStaysWithinBounds)
{
	allocate(20.f, 10.f, 1.f, -80.f);

	expectMotorOutputsWithinBounds();
}

TEST_F(ControlAllocationERPTestGenericOctocopter, InfeasibleMixedDemandPreservesDirection)
{
	constexpr float requested_roll = 1.0f * THRUST_MAX * BETA;
	constexpr float requested_pitch = 2.0f * THRUST_MAX * BETA;
	constexpr float trim_thrust_z = -6.0f * THRUST_MAX;

	allocate(requested_roll, requested_pitch, 0.f, trim_thrust_z);

	expectMotorOutputsWithinBounds();

	const Vector<float, ControlAllocation::NUM_AXES> allocated = _control_allocation.getAllocatedControl();
	const float roll_scale = allocated(ControlAllocation::ROLL) / requested_roll;
	const float pitch_scale = allocated(ControlAllocation::PITCH) / requested_pitch;

	EXPECT_NEAR(roll_scale, pitch_scale, 2e-2f);
	EXPECT_NEAR(allocated(ControlAllocation::THRUST_Z), 0.f, 2e-2f);
}
