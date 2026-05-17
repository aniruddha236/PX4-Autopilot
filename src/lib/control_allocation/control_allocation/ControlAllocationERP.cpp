#include "ControlAllocationERP.hpp"

#include <matrix/matrix/PseudoInverse.hpp>

int
ControlAllocationERP::rankOfEffectiveness(
	const matrix::Matrix<float, ERP_NUM_AXES, NUM_ACTUATORS> &effectiveness) const
{
	size_t rank = 0;
	const matrix::SquareMatrix<float, ERP_NUM_AXES> gram = effectiveness * effectiveness.transpose();
	matrix::fullRankCholesky(gram, rank);
	return static_cast<int>(rank);
}

void
ControlAllocationERP::setEffectivenessMatrix(
	const matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &effectiveness,
	const ActuatorVector &actuator_trim,
	const ActuatorVector &linearization_point,
	int num_actuators,
	bool update_normalization_scale)
{
	ControlAllocation::setEffectivenessMatrix(
		effectiveness,
		actuator_trim,
		linearization_point,
		num_actuators,
		update_normalization_scale);

	if (update_normalization_scale && !_had_actuator_failure) {
		updateControlAllocationMatrixScale();
	}
}

void
ControlAllocationERP::updateControlAllocationMatrixScale()
{
	matrix::Matrix<float, NUM_ACTUATORS, NUM_AXES> mix;
	matrix::geninv(_effectiveness, mix);

	// Same scale on roll and pitch
	if (_normalize_rpy) {

		int num_non_zero_roll_torque = 0;
		int num_non_zero_pitch_torque = 0;

		for (int i = 0; i < _num_actuators; i++) {

			if (fabsf(mix(i, 0)) > 1e-3f) {
				++num_non_zero_roll_torque;
			}

			if (fabsf(mix(i, 1)) > 1e-3f) {
				++num_non_zero_pitch_torque;
			}
		}

		float roll_norm_scale = 1.f;

		if (num_non_zero_roll_torque > 0) {
			roll_norm_scale = sqrtf(mix.col(0).norm_squared() / (num_non_zero_roll_torque / 2.f));
		}

		float pitch_norm_scale = 1.f;

		if (num_non_zero_pitch_torque > 0) {
			pitch_norm_scale = sqrtf(mix.col(1).norm_squared() / (num_non_zero_pitch_torque / 2.f));
		}

		_control_allocation_scale(0) = fmaxf(roll_norm_scale, pitch_norm_scale);
		_control_allocation_scale(1) = _control_allocation_scale(0);

		// Scale yaw separately
		_control_allocation_scale(2) = mix.col(2).max();

	} else {
		_control_allocation_scale(0) = 1.f;
		_control_allocation_scale(1) = 1.f;
		_control_allocation_scale(2) = 1.f;
	}

	// Scale thrust by the sum of the individual thrust axes, and use the scaling for the Z axis if there's no actuators
	// (for tilted actuators)
	_control_allocation_scale(THRUST_Z) = 1.f;

	for (int axis_idx = 2; axis_idx >= 0; --axis_idx) {
		int num_non_zero_thrust = 0;
		float norm_sum = 0.f;

		for (int i = 0; i < _num_actuators; i++) {
			float norm = fabsf(mix(i, 3 + axis_idx));
			norm_sum += norm;

			if (norm > FLT_EPSILON) {
				++num_non_zero_thrust;
			}
		}

		if (num_non_zero_thrust > 0) {
			_control_allocation_scale(3 + axis_idx) = norm_sum / num_non_zero_thrust;

		} else {
			_control_allocation_scale(3 + axis_idx) = _control_allocation_scale(THRUST_Z);
		}
	}
}

void
ControlAllocationERP::allocate()
{
	_prev_actuator_sp = _actuator_sp;

	ActuatorVector u = _actuator_trim;
	ActuatorVector u_delta;

	matrix::Vector<float, ERP_NUM_AXES> gamma_delta_erp;
	matrix::Vector<float, ERP_NUM_AXES> gamma_trim_erp;

	for (int axis = 0; axis < ERP_NUM_AXES; ++axis) {
		const int px4_axis = erpAxis(axis);
		gamma_trim_erp(axis) = _control_allocation_scale(px4_axis) * _control_trim(px4_axis);
		gamma_delta_erp(axis) = _control_sp(px4_axis) - gamma_trim_erp(axis);
	}

	float c_erp = 0.0f;

	bool active[NUM_ACTUATORS] {};

	for (int i = 0; i < _num_actuators; ++i) {
		active[i] = (_actuator_min(i) <= _actuator_max(i));
	}

	for (int k = 0; k < _num_actuators; ++k) {
		// Rebuild the active-set effectiveness matrix for this ERP iteration.
		matrix::Matrix<float, ERP_NUM_AXES, NUM_ACTUATORS> effectiveness_active;

		int num_active = 0;

		for (int actuator_idx = 0; actuator_idx < _num_actuators; ++actuator_idx) {
			if (!active[actuator_idx]) {
				continue;
			}

			for (int axis_idx = 0; axis_idx < ERP_NUM_AXES; ++axis_idx) {
				const int px4_axis = erpAxis(axis_idx);
				effectiveness_active(axis_idx, actuator_idx) =
					_control_allocation_scale(px4_axis) * _effectiveness(px4_axis, actuator_idx);
			}

			++num_active;
		}

		if (num_active == 0) {
			break;
		}

		const int active_rank = rankOfEffectiveness(effectiveness_active);

		if (active_rank < ERP_NUM_AXES) {
			break;
		}

		// This is A_s^+ in the ERP algorithm.
		matrix::Matrix<float, NUM_ACTUATORS, ERP_NUM_AXES> effectiveness_active_pinv;

		if (!matrix::geninv(effectiveness_active, effectiveness_active_pinv)) {
			break;
		}

		u_delta = effectiveness_active_pinv * gamma_delta_erp;

		// Force inactive actuators to stay fixed.
		for (int actuator_idx = 0; actuator_idx < _num_actuators; ++actuator_idx) {
			if (!active[actuator_idx]) {
				u_delta(actuator_idx) = 0.f;
			}
		}

		float d_min = FLT_MAX;
		int saturated_idx = -1;

		for (int actuator_idx = 0; actuator_idx < _num_actuators; ++actuator_idx) {
			if (!active[actuator_idx]) {
				continue;
			}

			float d_i = FLT_MAX;

			if (u_delta(actuator_idx) > FLT_EPSILON) {
				d_i = (_actuator_max(actuator_idx) - u(actuator_idx)) / u_delta(actuator_idx);

			} else if (u_delta(actuator_idx) < -FLT_EPSILON) {
				d_i = (_actuator_min(actuator_idx) - u(actuator_idx)) / u_delta(actuator_idx);
			}

			if (d_i >= 0.f && d_i < d_min) {
				d_min = d_i;
				saturated_idx = actuator_idx;
			}
		}

		if (saturated_idx < 0) {
			break;
		}

		// Full remaining command is feasible.
		if (d_min >= 1.f - c_erp) {
			const float d = 1.f - c_erp;
			u += d * u_delta;
			c_erp = 1.f;
			break;
		}

		// Move until the first actuator saturates.
		u += d_min * u_delta;
		c_erp += d_min;

		if (u_delta(saturated_idx) > 0.f) {
			u(saturated_idx) = _actuator_max(saturated_idx);

		} else {
			u(saturated_idx) = _actuator_min(saturated_idx);
		}

		active[saturated_idx] = false;
	}

	_actuator_sp = u;
	clipActuatorSetpoint();
}
