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
ControlAllocationERP::allocate()
{
	_prev_actuator_sp = _actuator_sp;

	ActuatorVector u = _actuator_trim;
	ActuatorVector u_delta;

	matrix::Vector<float, ERP_NUM_AXES> gamma_delta_erp;

	for (int axis = 0; axis < ERP_NUM_AXES; ++axis) {
		const int px4_axis = erpAxis(axis);
		gamma_delta_erp(axis) = _control_sp(px4_axis) - _control_trim(px4_axis);
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
				effectiveness_active(axis_idx, actuator_idx) =
					_effectiveness(erpAxis(axis_idx), actuator_idx);
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
