#pragma once

#include "ControlAllocation.hpp"

class ControlAllocationERP: public ControlAllocation
{
public:
	ControlAllocationERP() = default;
	virtual ~ControlAllocationERP() = default;

	void allocate() override;

	void setEffectivenessMatrix(const matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &effectiveness,
				    const ActuatorVector &actuator_trim, const ActuatorVector &linearization_point, int num_actuators,
				    bool update_normalization_scale) override;

private:
	static constexpr int ERP_NUM_AXES = 4;

	static constexpr int erpAxis(int axis)
	{
		return axis == 0 ? ControlAxis::ROLL :
		       axis == 1 ? ControlAxis::PITCH :
		       axis == 2 ? ControlAxis::YAW :
				   ControlAxis::THRUST_Z;
	}

	int rankOfEffectiveness(const matrix::Matrix<float, ERP_NUM_AXES, NUM_ACTUATORS> &effectiveness) const;

	void updateControlAllocationMatrixScale();

};
