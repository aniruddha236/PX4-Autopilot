/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/esc_status.h>

#include <drivers/drv_hrt.h>

#include <uavcan_stm32h7/can.hpp>

#include <cinttypes>
#include <cmath>

using namespace time_literals;

class EscCanDecoder : public ModuleBase, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	EscCanDecoder(uint32_t bitrate, uint32_t offset)
		: ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
		  _bitrate(bitrate),
		  _offset(offset)
	{
	}

	~EscCanDecoder() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	static int custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }
	static int run_trampoline(int argc, char *argv[]);
	static EscCanDecoder *instantiate(int argc, char *argv[]);
	int print_status() override;

	bool init();

private:
	static constexpr uint32_t TelemetryBase = 0x14A30000U;
	static constexpr uint32_t TelemetryPacket1 = 1U;
	static constexpr uint8_t RawCanIfaceIndex = 1U;

	uavcan_stm32h7::CanInitHelper<64> _can;
	uORB::Publication<esc_status_s> _esc_status_pub{ORB_ID(esc_status)};
	uORB::PublicationMulti<battery_status_s> _battery_status_pub{ORB_ID(battery_status)};

	uint32_t _bitrate{1000000};
	uint32_t _offset{0};
	uint16_t _frame_count{0};
	float _last_voltage_v{NAN};
	float _last_current_a{NAN};
	int32_t _last_rpm{0};
	hrt_abstime _last_update{0};

	static uint16_t le_u16(const uint8_t *data)
	{
		return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
	}

	static int16_t le_i16(const uint8_t *data)
	{
		return static_cast<int16_t>(le_u16(data));
	}

	static int32_t le_i32(const uint8_t *data)
	{
		return static_cast<int32_t>(static_cast<uint32_t>(data[0]) |
					    (static_cast<uint32_t>(data[1]) << 8) |
					    (static_cast<uint32_t>(data[2]) << 16) |
					    (static_cast<uint32_t>(data[3]) << 24));
	}

	uint32_t telemetry_packet_1_id() const { return TelemetryBase + _offset + TelemetryPacket1; }

	void publish_esc_status(hrt_abstime now);
	void publish_battery_status(hrt_abstime now);
	void process_frame(const uavcan::CanFrame &frame, hrt_abstime now);
	void Run() override;
};

ModuleBase::Descriptor EscCanDecoder::desc{task_spawn, custom_command, print_usage};

bool EscCanDecoder::init()
{
	const int ret = _can.driver.initRawIface(RawCanIfaceIndex, _bitrate);

	if (ret < 0) {
		PX4_ERR("CAN2 raw init failed (%d)", ret);
		return false;
	}

	ScheduleOnInterval(10_ms);
	return true;
}

void EscCanDecoder::publish_esc_status(hrt_abstime now)
{
	esc_status_s esc_status{};
	esc_status.timestamp = now;
	esc_status.counter = _frame_count;
	esc_status.esc_count = 1;
	esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_CAN;
	esc_status.esc_online_flags = 1;
	esc_status.esc_armed_flags = 1;
	esc_status.esc[0].timestamp = now;
	esc_status.esc[0].esc_errorcount = 0;
	esc_status.esc[0].esc_rpm = _last_rpm;
	esc_status.esc[0].esc_voltage = _last_voltage_v;
	esc_status.esc[0].esc_current = _last_current_a;
	esc_status.esc[0].esc_temperature = NAN;
	esc_status.esc[0].motor_temperature = INT16_MIN;
	esc_status.esc[0].esc_state = 0;
	esc_status.esc[0].actuator_function = esc_report_s::ACTUATOR_FUNCTION_MOTOR1;
	esc_status.esc[0].failures = 0;
	esc_status.esc[0].esc_power = -1;
	_esc_status_pub.publish(esc_status);
}

void EscCanDecoder::publish_battery_status(hrt_abstime now)
{
	battery_status_s battery_status{};
	battery_status.timestamp = now;
	battery_status.connected = true;
	battery_status.voltage_v = _last_voltage_v;
	battery_status.current_a = _last_current_a;
	battery_status.current_average_a = _last_current_a;
	battery_status.remaining = -1.f;
	battery_status.scale = 1.f;
	battery_status.temperature = NAN;
	battery_status.source = battery_status_s::SOURCE_ESCS;
	battery_status.priority = 0;
	battery_status.id = 1;
	battery_status.warning = battery_status_s::WARNING_NONE;
	_battery_status_pub.publish(battery_status);
}

void EscCanDecoder::process_frame(const uavcan::CanFrame &frame, hrt_abstime now)
{
	if (!frame.isExtended()) {
		return;
	}

	if ((frame.id & uavcan::CanFrame::MaskExtID) != telemetry_packet_1_id()) {
		return;
	}

	if (frame.dlc < 8) {
		return;
	}

	const uint16_t voltage_dv = le_u16(&frame.data[0]);
	const int16_t current_da = le_i16(&frame.data[2]);
	const int32_t rpm = le_i32(&frame.data[4]);

	_last_voltage_v = voltage_dv * 0.1f;
	_last_current_a = current_da * 0.1f;
	_last_rpm = rpm;
	_last_update = now;
	_frame_count++;

	publish_esc_status(now);
	publish_battery_status(now);
}

void EscCanDecoder::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	for (int i = 0; i < 16; ++i) {
		uavcan::CanFrame frame;
		uavcan::MonotonicTime mono;
		uavcan::UtcTime utc;
		uavcan::CanIOFlags flags = 0;
		const int ret = _can.driver.rawReceive(RawCanIfaceIndex, frame, mono, utc, flags);

		if (ret > 0) {
			process_frame(frame, hrt_absolute_time());

		} else if (ret == 0) {
			break;

		} else {
			PX4_ERR("raw receive failed (%d)", ret);
			break;
		}
	}
}

int EscCanDecoder::task_spawn(int argc, char *argv[])
{
	EscCanDecoder *instance = instantiate(argc, argv);

	if (instance == nullptr) {
		return PX4_ERROR;
	}

	desc.object.store(instance);
	desc.task_id = task_id_is_work_queue;

	if (!instance->init()) {
		delete instance;
		desc.object.store(nullptr);
		desc.task_id = -1;
		return PX4_ERROR;
	}

	return PX4_OK;
}

int EscCanDecoder::run_trampoline(int argc, char *argv[])
{
	return ModuleBase::run_trampoline_impl(desc, [](int ac, char *av[]) -> ModuleBase * {
		return EscCanDecoder::instantiate(ac, av);
	}, argc, argv);
}

EscCanDecoder *EscCanDecoder::instantiate(int argc, char *argv[])
{
	int bitrate = 1000000;
	int offset = 0;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "b:o:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			if (px4_get_parameter_value(myoptarg, bitrate) != 0) {
				PX4_ERR("bitrate parsing failed");
				return nullptr;
			}

			break;

		case 'o':
			if (px4_get_parameter_value(myoptarg, offset) != 0) {
				PX4_ERR("offset parsing failed");
				return nullptr;
			}

			break;

		default:
			return nullptr;
		}
	}

	if (bitrate <= 0) {
		PX4_ERR("bitrate must be positive");
		return nullptr;
	}

	if (offset < 0) {
		PX4_ERR("offset must be non-negative");
		return nullptr;
	}

	return new EscCanDecoder(static_cast<uint32_t>(bitrate), static_cast<uint32_t>(offset));
}

int EscCanDecoder::print_status()
{
	PX4_INFO("id=0x%08" PRIx32 " frames=%u", telemetry_packet_1_id(), _frame_count);

	return 0;
}

int EscCanDecoder::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_USAGE_NAME("esc_can_decoder", "system");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Decode CAN2 ESC telemetry");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	PRINT_MODULE_USAGE_PARAM_INT('b', 1000000, 1, 1000000, "CAN bitrate", true);
	PRINT_MODULE_USAGE_PARAM_INT('o', 0, 0, 0x1fffffff, "ESC offset", true);
	return 0;
}

extern "C" __EXPORT int esc_can_decoder_main(int argc, char *argv[]);

int esc_can_decoder_main(int argc, char *argv[])
{
	return ModuleBase::main(EscCanDecoder::desc, argc, argv);
}
