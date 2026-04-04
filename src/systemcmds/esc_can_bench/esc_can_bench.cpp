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
#include <px4_platform_common/posix.h>

#include <nuttx/can/can.h>

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstring>

class EscCanBench : public ModuleBase
{
public:
	static Descriptor desc;

	EscCanBench(const char *device, uint32_t offset, uint32_t rate_hz, uint16_t voltage_dv, int16_t current_da,
		    int32_t rpm)
		: _offset(offset), _rate_hz(rate_hz), _voltage_dv(voltage_dv), _current_da(current_da), _rpm(rpm)
	{
		strncpy(_device_path, device, sizeof(_device_path) - 1);
		_device_path[sizeof(_device_path) - 1] = '\0';
	}

	~EscCanBench() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int run_trampoline(int argc, char *argv[]);
	static EscCanBench *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }
	static int print_usage(const char *reason = nullptr);

	void run() override;
	int print_status() override;

private:
	static constexpr uint32_t TelemetryBase = 0x14A30000U;
	static constexpr uint32_t TelemetryPacket1 = 1U;

	char _device_path[32] {"/dev/can0"};
	uint32_t _offset{0};
	uint32_t _rate_hz{20};
	uint16_t _voltage_dv{240};
	int16_t _current_da{15};
	int32_t _rpm{1500};

	uint32_t telemetry_can_id() const { return TelemetryBase + _offset + TelemetryPacket1; }
	void fill_packet(can_msg_s &msg) const;
};

ModuleBase::Descriptor EscCanBench::desc{task_spawn, custom_command, print_usage};

void EscCanBench::fill_packet(can_msg_s &msg) const
{
	msg = {};
	msg.cm_hdr.ch_id = telemetry_can_id();
	msg.cm_hdr.ch_dlc = 8;
	msg.cm_hdr.ch_rtr = 0;
#ifdef CONFIG_CAN_EXTID
	msg.cm_hdr.ch_extid = 1;
#endif
#ifdef CONFIG_CAN_ERRORS
	msg.cm_hdr.ch_error = 0;
#endif

	msg.cm_data[0] = static_cast<uint8_t>(_voltage_dv & 0xff);
	msg.cm_data[1] = static_cast<uint8_t>((_voltage_dv >> 8) & 0xff);

	const uint16_t current_bits = static_cast<uint16_t>(_current_da);
	msg.cm_data[2] = static_cast<uint8_t>(current_bits & 0xff);
	msg.cm_data[3] = static_cast<uint8_t>((current_bits >> 8) & 0xff);

	const uint32_t rpm_bits = static_cast<uint32_t>(_rpm);
	msg.cm_data[4] = static_cast<uint8_t>(rpm_bits & 0xff);
	msg.cm_data[5] = static_cast<uint8_t>((rpm_bits >> 8) & 0xff);
	msg.cm_data[6] = static_cast<uint8_t>((rpm_bits >> 16) & 0xff);
	msg.cm_data[7] = static_cast<uint8_t>((rpm_bits >> 24) & 0xff);
}

int EscCanBench::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
Publishes dummy ESC telemetry packet 1 as an extended CAN frame for bench testing.

The message format is:
- CAN ID: 0x14A30000 + offset + 1
- Bytes 0..1: terminal voltage in 0.1 V units
- Bytes 2..3: average current in 0.1 A units
- Bytes 4..7: RPM in 1 RPM units
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME_SIMPLE("esc_can_bench", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start periodic dummy ESC telemetry publishing");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/can0", "<device>", "CAN char device path", true);
	PRINT_MODULE_USAGE_PARAM_INT('o', 0, 0, 0x1fffffff, "D09 offset added to base CAN ID", true);
	PRINT_MODULE_USAGE_PARAM_INT('r', 20, 1, 1000, "Publish rate in Hz", true);
	PRINT_MODULE_USAGE_PARAM_INT('v', 240, 0, 65535, "Voltage in 0.1 V units", true);
	PRINT_MODULE_USAGE_PARAM_INT('c', 15, -32768, 32767, "Current in 0.1 A units", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 1500, INT32_MIN, INT32_MAX, "RPM", true);
	return 0;
}

int EscCanBench::task_spawn(int argc, char *argv[])
{
	int task_id = px4_task_spawn_cmd("esc_can_bench", SCHED_DEFAULT,
					 SCHED_PRIORITY_SLOW_DRIVER, PX4_STACK_ADJUSTED(2048),
					 run_trampoline, (char *const *)argv);

	if (task_id < 0) {
		return -errno;
	}

	desc.task_id = task_id;
	return 0;
}

int EscCanBench::run_trampoline(int argc, char *argv[])
{
	return ModuleBase::run_trampoline_impl(desc, [](int ac, char *av[]) -> ModuleBase * {
		return EscCanBench::instantiate(ac, av);
	}, argc, argv);
}

EscCanBench *EscCanBench::instantiate(int argc, char *argv[])
{
	const char *device = "/dev/can0";
	int offset = 0;
	int rate_hz = 20;
	int voltage_dv = 240;
	int current_da = 15;
	int rpm = 1500;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:o:r:v:c:p:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device = myoptarg;
			break;

		case 'o':
			if (px4_get_parameter_value(myoptarg, offset) != 0) {
				PX4_ERR("offset parsing failed");
				return nullptr;
			}

			break;

		case 'r':
			if (px4_get_parameter_value(myoptarg, rate_hz) != 0) {
				PX4_ERR("rate parsing failed");
				return nullptr;
			}

			break;

		case 'v':
			if (px4_get_parameter_value(myoptarg, voltage_dv) != 0) {
				PX4_ERR("voltage parsing failed");
				return nullptr;
			}

			break;

		case 'c':
			if (px4_get_parameter_value(myoptarg, current_da) != 0) {
				PX4_ERR("current parsing failed");
				return nullptr;
			}

			break;

		case 'p':
			if (px4_get_parameter_value(myoptarg, rpm) != 0) {
				PX4_ERR("rpm parsing failed");
				return nullptr;
			}

			break;

		default:
			return nullptr;
		}
	}

	if (access(device, R_OK | W_OK) != 0) {
		PX4_ERR("invalid device %s", device);
		return nullptr;
	}

	if (rate_hz <= 0) {
		PX4_ERR("rate must be positive");
		return nullptr;
	}

	return new EscCanBench(device, static_cast<uint32_t>(offset), static_cast<uint32_t>(rate_hz),
			       static_cast<uint16_t>(voltage_dv), static_cast<int16_t>(current_da), rpm);
}

void EscCanBench::run()
{
	px4_prctl(PR_SET_NAME, "esc_can_bench", px4_getpid());

	const useconds_t interval_us = 1000000U / _rate_hz;

	while (!should_exit()) {
		const int fd = ::open(_device_path, O_WRONLY | O_NONBLOCK);

		if (fd < 0) {
			PX4_ERR("open %s failed (%d)", _device_path, errno);
			px4_usleep(1000000);
			continue;
		}

		can_msg_s msg {};
		fill_packet(msg);
		const size_t expected = CAN_MSGLEN(msg.cm_hdr.ch_dlc);
		const ssize_t nbytes = ::write(fd, &msg, expected);

		if (nbytes < 0) {
			if (errno != EAGAIN) {
				PX4_ERR("write failed (%d)", errno);
			}

		} else if (nbytes != static_cast<ssize_t>(expected)) {
			PX4_ERR("short write (%ld, expected %zu)", static_cast<long>(nbytes), expected);
		}

		::close(fd);
		px4_usleep(interval_us);
	}
}

int EscCanBench::print_status()
{
	PX4_INFO("device: %s", _device_path);
	PX4_INFO("can id: 0x%08" PRIx32, telemetry_can_id());
	PX4_INFO("rate: %" PRIu32 " Hz", _rate_hz);
	PX4_INFO("voltage: %" PRIu16 " dV", _voltage_dv);
	PX4_INFO("current: %" PRId16 " dA", _current_da);
	PX4_INFO("rpm: %" PRId32, _rpm);
	return 0;
}

extern "C" __EXPORT int esc_can_bench_main(int argc, char *argv[]);

int esc_can_bench_main(int argc, char *argv[])
{
	return ModuleBase::main(EscCanBench::desc, argc, argv);
}
