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

#include <uavcan_stm32h7/can.hpp>
#include <uavcan_stm32h7/clock.hpp>

#include <cinttypes>
#include <cstring>

namespace
{

static constexpr uint32_t ContactorBase = 0x14A40210U;
static constexpr uint32_t ContactorCommand = 0U;
static constexpr uint8_t RawCanIfaceIndex = 1U;
static constexpr uint8_t ContactorOnByteIndex = 0U;
static constexpr uint8_t ContactorOffByteIndex = 1U;
static constexpr uint8_t ContactorOnValue = 0xAAU;
static constexpr uint8_t ContactorOffValue = 0x55U;
static constexpr uint32_t TxDeadlineMs = 100U;
static constexpr unsigned CanRxQueueCapacity = 2U;
using CanHelper = uavcan_stm32h7::CanInitHelper<CanRxQueueCapacity>;
alignas(CanHelper) static uint8_t g_can_helper_storage[sizeof(CanHelper)] {};

uint32_t contactor_command_id(uint32_t offset)
{
	return ContactorBase + offset + ContactorCommand;
}

bool parse_contactor_state(const char *arg, bool &close)
{
	if ((strcmp(arg, "ON") == 0) || (strcmp(arg, "on") == 0)) {
		close = true;
		return true;
	}

	if ((strcmp(arg, "OFF") == 0) || (strcmp(arg, "off") == 0)) {
		close = false;
		return true;
	}

	return false;
}

void print_usage(const char *reason = nullptr)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
Sends a one-shot contactor command on CAN2.

Usage examples:
	 aaci_contactor ON
	 aaci_contactor -o 0x10 OFF
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("aaci_contactor", "command");
	PRINT_MODULE_USAGE_ARG("<ON|OFF>", "Contactor command state", false);
	PRINT_MODULE_USAGE_PARAM_INT('b', 1000000, 1, 1000000, "CAN bitrate", true);
	PRINT_MODULE_USAGE_PARAM_INT('o', 0, 0, 0x1fffffff, "Contactor offset", true);
}

int send_contactor_command(bool close, uint32_t bitrate, uint32_t offset)
{
	auto *can_helper = new (g_can_helper_storage) CanHelper();

	const int init_ret = can_helper->driver.initRawIface(RawCanIfaceIndex, bitrate);

	if (init_ret < 0) {
		PX4_ERR("CAN2 raw init failed (%d)", init_ret);
		return PX4_ERROR;
	}

	uavcan::CanFrame frame{};
	frame.id = (contactor_command_id(offset) & uavcan::CanFrame::MaskExtID) | uavcan::CanFrame::FlagEFF;
	frame.dlc = 8;

	if (close) {
		frame.data[ContactorOnByteIndex] = ContactorOnValue;

	} else {
		frame.data[ContactorOffByteIndex] = ContactorOffValue;
	}

	const uavcan::MonotonicTime tx_deadline =
		uavcan_stm32h7::clock::getMonotonic() + uavcan::MonotonicDuration::fromMSec(TxDeadlineMs);

	const int tx_ret = can_helper->driver.rawSend(RawCanIfaceIndex, frame, tx_deadline);

	if (tx_ret < 0) {
		PX4_ERR("raw send failed (%d)", tx_ret);
		return PX4_ERROR;
	}

	PX4_INFO("sent CAN2 id=0x%08" PRIx32 " state=%s", frame.id & uavcan::CanFrame::MaskExtID, close ? "ON" : "OFF");
	return PX4_OK;
}

} // namespace

extern "C" __EXPORT int aaci_contactor_main(int argc, char *argv[])
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
				return PX4_ERROR;
			}

			break;

		case 'o':
			if (px4_get_parameter_value(myoptarg, offset) != 0) {
				PX4_ERR("offset parsing failed");
				return PX4_ERROR;
			}

			break;

		default:
			print_usage("unknown option");
			return PX4_ERROR;
		}
	}

	if (bitrate <= 0) {
		PX4_ERR("bitrate must be positive");
		return PX4_ERROR;
	}

	if (offset < 0) {
		PX4_ERR("offset must be non-negative");
		return PX4_ERROR;
	}

	if (myoptind >= argc) {
		print_usage("missing state argument: ON or OFF");
		return PX4_ERROR;
	}

	if ((myoptind + 1) != argc) {
		print_usage("too many arguments");
		return PX4_ERROR;
	}

	bool close = false;

	if (!parse_contactor_state(argv[myoptind], close)) {
		print_usage("invalid state argument, use ON or OFF");
		return PX4_ERROR;
	}

	return send_contactor_command(close, static_cast<uint32_t>(bitrate), static_cast<uint32_t>(offset));
}

