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

#include <px4_platform_common/module.h>

#include <nuttx/can/can.h>

#include <fcntl.h>
#include <unistd.h>

#include <cctype>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace
{

void usage(const char *reason = nullptr)
{
	if (reason != nullptr) {
		PX4_INFO_RAW("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION("Dump frames from a legacy NuttX CAN char device.");
	PRINT_MODULE_USAGE_NAME("can_dump", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("send", "Transmit one CAN frame");
	PRINT_MODULE_USAGE_ARG("[device]", "Receive mode: device path, default /dev/can0", true);
	PRINT_MODULE_USAGE_ARG("[count]", "Receive mode: optional number of frames to read", true);
	PRINT_MODULE_USAGE_ARG("send [device] <frame>", "Frame like 123#11223344 or 1ABCDEFA#0102", true);
}

void print_frame(const can_msg_s &msg)
{
	const auto &hdr = msg.cm_hdr;

#ifdef CONFIG_CAN_EXTID
	const bool ext = hdr.ch_extid;
#else
	const bool ext = false;
#endif

#ifdef CONFIG_CAN_ERRORS
	const bool err = hdr.ch_error;
#else
	const bool err = false;
#endif

	PX4_INFO_RAW("%s %s id=0x%08lx dlc=%u data=",
		     err ? "ERR" : "RX ",
		     hdr.ch_rtr ? "RTR" : (ext ? "EXT" : "STD"),
		     static_cast<unsigned long>(hdr.ch_id),
		     static_cast<unsigned>(hdr.ch_dlc));

	for (unsigned i = 0; i < hdr.ch_dlc && i < CAN_MAXDATALEN; ++i) {
		PX4_INFO_RAW("%s%02x", (i == 0) ? "" : " ", msg.cm_data[i]);
	}

	PX4_INFO_RAW("\n");
}

bool parse_hex_nibble(char c, uint8_t &value)
{
	if (c >= '0' && c <= '9') {
		value = static_cast<uint8_t>(c - '0');
		return true;
	}

	c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));

	if (c >= 'A' && c <= 'F') {
		value = static_cast<uint8_t>(10 + (c - 'A'));
		return true;
	}

	return false;
}

bool parse_frame(const char *arg, can_msg_s &msg)
{
	const char *sep = std::strchr(arg, '#');

	if (sep == nullptr || sep == arg || *(sep + 1) == '\0') {
		return false;
	}

	const size_t id_len = static_cast<size_t>(sep - arg);

	if (id_len > 8) {
		return false;
	}

	char id_buf[9] {};
	std::memcpy(id_buf, arg, id_len);

	char *endptr = nullptr;
	const unsigned long can_id = std::strtoul(id_buf, &endptr, 16);

	if (endptr == id_buf || *endptr != '\0') {
		return false;
	}

	const char *data = sep + 1;
	const size_t data_len = std::strlen(data);

	if ((data_len % 2) != 0 || (data_len / 2) > CAN_MAXDATALEN) {
		return false;
	}

	msg = {};
	msg.cm_hdr.ch_id = can_id;
	msg.cm_hdr.ch_rtr = false;
	msg.cm_hdr.ch_dlc = data_len / 2;

#ifdef CONFIG_CAN_ERRORS
	msg.cm_hdr.ch_error = false;
#endif

#ifdef CONFIG_CAN_EXTID
	msg.cm_hdr.ch_extid = (id_len > 3);
#else
	if (id_len > 3 || can_id > CAN_MAX_STDMSGID) {
		return false;
	}
#endif

	for (size_t i = 0; i < data_len / 2; ++i) {
		uint8_t hi = 0;
		uint8_t lo = 0;

		if (!parse_hex_nibble(data[i * 2], hi) || !parse_hex_nibble(data[i * 2 + 1], lo)) {
			return false;
		}

		msg.cm_data[i] = static_cast<uint8_t>((hi << 4) | lo);
	}

	return true;
}

int send_frame(const char *device, const char *frame_arg)
{
	can_msg_s msg {};

	if (!parse_frame(frame_arg, msg)) {
		PX4_ERR("invalid frame, expected <id>#<data>, for example 123#11223344");
		return 1;
	}

	const int fd = ::open(device, O_WRONLY | O_NONBLOCK);

	if (fd < 0) {
		PX4_ERR("open %s failed (%d)", device, errno);
		return 1;
	}

	const size_t expected = CAN_MSGLEN(msg.cm_hdr.ch_dlc);
	const ssize_t nbytes = ::write(fd, &msg, expected);

	if (nbytes < 0) {
		if (errno == EAGAIN) {
			PX4_ERR("write would block");

		} else {
			PX4_ERR("write failed (%d)", errno);
		}

		::close(fd);
		return 1;
	}

	if (nbytes != static_cast<ssize_t>(expected)) {
		PX4_ERR("short write (%ld, expected %zu)", static_cast<long>(nbytes), expected);
		::close(fd);
		return 1;
	}

	PX4_INFO("sent on %s", device);
	print_frame(msg);

	::close(fd);
	return 0;
}

} // namespace

extern "C" __EXPORT int can_dump_main(int argc, char *argv[])
{
	if (argc >= 2 && std::strcmp(argv[1], "send") == 0) {
		const char *device = "/dev/can0";
		const char *frame = nullptr;

		if (argc == 3) {
			frame = argv[2];

		} else if (argc == 4) {
			device = argv[2];
			frame = argv[3];

		} else {
			usage("send mode expects: can_dump send [device] <frame>");
			return 1;
		}

		return send_frame(device, frame);
	}

	const char *device = "/dev/can0";
	long remaining = -1;

	if (argc > 3) {
		usage("too many arguments");
		return 1;
	}

	if (argc >= 2) {
		device = argv[1];
	}

	if (argc == 3) {
		char *endptr = nullptr;
		remaining = strtol(argv[2], &endptr, 10);

		if ((endptr == argv[2]) || (endptr && *endptr != '\0') || remaining <= 0) {
			usage("invalid count");
			return 1;
		}
	}

	const int fd = ::open(device, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("open %s failed (%d)", device, errno);
		return 1;
	}

	PX4_INFO("reading %s%s", device, (remaining > 0) ? "" : " until interrupted");

	while (remaining != 0) {
		can_msg_s msg {};
		const ssize_t nbytes = ::read(fd, &msg, sizeof(msg));

		if (nbytes < 0) {
			PX4_ERR("read failed (%d)", errno);
			::close(fd);
			return 1;
		}

		if (nbytes < static_cast<ssize_t>(CAN_MSGLEN(0))) {
			PX4_ERR("short read (%ld)", static_cast<long>(nbytes));
			::close(fd);
			return 1;
		}

		const size_t expected = CAN_MSGLEN(msg.cm_hdr.ch_dlc);

		if (static_cast<size_t>(nbytes) != expected) {
			PX4_ERR("unexpected read size (%ld, expected %zu)", static_cast<long>(nbytes), expected);
			::close(fd);
			return 1;
		}

		print_frame(msg);

		if (remaining > 0) {
			--remaining;
		}
	}

	::close(fd);
	return 0;
}
