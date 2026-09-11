/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <errno.h>
#include <fcntl.h>
#include <cstdio>
#include <stdint.h>
#include <unistd.h>

#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/time.h>

#ifndef MODULE_NAME
#define MODULE_NAME "extfin_heater_pwm"
#endif

using namespace time_literals;

namespace
{

constexpr const char *PWM_CHIP_PATH = "/sys/class/pwm/pwmchip10";
constexpr const char *PWM_CHANNEL_PATH = "/sys/class/pwm/pwmchip10/pwm0";
constexpr const char *PWM_EXPORT_PATH = "/sys/class/pwm/pwmchip10/export";
constexpr const char *PWM_ENABLE_PATH = "/sys/class/pwm/pwmchip10/pwm0/enable";
constexpr const char *PWM_PERIOD_PATH = "/sys/class/pwm/pwmchip10/pwm0/period";
constexpr const char *PWM_DUTY_PATH = "/sys/class/pwm/pwmchip10/pwm0/duty_cycle";
constexpr const char *PWM_POLARITY_PATH = "/sys/class/pwm/pwmchip10/pwm0/polarity";
constexpr unsigned PWM_PERIOD_NS = 1000000; // 1 kHz
bool pwm_initialized = false;

bool pwm_sysfs_ready()
{
	return ::access(PWM_ENABLE_PATH, W_OK) == 0
	       && ::access(PWM_PERIOD_PATH, W_OK) == 0
	       && ::access(PWM_DUTY_PATH, W_OK) == 0
	       && ::access(PWM_POLARITY_PATH, W_OK) == 0;
}

int write_sysfs_value(const char *path, unsigned value)
{
	const int fd = ::open(path, O_WRONLY | O_CLOEXEC);

	if (fd < 0) {
		return -errno;
	}

	char buffer[16] {};
	const int length = ::snprintf(buffer, sizeof(buffer), "%u", value);
	const ssize_t written = length > 0 ? ::write(fd, buffer, length) : -1;
	const int result = (written == length) ? 0 : (written < 0 ? -errno : -EIO);
	::close(fd);
	return result;
}

int write_sysfs_string(const char *path, const char *value)
{
	const int fd = ::open(path, O_WRONLY | O_CLOEXEC);

	if (fd < 0) {
		return -errno;
	}

	char buffer[16] {};
	const int length = ::snprintf(buffer, sizeof(buffer), "%s\n", value);
	const ssize_t written = length > 0 ? ::write(fd, buffer, length) : -1;
	const int result = (written == length) ? 0 : (written < 0 ? -errno : -EIO);
	::close(fd);
	return result;
}

bool initialize_pwm()
{
	if (::access(PWM_CHIP_PATH, F_OK) != 0) {
		PX4_ERR("heater PWM controller unavailable: %s", PWM_CHIP_PATH);
		return false;
	}

	if (::access(PWM_CHANNEL_PATH, F_OK) != 0) {
		const int export_result = write_sysfs_value(PWM_EXPORT_PATH, 0);

		if (export_result < 0 && export_result != -EBUSY) {
			PX4_ERR("failed to export heater PWM1-0 (%d)", export_result);
			return false;
		}
	}

	for (unsigned retry = 0; retry < 1000 && !pwm_sysfs_ready(); retry++) {
		px4_usleep(1_ms);
	}

	if (!pwm_sysfs_ready()) {
		PX4_ERR("heater PWM1-0 sysfs attributes were not created");
		return false;
	}

	// sunxi_pwm requires the period to be configured before polarity.
	if (write_sysfs_value(PWM_PERIOD_PATH, PWM_PERIOD_NS) < 0
	    || write_sysfs_string(PWM_POLARITY_PATH, "normal") < 0
	    || write_sysfs_value(PWM_DUTY_PATH, 0) < 0
	    || write_sysfs_value(PWM_ENABLE_PATH, 1) < 0) {
		PX4_ERR("failed to initialize heater PWM1-0");
		return false;
	}

	return true;
}

} // namespace

extern "C" int board_app_initialize(uintptr_t)
{
	pwm_initialized = initialize_pwm();
	return pwm_initialized ? PX4_OK : PX4_ERROR;
}

extern "C" void extfin_heater_output_en(int enabled)
{
	const unsigned duty_cycle = enabled ? PWM_PERIOD_NS : 0;

	if (write_sysfs_value(PWM_DUTY_PATH, duty_cycle) < 0) {
		PX4_ERR("failed to set heater PWM1-0 duty cycle");
		pwm_initialized = false;
	}
}
