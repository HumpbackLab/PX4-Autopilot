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

#include <drivers/drv_adc.h>
#include <px4_platform_common/log.h>

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>

namespace
{
int raw_fd = -1;
double scale_mv = 0.;
double offset = 0.;

bool read_number(int fd, double &value)
{
	char buffer[64] {};
	const ssize_t length = pread(fd, buffer, sizeof(buffer) - 1, 0);

	if (length <= 0 || length == sizeof(buffer) - 1) {
		return false;
	}

	char *end = nullptr;
	errno = 0;
	value = strtod(buffer, &end);

	if (end == buffer || errno != 0 || !std::isfinite(value)) {
		return false;
	}

	while (*end == ' ' || *end == '\n' || *end == '\t' || *end == '\r') {
		++end;
	}

	return *end == '\0';
}

bool read_attribute(const char *path, double &value)
{
	const int fd = open(path, O_RDONLY | O_CLOEXEC);

	if (fd < 0) {
		return false;
	}

	const bool ok = read_number(fd, value);
	close(fd);

	if (!ok) {
		errno = EINVAL;
	}

	return ok;
}
} // namespace

int px4_arch_adc_init(uint32_t base_address)
{
	px4_arch_adc_uninit(base_address);
	DIR *devices = opendir("/sys/bus/iio/devices");

	if (!devices) {
		return -errno;
	}

	int result = -ENODEV;

	while (const dirent *entry = readdir(devices)) {
		if (strncmp(entry->d_name, "iio:device", 10) != 0) {
			continue;
		}

		char path[512];
		snprintf(path, sizeof(path), "/sys/bus/iio/devices/%s/name", entry->d_name);
		FILE *name_file = fopen(path, "r");

		if (!name_file) {
			continue;
		}

		char name[64] {};
		const bool matches = fgets(name, sizeof(name), name_file) &&
				     (strcmp(name, "battery-voltage\n") == 0 || strcmp(name, "battery-voltage") == 0);
		fclose(name_file);

		if (!matches) {
			continue;
		}

		result = -EINVAL;
		snprintf(path, sizeof(path), "/sys/bus/iio/devices/%s/in_voltage0_scale", entry->d_name);

		if (!read_attribute(path, scale_mv) || scale_mv <= 0.) {
			break;
		}

		offset = 0.;
		snprintf(path, sizeof(path), "/sys/bus/iio/devices/%s/in_voltage0_offset", entry->d_name);

		if (!read_attribute(path, offset) && errno != ENOENT) {
			break;
		}

		snprintf(path, sizeof(path), "/sys/bus/iio/devices/%s/in_voltage0_raw", entry->d_name);
		raw_fd = open(path, O_RDONLY | O_CLOEXEC);

		if (raw_fd < 0) {
			result = -errno;
			break;
		}

		PX4_INFO("battery-voltage: %s, scale %.9f mV/count", entry->d_name, scale_mv);
		result = 0;
		break;
	}

	closedir(devices);

	if (result != 0) {
		PX4_ERR("battery-voltage IIO unavailable or invalid (%d)", result);
	}

	return result;
}

void px4_arch_adc_uninit(uint32_t base_address)
{
	if (raw_fd >= 0) {
		close(raw_fd);
		raw_fd = -1;
	}
}

uint32_t px4_arch_adc_sample(uint32_t base_address, unsigned channel)
{
	double raw = 0.;

	if (channel != 0 || raw_fd < 0 || !read_number(raw_fd, raw)) {
		return UINT32_MAX;
	}

	// IIO scale already includes the device-tree voltage divider. Report microvolts.
	const double voltage_uv = (raw + offset) * scale_mv * 1000.;

	if (!std::isfinite(voltage_uv) || voltage_uv < 0. || voltage_uv > INT32_MAX - 1.) {
		return UINT32_MAX;
	}

	return static_cast<uint32_t>(voltage_uv + 0.5);
}

float px4_arch_adc_reference_v()
{
	return 1.f;
}

uint32_t px4_arch_adc_dn_fullcount()
{
	return 1000000;
}

uint32_t px4_arch_adc_temp_sensor_mask()
{
	return 0;
}
