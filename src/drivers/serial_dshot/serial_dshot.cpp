#include "serial_dshot.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

#include <parameters/param.h>

#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

ModuleBase::Descriptor SerialDShot::desc{task_spawn, custom_command, print_usage};

namespace
{
constexpr uint8_t TX_HEADER = 0xaa;
constexpr uint8_t RX_HEADER = 0x55;
constexpr uint8_t FRAME_TIMEOUT = 0x02;
constexpr uint8_t FRAME_DSHOT_UPDATE = 0x03;
constexpr uint8_t FRAME_DSHOT_CONFIG = 0x04;
constexpr uint8_t FRAME_MOTOR_COMMAND = 0x10;
constexpr uint8_t RET_SUCCESS = 0x01;
constexpr uint8_t RET_PARSE_ERROR = 0x02;
constexpr uint8_t RET_PARAM_INVALID = 0x03;
constexpr uint8_t RET_TELEMETRY = 0x10;
constexpr uint8_t RET_CONFIG_UPLOAD = 0xcf;

uint8_t checksum(const uint8_t *data, size_t length)
{
	uint8_t sum = 0;

	for (size_t i = 0; i < length; ++i) {
		sum += data[i];
	}

	return sum;
}

void put_u16_le(uint8_t *destination, uint16_t value)
{
	destination[0] = static_cast<uint8_t>(value);
	destination[1] = static_cast<uint8_t>(value >> 8);
}

void put_u32_le(uint8_t *destination, uint32_t value)
{
	for (unsigned i = 0; i < 4; ++i) {
		destination[i] = static_cast<uint8_t>(value >> (8 * i));
	}
}

uint16_t get_u16_le(const uint8_t *source)
{
	return static_cast<uint16_t>(source[0]) | (static_cast<uint16_t>(source[1]) << 8);
}

int32_t get_param_int(const char *name, int32_t default_value)
{
	const param_t handle = param_find(name);
	int32_t value = default_value;

	if (handle != PARAM_INVALID) {
		param_get(handle, &value);
	}

	return value;
}

bool valid_rate(unsigned rate, bool bidirectional)
{
	if (rate != 150 && rate != 300 && rate != 600 && rate != 1200) {
		return false;
	}

	return !bidirectional || rate == 300 || rate == 600;
}
}

SerialDShot::SerialDShot(const char *device, uint32_t baudrate, unsigned dshot_rate, bool bidirectional,
			 uint32_t timeout_ms) :
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default),
	_serial(device, baudrate),
	_baudrate(baudrate),
	_dshot_rate(dshot_rate),
	_bidirectional(bidirectional),
	_timeout_ms(timeout_ms)
{
	_mixing_output.setMaxNumOutputs(DSHOT_CHANNELS);
	pthread_mutex_init(&_command_mutex, nullptr);
	update_params();
}

SerialDShot::~SerialDShot()
{
	if (_serial.isOpen()) {
		uint16_t stop[DSHOT_CHANNELS] {};
		send_motor_frame(stop);
		_serial.close();
	}

	pthread_mutex_destroy(&_command_mutex);
	perf_free(_cycle_perf);
	perf_free(_interval_perf);
	perf_free(_io_error_perf);
	perf_free(_protocol_error_perf);
}

int SerialDShot::init()
{
	if (!_serial.open()) {
		PX4_ERR("failed to open %s at %u baud", _serial.getPort(), _baudrate);
		return PX4_ERROR;
	}

	// The adapter is configured in RAM on every PX4 start. Do not change or
	// save its UART baud rate here, otherwise recovery after a mismatched
	// configuration would require probing multiple baud rates.
	uint8_t timeout_payload[4];
	put_u32_le(timeout_payload, _timeout_ms);

	uint8_t update_payload[9] {};
	update_payload[0] = 0; // synchronous: one DShot update per motor frame

	uint8_t config_payload[2] {
		static_cast<uint8_t>(_dshot_rate == 150 ? 0 : _dshot_rate == 300 ? 1 : _dshot_rate == 600 ? 2 : 3),
		static_cast<uint8_t>(_bidirectional ? 1 : 0)
	};

	if (!send_config_frame(FRAME_TIMEOUT, timeout_payload, sizeof(timeout_payload))
	    || !send_config_frame(FRAME_DSHOT_UPDATE, update_payload, sizeof(update_payload))
	    || !send_config_frame(FRAME_DSHOT_CONFIG, config_payload, sizeof(config_payload))) {
		PX4_ERR("failed to configure serial DShot adapter");
		_serial.close();
		return PX4_ERROR;
	}

	return PX4_OK;
}

int SerialDShot::task_spawn(int argc, char *argv[])
{
	int myoptind = 1;
	const char *myoptarg = nullptr;
	int ch;
	const char *device = DEFAULT_DEVICE;
	uint32_t baudrate = static_cast<uint32_t>(get_param_int("SD_DSHOT_BAUD", 1000000));
	unsigned rate = static_cast<unsigned>(get_param_int("SD_DSHOT_RATE", 300));
	bool bidirectional = get_param_int("SD_DSHOT_BIDIR", 1) != 0;
	uint32_t timeout_ms = static_cast<uint32_t>(get_param_int("SD_DSHOT_TMO", 500));

	while ((ch = px4_getopt(argc, argv, "d:b:r:tT", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd': device = myoptarg; break;
		case 'b': baudrate = strtoul(myoptarg, nullptr, 0); break;
		case 'r': rate = strtoul(myoptarg, nullptr, 0); break;
		case 't': bidirectional = true; break;
		case 'T': bidirectional = false; break;
		default: return print_usage("unrecognized flag");
		}
	}

	if (device == nullptr) {
		return print_usage("serial device is required");
	}

	if (!valid_rate(rate, bidirectional)) {
		PX4_ERR("invalid DShot rate %u for %s mode", rate, bidirectional ? "bidirectional" : "unidirectional");
		return PX4_ERROR;
	}

	if (timeout_ms < 100 || timeout_ms > 5000) {
		PX4_ERR("adapter timeout must be between 100 and 5000 ms");
		return PX4_ERROR;
	}

	SerialDShot *instance = new SerialDShot(device, baudrate, rate, bidirectional, timeout_ms);

	if (instance == nullptr) {
		PX4_ERR("failed to allocate instance");
		return PX4_ERROR;
	}

	if (instance->init() != PX4_OK) {
		delete instance;
		return PX4_ERROR;
	}

	desc.object.store(instance);
	desc.task_id = task_id_is_work_queue;
	instance->ScheduleNow();
	return PX4_OK;
}

bool SerialDShot::send_config_frame(uint8_t id, const uint8_t *payload, size_t payload_len)
{
	if (payload_len > 9) {
		return false;
	}

	uint8_t frame[12] {};
	frame[0] = TX_HEADER;
	frame[1] = id;
	memcpy(&frame[2], payload, payload_len);
	frame[payload_len + 2] = checksum(frame, payload_len + 2);
	const size_t frame_length = payload_len + 3;
	const ssize_t written = _serial.writeBlocking(frame, frame_length, 20);

	if (written != static_cast<ssize_t>(frame_length)) {
		perf_count(_io_error_perf);
		return false;
	}

	return true;
}

bool SerialDShot::send_motor_frame(const uint16_t values[DSHOT_CHANNELS])
{
	drain_serial();
	uint8_t frame[11] {TX_HEADER, FRAME_MOTOR_COMMAND};

	for (unsigned i = 0; i < DSHOT_CHANNELS; ++i) {
		put_u16_le(&frame[2 + 2 * i], values[i]);
	}

	frame[10] = checksum(frame, 10);
	const ssize_t written = _serial.writeBlocking(frame, sizeof(frame), 10);

	if (written != sizeof(frame)) {
		perf_count(_io_error_perf);
		return false;
	}

	return true;
}

bool SerialDShot::updateOutputs(float outputs[MAX_ACTUATORS], unsigned num_outputs,
				unsigned num_control_groups_updated)
{
	(void)num_control_groups_updated;
	uint16_t values[DSHOT_CHANNELS] {};
	bool command_sent = false;

	for (unsigned i = 0; i < DSHOT_CHANNELS; ++i) {
		if (i >= num_outputs) {
			continue;
		}

		const uint16_t output = static_cast<uint16_t>(lroundf(math::constrain(outputs[i], 0.f,
					static_cast<float>(DSHOT_MAX_THROTTLE))));

		if (output == DSHOT_DISARM_VALUE) {
			if (_current_command.valid() && (_current_command.motor_mask & (1u << i))) {
				values[i] = _current_command.command;
				command_sent = true;
			}

		} else {
			values[i] = output + DSHOT_COMMAND_OFFSET;
		}
	}

	const bool success = send_motor_frame(values);

	if (success) {
		memcpy(_last_outputs, values, sizeof(_last_outputs));

		if (command_sent && _current_command.valid() && --_current_command.num_repetitions == 0
		    && _current_command.save) {
			_current_command.command = DSHOT_CMD_SAVE_SETTINGS;
			_current_command.num_repetitions = 10;
			_current_command.save = false;
		}
	}

	return success;
}

void SerialDShot::drain_serial()
{
	uint8_t buffer[64];
	ssize_t count;

	while ((count = _serial.read(buffer, sizeof(buffer))) > 0) {
		for (ssize_t i = 0; i < count; ++i) {
			parse_byte(buffer[i]);
		}
	}
}

void SerialDShot::parse_byte(uint8_t byte)
{
	if (_rx_length == 0) {
		if (byte == RX_HEADER) {
			_rx_frame[_rx_length++] = byte;
		}

		return;
	}

	if (_rx_length == 1) {
		_rx_frame[_rx_length++] = byte;

		switch (byte) {
		case RET_SUCCESS:
		case RET_PARSE_ERROR:
		case RET_PARAM_INVALID: _rx_expected = 3; break;
		case RET_TELEMETRY: _rx_expected = 15; break;
		case RET_CONFIG_UPLOAD: _rx_expected = 23; break;
		default:
			perf_count(_protocol_error_perf);
			_rx_length = 0;
			_rx_expected = 0;
			break;
		}

		return;
	}

	_rx_frame[_rx_length++] = byte;

	if (_rx_expected > 0 && _rx_length == _rx_expected) {
		process_rx_frame(_rx_frame, _rx_length);
		_rx_length = 0;
		_rx_expected = 0;
	}
}

void SerialDShot::process_rx_frame(const uint8_t *frame, size_t length)
{
	if (length < 3 || checksum(frame, length - 1) != frame[length - 1]) {
		perf_count(_protocol_error_perf);
		return;
	}

	if (frame[1] == RET_PARSE_ERROR || frame[1] == RET_PARAM_INVALID) {
		perf_count(_protocol_error_perf);
		PX4_WARN("adapter rejected frame (response 0x%02x)", frame[1]);
		return;
	}

	if (frame[1] != RET_TELEMETRY || length != 15) {
		return;
	}

	_telemetry_valid_mask = 0;

	for (unsigned i = 0; i < DSHOT_CHANNELS; ++i) {
		const uint16_t period = get_u16_le(&frame[2 + 2 * i]);
		_last_period[i] = period;

		// Adapter failures are encoded as 0xff00 | error_code. A zero period
		// is also unusable for the RPM conversion.
		if (period != 0 && period < 0xff00) {
			_telemetry_valid_mask |= 1u << i;

		} else {
			++_telemetry_errors[i];
		}
	}

	_last_adc[0] = get_u16_le(&frame[10]);
	_last_adc[1] = get_u16_le(&frame[12]);
	_last_telemetry_time = hrt_absolute_time();
	publish_esc_status();
}

void SerialDShot::publish_esc_status()
{
	esc_status_s status{};
	status.timestamp = _last_telemetry_time;
	status.counter = _esc_status_counter++;
	status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_DSHOT;
	const unsigned pole_count = math::max(_param_motor_poles.get(), 2);
	unsigned telemetry_index = 0;

	for (unsigned channel = 0; channel < DSHOT_CHANNELS; ++channel) {
		if (!_mixing_output.isFunctionSet(channel) || telemetry_index >= esc_status_s::CONNECTED_ESC_MAX) {
			continue;
		}

		esc_report_s &esc = status.esc[telemetry_index];
		esc.actuator_function = static_cast<uint8_t>(_mixing_output.outputFunction(channel));
		esc.esc_errorcount = _telemetry_errors[channel];

		if (_telemetry_valid_mask & (1u << channel)) {
			esc.timestamp = status.timestamp;
			esc.esc_rpm = static_cast<int32_t>(120000000ULL /
					(static_cast<uint64_t>(_last_period[channel]) * pole_count));
			status.esc_online_flags |= 1u << telemetry_index;
		}

		if (_armed.load()) {
			status.esc_armed_flags |= 1u << telemetry_index;
		}

		++telemetry_index;
	}

	status.esc_count = telemetry_index;
	_esc_status_pub.publish(status);
}

int SerialDShot::enqueue_command(uint16_t command, int num_repetitions, uint8_t motor_mask, bool save)
{
	Command queued{command, num_repetitions, motor_mask, save};
	pthread_mutex_lock(&_command_mutex);

	if (_armed.load() || _command_queue_count >= COMMAND_QUEUE_SIZE) {
		pthread_mutex_unlock(&_command_mutex);
		PX4_WARN(_armed.load() ? "DShot commands require disarmed outputs" : "DShot command queue full");
		return -EBUSY;
	}

	_command_queue[_command_queue_tail] = queued;
	_command_queue_tail = (_command_queue_tail + 1) % COMMAND_QUEUE_SIZE;
	++_command_queue_count;
	pthread_mutex_unlock(&_command_mutex);
	ScheduleNow();
	return PX4_OK;
}

bool SerialDShot::dequeue_command(Command &command)
{
	pthread_mutex_lock(&_command_mutex);

	if (_command_queue_count == 0) {
		pthread_mutex_unlock(&_command_mutex);
		return false;
	}

	command = _command_queue[_command_queue_head];
	_command_queue_head = (_command_queue_head + 1) % COMMAND_QUEUE_SIZE;
	--_command_queue_count;
	pthread_mutex_unlock(&_command_mutex);
	return true;
}

void SerialDShot::clear_command_queue()
{
	pthread_mutex_lock(&_command_mutex);
	_command_queue_head = 0;
	_command_queue_tail = 0;
	_command_queue_count = 0;
	pthread_mutex_unlock(&_command_mutex);
}

void SerialDShot::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();
		exit_and_cleanup(desc);
		return;
	}

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);
	drain_serial();

	if (!_esc_init_done) {
		if (_esc_init_start == 0) {
			_esc_init_start = hrt_absolute_time();
			PX4_INFO("sending zero throttle frames for ESC init");
		}

		uint16_t stop[DSHOT_CHANNELS] {};
		send_motor_frame(stop);

		if (hrt_elapsed_time(&_esc_init_start) < ESC_INIT_DURATION) {
			perf_end(_cycle_perf);
			ScheduleDelayed(ESC_INIT_INTERVAL);
			return;
		}

		_esc_init_done = true;
		PX4_INFO("ESC init complete");
	}

	_mixing_output.update();
	const bool was_armed = _armed.load();
	_armed.store(_mixing_output.armed().armed);

	if (_armed.load()) {
		if (_current_command.valid()) {
			PX4_WARN("cancelling DShot command while armed");
			_current_command = {};
		}

		if (!was_armed) {
			clear_command_queue();
		}
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s update;
		_parameter_update_sub.copy(&update);
		update_params();
	}

	handle_vehicle_commands();

	if (!_armed.load() && !_current_command.valid()) {
		dequeue_command(_current_command);
	}

	_mixing_output.updateSubscriptions(true);
	perf_end(_cycle_perf);
}

void SerialDShot::handle_vehicle_commands()
{
	vehicle_command_s vehicle_command;

	while (_vehicle_command_sub.update(&vehicle_command)) {
		if (vehicle_command.command != vehicle_command_s::VEHICLE_CMD_CONFIGURE_ACTUATOR) {
			continue;
		}

		int function = static_cast<int>(vehicle_command.param5 + 0.5);

		if (function < 1000) {
			constexpr int first_motor_function = 1;
			constexpr int first_servo_function = 33;

			if (function >= first_motor_function && function < first_motor_function + actuator_test_s::MAX_NUM_MOTORS) {
				function = function - first_motor_function + actuator_test_s::FUNCTION_MOTOR1;

			} else if (function >= first_servo_function
				   && function < first_servo_function + actuator_test_s::MAX_NUM_SERVOS) {
				function = function - first_servo_function + actuator_test_s::FUNCTION_SERVO1;

			} else {
				function = INT32_MAX;
			}

		} else {
			function -= 1000;
		}

		const int type = static_cast<int>(vehicle_command.param1 + 0.5f);
		int index = -1;

		for (unsigned i = 0; i < DSHOT_CHANNELS; ++i) {
			if (static_cast<int>(_mixing_output.outputFunction(i)) == function) {
				index = i;
				break;
			}
		}

		vehicle_command_ack_s ack{};
		ack.command = vehicle_command.command;
		ack.target_system = vehicle_command.source_system;
		ack.target_component = vehicle_command.source_component;
		ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;

		if (index != -1) {
			uint16_t command = DSHOT_CMD_MOTOR_STOP;

			switch (type) {
			case 1: command = DSHOT_CMD_BEEP1; break;
			case 2: command = DSHOT_CMD_3D_MODE_ON; break;
			case 3: command = DSHOT_CMD_3D_MODE_OFF; break;
			case 4: command = DSHOT_CMD_SPIN_DIRECTION_1; break;
			case 5: command = DSHOT_CMD_SPIN_DIRECTION_2; break;
			}

			if (command != DSHOT_CMD_MOTOR_STOP) {
				const int result = enqueue_command(command, 10, 1u << index, true);
				ack.result = result == PX4_OK ? vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED :
					     vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			}
		}

		ack.timestamp = hrt_absolute_time();
		_command_ack_pub.publish(ack);
	}
}

int SerialDShot::custom_command(int argc, char *argv[])
{
	if (argc < 1 || strcmp(argv[0], "cmd")) {
		return print_usage("unknown command");
	}

	if (!is_running(desc)) {
		PX4_ERR("not running");
		return PX4_ERROR;
	}

	int argument = 1;
	int motor = 0;
	int command = 0;
	int repeat = 1;

	while (argument < argc) {
		const char *option = argv[argument++];

		if (!strcmp(option, "-m") && argument < argc) {
			motor = strtol(argv[argument++], nullptr, 0);

		} else if (!strcmp(option, "-c") && argument < argc) {
			command = strtol(argv[argument++], nullptr, 0);

		} else if (!strcmp(option, "-n") && argument < argc) {
			repeat = strtol(argv[argument++], nullptr, 0);

		} else {
			return print_usage("invalid cmd argument");
		}
	}

	if (motor < 0 || motor >= static_cast<int>(DSHOT_CHANNELS) || command < 0 || command > 47
	    || repeat < 1 || repeat > 10) {
		return print_usage("cmd values out of range");
	}

	SerialDShot *instance = get_instance<SerialDShot>(desc);
	return instance ? instance->enqueue_command(command, repeat, 1u << motor, false) : PX4_ERROR;
}

void SerialDShot::update_params()
{
	ModuleParams::updateParams();
	_mixing_output.setAllMinValues(math::constrain(static_cast<int>(_param_dshot_min.get()
				       * static_cast<float>(DSHOT_MAX_THROTTLE)),
				       static_cast<int>(DSHOT_MIN_THROTTLE), static_cast<int>(DSHOT_MAX_THROTTLE)));
}

int SerialDShot::print_status()
{
	PX4_INFO("device: %s, baud: %u", _serial.getPort(), _baudrate);
	PX4_INFO("DShot%u, bidirectional: %s, timeout: %u ms", _dshot_rate, _bidirectional ? "yes" : "no", _timeout_ms);
	PX4_INFO("ESC init: %s, outputs: %u %u %u %u", _esc_init_done ? "complete" : "running",
		 _last_outputs[0], _last_outputs[1], _last_outputs[2], _last_outputs[3]);

	if (_last_telemetry_time != 0) {
		PX4_INFO("telemetry: valid=0x%02x period=%u %u %u %u ADC=%u %u", _telemetry_valid_mask,
			 _last_period[0], _last_period[1], _last_period[2], _last_period[3], _last_adc[0], _last_adc[1]);
	}

	_mixing_output.printStatus();
	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);
	perf_print_counter(_io_error_perf);
	perf_print_counter(_protocol_error_perf);
	return PX4_OK;
}

int SerialDShot::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Drive four DShot outputs through the AT32 Serial-To-BiDirDSHOT adapter.

The adapter must already use the selected UART baud rate. At startup the
driver configures its DShot mode, synchronous updates, and failsafe timeout.
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("serial_dshot", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "Serial device", false);
	PRINT_MODULE_USAGE_PARAM_INT('b', 1000000, 9600, 3000000, "Serial baud rate", true);
	PRINT_MODULE_USAGE_PARAM_INT('r', 300, 150, 1200, "DShot rate (150, 300, 600, or 1200)", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('t', "Enable bidirectional DShot", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('T', "Disable bidirectional DShot", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("cmd", "Send DShot command to a motor");
	PRINT_MODULE_USAGE_PARAM_INT('m', 0, 0, DSHOT_CHANNELS - 1, "Motor index", true);
	PRINT_MODULE_USAGE_PARAM_INT('c', 0, 0, 47, "DShot command", true);
	PRINT_MODULE_USAGE_PARAM_INT('n', 1, 1, 10, "Repeat count", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int serial_dshot_main(int argc, char *argv[])
{
	return ModuleBase::main(SerialDShot::desc, argc, argv);
}
