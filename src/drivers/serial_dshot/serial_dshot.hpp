#pragma once

#include <px4_platform_common/Serial.hpp>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/module.h>

#include <drivers/drv_dshot.h>
#include <drivers/drv_hrt.h>
#include <lib/mixer_module/mixer_module.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/actuator_test.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>

#include <pthread.h>
#include <stdint.h>

using namespace time_literals;

class SerialDShot final : public ModuleBase, public OutputModuleInterface
{
public:
	static Descriptor desc;

	static constexpr unsigned DSHOT_CHANNELS = 4;
	static constexpr uint16_t DSHOT_DISARM_VALUE = 0;
	static constexpr uint16_t DSHOT_MIN_THROTTLE = 1;
	static constexpr uint16_t DSHOT_MAX_THROTTLE = 1999;
	static constexpr uint16_t DSHOT_COMMAND_OFFSET = DSHOT_CMD_MIN_THROTTLE;
	static constexpr hrt_abstime ESC_INIT_DURATION = 1200_ms;
	static constexpr hrt_abstime ESC_INIT_INTERVAL = 2_ms;
	static constexpr unsigned COMMAND_QUEUE_SIZE = 4;
	static constexpr const char *DEFAULT_DEVICE = nullptr;

	SerialDShot(const char *device, uint32_t baudrate, unsigned dshot_rate, bool bidirectional,
		    uint32_t timeout_ms);
	~SerialDShot() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	int init();
	int print_status() override;
	bool updateOutputs(float outputs[MAX_ACTUATORS], unsigned num_outputs,
			   unsigned num_control_groups_updated) override;

private:
	struct Command {
		uint16_t command{DSHOT_CMD_MOTOR_STOP};
		int num_repetitions{0};
		uint8_t motor_mask{0};
		bool save{false};

		bool valid() const { return num_repetitions > 0; }
	};

	void Run() override;
	bool send_motor_frame(const uint16_t values[DSHOT_CHANNELS]);
	bool send_config_frame(uint8_t id, const uint8_t *payload, size_t payload_len);
	void drain_serial();
	void parse_byte(uint8_t byte);
	void process_rx_frame(const uint8_t *frame, size_t length);
	void publish_esc_status();
	int enqueue_command(uint16_t command, int num_repetitions, uint8_t motor_mask, bool save);
	bool dequeue_command(Command &command);
	void clear_command_queue();
	void handle_vehicle_commands();
	void update_params();

	MixingOutput _mixing_output{PARAM_PREFIX, DSHOT_CHANNELS, *this, MixingOutput::SchedulingPolicy::Auto, false, false};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Publication<vehicle_command_ack_s> _command_ack_pub{ORB_ID(vehicle_command_ack)};
	uORB::Publication<esc_status_s> _esc_status_pub{ORB_ID(esc_status)};

	device::Serial _serial;
	uint32_t _baudrate;
	unsigned _dshot_rate;
	bool _bidirectional;
	uint32_t _timeout_ms;
	hrt_abstime _esc_init_start{0};
	bool _esc_init_done{false};
	uint16_t _last_outputs[DSHOT_CHANNELS] {};
	uint16_t _last_period[DSHOT_CHANNELS] {};
	uint16_t _last_adc[2] {};
	uint32_t _telemetry_errors[DSHOT_CHANNELS] {};
	uint8_t _telemetry_valid_mask{0};
	hrt_abstime _last_telemetry_time{0};
	uint16_t _esc_status_counter{0};
	uint8_t _rx_frame[23] {};
	size_t _rx_length{0};
	size_t _rx_expected{0};
	Command _current_command{};
	Command _command_queue[COMMAND_QUEUE_SIZE] {};
	unsigned _command_queue_head{0};
	unsigned _command_queue_tail{0};
	unsigned _command_queue_count{0};
	px4::atomic_bool _armed{false};
	pthread_mutex_t _command_mutex;

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};
	perf_counter_t _io_error_perf{perf_alloc(PC_COUNT, MODULE_NAME": io errors")};
	perf_counter_t _protocol_error_perf{perf_alloc(PC_COUNT, MODULE_NAME": protocol errors")};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::DSHOT_MIN>) _param_dshot_min,
		(ParamInt<px4::params::SD_MOT_POLES>) _param_motor_poles
	)
};
