/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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

/**
 * @file mavlink_main.cpp
 * MAVLink 1.0 protocol implementation.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <assert.h>
#include <math.h>
#include <poll.h>
#include <termios.h>
#include <time.h>
#include <math.h> /* isinf / isnan checks */

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>

#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <geo/geo.h>
#include <dataman/dataman.h>
#include <mathlib/mathlib.h>
#include <mavlink/mavlink_log.h>

#include <uORB/topics/parameter_update.h>

#include "mavlink_bridge_header.h"
#include "mavlink_main.h"
#include "mavlink_messages.h"
#include "mavlink_receiver.h"
#include "mavlink_rate_limiter.h"
#include "mavlink_commands.h"

#ifndef MAVLINK_CRC_EXTRA
#error MAVLINK_CRC_EXTRA has to be defined on PX4 systems
#endif

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#define DEFAULT_DEVICE_NAME	"/dev/ttyS2"
#define MAX_DATA_RATE	10000	// max data rate in bytes/s
#define MAIN_LOOP_DELAY 10000	// 100 Hz @ 1000 bytes/s data rate

static Mavlink *_mavlink_instances = nullptr;

/* TODO: if this is a class member it crashes */
static struct file_operations fops;

/**
 * mavlink app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mavlink_main(int argc, char *argv[]);

extern mavlink_system_t mavlink_system;

static uint64_t last_write_success_times[6] = {0};
static uint64_t last_write_try_times[6] = {0};

/*
 * Internal function to send the bytes through the right serial port
 */
void
mavlink_send_uart_bytes(mavlink_channel_t channel, const uint8_t *ch, int length)
{

	Mavlink *instance;

	switch (channel) {
	case MAVLINK_COMM_0:
		instance = Mavlink::get_instance(0);
		break;

	case MAVLINK_COMM_1:
		instance = Mavlink::get_instance(1);
		break;

	case MAVLINK_COMM_2:
		instance = Mavlink::get_instance(2);
		break;

	case MAVLINK_COMM_3:
		instance = Mavlink::get_instance(3);
		break;
#ifdef MAVLINK_COMM_4

	case MAVLINK_COMM_4:
		instance = Mavlink::get_instance(4);
		break;
#endif
#ifdef MAVLINK_COMM_5

	case MAVLINK_COMM_5:
		instance = Mavlink::get_instance(5);
		break;
#endif
#ifdef MAVLINK_COMM_6

	case MAVLINK_COMM_6:
		instance = Mavlink::get_instance(6);
		break;
#endif

	default:
		return;
	}

	int uart = instance->get_uart_fd();

	ssize_t desired = (sizeof(uint8_t) * length);

	/*
	 * Check if the OS buffer is full and disable HW
	 * flow control if it continues to be full
	 */
	int buf_free = 0;

	if (instance->get_flow_control_enabled()
	    && ioctl(uart, FIONWRITE, (unsigned long)&buf_free) == 0) {

		/* Disable hardware flow control:
		 * if no successful write since a defined time
		 * and if the last try was not the last successful write
		 */
		if (last_write_try_times[(unsigned)channel] != 0 &&
		    hrt_elapsed_time(&last_write_success_times[(unsigned)channel]) > 500 * 1000UL &&
		    last_write_success_times[(unsigned)channel] !=
		    last_write_try_times[(unsigned)channel]) {
			warnx("DISABLING HARDWARE FLOW CONTROL");
			instance->enable_flow_control(false);
		}

	}

	/* If the wait until transmit flag is on, only transmit after we've received messages.
	   Otherwise, transmit all the time. */
	if (instance->should_transmit()) {
		last_write_try_times[(unsigned)channel] = hrt_absolute_time();

		/* check if there is space in the buffer, let it overflow else */
		if (!ioctl(uart, FIONWRITE, (unsigned long)&buf_free)) {

			if (buf_free < desired) {
				/* we don't want to send anything just in half, so return */
				instance->count_txerr();
				instance->count_txerrbytes(desired);
				return;
			}
		}

		ssize_t ret = write(uart, ch, desired);

		if (ret != desired) {
			instance->count_txerr();
			instance->count_txerrbytes(desired);

		} else {
			last_write_success_times[(unsigned)channel] = last_write_try_times[(unsigned)channel];
			instance->count_txbytes(desired);
		}
	}
}

static void usage(void);

Mavlink::Mavlink() :
	_device_name(DEFAULT_DEVICE_NAME),
	_task_should_exit(false),
	next(nullptr),
	_instance_id(0),
	_mavlink_fd(-1),
	_task_running(false),
	_hil_enabled(false),
	_use_hil_gps(false),
	_is_usb_uart(false),
	_wait_to_transmit(false),
	_received_messages(false),
	_main_loop_delay(1000),
	_subscriptions(nullptr),
	_streams(nullptr),
	_mission_manager(nullptr),
	_mission_pub(-1),
	_mission_result_sub(-1),
	_mode(MAVLINK_MODE_NORMAL),
	_channel(MAVLINK_COMM_0),
	_logbuffer {},
	   _total_counter(0),
	   _receive_thread {},
	   _verbose(false),
	   _forwarding_on(false),
	   _passing_on(false),
	   _ftp_on(false),
	   _uart_fd(-1),
	   _baudrate(57600),
	   _datarate(10000),
	   _mavlink_param_queue_index(0),
	   mavlink_link_termination_allowed(false),
	   _subscribe_to_stream(nullptr),
	   _subscribe_to_stream_rate(0.0f),
	   _flow_control_enabled(true),
	   _bytes_tx(0),
	   _bytes_txerr(0),
	   _bytes_rx(0),
	   _bytes_timestamp(0),
	   _rate_tx(0.0f),
	   _rate_txerr(0.0f),
	   _rate_rx(0.0f),
	   _rstatus {},
	   _message_buffer {},
	   _message_buffer_mutex {},
	   _param_initialized(false),
	   _param_system_id(0),
	   _param_component_id(0),
	   _param_system_type(0),
	   _param_use_hil_gps(0),

	   /* performance counters */
	   _loop_perf(perf_alloc(PC_ELAPSED, "mavlink_el")),
	   _txerr_perf(perf_alloc(PC_COUNT, "mavlink_txe"))
{
	fops.ioctl = (int (*)(file *, int, long unsigned int))&mavlink_dev_ioctl;

	_instance_id = Mavlink::instance_count();

	/* set channel according to instance id */
	switch (_instance_id) {
	case 0:
		_channel = MAVLINK_COMM_0;
		break;

	case 1:
		_channel = MAVLINK_COMM_1;
		break;

	case 2:
		_channel = MAVLINK_COMM_2;
		break;

	case 3:
		_channel = MAVLINK_COMM_3;
		break;
#ifdef MAVLINK_COMM_4

	case 4:
		_channel = MAVLINK_COMM_4;
		break;
#endif
#ifdef MAVLINK_COMM_5

	case 5:
		_channel = MAVLINK_COMM_5;
		break;
#endif
#ifdef MAVLINK_COMM_6

	case 6:
		_channel = MAVLINK_COMM_6;
		break;
#endif

	default:
		errx(1, "instance ID is out of range");
		break;
	}
}

Mavlink::~Mavlink()
{
	perf_free(_loop_perf);
	perf_free(_txerr_perf);

	if (_task_running) {
		/* task wakes up every 10ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				//TODO store main task handle in Mavlink instance to allow killing task
				//task_delete(_mavlink_task);
				break;
			}
		} while (_task_running);
	}

	LL_DELETE(_mavlink_instances, this);
}

void
Mavlink::count_txerr()
{
	perf_count(_txerr_perf);
}

void
Mavlink::set_mode(enum MAVLINK_MODE mode)
{
	_mode = mode;
}

int
Mavlink::instance_count()
{
	unsigned inst_index = 0;
	Mavlink *inst;

	LL_FOREACH(::_mavlink_instances, inst) {
		inst_index++;
	}

	return inst_index;
}

Mavlink *
Mavlink::get_instance(unsigned instance)
{
	Mavlink *inst;
	unsigned inst_index = 0;
	LL_FOREACH(::_mavlink_instances, inst) {
		if (instance == inst_index) {
			return inst;
		}

		inst_index++;
	}

	return nullptr;
}

Mavlink *
Mavlink::get_instance_for_device(const char *device_name)
{
	Mavlink *inst;

	LL_FOREACH(::_mavlink_instances, inst) {
		if (strcmp(inst->_device_name, device_name) == 0) {
			return inst;
		}
	}

	return nullptr;
}

int
Mavlink::destroy_all_instances()
{
	/* start deleting from the end */
	Mavlink *inst_to_del = nullptr;
	Mavlink *next_inst = ::_mavlink_instances;

	unsigned iterations = 0;

	warnx("waiting for instances to stop");

	while (next_inst != nullptr) {
		inst_to_del = next_inst;
		next_inst = inst_to_del->next;

		/* set flag to stop thread and wait for all threads to finish */
		inst_to_del->_task_should_exit = true;

		while (inst_to_del->_task_running) {
			printf(".");
			fflush(stdout);
			usleep(10000);
			iterations++;

			if (iterations > 1000) {
				warnx("ERROR: Couldn't stop all mavlink instances.");
				return ERROR;
			}
		}
	}

	printf("\n");
	warnx("all instances stopped");
	return OK;
}

int
Mavlink::get_status_all_instances()
{
	Mavlink *inst = ::_mavlink_instances;

	unsigned iterations = 0;

	while (inst != nullptr) {

		printf("\ninstance #%u:\n", iterations);
		inst->display_status();

		/* move on */
		inst = inst->next;
		iterations++;
	}

	/* return an error if there are no instances */
	return (iterations == 0);
}

bool
Mavlink::instance_exists(const char *device_name, Mavlink *self)
{
	Mavlink *inst = ::_mavlink_instances;

	while (inst != nullptr) {

		/* don't compare with itself */
		if (inst != self && !strcmp(device_name, inst->_device_name)) {
			return true;
		}

		inst = inst->next;
	}

	return false;
}

void
Mavlink::forward_message(const mavlink_message_t *msg, Mavlink *self)
{

	Mavlink *inst;
	LL_FOREACH(_mavlink_instances, inst) {
		if (inst != self) {

			/* if not in normal mode, we are an onboard link
			 * onboard links should only pass on messages from the same system ID */
			if(!(self->_mode != MAVLINK_MODE_NORMAL && msg->sysid != mavlink_system.sysid)) {
				inst->pass_message(msg);
			}
		}
	}
}

int
Mavlink::get_uart_fd(unsigned index)
{
	Mavlink *inst = get_instance(index);

	if (inst) {
		return inst->get_uart_fd();
	}

	return -1;
}

int
Mavlink::get_uart_fd()
{
	return _uart_fd;
}

int
Mavlink::get_instance_id()
{
	return _instance_id;
}

mavlink_channel_t
Mavlink::get_channel()
{
	return _channel;
}

/****************************************************************************
 * MAVLink text message logger
 ****************************************************************************/

int
Mavlink::mavlink_dev_ioctl(struct file *filep, int cmd, unsigned long arg)
{
	switch (cmd) {
	case (int)MAVLINK_IOC_SEND_TEXT_INFO:
	case (int)MAVLINK_IOC_SEND_TEXT_CRITICAL:
	case (int)MAVLINK_IOC_SEND_TEXT_EMERGENCY: {

			const char *txt = (const char *)arg;
//		printf("logmsg: %s\n", txt);
			struct mavlink_logmessage msg;
			strncpy(msg.text, txt, sizeof(msg.text));
			msg.severity = (unsigned char)cmd;

			Mavlink *inst;
			LL_FOREACH(_mavlink_instances, inst) {
				if (!inst->_task_should_exit) {
					mavlink_logbuffer_write(&inst->_logbuffer, &msg);
					inst->_total_counter++;
				}
			}

			return OK;
		}

	default:
		return ENOTTY;
	}
}

void Mavlink::mavlink_update_system(void)
{
	if (!_param_initialized) {
		_param_system_id = param_find("MAV_SYS_ID");
		_param_component_id = param_find("MAV_COMP_ID");
		_param_system_type = param_find("MAV_TYPE");
		_param_use_hil_gps = param_find("MAV_USEHILGPS");
	}

	/* update system and component id */
	int32_t system_id;
	param_get(_param_system_id, &system_id);

	int32_t component_id;
	param_get(_param_component_id, &component_id);


	/* only allow system ID and component ID updates
	 * after reboot - not during operation */
	if (!_param_initialized) {
		if (system_id > 0 && system_id < 255) {
			mavlink_system.sysid = system_id;
		}

		if (component_id > 0 && component_id < 255) {
			mavlink_system.compid = component_id;
		}

		_param_initialized = true;
	}

	/* warn users that they need to reboot to take this
	 * into effect
	 */
	if (system_id != mavlink_system.sysid) {
		send_statustext_critical("Save params and reboot to change SYSID");
	}

	if (component_id != mavlink_system.compid) {
		send_statustext_critical("Save params and reboot to change COMPID");
	}

	int32_t system_type;
	param_get(_param_system_type, &system_type);

	if (system_type >= 0 && system_type < MAV_TYPE_ENUM_END) {
		mavlink_system.type = system_type;
	}

	int32_t use_hil_gps;
	param_get(_param_use_hil_gps, &use_hil_gps);

	_use_hil_gps = (bool)use_hil_gps;
}

int Mavlink::get_system_id()
{
	return mavlink_system.sysid;
}

int Mavlink::get_component_id()
{
	return mavlink_system.compid;
}

int Mavlink::mavlink_open_uart(int baud, const char *uart_name, struct termios *uart_config_original, bool *is_usb)
{
	/* process baud rate */
	int speed;

	switch (baud) {
	case 0:      speed = B0;      break;

	case 50:     speed = B50;     break;

	case 75:     speed = B75;     break;

	case 110:    speed = B110;    break;

	case 134:    speed = B134;    break;

	case 150:    speed = B150;    break;

	case 200:    speed = B200;    break;

	case 300:    speed = B300;    break;

	case 600:    speed = B600;    break;

	case 1200:   speed = B1200;   break;

	case 1800:   speed = B1800;   break;

	case 2400:   speed = B2400;   break;

	case 4800:   speed = B4800;   break;

	case 9600:   speed = B9600;   break;

	case 19200:  speed = B19200;  break;

	case 38400:  speed = B38400;  break;

	case 57600:  speed = B57600;  break;

	case 115200: speed = B115200; break;

	case 230400: speed = B230400; break;

	case 460800: speed = B460800; break;

	case 921600: speed = B921600; break;

	default:
		warnx("ERROR: Unsupported baudrate: %d\n\tsupported examples:\n\t9600, 19200, 38400, 57600\t\n115200\n230400\n460800\n921600\n",
		      baud);
		return -EINVAL;
	}

	/* open uart */
	_uart_fd = open(uart_name, O_RDWR | O_NOCTTY);

	if (_uart_fd < 0) {
		return _uart_fd;
	}


	/* Try to set baud rate */
	struct termios uart_config;
	int termios_state;
	*is_usb = false;

	/* Back up the original uart configuration to restore it after exit */
	if ((termios_state = tcgetattr(_uart_fd, uart_config_original)) < 0) {
		warnx("ERR GET CONF %s: %d\n", uart_name, termios_state);
		close(_uart_fd);
		return -1;
	}

	/* Fill the struct for the new configuration */
	tcgetattr(_uart_fd, &uart_config);

	/* Clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	/* USB serial is indicated by /dev/ttyACM0*/
	if (strcmp(uart_name, "/dev/ttyACM0") != OK && strcmp(uart_name, "/dev/ttyACM1") != OK) {

		/* Set baud rate */
		if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
			warnx("ERR SET BAUD %s: %d\n", uart_name, termios_state);
			close(_uart_fd);
			return -1;
		}

	}

	if ((termios_state = tcsetattr(_uart_fd, TCSANOW, &uart_config)) < 0) {
		warnx("ERR SET CONF %s\n", uart_name);
		close(_uart_fd);
		return -1;
	}

	if (!_is_usb_uart) {
		/*
		 * Setup hardware flow control. If the port has no RTS pin this call will fail,
		 * which is not an issue, but requires a separate call so we can fail silently.
		 */
		(void)tcgetattr(_uart_fd, &uart_config);
		uart_config.c_cflag |= CRTS_IFLOW;
		(void)tcsetattr(_uart_fd, TCSANOW, &uart_config);

		/* setup output flow control */
		if (enable_flow_control(true)) {
			warnx("hardware flow control not supported");
		}
	}

	return _uart_fd;
}

int
Mavlink::enable_flow_control(bool enabled)
{
	// We can't do this on USB - skip
	if (_is_usb_uart) {
		return OK;
	}

	struct termios uart_config;

	int ret = tcgetattr(_uart_fd, &uart_config);

	if (enabled) {
		uart_config.c_cflag |= CRTSCTS;

	} else {
		uart_config.c_cflag &= ~CRTSCTS;
	}

	ret = tcsetattr(_uart_fd, TCSANOW, &uart_config);

	if (!ret) {
		_flow_control_enabled = enabled;
	}

	return ret;
}

int
Mavlink::set_hil_enabled(bool hil_enabled)
{
	int ret = OK;

	/* enable HIL */
	if (hil_enabled && !_hil_enabled) {
		_hil_enabled = true;
		float rate_mult = _datarate / 1000.0f;
		configure_stream("HIL_CONTROLS", 15.0f * rate_mult);
	}

	/* disable HIL */
	if (!hil_enabled && _hil_enabled) {
		_hil_enabled = false;
		configure_stream("HIL_CONTROLS", 0.0f);

	} else {
		ret = ERROR;
	}

	return ret;
}

void
Mavlink::send_message(const mavlink_message_t *msg)
{
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
	mavlink_send_uart_bytes(_channel, buf, len);
}

void
Mavlink::handle_message(const mavlink_message_t *msg)
{
	/* handle packet with mission manager */
	_mission_manager->handle_message(msg);

	/* handle packet with parameter component */
	mavlink_pm_message_handler(_channel, msg);

	if (get_forwarding_on()) {
		/* forward any messages to other mavlink instances */
		Mavlink::forward_message(msg, this);
	}
}

int
Mavlink::mavlink_pm_queued_send()
{
	if (_mavlink_param_queue_index < param_count()) {
		mavlink_pm_send_param(param_for_index(_mavlink_param_queue_index));
		_mavlink_param_queue_index++;
		return 0;

	} else {
		return 1;
	}
}

void Mavlink::mavlink_pm_start_queued_send()
{
	_mavlink_param_queue_index = 0;
}

int Mavlink::mavlink_pm_send_param_for_index(uint16_t index)
{
	return mavlink_pm_send_param(param_for_index(index));
}

int Mavlink::mavlink_pm_send_param_for_name(const char *name)
{
	return mavlink_pm_send_param(param_find(name));
}

int Mavlink::mavlink_pm_send_param(param_t param)
{
	if (param == PARAM_INVALID) { return 1; }

	/* buffers for param transmission */
	char name_buf[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN];
	float val_buf;
	mavlink_message_t tx_msg;

	/* query parameter type */
	param_type_t type = param_type(param);
	/* copy parameter name */
	strncpy((char *)name_buf, param_name(param), MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);

	/*
	 * Map onboard parameter type to MAVLink type,
	 * endianess matches (both little endian)
	 */
	uint8_t mavlink_type;

	if (type == PARAM_TYPE_INT32) {
		mavlink_type = MAVLINK_TYPE_INT32_T;

	} else if (type == PARAM_TYPE_FLOAT) {
		mavlink_type = MAVLINK_TYPE_FLOAT;

	} else {
		mavlink_type = MAVLINK_TYPE_FLOAT;
	}

	/*
	 * get param value, since MAVLink encodes float and int params in the same
	 * space during transmission, copy param onto float val_buf
	 */

	int ret;

	if ((ret = param_get(param, &val_buf)) != OK) {
		return ret;
	}

	mavlink_msg_param_value_pack_chan(mavlink_system.sysid,
					  mavlink_system.compid,
					  _channel,
					  &tx_msg,
					  name_buf,
					  val_buf,
					  mavlink_type,
					  param_count(),
					  param_get_index(param));
	send_message(&tx_msg);
	return OK;
}

void Mavlink::mavlink_pm_message_handler(const mavlink_channel_t chan, const mavlink_message_t *msg)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
			mavlink_param_request_list_t req;
			mavlink_msg_param_request_list_decode(msg, &req);

			if (req.target_system == mavlink_system.sysid &&
			    (req.target_component == mavlink_system.compid || req.target_component == MAV_COMP_ID_ALL)) {
				/* Start sending parameters */
				mavlink_pm_start_queued_send();
				send_statustext_info("[pm] sending list");
			}
		} break;

	case MAVLINK_MSG_ID_PARAM_SET: {

			/* Handle parameter setting */

			if (msg->msgid == MAVLINK_MSG_ID_PARAM_SET) {
				mavlink_param_set_t mavlink_param_set;
				mavlink_msg_param_set_decode(msg, &mavlink_param_set);

				if (mavlink_param_set.target_system == mavlink_system.sysid
				    && ((mavlink_param_set.target_component == mavlink_system.compid)
					|| (mavlink_param_set.target_component == MAV_COMP_ID_ALL))) {
					/* local name buffer to enforce null-terminated string */
					char name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1];
					strncpy(name, mavlink_param_set.param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
					/* enforce null termination */
					name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN] = '\0';
					/* attempt to find parameter, set and send it */
					param_t param = param_find(name);

					if (param == PARAM_INVALID) {
						char buf[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
						sprintf(buf, "[pm] unknown: %s", name);
						send_statustext_info(buf);

					} else {
						/* set and send parameter */
						param_set(param, &(mavlink_param_set.param_value));
						mavlink_pm_send_param(param);
					}
				}
			}
		} break;

	case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
			mavlink_param_request_read_t mavlink_param_request_read;
			mavlink_msg_param_request_read_decode(msg, &mavlink_param_request_read);

			if (mavlink_param_request_read.target_system == mavlink_system.sysid
			    && ((mavlink_param_request_read.target_component == mavlink_system.compid)
				|| (mavlink_param_request_read.target_component == MAV_COMP_ID_ALL))) {
				/* when no index is given, loop through string ids and compare them */
				if (mavlink_param_request_read.param_index == -1) {
					/* local name buffer to enforce null-terminated string */
					char name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1];
					strncpy(name, mavlink_param_request_read.param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
					/* enforce null termination */
					name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN] = '\0';
					/* attempt to find parameter and send it */
					mavlink_pm_send_param_for_name(name);

				} else {
					/* when index is >= 0, send this parameter again */
					mavlink_pm_send_param_for_index(mavlink_param_request_read.param_index);
				}
			}

		} break;
	}
}

int
Mavlink::send_statustext_info(const char *string)
{
	return send_statustext(MAVLINK_IOC_SEND_TEXT_INFO, string);
}

int
Mavlink::send_statustext_critical(const char *string)
{
	return send_statustext(MAVLINK_IOC_SEND_TEXT_CRITICAL, string);
}

int
Mavlink::send_statustext_emergency(const char *string)
{
	return send_statustext(MAVLINK_IOC_SEND_TEXT_EMERGENCY, string);
}

int
Mavlink::send_statustext(unsigned severity, const char *string)
{
	const int len = MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN;
	mavlink_statustext_t statustext;

	int i = 0;

	while (i < len - 1) {
		statustext.text[i] = string[i];

		if (string[i] == '\0') {
			break;
		}

		i++;
	}

	if (i > 1) {
		/* Enforce null termination */
		statustext.text[i] = '\0';

		/* Map severity */
		switch (severity) {
		case MAVLINK_IOC_SEND_TEXT_INFO:
			statustext.severity = MAV_SEVERITY_INFO;
			break;

		case MAVLINK_IOC_SEND_TEXT_CRITICAL:
			statustext.severity = MAV_SEVERITY_CRITICAL;
			break;

		case MAVLINK_IOC_SEND_TEXT_EMERGENCY:
			statustext.severity = MAV_SEVERITY_EMERGENCY;
			break;
		}

		mavlink_msg_statustext_send(_channel, statustext.severity, statustext.text);
		return OK;

	} else {
		return ERROR;
	}
}

MavlinkOrbSubscription *Mavlink::add_orb_subscription(const orb_id_t topic)
{
	/* check if already subscribed to this topic */
	MavlinkOrbSubscription *sub;

	LL_FOREACH(_subscriptions, sub) {
		if (sub->get_topic() == topic) {
			/* already subscribed */
			return sub;
		}
	}

	/* add new subscription */
	MavlinkOrbSubscription *sub_new = new MavlinkOrbSubscription(topic);

	LL_APPEND(_subscriptions, sub_new);

	return sub_new;
}

int
Mavlink::configure_stream(const char *stream_name, const float rate)
{
	/* calculate interval in us, 0 means disabled stream */
	unsigned int interval = (rate > 0.0f) ? (1000000.0f / rate) : 0;

	/* search if stream exists */
	MavlinkStream *stream;
	LL_FOREACH(_streams, stream) {
		if (strcmp(stream_name, stream->get_name()) == 0) {
			if (interval > 0) {
				/* set new interval */
				stream->set_interval(interval);

			} else {
				/* delete stream */
				LL_DELETE(_streams, stream);
				delete stream;
				warnx("deleted stream %s", stream->get_name());
			}

			return OK;
		}
	}

	if (interval == 0) {
		/* stream was not active and is requested to be disabled, do nothing */
		return OK;
	}

	/* search for stream with specified name in supported streams list */
	for (unsigned int i = 0; streams_list[i] != nullptr; i++) {

		if (strcmp(stream_name, streams_list[i]->get_name()) == 0) {
			/* create new instance */
			stream = streams_list[i]->new_instance();
			stream->set_channel(get_channel());
			stream->set_interval(interval);
			stream->subscribe(this);
			LL_APPEND(_streams, stream);

			return OK;
		}
	}

	/* if we reach here, the stream list does not contain the stream */
	warnx("stream %s not found", stream_name);

	return ERROR;
}

void
Mavlink::configure_stream_threadsafe(const char *stream_name, const float rate)
{
	/* orb subscription must be done from the main thread,
	 * set _subscribe_to_stream and _subscribe_to_stream_rate fields
	 * which polled in mavlink main loop */
	if (!_task_should_exit) {
		/* wait for previous subscription completion */
		while (_subscribe_to_stream != nullptr) {
			usleep(MAIN_LOOP_DELAY / 2);
		}

		/* copy stream name */
		unsigned n = strlen(stream_name) + 1;
		char *s = new char[n];
		strcpy(s, stream_name);

		/* set subscription task */
		_subscribe_to_stream_rate = rate;
		_subscribe_to_stream = s;

		/* wait for subscription */
		do {
			usleep(MAIN_LOOP_DELAY / 2);
		} while (_subscribe_to_stream != nullptr);
	}
}

int
Mavlink::message_buffer_init(int size)
{

	_message_buffer.size = size;
	_message_buffer.write_ptr = 0;
	_message_buffer.read_ptr = 0;
	_message_buffer.data = (char *)malloc(_message_buffer.size);

	int ret;

	if (_message_buffer.data == 0) {
		ret = ERROR;
		_message_buffer.size = 0;

	} else {
		ret = OK;
	}

	return ret;
}

void
Mavlink::message_buffer_destroy()
{
	_message_buffer.size = 0;
	_message_buffer.write_ptr = 0;
	_message_buffer.read_ptr = 0;
	free(_message_buffer.data);
}

int
Mavlink::message_buffer_count()
{
	int n = _message_buffer.write_ptr - _message_buffer.read_ptr;

	if (n < 0) {
		n += _message_buffer.size;
	}

	return n;
}

int
Mavlink::message_buffer_is_empty()
{
	return _message_buffer.read_ptr == _message_buffer.write_ptr;
}


bool
Mavlink::message_buffer_write(const void *ptr, int size)
{
	// bytes available to write
	int available = _message_buffer.read_ptr - _message_buffer.write_ptr - 1;

	if (available < 0) {
		available += _message_buffer.size;
	}

	if (size > available) {
		// buffer overflow
		return false;
	}

	char *c = (char *) ptr;
	int n = _message_buffer.size - _message_buffer.write_ptr;	// bytes to end of the buffer

	if (n < size) {
		// message goes over end of the buffer
		memcpy(&(_message_buffer.data[_message_buffer.write_ptr]), c, n);
		_message_buffer.write_ptr = 0;

	} else {
		n = 0;
	}

	// now: n = bytes already written
	int p = size - n;	// number of bytes to write
	memcpy(&(_message_buffer.data[_message_buffer.write_ptr]), &(c[n]), p);
	_message_buffer.write_ptr = (_message_buffer.write_ptr + p) % _message_buffer.size;
	return true;
}

int
Mavlink::message_buffer_get_ptr(void **ptr, bool *is_part)
{
	// bytes available to read
	int available = _message_buffer.write_ptr - _message_buffer.read_ptr;

	if (available == 0) {
		return 0;	// buffer is empty
	}

	int n = 0;

	if (available > 0) {
		// read pointer is before write pointer, all available bytes can be read
		n = available;
		*is_part = false;

	} else {
		// read pointer is after write pointer, read bytes from read_ptr to end of the buffer
		n = _message_buffer.size - _message_buffer.read_ptr;
		*is_part = _message_buffer.write_ptr > 0;
	}

	*ptr = &(_message_buffer.data[_message_buffer.read_ptr]);
	return n;
}

void
Mavlink::message_buffer_mark_read(int n)
{
	_message_buffer.read_ptr = (_message_buffer.read_ptr + n) % _message_buffer.size;
}

void
Mavlink::pass_message(const mavlink_message_t *msg)
{
	if (_passing_on) {
		/* size is 8 bytes plus variable payload */
		int size = MAVLINK_NUM_NON_PAYLOAD_BYTES + msg->len;
		pthread_mutex_lock(&_message_buffer_mutex);
		message_buffer_write(msg, size);
		pthread_mutex_unlock(&_message_buffer_mutex);
	}
}

float
Mavlink::get_rate_mult()
{
	return _datarate / 1000.0f;
}

int
Mavlink::task_main(int argc, char *argv[])
{
	int ch;
	_baudrate = 57600;
	_datarate = 0;
	_mode = MAVLINK_MODE_NORMAL;

	/* work around some stupidity in task_create's argv handling */
	argc -= 2;
	argv += 2;

	/* don't exit from getopt loop to leave getopt global variables in consistent state,
	 * set error flag instead */
	bool err_flag = false;

	while ((ch = getopt(argc, argv, "b:r:d:m:fpvwx")) != EOF) {
		switch (ch) {
		case 'b':
			_baudrate = strtoul(optarg, NULL, 10);

			if (_baudrate < 9600 || _baudrate > 921600) {
				warnx("invalid baud rate '%s'", optarg);
				err_flag = true;
			}

			break;

		case 'r':
			_datarate = strtoul(optarg, NULL, 10);

			if (_datarate < 10 || _datarate > MAX_DATA_RATE) {
				warnx("invalid data rate '%s'", optarg);
				err_flag = true;
			}

			break;

		case 'd':
			_device_name = optarg;
			break;

//		case 'e':
//			mavlink_link_termination_allowed = true;
//			break;

		case 'm':
			if (strcmp(optarg, "custom") == 0) {
				_mode = MAVLINK_MODE_CUSTOM;

			} else if (strcmp(optarg, "camera") == 0) {
				_mode = MAVLINK_MODE_CAMERA;
			}

			break;

		case 'f':
			_forwarding_on = true;
			break;

		case 'p':
			_passing_on = true;
			break;

		case 'v':
			_verbose = true;
			break;

		case 'w':
			_wait_to_transmit = true;
			break;

		case 'x':
			_ftp_on = true;
			break;

		default:
			err_flag = true;
			break;
		}
	}

	if (err_flag) {
		usage();
		return ERROR;
	}

	if (_datarate == 0) {
		/* convert bits to bytes and use 1/2 of bandwidth by default */
		_datarate = _baudrate / 20;
	}

	if (_datarate > MAX_DATA_RATE) {
		_datarate = MAX_DATA_RATE;
	}

	if (Mavlink::instance_exists(_device_name, this)) {
		warnx("mavlink instance for %s already running", _device_name);
		return ERROR;
	}

	/* inform about mode */
	switch (_mode) {
	case MAVLINK_MODE_NORMAL:
		warnx("mode: NORMAL");
		break;

	case MAVLINK_MODE_CUSTOM:
		warnx("mode: CUSTOM");
		break;

	case MAVLINK_MODE_CAMERA:
		warnx("mode: CAMERA");
		break;

	default:
		warnx("ERROR: Unknown mode");
		break;
	}

	warnx("data rate: %d Bytes/s, port: %s, baud: %d", _datarate, _device_name, _baudrate);

	/* flush stdout in case MAVLink is about to take it over */
	fflush(stdout);

	struct termios uart_config_original;

	/* default values for arguments */
	_uart_fd = mavlink_open_uart(_baudrate, _device_name, &uart_config_original, &_is_usb_uart);

	if (_uart_fd < 0) {
		warn("could not open %s", _device_name);
		return ERROR;
	}

	/* initialize mavlink text message buffering */
	mavlink_logbuffer_init(&_logbuffer, 5);

	/* if we are passing on mavlink messages, we need to prepare a buffer for this instance */
	if (_passing_on || _ftp_on) {
		/* initialize message buffer if multiplexing is on or its needed for FTP.
		 * make space for two messages plus off-by-one space as we use the empty element
		 * marker ring buffer approach.
		 */
		if (OK != message_buffer_init(2 * MAVLINK_MAX_PACKET_LEN + 2)) {
			errx(1, "can't allocate message buffer, exiting");
		}

		/* initialize message buffer mutex */
		pthread_mutex_init(&_message_buffer_mutex, NULL);
	}

	/* create the device node that's used for sending text log messages, etc. */
	register_driver(MAVLINK_LOG_DEVICE, &fops, 0666, NULL);

	/* initialize logging device */
	_mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);

	/* Initialize system properties */
	mavlink_update_system();

	/* start the MAVLink receiver */
	_receive_thread = MavlinkReceiver::receive_start(this);

	_mission_result_sub = orb_subscribe(ORB_ID(mission_result));

	/* create mission manager */
	_mission_manager = new MavlinkMissionManager(this);
	_mission_manager->set_verbose(_verbose);

	_task_running = true;

	MavlinkOrbSubscription *param_sub = add_orb_subscription(ORB_ID(parameter_update));
	uint64_t param_time = 0;
	MavlinkOrbSubscription *status_sub = add_orb_subscription(ORB_ID(vehicle_status));
	uint64_t status_time = 0;

	struct vehicle_status_s status;
	status_sub->update(&status_time, &status);

	MavlinkCommandsStream commands_stream(this, _channel);

	/* add default streams depending on mode and intervals depending on datarate */
	float rate_mult = get_rate_mult();

	configure_stream("HEARTBEAT", 1.0f);

	switch (_mode) {
	case MAVLINK_MODE_NORMAL:
		configure_stream("SYS_STATUS", 1.0f);
		configure_stream("GPS_GLOBAL_ORIGIN", 0.5f);
		configure_stream("HIGHRES_IMU", 1.0f * rate_mult);
		configure_stream("ATTITUDE", 10.0f * rate_mult);
		configure_stream("VFR_HUD", 8.0f * rate_mult);
		configure_stream("GPS_RAW_INT", 1.0f * rate_mult);
		configure_stream("GLOBAL_POSITION_INT", 3.0f * rate_mult);
		configure_stream("LOCAL_POSITION_NED", 3.0f * rate_mult);
		configure_stream("RC_CHANNELS_RAW", 1.0f * rate_mult);
		configure_stream("NAMED_VALUE_FLOAT", 1.0f * rate_mult);
		configure_stream("GLOBAL_POSITION_SETPOINT_INT", 3.0f * rate_mult);
		configure_stream("ROLL_PITCH_YAW_THRUST_SETPOINT", 3.0f * rate_mult);
		configure_stream("DISTANCE_SENSOR", 0.5f);
		break;

	case MAVLINK_MODE_CAMERA:
		configure_stream("SYS_STATUS", 1.0f);
		configure_stream("ATTITUDE", 15.0f * rate_mult);
		configure_stream("GLOBAL_POSITION_INT", 15.0f * rate_mult);
		configure_stream("CAMERA_CAPTURE", 1.0f);
		break;

	default:
		break;
	}

	/* don't send parameters on startup without request */
	_mavlink_param_queue_index = param_count();

	MavlinkRateLimiter fast_rate_limiter(30000.0f / rate_mult);

	/* set main loop delay depending on data rate to minimize CPU overhead */
	_main_loop_delay = MAIN_LOOP_DELAY / rate_mult;

	/* now the instance is fully initialized and we can bump the instance count */
	LL_APPEND(_mavlink_instances, this);

	while (!_task_should_exit) {
		/* main loop */
		usleep(_main_loop_delay);

		perf_begin(_loop_perf);

		hrt_abstime t = hrt_absolute_time();

		if (param_sub->update(&param_time, nullptr)) {
			/* parameters updated */
			mavlink_update_system();
		}

		if (status_sub->update(&status_time, &status)) {
			/* switch HIL mode if required */
			set_hil_enabled(status.hil_state == HIL_STATE_ON);
		}

		/* update commands stream */
		commands_stream.update(t);

		/* check for requested subscriptions */
		if (_subscribe_to_stream != nullptr) {
			if (OK == configure_stream(_subscribe_to_stream, _subscribe_to_stream_rate)) {
				if (_subscribe_to_stream_rate > 0.0f) {
					warnx("stream %s on device %s enabled with rate %.1f Hz", _subscribe_to_stream, _device_name,
					      (double)_subscribe_to_stream_rate);

				} else {
					warnx("stream %s on device %s disabled", _subscribe_to_stream, _device_name);
				}

			} else {
				warnx("stream %s on device %s not found", _subscribe_to_stream, _device_name);
			}

			delete _subscribe_to_stream;
			_subscribe_to_stream = nullptr;
		}

		/* update streams */
		MavlinkStream *stream;
		LL_FOREACH(_streams, stream) {
			stream->update(t);
		}

		if (fast_rate_limiter.check(t)) {
			mavlink_pm_queued_send();
			_mission_manager->eventloop();

			if (!mavlink_logbuffer_is_empty(&_logbuffer)) {
				struct mavlink_logmessage msg;
				int lb_ret = mavlink_logbuffer_read(&_logbuffer, &msg);

				if (lb_ret == OK) {
					send_statustext(msg.severity, msg.text);
				}
			}
		}

		/* pass messages from other UARTs or FTP worker */
		if (_passing_on || _ftp_on) {

			bool is_part;
			uint8_t *read_ptr;
			uint8_t *write_ptr;

			pthread_mutex_lock(&_message_buffer_mutex);
			int available = message_buffer_get_ptr((void **)&read_ptr, &is_part);
			pthread_mutex_unlock(&_message_buffer_mutex);

			if (available > 0) {
				// Reconstruct message from buffer

				mavlink_message_t msg;
				write_ptr = (uint8_t *)&msg;

				// Pull a single message from the buffer
				size_t read_count = available;

				if (read_count > sizeof(mavlink_message_t)) {
					read_count = sizeof(mavlink_message_t);
				}

				memcpy(write_ptr, read_ptr, read_count);

				// We hold the mutex until after we complete the second part of the buffer. If we don't
				// we may end up breaking the empty slot overflow detection semantics when we mark the
				// possibly partial read below.
				pthread_mutex_lock(&_message_buffer_mutex);

				message_buffer_mark_read(read_count);

				/* write second part of buffer if there is some */
				if (is_part && read_count < sizeof(mavlink_message_t)) {
					write_ptr += read_count;
					available = message_buffer_get_ptr((void **)&read_ptr, &is_part);
					read_count = sizeof(mavlink_message_t) - read_count;
					memcpy(write_ptr, read_ptr, read_count);
					message_buffer_mark_read(available);
				}

				pthread_mutex_unlock(&_message_buffer_mutex);

				_mavlink_resend_uart(_channel, &msg);
			}
		}

		/* update TX/RX rates*/
		if (t > _bytes_timestamp + 1000000) {
			if (_bytes_timestamp != 0) {
				float dt = (t - _bytes_timestamp) / 1000.0f;
				_rate_tx = _bytes_tx / dt;
				_rate_txerr = _bytes_txerr / dt;
				_rate_rx = _bytes_rx / dt;
				_bytes_tx = 0;
				_bytes_txerr = 0;
				_bytes_rx = 0;
			}
			_bytes_timestamp = t;
		}

		perf_end(_loop_perf);
	}

	delete _mission_manager;

	delete _subscribe_to_stream;
	_subscribe_to_stream = nullptr;

	/* delete streams */
	MavlinkStream *stream_to_del = nullptr;
	MavlinkStream *stream_next = _streams;

	while (stream_next != nullptr) {
		stream_to_del = stream_next;
		stream_next = stream_to_del->next;
		delete stream_to_del;
	}

	_streams = nullptr;

	/* delete subscriptions */
	MavlinkOrbSubscription *sub_to_del = nullptr;
	MavlinkOrbSubscription *sub_next = _subscriptions;

	while (sub_next != nullptr) {
		sub_to_del = sub_next;
		sub_next = sub_to_del->next;
		delete sub_to_del;
	}

	_subscriptions = nullptr;

	warnx("waiting for UART receive thread");

	/* wait for threads to complete */
	pthread_join(_receive_thread, NULL);

	/* reset the UART flags to original state */
	tcsetattr(_uart_fd, TCSANOW, &uart_config_original);

	/* close UART */
	close(_uart_fd);

	/* close mavlink logging device */
	close(_mavlink_fd);

	if (_passing_on || _ftp_on) {
		message_buffer_destroy();
		pthread_mutex_destroy(&_message_buffer_mutex);
	}

	/* destroy log buffer */
	mavlink_logbuffer_destroy(&_logbuffer);

	warnx("exiting");
	_task_running = false;

	return OK;
}

int Mavlink::start_helper(int argc, char *argv[])
{
	/* create the instance in task context */
	Mavlink *instance = new Mavlink();

	int res;

	if (!instance) {

		/* out of memory */
		res = -ENOMEM;
		warnx("OUT OF MEM");

	} else {
		/* this will actually only return once MAVLink exits */
		res = instance->task_main(argc, argv);

		/* delete instance on main thread end */
		delete instance;
	}

	return res;
}

int
Mavlink::start(int argc, char *argv[])
{
	// Wait for the instance count to go up one
	// before returning to the shell
	int ic = Mavlink::instance_count();

	// Instantiate thread
	char buf[24];
	sprintf(buf, "mavlink_if%d", ic);

	// This is where the control flow splits
	// between the starting task and the spawned
	// task - start_helper() only returns
	// when the started task exits.
	task_spawn_cmd(buf,
		       SCHED_DEFAULT,
		       SCHED_PRIORITY_DEFAULT,
		       2700,
		       (main_t)&Mavlink::start_helper,
		       (const char **)argv);

	// Ensure that this shell command
	// does not return before the instance
	// is fully initialized. As this is also
	// the only path to create a new instance,
	// this is effectively a lock on concurrent
	// instance starting. XXX do a real lock.

	// Sleep 500 us between each attempt
	const unsigned sleeptime = 500;

	// Wait 100 ms max for the startup.
	const unsigned limit = 100 * 1000 / sleeptime;

	unsigned count = 0;

	while (ic == Mavlink::instance_count() && count < limit) {
		::usleep(sleeptime);
		count++;
	}

	return OK;
}

void
Mavlink::display_status()
{

	if (_rstatus.heartbeat_time > 0) {
		printf("\tGCS heartbeat:\t%llu us ago\n", hrt_elapsed_time(&_rstatus.heartbeat_time));
	}

	if (_rstatus.timestamp > 0) {

		printf("\ttype:\t\t");

		switch (_rstatus.type) {
		case TELEMETRY_STATUS_RADIO_TYPE_3DR_RADIO:
			printf("3DR RADIO\n");
			break;

		default:
			printf("UNKNOWN RADIO\n");
			break;
		}

		printf("\trssi:\t\t%d\n", _rstatus.rssi);
		printf("\tremote rssi:\t%u\n", _rstatus.remote_rssi);
		printf("\ttxbuf:\t\t%u\n", _rstatus.txbuf);
		printf("\tnoise:\t\t%d\n", _rstatus.noise);
		printf("\tremote noise:\t%u\n", _rstatus.remote_noise);
		printf("\trx errors:\t%u\n", _rstatus.rxerrors);
		printf("\tfixed:\t\t%u\n", _rstatus.fixed);

	} else {
		printf("\tno telem status.\n");
	}
	printf("\trates:\n");
	printf("\ttx: %.3f kB/s\n", (double)_rate_tx);
	printf("\ttxerr: %.3f kB/s\n", (double)_rate_txerr);
	printf("\trx: %.3f kB/s\n", (double)_rate_rx);
}

int
Mavlink::stream_command(int argc, char *argv[])
{
	const char *device_name = DEFAULT_DEVICE_NAME;
	float rate = -1.0f;
	const char *stream_name = nullptr;

	argc -= 2;
	argv += 2;

	/* don't exit from getopt loop to leave getopt global variables in consistent state,
	 * set error flag instead */
	bool err_flag = false;

	int i = 0;

	while (i < argc) {

		if (0 == strcmp(argv[i], "-r") && i < argc - 1) {
			rate = strtod(argv[i + 1], nullptr);

			if (rate < 0.0f) {
				err_flag = true;
			}

			i++;

		} else if (0 == strcmp(argv[i], "-d") && i < argc - 1) {
			device_name = argv[i + 1];
			i++;

		} else if (0 == strcmp(argv[i], "-s") && i < argc - 1) {
			stream_name = argv[i + 1];
			i++;

		} else {
			err_flag = true;
		}

		i++;
	}

	if (!err_flag && rate >= 0.0f && stream_name != nullptr) {
		Mavlink *inst = get_instance_for_device(device_name);

		if (inst != nullptr) {
			inst->configure_stream_threadsafe(stream_name, rate);

		} else {

			// If the link is not running we should complain, but not fall over
			// because this is so easy to get wrong and not fatal. Warning is sufficient.
			errx(0, "mavlink for device %s is not running", device_name);
		}

	} else {
		errx(1, "usage: mavlink stream [-d device] -s stream -r rate");
	}

	return OK;
}

static void usage()
{
	warnx("usage: mavlink {start|stop-all|stream} [-d device] [-b baudrate]\n\t[-r rate][-m mode] [-s stream] [-f] [-p] [-v] [-w] [-x]");
}

int mavlink_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
		exit(1);
	}

	if (!strcmp(argv[1], "start")) {
		return Mavlink::start(argc, argv);

	} else if (!strcmp(argv[1], "stop")) {
		warnx("mavlink stop is deprecated, use stop-all instead");
		usage();
		exit(1);

	} else if (!strcmp(argv[1], "stop-all")) {
		return Mavlink::destroy_all_instances();

	} else if (!strcmp(argv[1], "status")) {
		return Mavlink::get_status_all_instances();

	} else if (!strcmp(argv[1], "stream")) {
		return Mavlink::stream_command(argc, argv);

	} else {
		usage();
		exit(1);
	}

	return 0;
}
