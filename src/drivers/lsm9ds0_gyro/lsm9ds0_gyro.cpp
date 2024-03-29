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

/**
 * @file lsm9ds0_gyro.cpp
 * Driver for the ST LSM9DS0_GYRO MEMS gyro connected via SPI.
 */

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>

#include <drivers/drv_hrt.h>
#include <drivers/device/spi.h>
#include <drivers/drv_gyro.h>
#include <drivers/device/ringbuffer.h>

#include <board_config.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>

#define LSM9DS0_GYRO_DEVICE_PATH "/dev/lsm9ds0_gyro"

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

/* Orientation on board */
#define SENSOR_BOARD_ROTATION_000_DEG	0
#define SENSOR_BOARD_ROTATION_090_DEG	1
#define SENSOR_BOARD_ROTATION_180_DEG	2
#define SENSOR_BOARD_ROTATION_270_DEG	3

/* SPI protocol address bits */
#define DIR_READ				(1<<7)
#define DIR_WRITE				(0<<7)
#define ADDR_INCREMENT				(1<<6)

/* register addresses */
#define ADDR_WHO_AM_I			0x0F
#define WHO_I_AM_H 				0xD7
#define WHO_I_AM				0xD4

#define ADDR_CTRL_REG1			0x20
#define REG1_RATE_LP_MASK			0xF0 /* Mask to guard partial register update */
/* keep lowpass low to avoid noise issues */
#define RATE_95HZ_LP_25HZ		((0<<7) | (0<<6) | (0<<5) | (1<<4))
#define RATE_190HZ_LP_25HZ		((0<<7) | (1<<6) | (0<<5) | (1<<4))
#define RATE_190HZ_LP_50HZ		((0<<7) | (1<<6) | (1<<5) | (0<<4))
#define RATE_190HZ_LP_70HZ		((0<<7) | (1<<6) | (1<<5) | (1<<4))
#define RATE_380HZ_LP_20HZ		((1<<7) | (0<<6) | (1<<5) | (0<<4))
#define RATE_380HZ_LP_25HZ		((1<<7) | (0<<6) | (0<<5) | (1<<4))
#define RATE_380HZ_LP_50HZ		((1<<7) | (0<<6) | (1<<5) | (0<<4))
#define RATE_380HZ_LP_100HZ		((1<<7) | (0<<6) | (1<<5) | (1<<4))
#define RATE_760HZ_LP_30HZ		((1<<7) | (1<<6) | (0<<5) | (0<<4))
#define RATE_760HZ_LP_35HZ		((1<<7) | (1<<6) | (0<<5) | (1<<4))
#define RATE_760HZ_LP_50HZ		((1<<7) | (1<<6) | (1<<5) | (0<<4))
#define RATE_760HZ_LP_100HZ		((1<<7) | (1<<6) | (1<<5) | (1<<4))

#define ADDR_CTRL_REG2			0x21
#define ADDR_CTRL_REG3			0x22
#define ADDR_CTRL_REG4			0x23
#define REG4_RANGE_MASK				0x30 /* Mask to guard partial register update */
#define RANGE_250DPS				(0<<4)
#define RANGE_500DPS				(1<<4)
#define RANGE_2000DPS				(3<<4)

#define ADDR_CTRL_REG5			0x24
#define ADDR_REFERENCE			0x25
#define ADDR_OUT_TEMP			0x26
#define ADDR_STATUS_REG			0x27
#define ADDR_OUT_X_L			0x28
#define ADDR_OUT_X_H			0x29
#define ADDR_OUT_Y_L			0x2A
#define ADDR_OUT_Y_H			0x2B
#define ADDR_OUT_Z_L			0x2C
#define ADDR_OUT_Z_H			0x2D
#define ADDR_FIFO_CTRL_REG		0x2E
#define ADDR_FIFO_SRC_REG		0x2F
#define ADDR_INT1_CFG			0x30
#define ADDR_INT1_SRC			0x31
#define ADDR_INT1_TSH_XH		0x32
#define ADDR_INT1_TSH_XL		0x33
#define ADDR_INT1_TSH_YH		0x34
#define ADDR_INT1_TSH_YL		0x35
#define ADDR_INT1_TSH_ZH		0x36
#define ADDR_INT1_TSH_ZL		0x37
#define ADDR_INT1_DURATION		0x38


/* Internal configuration values */
#define REG1_POWER_NORMAL			(1<<3)
#define REG1_Z_ENABLE				(1<<2)
#define REG1_Y_ENABLE				(1<<1)
#define REG1_X_ENABLE				(1<<0)

#define REG4_BDU				(1<<7)
#define REG4_BLE				(1<<6)
//#define REG4_SPI_3WIRE			(1<<0)

#define REG5_FIFO_ENABLE			(1<<6)
#define REG5_REBOOT_MEMORY			(1<<7)

#define STATUS_ZYXOR				(1<<7)
#define STATUS_ZOR				(1<<6)
#define STATUS_YOR				(1<<5)
#define STATUS_XOR				(1<<4)
#define STATUS_ZYXDA				(1<<3)
#define STATUS_ZDA				(1<<2)
#define STATUS_YDA				(1<<1)
#define STATUS_XDA				(1<<0)

#define FIFO_CTRL_BYPASS_MODE			(0<<5)
#define FIFO_CTRL_FIFO_MODE			(1<<5)
#define FIFO_CTRL_STREAM_MODE			(1<<6)
#define FIFO_CTRL_STREAM_TO_FIFO_MODE		(3<<5)
#define FIFO_CTRL_BYPASS_TO_STREAM_MODE		(1<<7)

#define LSM9DS0_GYRO_DEFAULT_RATE			760
#define LSM9DS0_GYRO_DEFAULT_RANGE_DPS		2000
#define LSM9DS0_GYRO_DEFAULT_FILTER_FREQ		30

extern "C" { __EXPORT int lsm9ds0_gyro_main(int argc, char *argv[]); }

class LSM9DS0_GYRO : public device::SPI
{
public:
	LSM9DS0_GYRO(int bus, const char* path, spi_dev_e device);
	virtual ~LSM9DS0_GYRO();

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

protected:
	virtual int		probe();

private:

	struct hrt_call		_call;
	unsigned		_call_interval;
	
	RingBuffer		*_reports;

	struct gyro_scale	_gyro_scale;
	float			_gyro_range_scale;
	float			_gyro_range_rad_s;
	orb_advert_t		_gyro_topic;
	orb_id_t			_orb_id;
	int			_class_instance;

	unsigned		_current_rate;
	unsigned		_orientation;

	unsigned		_read;

	perf_counter_t		_sample_perf;
	perf_counter_t		_reschedules;
	perf_counter_t		_errors;

	math::LowPassFilter2p	_gyro_filter_x;
	math::LowPassFilter2p	_gyro_filter_y;
	math::LowPassFilter2p	_gyro_filter_z;

	/**
	 * Start automatic measurement.
	 */
	void			start();

	/**
	 * Stop automatic measurement.
	 */
	void			stop();

	/**
	 * Reset the driver
	 */
	void			reset();

	/**
	 * disable I2C on the chip
	 */
	void			disable_i2c();

	/**
	 * Static trampoline from the hrt_call context; because we don't have a
	 * generic hrt wrapper yet.
	 *
	 * Called by the HRT in interrupt context at the specified rate if
	 * automatic polling is enabled.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		measure_trampoline(void *arg);

	/**
	 * Fetch measurements from the sensor and update the report ring.
	 */
	void			measure();

	/**
	 * Read a register from the LSM9DS0_GYRO
	 *
	 * @param		The register to read.
	 * @return		The value that was read.
	 */
	uint8_t			read_reg(unsigned reg);

	/**
	 * Write a register in the LSM9DS0_GYRO
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_reg(unsigned reg, uint8_t value);

	/**
	 * Modify a register in the LSM9DS0_GYRO
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg		The register to modify.
	 * @param clearbits	Bits in the register to clear.
	 * @param setbits	Bits in the register to set.
	 */
	void			modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

	/**
	 * Set the LSM9DS0_GYRO measurement range.
	 *
	 * @param max_dps	The measurement range is set to permit reading at least
	 *			this rate in degrees per second.
	 *			Zero selects the maximum supported range.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			set_range(unsigned max_dps);

	/**
	 * Set the LSM9DS0_GYRO internal sampling frequency.
	 *
	 * @param frequency	The internal sampling frequency is set to not less than
	 *			this value.
	 *			Zero selects the maximum rate supported.
	 * @return		OK if the value can be supported.
	 */
	int			set_samplerate(unsigned frequency);

	/**
	 * Set the lowpass filter of the driver
	 *
	 * @param samplerate	The current samplerate
	 * @param frequency	The cutoff frequency for the lowpass filter
	 */
	void			set_driver_lowpass_filter(float samplerate, float bandwidth);

	/**
	 * Self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	 int 			self_test();
};

LSM9DS0_GYRO::LSM9DS0_GYRO(int bus, const char* path, spi_dev_e device) :
	SPI("LSM9DS0_GYRO", path, bus, device, SPIDEV_MODE3, 11*1000*1000 /* will be rounded to 10.4 MHz, within margins for LSM9DS0_GYRO */),
	_call_interval(0),
	_reports(nullptr),
	_gyro_range_scale(0.0f),
	_gyro_range_rad_s(0.0f),
	_gyro_topic(-1),
	_orb_id(nullptr),
	_class_instance(-1),
	_current_rate(0),
	_orientation(SENSOR_BOARD_ROTATION_000_DEG),
	_read(0),
	_sample_perf(perf_alloc(PC_ELAPSED, "lsm9ds0_gyro_read")),
	_reschedules(perf_alloc(PC_COUNT, "lsm9ds0_gyro_reschedules")),
	_errors(perf_alloc(PC_COUNT, "lsm9ds0_gyro_errors")),
	_gyro_filter_x(LSM9DS0_GYRO_DEFAULT_RATE, LSM9DS0_GYRO_DEFAULT_FILTER_FREQ),
	_gyro_filter_y(LSM9DS0_GYRO_DEFAULT_RATE, LSM9DS0_GYRO_DEFAULT_FILTER_FREQ),
	_gyro_filter_z(LSM9DS0_GYRO_DEFAULT_RATE, LSM9DS0_GYRO_DEFAULT_FILTER_FREQ)
{
	// enable debug() calls
	_debug_enabled = true;

	// default scale factors
	_gyro_scale.x_offset = 0;
	_gyro_scale.x_scale  = 1.0f;
	_gyro_scale.y_offset = 0;
	_gyro_scale.y_scale  = 1.0f;
	_gyro_scale.z_offset = 0;
	_gyro_scale.z_scale  = 1.0f;
}

LSM9DS0_GYRO::~LSM9DS0_GYRO()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr)
		delete _reports;

	if (_class_instance != -1)
		unregister_class_devname(GYRO_DEVICE_PATH, _class_instance);

	/* delete the perf counter */
	perf_free(_sample_perf);
	perf_free(_reschedules);
	perf_free(_errors);
}

int
LSM9DS0_GYRO::init()
{
	int ret = ERROR;

	/* do SPI init (and probe) first */
	if (SPI::init() != OK)
		goto out;

	/* allocate basic report buffers */
	_reports = new RingBuffer(2, sizeof(gyro_report));

	if (_reports == nullptr)
		goto out;

	_class_instance = register_class_devname(GYRO_DEVICE_PATH);

		switch (_class_instance) {
		case CLASS_DEVICE_PRIMARY:
			_orb_id = ORB_ID(sensor_gyro0);
			break;

		case CLASS_DEVICE_SECONDARY:
			_orb_id = ORB_ID(sensor_gyro1);
			break;

		case CLASS_DEVICE_TERTIARY:
			_orb_id = ORB_ID(sensor_gyro2);
			break;
	}
	reset();

	measure();


	/* advertise sensor topic, measure manually to initialize valid report */
	struct gyro_report grp;
	_reports->get(&grp);

	_gyro_topic = orb_advertise(_orb_id, &grp);

	if (_gyro_topic < 0) {
		debug("failed to create sensor_gyro publication");
	}

	ret = OK;
out:
	return ret;
}

int
LSM9DS0_GYRO::probe()
{
	/* read dummy value to void to clear SPI statemachine on sensor */
	(void)read_reg(ADDR_WHO_AM_I);

	bool success = false;

	/* verify that the device is attached and functioning, accept LSM9DS0_GYRO and LSM9DS0_GYROH */
	if (read_reg(ADDR_WHO_AM_I) == WHO_I_AM) {

		#ifdef CONFIG_ARCH_BOARD_PX4FMU_V1
			_orientation = SENSOR_BOARD_ROTATION_270_DEG;
		#elif CONFIG_ARCH_BOARD_PX4FMU_V2
			_orientation = SENSOR_BOARD_ROTATION_270_DEG;
		#elif CONFIG_ARCH_BOARD_LEOFMU_V1
			_orientation = SENSOR_BOARD_ROTATION_000_DEG;
		#else
			#error This driver needs a board selection, either CONFIG_ARCH_BOARD_PX4FMU_V1 or CONFIG_ARCH_BOARD_PX4FMU_V2
		#endif

		success = true;
	}


	/*if (read_reg(ADDR_WHO_AM_I) == WHO_I_AM_H) {
		_orientation = SENSOR_BOARD_ROTATION_180_DEG;
		success = true;
	}*/

	if (success)
		return OK;

	return -EIO;
}

ssize_t
LSM9DS0_GYRO::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct gyro_report);
	struct gyro_report *gbuf = reinterpret_cast<struct gyro_report *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1)
		return -ENOSPC;

	/* if automatic measurement is enabled */
	if (_call_interval > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the measurement code while we are doing this;
		 * we are careful to avoid racing with it.
		 */
		while (count--) {
			if (_reports->get(gbuf)) {
				ret += sizeof(*gbuf);
				gbuf++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement */
	_reports->flush();
	measure();

	/* measurement will have generated a report, copy it out */
	if (_reports->get(gbuf)) {
		ret = sizeof(*gbuf);
	}

	return ret;
}

int
LSM9DS0_GYRO::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

				/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_call_interval = 0;
				return OK;

				/* external signalling not supported */
			case SENSOR_POLLRATE_EXTERNAL:

				/* zero would be bad */
			case 0:
				return -EINVAL;

				/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT:
				return ioctl(filp, SENSORIOCSPOLLRATE, LSM9DS0_GYRO_DEFAULT_RATE);

				/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_call_interval == 0);

					/* convert hz to hrt interval via microseconds */
					unsigned ticks = 1000000 / arg;

					/* check against maximum sane rate */
					if (ticks < 1000)
						return -EINVAL;

					/* update interval for next measurement */
					/* XXX this is a bit shady, but no other way to adjust... */
					_call.period = _call_interval = ticks;

					/* adjust filters */
					float cutoff_freq_hz = _gyro_filter_x.get_cutoff_freq();
					float sample_rate = 1.0e6f/ticks;
					set_driver_lowpass_filter(sample_rate, cutoff_freq_hz);

					/* if we need to start the poll state machine, do it */
					if (want_start)
						start();

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_call_interval == 0)
			return SENSOR_POLLRATE_MANUAL;

		return 1000000 / _call_interval;

	case SENSORIOCSQUEUEDEPTH: {
		/* lower bound is mandatory, upper bound is a sanity check */
		if ((arg < 1) || (arg > 100))
			return -EINVAL;

		irqstate_t flags = irqsave();
		if (!_reports->resize(arg)) {
			irqrestore(flags);
			return -ENOMEM;
		}
		irqrestore(flags);
		
		return OK;
	}

	case SENSORIOCGQUEUEDEPTH:
		return _reports->size();

	case SENSORIOCRESET:
		reset();
		return OK;

	case GYROIOCSSAMPLERATE:
		return set_samplerate(arg);

	case GYROIOCGSAMPLERATE:
		return _current_rate;

	case GYROIOCSLOWPASS: {
		float cutoff_freq_hz = arg;
		float sample_rate = 1.0e6f / _call_interval;
		set_driver_lowpass_filter(sample_rate, cutoff_freq_hz);

		return OK;
	}

	case GYROIOCGLOWPASS:
		return _gyro_filter_x.get_cutoff_freq();

	case GYROIOCSSCALE:
		/* copy scale in */
		memcpy(&_gyro_scale, (struct gyro_scale *) arg, sizeof(_gyro_scale));
		return OK;

	case GYROIOCGSCALE:
		/* copy scale out */
		memcpy((struct gyro_scale *) arg, &_gyro_scale, sizeof(_gyro_scale));
		return OK;

	case GYROIOCSRANGE:
		/* arg should be in dps */
		return set_range(arg);

	case GYROIOCGRANGE:
		/* convert to dps and round */
		return (unsigned long)(_gyro_range_rad_s * 180.0f / M_PI_F + 0.5f);

	case GYROIOCSELFTEST:
		return self_test();

	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
	}
}

uint8_t
LSM9DS0_GYRO::read_reg(unsigned reg)
{
	uint8_t cmd[2];

	cmd[0] = reg | DIR_READ;
	cmd[1] = 0;

	transfer(cmd, cmd, sizeof(cmd));

	return cmd[1];
}

void
LSM9DS0_GYRO::write_reg(unsigned reg, uint8_t value)
{
	uint8_t	cmd[2];

	cmd[0] = reg | DIR_WRITE;
	cmd[1] = value;

	transfer(cmd, nullptr, sizeof(cmd));
}

void
LSM9DS0_GYRO::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t	val;

	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_reg(reg, val);
}

int
LSM9DS0_GYRO::set_range(unsigned max_dps)
{
	uint8_t bits = REG4_BDU;
	float new_range_scale_dps_digit;
	float new_range;

	if (max_dps == 0) {
		max_dps = 2000;
	}
	if (max_dps <= 250) {
		new_range = 250;
		bits |= RANGE_250DPS;
		new_range_scale_dps_digit = 8.75e-3f;

	} else if (max_dps <= 500) {
		new_range = 500;
		bits |= RANGE_500DPS;
		new_range_scale_dps_digit = 17.5e-3f;

	} else if (max_dps <= 2000) {
		new_range = 2000;
		bits |= RANGE_2000DPS;
		new_range_scale_dps_digit = 70e-3f;

	} else {
		return -EINVAL;
	}

	_gyro_range_rad_s = new_range / 180.0f * M_PI_F;
	_gyro_range_scale = new_range_scale_dps_digit / 180.0f * M_PI_F;
	write_reg(ADDR_CTRL_REG4, bits);

	return OK;
}

int
LSM9DS0_GYRO::set_samplerate(unsigned frequency)
{
	uint8_t bits = REG1_POWER_NORMAL | REG1_Z_ENABLE | REG1_Y_ENABLE | REG1_X_ENABLE;

	if (frequency == 0)
		frequency = 760;

	/* use limits good for H or non-H models */
	if (frequency <= 100) {
		_current_rate = 95;
		bits |= RATE_95HZ_LP_25HZ;

	} else if (frequency <= 200) {
		_current_rate = 190;
		bits |= RATE_190HZ_LP_50HZ;

	} else if (frequency <= 400) {
		_current_rate = 380;
		bits |= RATE_380HZ_LP_50HZ;

	} else if (frequency <= 800) {
		_current_rate = 760;
		bits |= RATE_760HZ_LP_50HZ;
	} else {
		return -EINVAL;
	}

	write_reg(ADDR_CTRL_REG1, bits);

	return OK;
}

void
LSM9DS0_GYRO::set_driver_lowpass_filter(float samplerate, float bandwidth)
{
	_gyro_filter_x.set_cutoff_frequency(samplerate, bandwidth);
	_gyro_filter_y.set_cutoff_frequency(samplerate, bandwidth);
	_gyro_filter_z.set_cutoff_frequency(samplerate, bandwidth);
}

void
LSM9DS0_GYRO::start()
{
	/* make sure we are stopped first */
	stop();

	/* reset the report ring */
	_reports->flush();

	/* start polling at the specified rate */
	hrt_call_every(&_call, 1000, _call_interval, (hrt_callout)&LSM9DS0_GYRO::measure_trampoline, this);
}

void
LSM9DS0_GYRO::stop()
{
	hrt_cancel(&_call);
}

void
LSM9DS0_GYRO::disable_i2c(void)
{
	uint8_t retries = 10;
	while (retries--) {
		// add retries
		return;
		uint8_t a = read_reg(0x05);
		write_reg(0x05, (0x20 | a));
		if (read_reg(0x05) == (a | 0x20)) {
			return;
		}
	}
	debug("FAILED TO DISABLE I2C");
}

void
LSM9DS0_GYRO::reset()
{
	// ensure the chip doesn't interpret any other bus traffic as I2C
	disable_i2c();

	/* set default configuration */
	write_reg(ADDR_CTRL_REG1, REG1_POWER_NORMAL | REG1_Z_ENABLE | REG1_Y_ENABLE | REG1_X_ENABLE);
	write_reg(ADDR_CTRL_REG2, 0);		/* disable high-pass filters */
	write_reg(ADDR_CTRL_REG3, 0x08);        /* DRDY enable */
	write_reg(ADDR_CTRL_REG4, REG4_BDU);
	write_reg(ADDR_CTRL_REG5, 0);

	write_reg(ADDR_CTRL_REG5, REG5_FIFO_ENABLE);		/* disable wake-on-interrupt */

	/* disable FIFO. This makes things simpler and ensures we
	 * aren't getting stale data. It means we must run the hrt
	 * callback fast enough to not miss data. */
	write_reg(ADDR_FIFO_CTRL_REG, FIFO_CTRL_BYPASS_MODE);

	set_samplerate(0); // 760Hz
	set_range(LSM9DS0_GYRO_DEFAULT_RANGE_DPS);
	set_driver_lowpass_filter(LSM9DS0_GYRO_DEFAULT_RATE, LSM9DS0_GYRO_DEFAULT_FILTER_FREQ);

	_read = 0;
}

void
LSM9DS0_GYRO::measure_trampoline(void *arg)
{
	LSM9DS0_GYRO *dev = (LSM9DS0_GYRO *)arg;

	/* make another measurement */
	dev->measure();
}

#ifdef GPIO_EXTI_GYRO_DRDY
# define LSM9DS0_GYRO_USE_DRDY 1
#else
# define LSM9DS0_GYRO_USE_DRDY 0
#endif

void
LSM9DS0_GYRO::measure()
{
#if LSM9DS0_GYRO_USE_DRDY
	// if the gyro doesn't have any data ready then re-schedule
	// for 100 microseconds later. This ensures we don't double
	// read a value and then miss the next value
	if (stm32_gpioread(GPIO_EXTI_GYRO_DRDY) == 0) {
		perf_count(_reschedules);
		hrt_call_delay(&_call, 100);
		return;
	}
#endif

	/* status register and data as read back from the device */
#pragma pack(push, 1)
	struct {
		uint8_t		cmd;
		uint8_t		temp;
		uint8_t		status;
		int16_t		x;
		int16_t		y;
		int16_t		z;
	} raw_report;
#pragma pack(pop)

	gyro_report report;

	/* start the performance counter */
	perf_begin(_sample_perf);

	/* fetch data from the sensor */
	memset(&raw_report, 0, sizeof(raw_report));
	raw_report.cmd = ADDR_OUT_TEMP | DIR_READ | ADDR_INCREMENT;
	transfer((uint8_t *)&raw_report, (uint8_t *)&raw_report, sizeof(raw_report));

#if LSM9DS0_GYRO_USE_DRDY
        if ((raw_report.status & 0xF) != 0xF) {
            /*
              we waited for DRDY, but did not see DRDY on all axes
              when we captured. That means a transfer error of some sort
             */
            perf_count(_errors);            
            return;
        }
#endif
	/*
	 * 1) Scale raw value to SI units using scaling from datasheet.
	 * 2) Subtract static offset (in SI units)
	 * 3) Scale the statically calibrated values with a linear
	 *    dynamically obtained factor
	 *
	 * Note: the static sensor offset is the number the sensor outputs
	 * 	 at a nominally 'zero' input. Therefore the offset has to
	 * 	 be subtracted.
	 *
	 *	 Example: A gyro outputs a value of 74 at zero angular rate
	 *	 	  the offset is 74 from the origin and subtracting
	 *		  74 from all measurements centers them around zero.
	 */
	report.timestamp = hrt_absolute_time();
        report.error_count = 0; // not recorded
	
	switch (_orientation) {

		case SENSOR_BOARD_ROTATION_000_DEG:
			/* keep axes in place */
			report.x_raw = raw_report.x;
			report.y_raw = raw_report.y;
			break;

		case SENSOR_BOARD_ROTATION_090_DEG:
			/* swap x and y */
			report.x_raw = raw_report.y;
			report.y_raw = raw_report.x;
			break;

		case SENSOR_BOARD_ROTATION_180_DEG:
			/* swap x and y and negate both */
			report.x_raw = ((raw_report.x == -32768) ? 32767 : -raw_report.x);
			report.y_raw = ((raw_report.y == -32768) ? 32767 : -raw_report.y);
			break;

		case SENSOR_BOARD_ROTATION_270_DEG:
			/* swap x and y and negate y */
			report.x_raw = raw_report.y;
			report.y_raw = ((raw_report.x == -32768) ? 32767 : -raw_report.x);
			break;
	}

	report.z_raw = raw_report.z;

	report.x = ((report.x_raw * _gyro_range_scale) - _gyro_scale.x_offset) * _gyro_scale.x_scale;
	report.y = ((report.y_raw * _gyro_range_scale) - _gyro_scale.y_offset) * _gyro_scale.y_scale;
	report.z = ((report.z_raw * _gyro_range_scale) - _gyro_scale.z_offset) * _gyro_scale.z_scale;

	report.x = _gyro_filter_x.apply(report.x);
	report.y = _gyro_filter_y.apply(report.y);
	report.z = _gyro_filter_z.apply(report.z);

	report.scaling = _gyro_range_scale;
	report.range_rad_s = _gyro_range_rad_s;

	_reports->force(&report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	/* publish for subscribers */
	if (!(_pub_blocked)) {
		/* publish it */
		orb_publish(_orb_id, _gyro_topic, &report);
	}

	_read++;

	/* stop the perf counter */
	perf_end(_sample_perf);
}

void
LSM9DS0_GYRO::print_info()
{
	printf("gyro reads:          %u\n", _read);
	perf_print_counter(_sample_perf);
	perf_print_counter(_reschedules);
	perf_print_counter(_errors);
	_reports->print_info("report queue");
}

int
LSM9DS0_GYRO::self_test()
{
	/* evaluate gyro offsets, complain if offset -> zero or larger than 6 dps */
	if (fabsf(_gyro_scale.x_offset) > 0.5f || fabsf(_gyro_scale.x_offset) < 0.000001f)
		return 1;
	if (fabsf(_gyro_scale.x_scale - 1.0f) > 0.3f)
		return 1;

	if (fabsf(_gyro_scale.y_offset) > 0.5f || fabsf(_gyro_scale.y_offset) < 0.000001f)
		return 1;
	if (fabsf(_gyro_scale.y_scale - 1.0f) > 0.3f)
		return 1;

	if (fabsf(_gyro_scale.z_offset) > 0.5f || fabsf(_gyro_scale.z_offset) < 0.000001f)
		return 1;
	if (fabsf(_gyro_scale.z_scale - 1.0f) > 0.3f)
		return 1;

	return 0;
}

/**
 * Local functions in support of the shell command.
 */
namespace lsm9ds0_gyro
{

LSM9DS0_GYRO	*g_dev;

void	start();
void	test();
void	reset();
void	info();

/**
 * Start the driver.
 */
void
start()
{
	int fd;

	if (g_dev != nullptr)
		errx(0, "already started");

	/* create the driver */
	g_dev = new LSM9DS0_GYRO(1 /* SPI bus 1 */, LSM9DS0_GYRO_DEVICE_PATH, (spi_dev_e)PX4_SPIDEV_GYRO);

	if (g_dev == nullptr)
		goto fail;

	if (OK != g_dev->init())
		goto fail;

	/* set the poll rate to default, starts automatic data collection */
	fd = open(LSM9DS0_GYRO_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		goto fail;

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		goto fail;

        close(fd);

	exit(0);
fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{
	int fd_gyro = -1;
	struct gyro_report g_report;
	ssize_t sz;

	/* get the driver */
	fd_gyro = open(LSM9DS0_GYRO_DEVICE_PATH, O_RDONLY);

	if (fd_gyro < 0)
		err(1, "%s open failed", LSM9DS0_GYRO_DEVICE_PATH);

	/* reset to manual polling */
	if (ioctl(fd_gyro, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MANUAL) < 0)
		err(1, "reset to manual polling");

	/* do a simple demand read */
	sz = read(fd_gyro, &g_report, sizeof(g_report));

	if (sz != sizeof(g_report))
		err(1, "immediate gyro read failed");

	warnx("gyro x: \t% 9.5f\trad/s", (double)g_report.x);
	warnx("gyro y: \t% 9.5f\trad/s", (double)g_report.y);
	warnx("gyro z: \t% 9.5f\trad/s", (double)g_report.z);
	warnx("gyro x: \t%d\traw", (int)g_report.x_raw);
	warnx("gyro y: \t%d\traw", (int)g_report.y_raw);
	warnx("gyro z: \t%d\traw", (int)g_report.z_raw);
	warnx("gyro range: %8.4f rad/s (%d deg/s)", (double)g_report.range_rad_s,
	      (int)((g_report.range_rad_s / M_PI_F) * 180.0f + 0.5f));

        close(fd_gyro);

	/* XXX add poll-rate tests here too */

	reset();
	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(LSM9DS0_GYRO_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "failed ");

	if (ioctl(fd, SENSORIOCRESET, 0) < 0)
		err(1, "driver reset failed");

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		err(1, "accel pollrate reset failed");

        close(fd);

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr)
		errx(1, "driver not running\n");

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}


} // namespace

int
lsm9ds0_gyro_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.

	 */
	if (!strcmp(argv[1], "start"))
		lsm9ds0_gyro::start();

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test"))
		lsm9ds0_gyro::test();

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset"))
		lsm9ds0_gyro::reset();

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info"))
		lsm9ds0_gyro::info();

	errx(1, "unrecognized command, try 'start', 'test', 'reset' or 'info'");
}
