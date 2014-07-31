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
 * @file lps331.cpp
 * Driver for the ST LPS331 MEMS baro connected via SPI.
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
//#include <nuttx/wqueue.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>

#include <drivers/drv_hrt.h>
#include <drivers/device/spi.h>
#include <drivers/drv_baro.h>
#include <drivers/device/ringbuffer.h>

#include <board_config.h>


#define LPS331_DEVICE_PATH "/dev/lps331"

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

/* SPI protocol address bits */
#define DIR_READ				(1<<7)
#define DIR_WRITE				(0<<7)
#define ADDR_INCREMENT				(1<<6)

/* register addresses */
#define ADDR_WHO_AM_I			0x0F
#define WHO_I_AM				0xBB

#define ADDR_RES_CONF				0x10
#define ADDR_CTRL_REG1				0x20
#define ADDR_CTRL_REG2				0x21
#define ADDR_CTRL_REG3				0x22
#define ADDR_INT_CFG_REG			0x23
#define ADDR_INT_SOURCE_REG			0x24
#define ADDR_THS_P_LOW_REG			0x25
#define ADDR_THS_P_HIGH_REG			0x26
#define ADDR_STATUS_REG				0x27
#define ADDR_PRESS_POUT_XL_REH			0x28
#define ADDR_PRESS_OUT_L 			0x29
#define ADDR_PRESS_OUT_H			0x2A
#define ADDR_TEMP_OUT_L				0x2B
#define ADDR_TEMP_OUT_H				0x2C

#define PRESS_AVG_1				0x00
#define PRESS_AVG_2				0x01
#define PRESS_AVG_4				0x02
#define PRESS_AVG_8				0x03
#define PRESS_AVG_16				0x04
#define PRESS_AVG_32				0x05
#define PRESS_AVG_64				0x06
#define PRESS_AVG_128				0x07
#define PRESS_AVG_256				0x08
#define PRESS_AVG_384				0x09
#define PRESS_AVG_512				0x0A

#define TEMP_AVG_1				(0<<6) | (0<<5) | (0<<4)
#define TEMP_AVG_2				(0<<6) | (0<<5) | (1<<4)
#define TEMP_AVG_4				(0<<6) | (1<<5) | (0<<4)
#define TEMP_AVG_8				(0<<6) | (1<<5) | (1<<4)
#define TEMP_AVG_16				(1<<6) | (0<<5) | (0<<4)
#define TEMP_AVG_32				(1<<6) | (0<<5) | (1<<4)
#define TEMP_AVG_64				(1<<6) | (1<<5) | (0<<4)
#define TEMP_AVG_128				(1<<6) | (1<<5) | (1<<4)

#define REG1_ACTIVE_MODE			(1<<7)
#define REG1_RATE_ONESHOT			(0<<6) | (0<<5) | (0<<4)
#define REG1_RATE_P_1HZ_T_1HZ			(0<<6) | (0<<5) | (1<<4)
#define REG1_RATE_P_7HZ_T_1HZ			(0<<6) | (1<<5) | (0<<4)
#define REG1_RATE_P_12HZ_T_1HZ			(0<<6) | (1<<5) | (1<<4)
#define REG1_RATE_P_25HZ_T_1HZ			(1<<6) | (0<<5) | (0<<4)
#define REG1_RATE_P_7HZ_T_7HZ			(1<<6) | (0<<5) | (1<<4)
#define REG1_RATE_P_12HZ_T_12HZ			(1<<6) | (1<<5) | (0<<4)
#define REG1_RATE_P_25HZ_T_25HZ			(1<<6) | (1<<5) | (1<<4)
#define REG1_DIFF_EN				(1<<3)
#define REG1_BDU				(1<<2)
#define REG1_DELTA_EN				(1<<1)


#define LPS331_DEFAULT_RATE			25
#define LPS331_CONVERSION_INTERVAL		40000


extern "C" { __EXPORT int lps331_main(int argc, char *argv[]); }

class LPS331 : public device::SPI
{
public:
	LPS331(int bus, const char* path, spi_dev_e device);
	virtual ~LPS331();

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

protected:
	virtual int		probe();
	//struct work_s		_work;

private:

	struct hrt_call		_call;
	unsigned		_call_interval;
	
	RingBuffer		*_reports;

	/* altitude conversion calibration */
	unsigned		_msl_pressure;	/* in kPa */
	orb_advert_t		_baro_topic;
	orb_id_t			_orb_id;
	int			_class_instance;

	

	unsigned		_read;

	perf_counter_t		_sample_perf;
	perf_counter_t		_reschedules;
	perf_counter_t		_errors;


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
	 * Read a register from the LPS331
	 *
	 * @param		The register to read.
	 * @return		The value that was read.
	 */
	uint8_t			read_reg(unsigned reg);

	/**
	 * Write a register in the LPS331
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_reg(unsigned reg, uint8_t value);

	/**
	 * Modify a register in the LPS331
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg		The register to modify.
	 * @param clearbits	Bits in the register to clear.
	 * @param setbits	Bits in the register to set.
	 */
	void			modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

	/**
	 * Set the LPS331 measurement range.
	 *
	 * @param max_dps	The measurement range is set to permit reading at least
	 *			this rate in degrees per second.
	 *			Zero selects the maximum supported range.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			set_range(unsigned max_dps);

	/**
	 * Set the LPS331 internal sampling frequency.
	 *
	 * @param frequency	The internal sampling frequency is set to not less than
	 *			this value.
	 *			Zero selects the maximum rate supported.
	 * @return		OK if the value can be supported.
	 */
	int			set_samplerate(unsigned frequency);



	/**
	 * Self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	 int 			self_test();
};

LPS331::LPS331(int bus, const char* path, spi_dev_e device) :
	SPI("LPS331", path, bus, device, SPIDEV_MODE3, 11*1000*1000 /* will be rounded to 10.4 MHz, within margins for LPS331 */),
	_call_interval(0),
	_reports(nullptr),
	_msl_pressure(101325),
	_baro_topic(-1),
	_class_instance(-1),
	_read(0),
	_sample_perf(perf_alloc(PC_ELAPSED, "lps331_read")),
	_reschedules(perf_alloc(PC_COUNT, "lps331_reschedules")),
	_errors(perf_alloc(PC_COUNT, "lps331_errors"))
{
	// enable debug() calls
	//memset(&_work, 0, sizeof(_work));
	_debug_enabled = true;

}

LPS331::~LPS331()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr)
		delete _reports;

	if (_class_instance != -1)
		unregister_class_devname(BARO_DEVICE_PATH, _class_instance);

	/* delete the perf counter */
	perf_free(_sample_perf);
	perf_free(_reschedules);
	perf_free(_errors);
}

int
LPS331::init()
{
	int ret = ERROR;

	/* do SPI init (and probe) first */
	if (SPI::init() != OK)
		goto out;

	/* allocate basic report buffers */
	_reports = new RingBuffer(2, sizeof(baro_report));

	if (_reports == nullptr)
		goto out;

	_class_instance = register_class_devname(BARO_DEVICE_PATH);

	switch (_class_instance) {
		case CLASS_DEVICE_PRIMARY:
			_orb_id = ORB_ID(sensor_baro0);
			break;

		case CLASS_DEVICE_SECONDARY:
			_orb_id = ORB_ID(sensor_baro1);
			break;
	}

	reset();

	measure();

	/* advertise sensor topic, measure manually to initialize valid report */
	struct baro_report grp;
	_reports->get(&grp);

	_baro_topic = orb_advertise(_orb_id, &grp);

	if (_baro_topic < 0) {
		debug("failed to create sensor_baro publication");
	}

	ret = OK;
out:
	return ret;
}

// int
// LPS331::init()
// {
// 	int ret = ERROR;

// 	/* do SPI init (and probe) first */
// 	if (SPI::init() != OK)
// 		goto out;

// 	/* allocate basic report buffers */
// 	_reports = new RingBuffer(2, sizeof(baro_report));

// 	if (_reports == nullptr)
// 		goto out;

// 	_class_instance = register_class_devname(BARO_DEVICE_PATH);

// 	reset();

// 	measure();

// 	switch (_class_instance) {
// 			case CLASS_DEVICE_PRIMARY:
// 				_baro_topic = orb_advertise(ORB_ID(sensor_baro0), &brp);
// 				break;
// 			case CLASS_DEVICE_SECONDARY:
// 				_baro_topic = orb_advertise(ORB_ID(sensor_baro1), &brp);
// 				break;
// 		}

// 		if (_baro_topic < 0)
// 			warnx("failed to create sensor_baro publication");

	

// 	ret = OK;
// out:
// 	return ret;
// }

int
LPS331::probe()
{
	/* read dummy value to void to clear SPI statemachine on sensor */
	(void)read_reg(ADDR_WHO_AM_I);

	bool success = false;

	/* verify that the device is attached and functioning, accept LPS331 */
	if (read_reg(ADDR_WHO_AM_I) == WHO_I_AM) {

		success = true;
	}


	if (success)
		return OK;

	return -EIO;
}

ssize_t
LPS331::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct baro_report);
	struct baro_report *gbuf = reinterpret_cast<struct baro_report *>(buffer);
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
LPS331::ioctl(struct file *filp, int cmd, unsigned long arg)
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
				return ioctl(filp, SENSORIOCSPOLLRATE, LPS331_DEFAULT_RATE);

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

	case BAROIOCSMSLPRESSURE:

		/* range-check for sanity */
		if ((arg < 80000) || (arg > 120000))
			return -EINVAL;

		_msl_pressure = arg;
		return OK;

	case BAROIOCGMSLPRESSURE:
		return _msl_pressure;

	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
	}
}

uint8_t
LPS331::read_reg(unsigned reg)
{
	uint8_t cmd[2];

	cmd[0] = reg | DIR_READ;
	cmd[1] = 0;

	transfer(cmd, cmd, sizeof(cmd));

	return cmd[1];
}

void
LPS331::write_reg(unsigned reg, uint8_t value)
{
	uint8_t	cmd[2];

	cmd[0] = reg | DIR_WRITE;
	cmd[1] = value;

	transfer(cmd, nullptr, sizeof(cmd));
}

void
LPS331::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t	val;

	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_reg(reg, val);
}


int
LPS331::set_samplerate(unsigned frequency)
{



	uint8_t bits = REG1_ACTIVE_MODE | REG1_BDU;

	if (frequency == 0)
		frequency = 25;

	/* use limits good for H or non-H models */
	if (frequency <= 1) {
		
		bits |= REG1_RATE_P_1HZ_T_1HZ;

	} else if (frequency <= 7) {
		bits |= REG1_RATE_P_7HZ_T_1HZ;

	} else if (frequency <= 12) {
		bits |= REG1_RATE_P_12HZ_T_1HZ;

	} else if (frequency <= 25) {
		bits |= REG1_RATE_P_25HZ_T_1HZ;
	} else {
		return -EINVAL;
	}

	write_reg(ADDR_CTRL_REG1, bits);

	return OK;
}



void
LPS331::start()
{
	/* make sure we are stopped first */
	stop();

	/* reset the report ring */
	_reports->flush();

	/* start polling at the specified rate */
	hrt_call_every(&_call, LPS331_CONVERSION_INTERVAL, _call_interval, (hrt_callout)&LPS331::measure_trampoline, this);
	/*work_queue(HPWORK,
		   &_work,
		   (worker_t)&LPS331::measure_trampoline,
		   this,
		   USEC2TICK(LPS331_CONVERSION_INTERVAL));*/
}

void
LPS331::stop()
{
	//work_cancel(HPWORK, &_work);
	hrt_cancel(&_call);
}

void
LPS331::disable_i2c(void)
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
LPS331::reset()
{
	// ensure the chip doesn't interpret any other bus traffic as I2C
	//disable_i2c();

	/* set default configuration */
	write_reg(ADDR_CTRL_REG1, REG1_ACTIVE_MODE | REG1_BDU | REG1_RATE_P_25HZ_T_1HZ);
	write_reg(ADDR_CTRL_REG2, 0);		/* disable high-pass filters */
	write_reg(ADDR_CTRL_REG3, 0);        /* DRDY enable */

	write_reg(ADDR_RES_CONF,TEMP_AVG_64 | PRESS_AVG_512);


	set_samplerate(25); // 25Hz

	_read = 0;
}

void
LPS331::measure_trampoline(void *arg)
{
	LPS331 *dev = (LPS331 *)arg;

	/* make another measurement */
	dev->measure();
}



void
LPS331::measure()
{


	/* status register and data as read back from the device */
#pragma pack(push, 1)
	struct {
		uint8_t		cmd;
		uint8_t		press_xl;
		uint8_t		press_l;
		uint8_t		press_h;
		uint8_t		temp_l;
		uint8_t		temp_h;
		
	} raw_report;
#pragma pack(pop)

	baro_report report;

	/* start the performance counter */
	perf_begin(_sample_perf);

	/* fetch data from the sensor */
	memset(&raw_report, 0, sizeof(raw_report));
	raw_report.cmd = ADDR_PRESS_POUT_XL_REH | DIR_READ | ADDR_INCREMENT;
	transfer((uint8_t *)&raw_report, (uint8_t *)&raw_report, sizeof(raw_report));



	report.timestamp = hrt_absolute_time();
        report.error_count = 0; // not recorded
	int32_t pressure_int =  (int32_t)(raw_report.press_h)<<16 | (int32_t)(raw_report.press_l)<<8 | (int32_t)raw_report.press_xl;
	int16_t temperature_int = (int16_t)(raw_report.temp_h)<<8 | (int16_t)(raw_report.temp_l);
	double pressure_dbl = (double)pressure_int/4096.0;//pressure in millibar
	double temperature_dbl = 42.5 + (double)temperature_int/480.0;//temperature in C
	/* START - Exponential Moving Average Filter */
	const double alpha = 0.002;
	static bool filter_starting = true;
	static double pressure_av_prev = 0, pressure_av = 0;
	if(filter_starting)
	{
		pressure_av_prev = pressure_dbl;
		if(pressure_dbl > 200 && pressure_dbl < 1200)
		{
			
			filter_starting = false;
		}
	}
	pressure_av = alpha * pressure_dbl + (1 - alpha) * pressure_av_prev;
	pressure_av_prev = pressure_av;


	/* END - Exponential Moving Average Filter */
	//report.pressure= pressure_dbl;
	report.pressure= pressure_av;
	report.temperature= temperature_dbl;


	/* altitude calculations based on http://www.kansasflyer.org/index.asp?nav=Avi&sec=Alti&tab=Theory&pg=1 */

		/*
		 * PERFORMANCE HINT:
		 *
		 * The single precision calculation is 50 microseconds faster than the double
		 * precision variant. It is however not obvious if double precision is required.
		 * Pending more inspection and tests, we'll leave the double precision variant active.
		 *
		 * Measurements:
		 * 	double precision: ms5611_read: 992 events, 258641us elapsed, min 202us max 305us
		 *	single precision: ms5611_read: 963 events, 208066us elapsed, min 202us max 241us
		 */

		/* tropospheric properties (0-11km) for standard atmosphere */
		const double T1 = 15.0 + 273.15;	/* temperature at base height in Kelvin */
		const double a  = -6.5 / 1000;	/* temperature gradient in degrees per metre */
		const double g  = 9.80665;	/* gravity constant in m/s/s */
		const double R  = 287.05;	/* ideal gas constant in J/kg/K */

		/* current pressure at MSL in kPa */
		double p1 = _msl_pressure / 1000.0;

		/* measured pressure in kPa */
		//double p = pressure_dbl / 10.0;
		double p = pressure_av / 10.0;


/*
		 * Solve:
		 *
		 *     /        -(aR / g)     \
		 *    | (p / p1)          . T1 | - T1
		 *     \                      /
		 * h = -------------------------------  + h1
		 *                   a
		 */
		report.altitude = (((pow((p / p1), (-(a * R) / g))) * T1) - T1) / a;

		

	

	_reports->force(&report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	/* publish for subscribers */
	if (_baro_topic > 0 && !(_pub_blocked)) {
		/* publish it */
		orb_publish(_orb_id, _baro_topic, &report);
	}

	_read++;

	/* stop the perf counter */
	perf_end(_sample_perf);

	/*work_queue(HPWORK,
		   &_work,
		   (worker_t)&LPS331::measure_trampoline,
		   this,
		   USEC2TICK(LPS331_CONVERSION_INTERVAL));*/
}

void
LPS331::print_info()
{
	printf("baro reads:          %u\n", _read);
	perf_print_counter(_sample_perf);
	perf_print_counter(_reschedules);
	perf_print_counter(_errors);
	_reports->print_info("report queue");
}

int
LPS331::self_test()
{


	return 0;
}

/**
 * Local functions in support of the shell command.
 */
namespace lps331
{

LPS331	*g_dev;

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
	g_dev = new LPS331(1 /* SPI bus 1 */, LPS331_DEVICE_PATH, (spi_dev_e)PX4_SPIDEV_BARO);

	if (g_dev == nullptr)
		goto fail;

	if (OK != g_dev->init())
		goto fail;

	/* set the poll rate to default, starts automatic data collection */
	fd = open(LPS331_DEVICE_PATH, O_RDONLY);

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
	int fd_baro = -1;
	struct baro_report b_report;
	ssize_t sz;

	/* get the driver */
	fd_baro = open(LPS331_DEVICE_PATH, O_RDONLY);

	if (fd_baro < 0)
		err(1, "%s open failed", LPS331_DEVICE_PATH);

	/* reset to manual polling */
	if (ioctl(fd_baro, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MANUAL) < 0)
		err(1, "reset to manual polling");

	/* do a simple demand read */
	sz = read(fd_baro, &b_report, sizeof(b_report));

	if (sz != sizeof(b_report))
		err(1, "immediate baro read failed");

	warnx("baro pressure :    %10.4f", (double)b_report.pressure);
	warnx("baro altitude :    %11.4f", (double)b_report.altitude);
	warnx("baro temperature : %8.4f", (double)b_report.temperature);
	warnx("baro time :        %lld", b_report.timestamp);

        close(fd_baro);

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
	int fd = open(LPS331_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "failed ");

	if (ioctl(fd, SENSORIOCRESET, 0) < 0)
		err(1, "driver reset failed");

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		err(1, "baro pollrate reset failed");

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
lps331_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.

	 */
	if (!strcmp(argv[1], "start"))
		lps331::start();

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test"))
		lps331::test();

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset"))
		lps331::reset();

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info"))
		lps331::info();

	errx(1, "unrecognized command, try 'start', 'test', 'reset' or 'info'");
}
