/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file motor_power_switch.c
 *  Application to control a motor power "kill switch" example for PX4 autopilot
 *
 * @author Per Bröms <per.broms@ri.se>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/getopt.h>

#include <px4_platform_common/posix.h>

#include <drivers/drv_hrt.h>
#include <board_config.h>

#define RELAY_PIN (GPIO_OUTPUT | GPIO_PORTI | GPIO_PIN0 | GPIO_OPENDRAIN )
#define _STRINGIFY(s) #s
#define STRINGIFY(s) _STRINGIFY(s)

// Function to control motor power via FMU_CAP1 pin
void set_motor_power(bool on)
{
	if (on) {
		px4_arch_gpiowrite(RELAY_PIN, 1);
		PX4_INFO("Motor power ON");

	} else {
		px4_arch_gpiowrite(RELAY_PIN, 0);
		PX4_INFO("Motor power OFF");
	}
}

__EXPORT int motor_power_switch_main(int argc, char *argv[]);

int motor_power_switch_main(int argc, char *argv[])
{
	PX4_INFO("RELAY_PIN:" STRINGIFY(RELAY_PIN));
	PX4_INFO("FMU_CAP1:" STRINGIFY(GPIO_FMU_CAP1));

	if (argc < 2) {
		PX4_ERR("Usage: motor_power <on|off>");
		return 1;
	}

	// Configure the relay pin as output with pull-up and slow slew rate
	px4_arch_configgpio(RELAY_PIN);

	if (strcmp(argv[1], "on") == 0) {
		set_motor_power(true);

	} else if (strcmp(argv[1], "off") == 0) {
		set_motor_power(false);

	} else {
		PX4_ERR("Invalid argument: use 'on' or 'off'");
		return 1;
	}

	return 0;
}
