

#include "26MhzOut.pio.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/pll.h"
#include "hardware/xosc.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <stdlib.h>

#define EXT_CLK                // comment out to use internal TCXO intead of external clock
#define EXT_CLK_FREQ 10 * MHZ  // change to you external clock frequency
#define TARGET_FREQ 130 * MHZ  // change to you external clock frequency
#define EXT_CLK_GPIO 20        // CAN NOT BE CHANGED, because of RP2040 gpio usages.

int main() {
	sleep_ms(1000);

	//set_sys_clock_khz(128000, true);

	//set_sys_clock_pll(PLL_SYS_VCO_FREQ_KHZ, PLL_SYS_POSTDIV1, PLL_SYS_POSTDIV2);

	clock_gpio_init(21, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS, 5);

	//clock_gpio_init(21, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS, 26);

	int i = 0;
	//stdio_init_all();
	const uint LED_PIN = PICO_DEFAULT_LED_PIN;

	uint32_t sys_freq = TARGET_FREQ;

	gpio_init(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);

	gpio_put(LED_PIN, 1);

	sleep_ms(5000);

	gpio_put(LED_PIN, 0);

	while (i < 20) {
		gpio_put(LED_PIN, 1);
		//gpio_put(_26MHzOut, 1);
		sleep_ms(250);
		gpio_put(LED_PIN, 0);
		//gpio_put(_26MHzOut, 0);
		sleep_ms(250);
	}

	//	AIRCR_Register = 0x5FA0004;
}
