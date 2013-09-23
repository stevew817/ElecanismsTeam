#include <p24FJ128GB206.h>
#include "config.h"
#include "common.h"
#include "ui.h"
#include "oc.h"
#include "timer.h"

int16_t main(void) {
    init_clock();
	init_pin();
    init_ui();
    init_timer();
	init_oc();
	
	//Servo control on D13
	//Potentiometer input on A0
	pin_analogIn(&A[0]);
	pin_digitalOut(&D[13]);
	
	//According to HobbyKing documentation: range .8 through 2.2 msec
	oc_servo(&oc1, &D[13], &timer1, 0.02f, 0.0008f, 0.0022f, 0x8000);

    led_on(&led1);
    timer_setPeriod(&timer2, 0.2);
    timer_start(&timer2);

    while (1) {
        if (timer_flag(&timer2)) {
            timer_lower(&timer2);
            led_toggle(&led1);
        }
        led_write(&led2, !sw_read(&sw3));
        led_write(&led3, !sw_read(&sw2));
		
		//analog read on A0 will give us 10bits left-justified as a fraction
		//which is exactly what a write on a servo-initialized pin expects...
		pin_write(&D[13], pin_read(&A[0]));
    }
}

