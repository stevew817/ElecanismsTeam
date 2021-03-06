/***************************************************************************
*	Elecanisms 2013 Miniproject 2:
*	Servo control using USB and a Python GUI
*	Geeta, Sarah and Steven
***************************************************************************/

#include <p24FJ128GB206.h>
#include "config.h"
#include "common.h"
#include "usb.h"
#include "pin.h"
#include "uart.h"
#include "ui.h"
#include "oc.h"
#include "timer.h"
#include <stdio.h>

#define HELLO       0   // Vendor request that prints "Hello World!"
#define SET_VALS    1   // Vendor request that receives 2 unsigned integer values
#define GET_VALS    2   // Vendor request that returns 2 unsigned integer values
#define PRINT_VALS  3   // Vendor request that prints 2 unsigned integer values 

#define PAN_PIN		&D[2]
#define TILT_PIN	&D[3]

#define SERVO_PERIOD	0.02f
#define SERVO_MIN		0.0008f
#define SERVO_MAX		0.0022f

#define SERVO1_OC	&oc1
#define SERVO2_OC	&oc2

#define SERVO1_TIM	&timer1
#define SERVO2_TIM	&timer2
#define LED_TIM		&timer3

#define LED 		&led1

uint16_t pan_set_val = 0, tilt_set_val = 0;

//void ClassRequests(void) {
//    switch (USB_setup.bRequest) {
//        default:
//            USB_error_flags |= 0x01;                    // set Request Error Flag
//    }
//}

void VendorRequests(void) {
    WORD temp;

    switch (USB_setup.bRequest) {
        case HELLO:
            printf("Hello World!\n");
            BD[EP0IN].bytecount = 0;    // set EP0 IN byte count to 0 
            BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
            break;
        case SET_VALS:
            pan_set_val = USB_setup.wValue.w;
            tilt_set_val = USB_setup.wIndex.w;
            BD[EP0IN].bytecount = 0;    // set EP0 IN byte count to 0 
            BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
            break;
        case GET_VALS:
            temp.w = pan_set_val;
            BD[EP0IN].address[0] = temp.b[0];
            BD[EP0IN].address[1] = temp.b[1];
            temp.w = tilt_set_val;
            BD[EP0IN].address[2] = temp.b[0];
            BD[EP0IN].address[3] = temp.b[1];
            BD[EP0IN].bytecount = 4;    // set EP0 IN byte count to 4
            BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
            break;            
        case PRINT_VALS:
            printf("pan_set_val = %u, tilt_set_val = %u\n", pan_set_val, tilt_set_val);
            BD[EP0IN].bytecount = 0;    // set EP0 IN byte count to 0
            BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
            break;
        default:
            USB_error_flags |= 0x01;    // set Request Error Flag
    }
}

void VendorRequestsIn(void) {
    switch (USB_request.setup.bRequest) {
        default:
            USB_error_flags |= 0x01;                    // set Request Error Flag
    }
}

void VendorRequestsOut(void) {
    switch (USB_request.setup.bRequest) {
        default:
            USB_error_flags |= 0x01;                    // set Request Error Flag
    }
}

int16_t main(void) {
	//initialize all system clocks
    init_clock();
	//initialize serial communications
    init_uart();
	//initialize pin driving library (to be able to use the &D[x] defs)
	init_pin();
	//initialize the UI library
    init_ui();
	//initialize the timer module
    init_timer();
	//initialize the OC module (used by the servo driving code)
	init_oc();
	
	//Set servo control pins as output
	pin_digitalOut(PAN_PIN);
	pin_digitalOut(TILT_PIN);
	
	//Set LED off
	led_off(LED);
	//Configure blinking rate for LED when connected
    timer_setPeriod(LED_TIM, 0.2);
    timer_start(LED_TIM);
	
	//According to HobbyKing documentation: range .8 through 2.2 msec
	//Set servo control pins as OC outputs on their respective timers
	oc_servo(SERVO1_OC, PAN_PIN, 	SERVO1_TIM, SERVO_PERIOD, SERVO_MIN, SERVO_MAX, pan_set_val);
	oc_servo(SERVO2_OC, TILT_PIN, 	SERVO2_TIM, SERVO_PERIOD, SERVO_MIN, SERVO_MAX, tilt_set_val);

    InitUSB();                              // initialize the USB registers and serial interface engine
    while (USB_USWSTAT!=CONFIG_STATE) {     // while the peripheral is not configured...
        ServiceUSB();                       // ...service USB requests
		led_on(LED);
		//There's no point in driving the servos when there's no one connected yet.
    }
	
    while (1) {
        ServiceUSB();                       // service any pending USB requests
		
		//blink the LED
		if (timer_flag(LED_TIM)) {
            timer_lower(LED_TIM);
            led_toggle(LED);
        }
		
		//Update the servo control values.
		pin_write(PAN_PIN, pan_set_val);
		pin_write(TILT_PIN, tilt_set_val);
    }
}

