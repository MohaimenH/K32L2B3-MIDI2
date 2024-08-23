#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "bsp/board_api.h"
#include "tusb.h"

#include "K32L2B31A.h"

#define MASK(x) (1UL << (x))

// Define pins for buttons and potentiometers
#define BUTTON1_PIN 3   // D3
#define BUTTON2_PIN 12  // A12
#define BUTTON3_PIN 4   // A4
#define POT1_PIN 1      // B1
#define POT2_PIN 0      // B0

//--------------------------------------------------------------------+
// Variables
//--------------------------------------------------------------------+

// Button states
static bool button1_pressed = false;
static bool button2_pressed = false;
static bool button3_pressed = false;

static bool button1_toggle = false;
static bool button2_toggle = false;
static bool button3_toggle = false;

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum {
	BLINK_NOT_MOUNTED = 250, BLINK_MOUNTED = 1000, BLINK_SUSPENDED = 2500,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

void led_blinking_task(void);
void ump_task(void);
void init_gpio(void);
void init_adc(void);
void read_inputs(void);
uint32_t read_adc(uint8_t channel);
uint32_t scale_adc_to_midi(uint32_t adc_value);

/*------------- MAIN -------------*/
int main(void) {
	// silly little change.
	board_init();
	init_gpio();
	init_adc();

	// init device stack on configured roothub port
	tud_init(BOARD_TUD_RHPORT);

	if (board_init_after_tusb) {
		board_init_after_tusb();
	}

	while (1) {
		tud_task(); // tinyusb device task
		led_blinking_task();
		ump_task();
	}
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void) {
	blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void) {
	blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en) {
	(void) remote_wakeup_en;
	blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void) {
	blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
}

//--------------------------------------------------------------------+
// Device Setup
//--------------------------------------------------------------------+

void init_gpio(void) {
	// Enable clock for PORTB, PORTD, and PORTA
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTD_MASK
			| SIM_SCGC5_PORTA_MASK;

	// Configure buttons as inputs with pull-ups
	PORTD->PCR[BUTTON1_PIN] = PORT_PCR_MUX(
			1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
	PORTA->PCR[BUTTON2_PIN] = PORT_PCR_MUX(
			1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
	PORTA->PCR[BUTTON3_PIN] = PORT_PCR_MUX(
			1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;

	// Configure potentiometer pins as analog inputs
	PORTB->PCR[POT1_PIN] = PORT_PCR_MUX(0);
	PORTB->PCR[POT2_PIN] = PORT_PCR_MUX(0);
}

void init_adc(void) {
	// Enable ADC0 clock
	SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;

	// Configure ADC
	ADC0->CFG1 = ADC_CFG1_MODE(3) |    // 16-bit conversion
			ADC_CFG1_ADLSMP_MASK; // Long sample time

	ADC0->SC3 = ADC_SC3_AVGE_MASK |    // Enable hardware average
			ADC_SC3_AVGS(3);       // 32 samples averaged

	// Calibrate the ADC
	ADC0->SC3 |= ADC_SC3_CAL_MASK;
	while (ADC0->SC3 & ADC_SC3_CAL_MASK)
		;
}

void read_inputs(void) {
	// Read button states (active low due to pull-ups)
	button1_pressed = !(PTD->PDIR & MASK(BUTTON1_PIN));
	button2_pressed = !(PTA->PDIR & MASK(BUTTON2_PIN));
	button3_pressed = !(PTA->PDIR & MASK(BUTTON3_PIN));
}

uint32_t read_adc(uint8_t channel) {
	ADC0->SC1[0] = ADC_SC1_ADCH(channel);
	while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK))
		;
	return ADC0->R[0];
}

uint32_t scale_adc_to_midi(uint32_t adc_value) {
	// Scale 16-bit ADC value (0-65535) to 32-bit MIDI value (0-4294967295)
	return (uint32_t) (((uint64_t) adc_value * 4294967295ULL) / 65535ULL);
}

//--------------------------------------------------------------------+
// Helpers
//--------------------------------------------------------------------+

// function to convert decimal to binary

static const uint32_t steppedValues[] = {
    0x00000000u, 0x03E0F83Fu, 0x07C1F07Eu, 0x0BA2E8BDu, 0x0F83E0FCu, 0x1364D93Bu, 0x1745D17Au, 0x1B26C9B9u,
    0x1F07C1F8u, 0x22E8BA37u, 0x26C9B276u, 0x2AAAAAb5u, 0x2E8BA2F4u, 0x326C9B33u, 0x364D9372u, 0x3A2E8BB1u,
    0x3E0F83F0u, 0x41F07C2Fu, 0x45D1746Eu, 0x49B266ADu, 0x4D9358ECu, 0x7FFFFFFFu
};

uint32_t pitchCounter = 0;
uint32_t polyCounter = 0;

//--------------------------------------------------------------------+
// UMP Task
//--------------------------------------------------------------------+

void ump_task(void) {
	static uint32_t start_ms = 0;
	uint8_t const channel = 0; // MIDI jack associated with USB endpoint

	// Read inputs
	read_inputs();

	// The MIDI interface always creates input and output port/jack descriptors
	// regardless of these being used or not. Therefore incoming traffic should be read
	// (possibly just discarded) to avoid the sender blocking in IO
#define RECEIVE_PACKET_WIDTH 2
	uint32_t packet[RECEIVE_PACKET_WIDTH];
	while (tud_ump_n_available(channel)) {
		tud_ump_read(channel, packet, RECEIVE_PACKET_WIDTH);
		tud_ump_write(channel, packet, RECEIVE_PACKET_WIDTH);
	}

#define SEND_PACKET_WIDTH 2
	// send note periodically
	if (board_millis() - start_ms < 286)
		return; // not enough time
	start_ms += 286;

	// Send Note On when BUTTON2 is pressed
	if (button2_pressed && !button2_toggle) {
		uint32_t note_on_packet[2];
		note_on_packet[0] = 0b01000000100100000011110001100000u; // Group 0, Note On (1001), Note 60 (middle C)
		note_on_packet[1] = 0b01111111111111110000000000000000u; // Velocity 96 (32-bit resolution)
		tud_ump_write(channel, note_on_packet, SEND_PACKET_WIDTH);
		button2_toggle = true;
	} else if (!button2_pressed && button2_toggle) {
		uint32_t note_off_packet[2];
		note_off_packet[0] = 0b01000000100000000011110001100000u; // Group 0, Note Off (1000), Note 60 (middle C)
		note_off_packet[1] = 0b00000000000000000000000000000000u; // Velocity 0 (32-bit resolution)
		tud_ump_write(channel, note_off_packet, SEND_PACKET_WIDTH);
		button2_toggle = false;
	}

	// Send Pitch Bend when BUTTON1 is pressed, use POT1 for value
	if (button1_pressed && !button1_toggle) {
//		uint32_t pot1_value = read_adc(POT1_PIN);
//		uint32_t pitch_bend_value = scale_adc_to_midi(pot1_value);

		uint32_t pitch_bend_packet[2];
		pitch_bend_packet[0] = 0b01000000011000000011110000000000u;
//	    pitch_bend_packet[1] = pitch_bend_value;
		pitch_bend_packet[1] = steppedValues[pitchCounter];
		pitchCounter++;
		if (pitchCounter >= sizeof(steppedValues)/sizeof(steppedValues[0])) pitchCounter = 0;
		tud_ump_write(channel, pitch_bend_packet, SEND_PACKET_WIDTH);
		button1_toggle = true;
	} else if (!button1_pressed && button1_toggle) {
		// When button is released, send center pitch bend value
//		uint32_t pitch_bend_packet[2];
//		pitch_bend_packet[0] = 0b01000000011000000011110000000000u;
//		pitch_bend_packet[1] = 0x80000000u; // Center pitch bend value
//		tud_ump_write(channel, pitch_bend_packet, SEND_PACKET_WIDTH);
		button1_toggle = false;
	}

	// Send Poly Pressure when BUTTON3 is pressed, use POT2 for value
	if (button3_pressed && !button3_toggle) {
//		uint32_t pot2_value = read_adc(POT2_PIN);
//		uint32_t pressure_value = scale_adc_to_midi(pot2_value);

		uint32_t poly_pressure_packet[2];
		poly_pressure_packet[0] = 0b01000000101000000011110000000000u; // Group 0, Poly Pressure (1010), 32-bit message
		poly_pressure_packet[1] = steppedValues[polyCounter];
		polyCounter++;
		if (polyCounter >= sizeof(steppedValues)/sizeof(steppedValues[0])) polyCounter = 0;
		tud_ump_write(channel, poly_pressure_packet, SEND_PACKET_WIDTH);
		button3_toggle = true;
	} else if (!button3_pressed && button3_toggle) {
		button3_toggle = false;
	}
}
//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void) {
	static uint32_t start_ms = 0;
	static bool led_state = false;

	// Blink every interval ms
	if (board_millis() - start_ms < blink_interval_ms)
		return; // not enough time
	start_ms += blink_interval_ms;

	board_led_write(led_state);
	led_state = 1 - led_state; // toggle

	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

	PORTE->PCR[0] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[0] |= PORT_PCR_MUX(3);

	PORTE->PCR[1] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[1] |= PORT_PCR_MUX(3);
}
