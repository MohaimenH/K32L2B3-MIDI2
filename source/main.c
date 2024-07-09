/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "bsp/board_api.h"
#include "tusb.h"

#include "K32L2B31A.h"

#define MASK(x) (1UL << (x))

/* This MIDI example send sequence of note (on/off) repeatedly. To test on PC, you need to install
 * synth software and midi connection management software. On
 * - Linux (Ubuntu): install qsynth, qjackctl. Then connect TinyUSB output port to FLUID Synth input port
 * - Windows: install MIDI-OX
 * - MacOS: SimpleSynth
 */

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum  {
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

void led_blinking_task(void);
void ump_task(void);

/*------------- MAIN -------------*/
int main(void)
{
  board_init();

  // init device stack on configured roothub port
  tud_init(BOARD_TUD_RHPORT);

  if (board_init_after_tusb) {
    board_init_after_tusb();
  }

  while (1)
  {
    tud_task(); // tinyusb device task
    led_blinking_task();
    ump_task();
  }
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
}

//--------------------------------------------------------------------+
// MIDI Task
//--------------------------------------------------------------------+

// Variable that holds the current position in the sequence.
uint32_t note_pos = 0;

// Store example melody as an array of note values
uint8_t note_sequence[] =
{
  74,78,81,86,90,93,98,102,57,61,66,69,73,78,81,85,88,92,97,100,97,92,88,85,81,78,
  74,69,66,62,57,62,66,69,74,78,81,86,90,93,97,102,97,93,90,85,81,78,73,68,64,61,
  56,61,64,68,74,78,81,86,90,93,98,102
};

void ump_task(void)
{
  static uint32_t start_ms = 0;

  uint8_t const cable_num = 0; // MIDI jack associated with USB endpoint
//  uint8_t const channel   = 0; // 0 for channel 1

  // The MIDI interface always creates input and output port/jack descriptors
  // regardless of these being used or not. Therefore incoming traffic should be read
  // (possibly just discarded) to avoid the sender blocking in IO
  #define NUM_WORDS 1
  uint32_t packet[NUM_WORDS];
  while ( tud_ump_n_available(cable_num) ) tud_ump_read(cable_num, packet, NUM_WORDS);

  // send note periodically
  if (board_millis() - start_ms < 286) return; // not enough time
  start_ms += 286;

  // Previous positions in the note sequence.
  int previous = (int) (note_pos - 1);

  // If we currently are at position 0, set the
  // previous position to the last note in the sequence.
  if (previous < 0) previous = sizeof(note_sequence) - 1;

  // Send Note On for current position at full velocity (127) on channel 1.
//  uint8_t note_on[3] = { 0x90 | channel, note_sequence[note_pos], 127 };
//  uint32_t note_on_packet[NUM_WORDS];
//  note_on_packet[0] = (0x0 << 28) | (0x9 << 24) | (0x0 << 20) | (note_sequence[note_pos] << 8) | 0x40;
//  tud_ump_write(cable_num, note_on_packet, NUM_WORDS);
//  printf("Note On");
//
//  // Send Note Off for previous note.
////  uint8_t note_off[3] = { 0x80 | channel, note_sequence[previous], 0};
//  uint32_t note_off_packet[NUM_WORDS];
//  note_off_packet[0] = (0x0 << 28) | (0x8 << 24) | (0x0 << 20) | (note_sequence[previous] << 8) | 0x40;
//  tud_ump_write(cable_num, note_off_packet, NUM_WORDS);
//  printf("Note Off");


  uint32_t note_on_packet[2];
  note_on_packet[0] = 0b01000000100100000011110001100000; // Group 0, Note On (1001), Note 60 (middle C)
  note_on_packet[1] = 0b00000000011000000000000000000000; // Velocity 96 (32-bit resolution)
  tud_ump_write(cable_num, note_on_packet, 2);

  // Pitch bend packet using binary
  uint32_t pitch_bend_packet[2];
  pitch_bend_packet[0] = 0b01000001111000000011110000000000;  // Group 0, Pitch Bend (1110), 32-bit message
  pitch_bend_packet[1] = 0b00000000000000000100000000000000; // Pitch Bend value (32-bit, centered at 0x80000000)
  tud_ump_write(cable_num, pitch_bend_packet, 2);

  // Poly pressure packet using binary
  uint32_t poly_pressure_packet[2];
  poly_pressure_packet[0] = 0b01000001101000000011110000000000; // Group 0, Poly Pressure (1010), 32-bit message
  poly_pressure_packet[1] = 0b00000000000000001000000000000000; // Poly Pressure value (32-bit, range from 0 to 0xFFFFFFFF)
  tud_ump_write(cable_num, poly_pressure_packet, 2);
  // Increment position
  note_pos++;

  // If we are at the end of the sequence, start over.
  if (note_pos >= sizeof(note_sequence)) note_pos = 0;
}
//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void)
{
  static uint32_t start_ms = 0;
  static bool led_state = false;

  // Blink every interval ms
  if ( board_millis() - start_ms < blink_interval_ms) return; // not enough time
  start_ms += blink_interval_ms;

  board_led_write(led_state);
  led_state = 1 - led_state; // toggle

  SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

  PORTE->PCR[0] &= ~PORT_PCR_MUX_MASK;
  PORTE->PCR[0] |= PORT_PCR_MUX(3);

  PORTE->PCR[1] &= ~PORT_PCR_MUX_MASK;
  PORTE->PCR[1] |= PORT_PCR_MUX(3);
}
