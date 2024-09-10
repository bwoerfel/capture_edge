/*
 * -------------------------------------------------------------------------------
 *
 * Copyright (c) 2022, Daniel Gorbea
 * All rights reserved.
 *
 * This source code is licensed under the MIT-style license found in the
 * LICENSE file in the root directory of this source tree.
 *
 * -------------------------------------------------------------------------------
 *
 *  A pio program to capture signal edges on any pins of the RP2040:
 *
 *  Base pin is 0. CAPTURE_EDGE_PIN_COUNT is 2 -> Pins 0 & 1 can be captured
 *
 *  Set the number of pins to capture in capture_edge.pio with CAPTURE_EDGE_PIN_COUNT
 *
 *  Connect a signal to pins 0 and/or 1 and check output at 115200
 *
 * -------------------------------------------------------------------------------
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "capture_edge.h"

#include "ws2812.pio.h"
#include "ppm.pio.h"
#define PPM_io 14

float clk_div = 1;
volatile uint counter, pin;
volatile float frequency, duty, duration;
volatile bool is_captured;
volatile edge_type_t edge_type;

PIO pio[] = {pio0, pio1};
uint sm[] = {0, 0};

void set_value_and_log(uint channel, uint value_usec) {
    //printf("Channel %d is set to value %5.3f ms\r\n", channel, (float)value_usec / 1000.0f);
    ppm_set_value(channel, value_usec);

}

// NEOPIXEL STUFF
static inline void put_pixel(uint32_t pixel_grb) {    
    pio_sm_put_blocking(pio[1], sm[1], pixel_grb << 8u);
}

static void capture_pin_0_handler(uint counter, edge_type_t edge)
{
    static uint counter_edge_rising = 0, counter_edge_falling = 0;
    pin = 0;
    is_captured = true;
    edge_type = edge;

    if (edge == EDGE_RISING)
    {
        duration = (float)(counter - counter_edge_rising) / clock_get_hz(clk_sys) * clk_div * COUNTER_CYCLES;
        frequency = 1 / duration;
        counter_edge_rising = counter;
    }
    if (edge == EDGE_FALLING)
    {
        float duration_pulse = (float)(counter - counter_edge_rising) / clock_get_hz(clk_sys) * clk_div * COUNTER_CYCLES;
        duty = duration_pulse / duration * 100;
        counter_edge_falling = counter;
    }
}

static void capture_pin_1_handler(uint counter, edge_type_t edge)
{
    static uint counter_edge_rising = 0, counter_edge_falling = 0;
    pin = 1;
    is_captured = true;
    edge_type = edge;

    if (edge == EDGE_RISING)
    {
        duration = (float)(counter - counter_edge_rising) / clock_get_hz(clk_sys) * clk_div * COUNTER_CYCLES;
        frequency = 1 / duration;
        counter_edge_rising = counter;
    }
    if (edge == EDGE_FALLING)
    {
        float duration_pulse = (float)(counter - counter_edge_rising) / clock_get_hz(clk_sys) * clk_div * COUNTER_CYCLES;
        duty = duration_pulse / duration * 100;
        counter_edge_falling = counter;
    }
}

int main()
{
    PIO pio = pio0;        // values: pio0, pio1
    uint pin_base = 0;     // starting gpio to capture
    uint irq = PIO0_IRQ_0; // values for pio0: PIO0_IRQ_0, PIO0_IRQ_1. values for pio1: PIO1_IRQ_0, PIO1_IRQ_1

    // PIO Blinking example
    int offset[] = {0, pio_add_program(pio[1], &ws2812_program)};
    printf("Loaded programs at [%d, %d]\n", offset[0], offset[1]);    
    //ws2812_program_init(pio[1], sm[1], offset[1], 21, 20, 800000, false);    
    ppm_program_init(pio1, PPM_io);

    sleep_ms(1000);

    stdio_init_all();

    capture_edge_init(pio, pin_base, clk_div, irq);
    capture_edge_set_handler(0, capture_pin_0_handler);
    capture_edge_set_handler(1, capture_pin_1_handler);

    while (true)
    {
        if (is_captured)
        {
            printf("\n\rCapture pin %u. Counter: %u State: %s Duration(us): %.0f", pin, counter, edge_type == EDGE_FALLING ? "High" : "Low ", duration * 1000000);
            if (edge_type == EDGE_RISING)
                printf(" Freq(Hz): %.1f Duty: %.1f", frequency, duty);
            is_captured = false;
            set_value_and_log(5, duty/5.0f*1000.0f);
        }
    }
}