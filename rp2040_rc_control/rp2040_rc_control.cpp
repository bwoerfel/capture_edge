#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/divider.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"


#include "capture_edge.h"

float clk_div = 1;
volatile uint capture_counter, pin;
volatile float frequency, duty, duration_cycle;
volatile bool is_captured;
volatile edge_type_t edge_type;

static void capture_pin_0_handler(uint counter, edge_type_t edge) {
    static uint counter_edge_rising = 0, counter_edge_falling = 0;
    capture_counter = counter;
    pin = 0;
    is_captured = true;
    edge_type = edge;

    if (edge == EDGE_RISING) {
        duration_cycle = (float)(counter - counter_edge_rising) / clock_get_hz(clk_sys) * COUNTER_CYCLES;
        frequency = 1 / duration_cycle;
        counter_edge_rising = counter;
    }
    if (edge == EDGE_FALLING) {
        float duration_pulse = (float)(counter - counter_edge_rising) / clock_get_hz(clk_sys) * COUNTER_CYCLES;
        duty = duration_pulse / duration_cycle * 100;
        counter_edge_falling = counter;
    }
}

static void capture_pin_1_handler(uint counter, edge_type_t edge) {
    static uint counter_edge_rising = 0, counter_edge_falling = 0;
    capture_counter = counter;
    pin = 1;
    is_captured = true;
    edge_type = edge;

    if (edge == EDGE_RISING) {
        duration_cycle = (float)(counter - counter_edge_rising) / clock_get_hz(clk_sys) * COUNTER_CYCLES;
        frequency = 1 / duration_cycle;
        counter_edge_rising = counter;
    }
    if (edge == EDGE_FALLING) {
        float duration_pulse = (float)(counter - counter_edge_rising) / clock_get_hz(clk_sys) * COUNTER_CYCLES;
        duty = duration_pulse / duration_cycle * 100;
        counter_edge_falling = counter;
    }
}

/*
 * 8-channel Radio Control PPM example
 * Output on GPIO3
 *

 *  synchro    |  ch0 value 1-2ms    |  ch1 value 1-2ms  |...|  ch8 value 1-2ms  |
 *  >=5ms       _______               _______                 _______              _______
 * _______...__| 0.5ms |_____________| 0.5ms |___________|...| 0.5ms |____________| 0.5ms |_...
 *
 */

// UART defines
// By default the stdout UART is `uart0`, so we will use the second one
#define UART_ID uart1
#define BAUD_RATE 9600

// Use pins 4 and 5 for UART1
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define UART_TX_PIN 24
#define UART_RX_PIN 25

// GPIO defines
// Example uses GPIO 2
#define GPIO_TEST 2

// This example drives a PWM output at a range of duty cycles, and uses
// another PWM slice in input mode to measure the duty cycle. You'll need to
// connect these two pins with a jumper wire:
const uint OUTPUT_PIN = 2;
const uint MEASURE_PIN = 5;

// Data will be copied from src to dst
const char src[] = "Hello, world! (from DMA)";
char dst[count_of(src)];

#include "blink.pio.h"
#include "ws2812.pio.h"
#include "ppm.pio.h"
#define PPM_io 14
#include "PwmIn.pio.h"
#define PWMIN_io 5

PIO pio[] = {pio0, pio1};
uint sm[] = {0, 0};

void set_value_and_log(uint channel, uint value_usec) {
    //printf("Channel %d is set to value %5.3f ms\r\n", channel, (float)value_usec / 1000.0f);
    ppm_set_value(channel, value_usec);

}

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    pio->txf[sm] = (125000000 / (2 * freq)) - 3;
}

// NEOPIXEL STUFF
static inline void put_pixel(uint32_t pixel_grb) {    
    pio_sm_put_blocking(pio[1], sm[1], pixel_grb << 8u);
}

int main()
{
    stdio_init_all();
    sleep_ms(500);   

    // PIO Blinking example
    int offset[] = {0, pio_add_program(pio[1], &ws2812_program)};
    printf("Loaded programs at [%d, %d]\n", offset[0], offset[1]);    
    ws2812_program_init(pio[1], sm[1], offset[1], 21, 20, 800000, false);    
    //ppm_program_init(pio[0], PPM_io);
    
    uint pin_base = 0;      // starting gpio to capture
    uint irq = PIO0_IRQ_0;  // values for pio0: PIO0_IRQ_0, PIO0_IRQ_1. values for pio1: PIO1_IRQ_0, PIO1_IRQ_1    
    capture_edge_init(pio[0], pin_base, clk_div, irq);
    capture_edge_set_handler(0, capture_pin_0_handler);
    capture_edge_set_handler(1, capture_pin_1_handler);    
    // For more pio examples see https://github.com/raspberrypi/pico-examples/tree/master/pio

    // Watchdog example code
    if (watchdog_caused_reboot()) {
        printf("Rebooted by Watchdog!\n");
        put_pixel(0x00FF00);
        // Whatever action you may take if a watchdog caused a reboot
    } else {
        put_pixel(0x800080);
    } 



    // Enable the watchdog, requiring the watchdog to be updated every 100ms or the chip will reboot
    // second arg is pause on debug which means the watchdog will pause when stepping through code
    watchdog_enable(5000, 1);

    while (true) {

        
        /*
        sleep_ms(250);
        set_value_and_log(5, 1100);
        gpio_put(13, !gpio_get(13));
        gpio_put(12, !gpio_get(12));
        sleep_ms(250);
        set_value_and_log(1, 1200);
        gpio_put(13, !gpio_get(13));
        gpio_put(12, !gpio_get(12));
        */
        
        if (is_captured) {
        
            printf("\n\r[%9d] Capture pin %u. Counter: %u State: %s Duration(us): %.0f", to_ms_since_boot(get_absolute_time()), pin, capture_counter,
                   edge_type == EDGE_FALLING ? "High" : "Low ", duration_cycle * 1000000);
            if (edge_type == EDGE_RISING) printf(" Freq(Hz): %.1f Duty: %.1f[%.1f]", frequency, duty, duty/5.0f*1000.0f);
            is_captured = false;
        }
       

        

                     

        // You need to call this function at least more often than the 100ms in the enable call to prevent a reboot
        watchdog_update();
    }
}
