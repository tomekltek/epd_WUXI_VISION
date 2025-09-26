/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/spi.h"
#include "ws2812.pio.h"

/**
 * NOTE:
 *  Take into consideration if your WS2812 is a RGB or RGBW variant.
 *
 *  If it is RGBW, you need to set IS_RGBW to true and provide 4 bytes per 
 *  pixel (Red, Green, Blue, White) and use urgbw_u32().
 *
 *  If it is RGB, set IS_RGBW to false and provide 3 bytes per pixel (Red,
 *  Green, Blue) and use urgb_u32().
 *
 *  When RGBW is used with urgb_u32(), the White channel will be ignored (off).
 *
 */
#define IS_RGBW false
#define NUM_PIXELS 5

// default to pin 2 if the board doesn't have a default WS2812 pin defined
#define WS2812_PIN 16   //oryginalna płytka RP2040-zero

// ====== KONFIGURACJA PINÓW (Opcja A) ======
#define EPD_SPI         spi0
#define PIN_SCK         2   // GP2  -> SCK
#define PIN_MOSI        3   // GP3  -> MOSI
#define PIN_MISO        4   // GP4  -> MISO (opcjonalnie, dla e-paper zwykle niepotrzebne)
#define PIN_CS          1   // GP1  -> CS#
#define PIN_DC          6   // GP6  -> D/C
#define PIN_RST         7   // GP7  -> RST#
#define PIN_BUSY        8   // GP8  -> BUSY

// Check the pin is compatible with the platform
#if WS2812_PIN >= NUM_BANK0_GPIOS
#error Attempting to use a pin>=32 on a platform that does not support it
#endif


#define SPI_BAUD  (2*1000*1000)
#define BUSY_ACTIVE_HIGH 1   // 1 = zajęty (wg Twojej obserwacji)


static inline void put_pixel(PIO pio, uint sm, uint32_t pixel_grb) {
    pio_sm_put_blocking(pio, sm, pixel_grb << 8u);
}

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return
            ((uint32_t) (r) << 8) |
            ((uint32_t) (g) << 16) |
            (uint32_t) (b);
}

static inline uint32_t urgbw_u32(uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
    return
            ((uint32_t) (r) << 8) |
            ((uint32_t) (g) << 16) |
            ((uint32_t) (w) << 24) |
            (uint32_t) (b);
}
//            put_pixel(pio, sm, urgb_u32(0xff, 0, 0));

// ====== Parametry panelu 2.15" (GDEW0215T11 / UC8151D) ======
#define EPD_WIDTH   112
#define EPD_HEIGHT  208
#define EPD_ARRAY   (EPD_WIDTH * EPD_HEIGHT / 8)

// ====== Niskopoziomowe I/O ======
static inline void epd_cs(bool level){  gpio_put(PIN_CS,  level); }
static inline void epd_dc(bool level){  gpio_put(PIN_DC,  level); }
static inline void epd_rst(bool level){ gpio_put(PIN_RST, level); }

static inline void epd_write_bytes(const uint8_t *buf, size_t len){
    spi_write_blocking(spi0, buf, len);
}
static inline void epd_write_cmd(uint8_t c){
    epd_cs(false);
    epd_dc(false);
    epd_write_bytes(&c, 1);
    epd_cs(true);
}
static inline void epd_write_data(uint8_t d){
    epd_cs(false);
    epd_dc(true);
    epd_write_bytes(&d, 1);
    epd_cs(true);
}

// Czekaj aż BUSY=1 (gotowy) — jak w Twojej wersji Arduino_UNO
static void epd_wait_ready(void){
    // timeout awaryjny ~10s, żeby nie zawiesić się na wieki
//    const uint64_t t0 = time_us_64();
    while(gpio_get(PIN_BUSY) == 0){
        tight_loop_contents();
     //   if (time_us_64() - t0 > 10ULL*1000*1000) break;
    }
}

// ====== Komendy wysokiego poziomu (z Twojego .cpp) ======
// Pełna inicjalizacja (wersja z EPD_Init)
static void epd_init_full(void){
    // reset x3 (jak w źródle)
    for(int i=0;i<3;i++){
        epd_rst(false); sleep_ms(10);
        epd_rst(true);  sleep_ms(10);
    }
    epd_wait_ready();

    // POWER SETTING
    epd_write_cmd(0x01);
    epd_write_data(0x03);  
    epd_write_data(0x00);
    epd_write_data(0x2b);
    epd_write_data(0x2b);
    epd_write_data(0x13);

    //BOOSTER SOFT
/*     epd_write_cmd(0x06);
    epd_write_data(0x17);  
    epd_write_data(0x17);
    epd_write_data(0x17); */

    // 0x00 panel setting: 0x1F, 0x0D
    epd_write_cmd(0x00);
    epd_write_data(0x1F);  // LUT from OTP (BW OTP)
    epd_write_data(0x0D);

    // 0x61 resolution: width, height high, height low
    epd_write_cmd(0x61);
    epd_write_data(EPD_WIDTH);
    epd_write_data(EPD_HEIGHT >> 8);
    epd_write_data(EPD_HEIGHT & 0xFF);

    // 0x04 POWER ON
    epd_write_cmd(0x04);
    epd_wait_ready();

    // 0x50 VCOM & data interval
    epd_write_cmd(0x50);
    epd_write_data(0x57);
}

// Wyślij pełną ramkę: najpierw "stare" (0x10) = biel, potem "nowe" (0x13) = bufor
static void epd_frame_push(const uint8_t *newbuf){
    // Old data
    epd_write_cmd(0x10);
    for (int i=0; i<EPD_ARRAY; i++) epd_write_data(0x00); // białe tło

    // New data
    epd_write_cmd(0x13);
    for (int i=0; i<EPD_ARRAY; i++) epd_write_data(newbuf ? newbuf[i] : 0xff);
}

// Wyzwól odświeżenie (0x12) i poczekaj
static void epd_update(void){
    epd_write_cmd(0x12);
    sleep_ms(1);
    epd_wait_ready();
}

// Usypianie jak w Twoim .cpp
static void epd_deep_sleep(void){
    epd_write_cmd(0x50);
    epd_write_data(0xF7);
    epd_write_cmd(0x02);      // power off
    epd_wait_ready();
    sleep_ms(200);//!!!The delay here is necessary,100mS at least!!!  
    epd_write_cmd(0x07);      // deep sleep
    epd_write_data(0xA5);
}

// ====== Prosta grafika testowa ======
static uint8_t fb[EPD_ARRAY];

// Pasy poziome: górna połowa czarna, dolna biała
static void make_test_bands(void){
    // UWAGA: 1 bit = 1 piksel, 0 = CZARNY, 1 = BIAŁY
    // Każdy wiersz ma EPD_WIDTH/8 = 14 bajtów
    const int stride = EPD_WIDTH / 8;
    for(int y=0; y<EPD_HEIGHT; y++){
        uint8_t v = (y < (EPD_HEIGHT/2)) ? 0x00 : 0xFF; // pół ekranu czarne
        for(int x=0; x<stride; x++){
            fb[y*stride + x] = v;
        }
    }
}

int main() {
    PIO pio;
    uint sm;
    uint offset;


    //set_sys_clock_48();
    sleep_ms(1000);
    printf("\n=== EPD quick tester (RP2040) ===\n");

    // GPIO
    gpio_init(PIN_CS);  gpio_set_dir(PIN_CS, GPIO_OUT);  gpio_put(PIN_CS,  true);
    gpio_init(PIN_DC);  gpio_set_dir(PIN_DC, GPIO_OUT);  gpio_put(PIN_DC, true);
    gpio_init(PIN_RST); gpio_set_dir(PIN_RST, GPIO_OUT); gpio_put(PIN_RST, true);
    gpio_init(PIN_BUSY);gpio_set_dir(PIN_BUSY, GPIO_IN);
    // todo get free sm
//while (1) ;
    // This will find a free pio and state machine for our program and load it for us
    // We use pio_claim_free_sm_and_add_program_for_gpio_range (for_gpio_range variant)
    // so we will get a PIO instance suitable for addressing gpios >= 32 if needed and supported by the hardware
    bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&ws2812_program, &pio, &sm, &offset, WS2812_PIN, 1, true);
    hard_assert(success);

    ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, IS_RGBW);

    // SPI
    spi_init(EPD_SPI, SPI_BAUD);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    // MISO opcjonalnie
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    put_pixel(pio, sm, 0x101010);
    sleep_ms(1000);

// ——— Test 1: pełna inicjalizacja i białe czyszczenie ———
    epd_init_full();

    // Na początek biel (0xFF)
    for (int i=0;i<EPD_ARRAY;i++) fb[i]=0b10000000;
    epd_frame_push(fb);
       put_pixel(pio, sm, 0x100000);
    epd_update();
       put_pixel(pio, sm, 0x002000);

    // ——— Test 2: pasy (góra czarna, dół biała) ———
/*    make_test_bands();
  epd_frame_push(fb);
   epd_update();
       put_pixel(pio, sm, 0x004000);
     sleep_ms(25000); */
    // Zostaw obraz, uśpij panel
    sleep_ms(1000);
    epd_deep_sleep();


    put_pixel(pio, sm, 0x000040);
    // Zostaw logi dostępne
    while (1) { sleep_ms(1000); }
    return 0;


}
