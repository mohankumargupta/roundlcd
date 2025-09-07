// GC9A01 Rounded Display Simulation Model
//
// This simulation model implements the GC9A01 1.2" 240x240 LCD chip
// with a “rounded‐display” area. Commands sent via SPI (using the DC pin)
// are parsed incrementally (the SPI buffer may contain only a few bytes at a time).
// When pixel data is received (after a RAMWR command) the pixel is written
// only if its (x,y) lies inside a centered circle; otherwise the pixel is forced black.
//
// Compatible with the Adafruit_GC9A01A library.
// 
// SPDX-License-Identifier: MIT
// (c) 2025 CodeMagic LTD

#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/*-----------------------------------------------------------
   GC9A01 Command Codes
-----------------------------------------------------------*/
#define GC9A01_SWRESET  0x01   // Software reset
#define GC9A01_SLPOUT   0x11   // Sleep out
#define GC9A01_DISPON   0x29   // Display ON
#define GC9A01_DISPOFF  0x28   // Display OFF
#define GC9A01_CASET    0x2A   // Column address set – 4 args (startHi, startLo, endHi, endLo)
#define GC9A01_RASET    0x2B   // Row address set – 4 args
#define GC9A01_RAMWR    0x2C   // Memory write
#define GC9A01_MADCTL   0x36   // Memory Access Control – 1 arg
#define GC9A01_COLMOD   0x3A   // Pixel Format Set – 1 arg
#define GC9A01_INVOFF   0x20   // Inversion OFF
#define GC9A01_INVON    0x21   // Inversion ON

/*-----------------------------------------------------------
   SPI Mode: Command vs Data
-----------------------------------------------------------*/
typedef enum {
  GC9A01_MODE_COMMAND,
  GC9A01_MODE_DATA
} gc9a01_mode_t;

/*-----------------------------------------------------------
   GC9A01 State Structure
-----------------------------------------------------------*/
typedef struct {
  /* SPI related */
  spi_dev_t spi;
  pin_t cs_pin;
  pin_t dc_pin;
  pin_t rst_pin;
  uint8_t spi_buffer[256]; // Added SPI buffer for receiving SPI data packets

  /* Display framebuffer and dimensions */
  buffer_t framebuffer;
  uint32_t width;
  uint32_t height;

  /* Command/Data parser state */
  gc9a01_mode_t mode;      // Determined by the DC pin (LOW=command, HIGH=data)
  bool is_receiving_command;
  uint8_t current_command;
  uint8_t expected_args;
  uint8_t received_args;
  uint8_t command_args[16];  // Buffer for command parameters

  /* Data mode: pending byte when receiving 16-bit pixel data */
  uint8_t pending_data;
  bool pending_data_valid;

  /* Address window (set via CASET/RASET) and current pixel pointer */
  uint16_t col_start;
  uint16_t col_end;
  uint16_t row_start;
  uint16_t row_end;
  uint16_t current_col;
  uint16_t current_row;

  /* RAM write flag: true when RAMWR command is active */
  bool ram_write;

  /* Other display flags */
  bool display_on;
  bool inverted;  // Inversion flag (INVON/INVOFF)
} gc9a01_state_t;

/*-----------------------------------------------------------
   Helper: Expected argument count for each command.
-----------------------------------------------------------*/
static uint8_t get_expected_arg_count(uint8_t command) {
  switch (command) {
    case GC9A01_SWRESET:
    case GC9A01_SLPOUT:
    case GC9A01_DISPON:
    case GC9A01_DISPOFF:
    case GC9A01_RAMWR:
    case GC9A01_INVOFF:
    case GC9A01_INVON:
      return 0;
    case GC9A01_CASET:
    case GC9A01_RASET:
      return 4;
    case GC9A01_MADCTL:
      return 1;
    case GC9A01_COLMOD:
      return 1;
    default:
      return 0;
  }
}

/*-----------------------------------------------------------
   Helper: Convert RGB565 to 32-bit RGBA.
   This conversion follows the GC9A01 driver model.
-----------------------------------------------------------*/
static uint32_t rgb565_to_rgba(uint16_t value) {
  return 0xff000000 | ((value & 0x001F) << 19) | ((value & 0x07E0) << 5) | ((value & 0xF800) >> 8);
}

/*-----------------------------------------------------------
   Process a complete command (command byte and parameters).
-----------------------------------------------------------*/
static void process_command(gc9a01_state_t *state, uint8_t command, uint8_t *args, uint8_t len) {
  switch (command) {
    case GC9A01_SWRESET:
      {
        // Fill framebuffer with black.
        uint32_t black = 0xff000000;
        for (uint32_t y = 0; y < state->height; y++) {
          for (uint32_t x = 0; x < state->width; x++) {
            uint32_t offset = (y * state->width + x) * 4;
            buffer_write(state->framebuffer, offset, &black, sizeof(black));
          }
        }
        state->display_on = false;
        state->inverted = false;
        state->ram_write = false;
        state->col_start = 0;
        state->col_end = state->width - 1;
        state->row_start = 0;
        state->row_end = state->height - 1;
        state->current_col = 0;
        state->current_row = 0;
      }
      break;
    case GC9A01_SLPOUT:
      break;
    case GC9A01_DISPON:
      state->display_on = true;
      break;
    case GC9A01_DISPOFF:
      state->display_on = false;
      break;
    case GC9A01_CASET:
      if (len == 4) {
        state->col_start = (args[0] << 8) | args[1];
        state->col_end   = (args[2] << 8) | args[3];
        state->current_col = state->col_start;
      }
      break;
    case GC9A01_RASET:
      if (len == 4) {
        state->row_start = (args[0] << 8) | args[1];
        state->row_end   = (args[2] << 8) | args[3];
        state->current_row = state->row_start;
      }
      break;
    case GC9A01_RAMWR:
      state->ram_write = true;
      state->pending_data_valid = false;
      break;
    case GC9A01_MADCTL:
      break;
    case GC9A01_COLMOD:
      break;
    case GC9A01_INVOFF:
      state->inverted = false;
      break;
    case GC9A01_INVON:
      state->inverted = true;
      break;
    default:
      break;
  }
}

/*-----------------------------------------------------------
   Process one pixel (16-bit RGB565 value) received during RAMWR.
   Pixels are written into the defined address window and then
   converted to 32-bit RGBA. A rounded‐mask is applied so that
   pixels outside a centered circle (radius = width/2) are forced black.
-----------------------------------------------------------*/
static void process_pixel(gc9a01_state_t *state, uint16_t pixel_val) {
  uint32_t color = rgb565_to_rgba(pixel_val);

  if (state->inverted) {
    uint8_t r = 255 - ((color >> 16) & 0xFF);
    uint8_t g = 255 - ((color >> 8)  & 0xFF);
    uint8_t b = 255 - (color & 0xFF);
    color = 0xff000000 | (r << 16) | (g << 8) | b;
  }

  if (state->current_col < state->col_start || state->current_col > state->col_end ||
      state->current_row < state->row_start || state->current_row > state->row_end) {
    return;
  }

  const int center = state->width / 2;
  int dx = (int)state->current_col - center;
  int dy = (int)state->current_row - center;
  if ((dx * dx + dy * dy) > (center * center)) {
    color = 0xff000000;
  }

  uint32_t offset = (state->current_row * state->width + state->current_col) * 4;
  buffer_write(state->framebuffer, offset, &color, sizeof(color));

  state->current_col++;
  if (state->current_col > state->col_end) {
    state->current_col = state->col_start;
    state->current_row++;
    if (state->current_row > state->row_end) {
      state->current_row = state->row_start;
    }
  }
}

/*-----------------------------------------------------------
   SPI callback: Called when an SPI packet is received.
   The packet may be incomplete; we process each byte according
   to the current DC mode.
-----------------------------------------------------------*/
static void gc9a01_spi_done(void *user_data, uint8_t *buffer, uint32_t count) {
  gc9a01_state_t *state = (gc9a01_state_t *)user_data;

  if (count == 0)
    return;

  for (uint32_t i = 0; i < count; i++) {
    uint8_t b = buffer[i];

    if (state->mode == GC9A01_MODE_COMMAND) {
      if (!state->is_receiving_command) {
        state->current_command = b;
        state->is_receiving_command = true;
        state->received_args = 0;
        state->expected_args = get_expected_arg_count(b);
        if (state->expected_args == 0) {
          process_command(state, state->current_command, NULL, 0);
          state->is_receiving_command = false;
        }
      } else {
        state->command_args[state->received_args++] = b;
        if (state->received_args >= state->expected_args) {
          process_command(state, state->current_command, state->command_args, state->expected_args);
          state->is_receiving_command = false;
        }
      }
    }
    else { // Data mode.
      if (state->ram_write) {
        if (state->pending_data_valid) {
          uint16_t pixel_val = (state->pending_data << 8) | b;
          state->pending_data_valid = false;
          process_pixel(state, pixel_val);
        } else {
          state->pending_data = b;
          state->pending_data_valid = true;
        }
      }
    }
  }

  if (pin_read(state->cs_pin) == LOW) {
    spi_start(state->spi, state->spi_buffer, 256);
  }
}

/*-----------------------------------------------------------
   Pin-change callback.
   Monitors the CS, DC and RST lines.
-----------------------------------------------------------*/
static void gc9a01_pin_change(void *user_data, pin_t pin, uint32_t value) {
  gc9a01_state_t *state = (gc9a01_state_t *)user_data;

  if (pin == state->cs_pin) {
    if (value == LOW) {
      state->is_receiving_command = false;
      state->pending_data_valid = false;
      spi_start(state->spi, state->spi_buffer, 256);
    } else {
      spi_stop(state->spi);
      state->ram_write = false;
      state->is_receiving_command = false;
      state->pending_data_valid = false;
    }
  }

  if (pin == state->dc_pin) {
    if (value == LOW)
      state->mode = GC9A01_MODE_COMMAND;
    else
      state->mode = GC9A01_MODE_DATA;

    spi_stop(state->spi);
    if (pin_read(state->cs_pin) == LOW)
      spi_start(state->spi, state->spi_buffer, 256);
  }

  if (pin == state->rst_pin && value == LOW) {
    spi_stop(state->spi);
    uint32_t black = 0xff000000;
    for (uint32_t y = 0; y < state->height; y++) {
      for (uint32_t x = 0; x < state->width; x++) {
        uint32_t offset = (y * state->width + x) * 4;
        buffer_write(state->framebuffer, offset, &black, sizeof(black));
      }
    }
    state->display_on = false;
    state->inverted = false;
    state->ram_write = false;
    state->col_start = 0;
    state->col_end = state->width - 1;
    state->row_start = 0;
    state->row_end = state->height - 1;
    state->current_col = 0;
    state->current_row = 0;
  }
}

/*-----------------------------------------------------------
   Chip Initialization.
-----------------------------------------------------------*/
void chip_init(void) {
  gc9a01_state_t *state = calloc(1, sizeof(gc9a01_state_t));
  if (!state) {
    printf("GC9A01: Failed to allocate state memory!\n");
    return;
  }

  state->width  = 240;
  state->height = 240;

  state->display_on = false;
  state->inverted = false;
  state->ram_write = false;
  state->mode = GC9A01_MODE_COMMAND;
  state->is_receiving_command = false;
  state->pending_data_valid = false;

  state->col_start = 0;
  state->col_end   = state->width - 1;
  state->row_start = 0;
  state->row_end   = state->height - 1;
  state->current_col = 0;
  state->current_row = 0;

  state->cs_pin  = pin_init("CS", INPUT_PULLUP);
  state->dc_pin  = pin_init("DC", INPUT);
  state->rst_pin = pin_init("RST", INPUT_PULLUP);
  pin_init("SCL", INPUT_PULLUP);
  pin_init("SDA", INPUT_PULLUP);

  const pin_watch_config_t watch_config = {
    .edge = BOTH,
    .pin_change = gc9a01_pin_change,
    .user_data = state,
  };
  pin_watch(state->cs_pin,  &watch_config);
  pin_watch(state->dc_pin,  &watch_config);
  pin_watch(state->rst_pin, &watch_config);

  spi_config_t spi_conf = {
    .sck = pin_init("SCL", INPUT_PULLUP),
    .mosi = pin_init("SDA", INPUT_PULLUP),
    .miso = NO_PIN,
    .done = gc9a01_spi_done,
    .user_data = state
  };
  state->spi = spi_init(&spi_conf);

  state->framebuffer = framebuffer_init(&state->width, &state->height);

  uint32_t black = 0xff000000;
  for (uint32_t y = 0; y < state->height; y++) {
    for (uint32_t x = 0; x < state->width; x++) {
      uint32_t offset = (y * state->width + x) * 4;
      buffer_write(state->framebuffer, offset, &black, sizeof(black));
    }
  }

  printf("GC9A01 1.2\" 240x240 Rounded Display initialized!\n");
}
