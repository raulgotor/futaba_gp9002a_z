/*!
 *******************************************************************************
 * @file gp9002a.c
 *
 * @brief 
 *
 * @author Raúl Gotor (raulgotor@gmail.com)
 * @date 12.10.24
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2022 Raúl Gotor
 * All rights reserved.
 *******************************************************************************
 */

/*
 *******************************************************************************
 * #include Statements                                                         *
 *******************************************************************************
 */

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#include "gp9002a.h"

/*
 *******************************************************************************
 * Private Macros                                                              *
 *******************************************************************************
 */

#define SPIBB_NODE      DT_NODELABEL(arduino_spi)
#define GPIOA_NODE      DT_NODELABEL(gpioa)

#define CD_PIN          (9)
#define DISPLAY_MODE 0x14
#define DISPLAY_OR_1_2 0x10
#define MONOCHROME 0x10
#define DISPLAY_1_ON 0x01
#define DISPLAY_2_ON 0x02
#define LOWER_ADDR_START_1 0x0A
#define UPPER_ADDR_START_1 0x0B
#define LOWER_ADDR_START_2 0x0C
#define UPPER_ADDR_START_2 0x0D
#define CLEAR 0x06
#define HELD 0x05
#define NO_HELD 0x04
#define WRITE_DATA 0x08
#define GP9002_DATA_READ 0x09
#define GP9002_ADDRL 0xE
#define GP9002_ADDRH 0xF

/*
 *******************************************************************************
 * Data types                                                                  *
 *******************************************************************************
 */

/*
 *******************************************************************************
 * Constants                                                                   *
 *******************************************************************************
 */

/*
 *******************************************************************************
 * Private Function Prototypes                                                 *
 *******************************************************************************
 */

static int gp9002a_n_bytes_read(uint8_t * const p_data, size_t const size);

static int gp9002a_n_bytes_write(uint8_t const * const p_data, size_t const size, bool is_cmd);

static int gp9002a_command(uint8_t const * const p_data, size_t const size);

static int gp9002a_data_write(uint8_t const * const p_data, size_t const size);

static int gp9002a_data_read(uint8_t * const p_data, size_t const size);

static int command(uint8_t byte);

static int gp9002a_one_byte_write(uint8_t byte);

static int gp9002a_one_byte_read(uint8_t * const p_byte);


/*
 *******************************************************************************
 * Public Data Declarations                                                    *
 *******************************************************************************
 */

/*
 *******************************************************************************
 * Static Data Declarations                                                    *
 *******************************************************************************
 */

static uint8_t buffer[2024];


#define SLEEP_TIME_MS   100
#define SLEEP_TIME_MS_one   1

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);


const struct device *const dev = DEVICE_DT_GET(SPIBB_NODE);
const struct device *const gpioa = DEVICE_DT_GET(GPIOA_NODE);

struct spi_cs_control cs_ctrl = (struct spi_cs_control){
        .gpio = GPIO_DT_SPEC_GET(SPIBB_NODE, cs_gpios),
        .delay = 0u,
};

struct spi_config spi_cfg = {
        .frequency = 500000U,
        .operation = SPI_WORD_SET(8) | SPI_TRANSFER_LSB | SPI_OP_MODE_MASTER,

        .cs = &cs_ctrl,

};

/*
 *******************************************************************************
 * Public Function Bodies                                                      *
 *******************************************************************************
 */

void gp9002a_init(void)
{
        if (!gpio_is_ready_dt(&led)) {
                return;
        }

        if (!gpio_is_ready_dt(&gpioa)) {
                return;
        }

        gpio_pin_configure(gpioa, CD_PIN, GPIO_OUTPUT);

        command(DISPLAY_MODE);
        gp9002a_one_byte_write(MONOCHROME);

        command(LOWER_ADDR_START_1);
        gp9002a_one_byte_write(0x00);

        command(UPPER_ADDR_START_1);
        gp9002a_one_byte_write(0x00);

        command(LOWER_ADDR_START_2);
        gp9002a_one_byte_write(0x00);

        command(UPPER_ADDR_START_2);
        gp9002a_one_byte_write(0x04);

        command(DISPLAY_OR_1_2);
        command(CLEAR);
        command(DISPLAY_1_ON);
        command(HELD);

}

void gp9002a_draw_pixel(uint8_t const x, uint8_t const y)
{
        uint16_t address = x * 8 + (63 - y) / 8;
        uint8_t bit = 0x01 << (y % 8);
        uint8_t read_byte;


        command(HELD);
        command(GP9002_ADDRL);
        gp9002a_one_byte_write(address & 0xFF);

        command(GP9002_ADDRH);
        gp9002a_one_byte_write(address >> 8);

        command(GP9002_DATA_READ);

        k_msleep(1);
        gp9002a_one_byte_read(&read_byte);
        gp9002a_one_byte_read(&read_byte);
        gp9002a_one_byte_read(&read_byte);
        k_msleep(1);

        if (1) {
                bit |= buffer[address];
        } else {
                bit |= read_byte;
        }

        command(WRITE_DATA);
        gp9002a_one_byte_write(bit);
        buffer[address] = bit;
}
/*
 *******************************************************************************
 * Private Function Bodies                                                     *
 *******************************************************************************
 */

static int gp9002a_n_bytes_read(uint8_t * const p_data, size_t const size)
{
        int ret;

        struct spi_buf bufs[] = {
                {
                        .buf = p_data,
                        .len = size
                },
        };

        struct spi_buf_set tx = {
                .buffers =  bufs,
                .count = 1
        };

        ret = gpio_pin_set(gpioa, CD_PIN, 0);

        if (0 == ret) {
                ret = spi_read(dev, &spi_cfg, &tx);

        }
        //*p_data = 0xff;
        k_usleep(SLEEP_TIME_MS_one);

        return ret;
}

static int gp9002a_n_bytes_write(uint8_t const * const p_data, size_t const size, bool is_cmd)
{
        int ret;
        int cmd = is_cmd ? 1 : 0;

        struct spi_buf bufs[] = {
                {
                        .buf = p_data,
                        .len = size
                },
        };

        struct spi_buf_set tx = {
                .buffers =  bufs,
                .count = 1
        };

        ret = gpio_pin_set(gpioa, CD_PIN, cmd);
        if (0 == ret) {
                ret = spi_write(dev, &spi_cfg, &tx);
        }
        k_usleep(SLEEP_TIME_MS_one);

        return ret;
}

static int gp9002a_command(uint8_t const * const p_data, size_t const size)
{
        return gp9002a_n_bytes_write(p_data, size, true);
}

static int gp9002a_data_write(uint8_t const * const p_data, size_t const size)
{
        return gp9002a_n_bytes_write(p_data, size, false);
}

static int gp9002a_data_read(uint8_t * const p_data, size_t const size)
{
        return gp9002a_n_bytes_read(p_data, size);
}

static int command(uint8_t byte)
{
        return gp9002a_command(&byte, 1);
}

static int gp9002a_one_byte_write(uint8_t byte)
{
        return gp9002a_data_write(&byte, 1);
}

static int gp9002a_one_byte_read(uint8_t * const p_byte)
{
        return gp9002a_data_read(p_byte, 1);
}

/*
 *******************************************************************************
 * Interrupt Service Routines / Tasks / Thread Main Functions                  *
 *******************************************************************************
 */
