#include "MLX90640_I2C_Driver.h"
#include "pin_config.h"
#include "pio_i2c.h"
#include "i2c.pio.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include <stdbool.h>

#define MLX_I2C_FINAL_LSB 9u
#define MLX_I2C_DATA_LSB 1u
#define MLX_I2C_NAK_LSB 0u

static PIO mlx_pio = pio0;
static uint mlx_sm = 0;
static bool mlx_initialized = false;

static void i2c_rx_enable(PIO pio, uint sm, bool en) {
    if (en) {
        hw_set_bits(&pio->sm[sm].shiftctrl, PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS);
    } else {
        hw_clear_bits(&pio->sm[sm].shiftctrl, PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS);
    }
}

static void i2c_wait_idle(PIO pio, uint sm) {
    pio->fdebug = 1u << (PIO_FDEBUG_TXSTALL_LSB + sm);
    while (!(pio->fdebug & (1u << (PIO_FDEBUG_TXSTALL_LSB + sm))) &&
           !pio_i2c_check_error(pio, sm)) {
        tight_loop_contents();
    }
}

void MLX90640_I2CInit(void) {
    if (mlx_initialized) {
        return;
    }

    uint offset = pio_add_program(mlx_pio, &i2c_program);
    mlx_sm = pio_claim_unused_sm(mlx_pio, true);
    i2c_program_init(mlx_pio, mlx_sm, offset, PIN_MLX_SDA, PIN_MLX_SCL);

    // Default to 400 kHz I2C clock
    MLX90640_I2CFreqSet(400);
    mlx_initialized = true;
}

void MLX90640_I2CFreqSet(int freq) {
    if (freq <= 0) {
        return;
    }

    // Accept either kHz (e.g., 400) or Hz (e.g., 400000)
    float freq_hz = (freq <= 2000) ? ((float)freq * 1000.0f) : (float)freq;
    float div = (float)clock_get_hz(clk_sys) / (32.0f * freq_hz);
    if (div < 1.0f) {
        div = 1.0f;
    }
    pio_sm_set_clkdiv(mlx_pio, mlx_sm, div);
}

int MLX90640_I2CGeneralReset(void) {
    uint8_t cmd = 0x06;
    int err = pio_i2c_write_blocking(mlx_pio, mlx_sm, 0x00, &cmd, 1);
    return (err == 0) ? MLX90640_NO_ERROR : -MLX90640_I2C_NACK_ERROR;
}

int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data) {
    uint8_t buf[4];
    buf[0] = (uint8_t)(writeAddress >> 8);
    buf[1] = (uint8_t)(writeAddress & 0xFFu);
    buf[2] = (uint8_t)(data >> 8);
    buf[3] = (uint8_t)(data & 0xFFu);

    int err = pio_i2c_write_blocking(mlx_pio, mlx_sm, slaveAddr, buf, sizeof(buf));
    return (err == 0) ? MLX90640_NO_ERROR : -MLX90640_I2C_NACK_ERROR;
}

int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data) {
    if (nMemAddressRead == 0) {
        return MLX90640_NO_ERROR;
    }
    if (!data) {
        return -MLX90640_I2C_NACK_ERROR;
    }

    // Phase 1: Send 16-bit big-endian register address (no STOP)
    i2c_rx_enable(mlx_pio, mlx_sm, false);
    pio_i2c_start(mlx_pio, mlx_sm);
    pio_i2c_put_or_err(mlx_pio, mlx_sm, ((uint16_t)slaveAddr << 2) | (1u << MLX_I2C_NAK_LSB));
    pio_i2c_put_or_err(mlx_pio, mlx_sm, (((uint16_t)(startAddress >> 8)) << MLX_I2C_DATA_LSB) | (1u << MLX_I2C_NAK_LSB));
    pio_i2c_put_or_err(mlx_pio, mlx_sm, (((uint16_t)(startAddress & 0xFFu)) << MLX_I2C_DATA_LSB) | (1u << MLX_I2C_NAK_LSB));
    i2c_wait_idle(mlx_pio, mlx_sm);

    if (pio_i2c_check_error(mlx_pio, mlx_sm)) {
        pio_i2c_resume_after_error(mlx_pio, mlx_sm);
        pio_i2c_stop(mlx_pio, mlx_sm);
        i2c_wait_idle(mlx_pio, mlx_sm);
        return -MLX90640_I2C_NACK_ERROR;
    }

    // Flush RX FIFO and reset ISR shift counter before enabling autopush
    // (prevents 1-bit shift corruption after writes in pico-examples i2c.pio)
    while (!pio_sm_is_rx_fifo_empty(mlx_pio, mlx_sm)) {
        (void)pio_i2c_get(mlx_pio, mlx_sm);
    }
    pio_sm_exec_wait_blocking(mlx_pio, mlx_sm, pio_encode_mov(pio_isr, pio_null));

    // Phase 2: Repeated START + read (2 * nMemAddressRead) bytes
    i2c_rx_enable(mlx_pio, mlx_sm, true);
    pio_i2c_repstart(mlx_pio, mlx_sm);
    pio_i2c_put_or_err(mlx_pio, mlx_sm, ((uint16_t)slaveAddr << 2) | (1u << MLX_I2C_DATA_LSB) | (1u << MLX_I2C_NAK_LSB));

    uint32_t tx_remain = (uint32_t)nMemAddressRead * 2u;
    uint32_t rx_remain = tx_remain;
    bool discard_addr = true;
    uint32_t rx_idx = 0;
    uint8_t msb = 0;

    while ((tx_remain || rx_remain) && !pio_i2c_check_error(mlx_pio, mlx_sm)) {
        if (tx_remain && !pio_sm_is_tx_fifo_full(mlx_pio, mlx_sm)) {
            --tx_remain;
            uint16_t cmd = (0xFFu << MLX_I2C_DATA_LSB) |
                           (tx_remain ? 0u : ((1u << MLX_I2C_FINAL_LSB) | (1u << MLX_I2C_NAK_LSB)));
            pio_i2c_put_or_err(mlx_pio, mlx_sm, cmd);
        }

        if (!pio_sm_is_rx_fifo_empty(mlx_pio, mlx_sm)) {
            uint8_t byte = pio_i2c_get(mlx_pio, mlx_sm);
            if (discard_addr) {
                discard_addr = false; // Ignore echoed (slaveAddr << 1 | 1) byte
            } else if (rx_remain > 0) {
                --rx_remain;
                if ((rx_idx & 1u) == 0u) {
                    msb = byte;
                } else {
                    data[rx_idx >> 1] = ((uint16_t)msb << 8) | byte;
                }
                rx_idx++;
            }
        }
    }

    pio_i2c_stop(mlx_pio, mlx_sm);
    i2c_wait_idle(mlx_pio, mlx_sm);

    if (pio_i2c_check_error(mlx_pio, mlx_sm)) {
        pio_i2c_resume_after_error(mlx_pio, mlx_sm);
        pio_i2c_stop(mlx_pio, mlx_sm);
        i2c_wait_idle(mlx_pio, mlx_sm);
        return -MLX90640_I2C_NACK_ERROR;
    }

    return MLX90640_NO_ERROR;
}