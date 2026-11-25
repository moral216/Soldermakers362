#include "MLX90640_I2C_Driver.h"
#include "pin_config.h"
#include "pio_i2c.h"
#include "i2c.pio.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/timer.h"
#include <stdbool.h>

#define MLX_I2C_FINAL_LSB     9u
#define MLX_I2C_DATA_LSB      1u
#define MLX_I2C_NAK_LSB       0u
#define MLX_DMA_TIMEOUT_US    500000u // 500 ms safety timeout

// 16-bit PIO command for "read 1 byte + send ACK" placed in bits [31:16]
// (i2c.pio shifts left with a 16-bit autopull threshold)
static const uint32_t tx_cmd_read_ack = ((uint32_t)(0xFFu << MLX_I2C_DATA_LSB)) << 16;

static PIO mlx_pio = pio0;
static uint mlx_sm = 0;
static int dma_tx_chan = -1;
static int dma_rx_chan = -1;
static dma_channel_config dma_tx_cfg;
static dma_channel_config dma_rx_cfg;
static bool mlx_initialized = false;

static void i2c_rx_enable(PIO pio, uint sm, bool en) {
    if (en) {
        hw_set_bits(&pio->sm[sm].shiftctrl, PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS);
    } else {
        hw_clear_bits(&pio->sm[sm].shiftctrl, PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS);
    }
}

static void i2c_set_push_thresh(PIO pio, uint sm, uint thresh) {
    hw_write_masked(
        &pio->sm[sm].shiftctrl,
        (thresh << PIO_SM0_SHIFTCTRL_PUSH_THRESH_LSB) & PIO_SM0_SHIFTCTRL_PUSH_THRESH_BITS,
        PIO_SM0_SHIFTCTRL_PUSH_THRESH_BITS
    );
}

static void i2c_wait_idle(PIO pio, uint sm) {
    pio->fdebug = 1u << (PIO_FDEBUG_TXSTALL_LSB + sm);
    while (!(pio->fdebug & (1u << (PIO_FDEBUG_TXSTALL_LSB + sm))) &&
           !pio_i2c_check_error(pio, sm)) {
        tight_loop_contents();
    }
}

static bool dma_wait_with_timeout(int chan, uint32_t timeout_us) {
    uint32_t start = time_us_32();
    while (dma_channel_is_busy(chan)) {
        if (pio_i2c_check_error(mlx_pio, mlx_sm) || (time_us_32() - start) > timeout_us) {
            return false;
        }
        tight_loop_contents();
    }
    return true;
}

static void i2c_recover_bus(void) {
    i2c_rx_enable(mlx_pio, mlx_sm, false);
    i2c_set_push_thresh(mlx_pio, mlx_sm, 8u);
    pio_i2c_resume_after_error(mlx_pio, mlx_sm);
    pio_i2c_stop(mlx_pio, mlx_sm);
    i2c_wait_idle(mlx_pio, mlx_sm);
}

void MLX90640_I2CInit(void) {
    if (mlx_initialized) {
        return;
    }

    uint offset = pio_add_program(mlx_pio, &i2c_program);
    mlx_sm = pio_claim_unused_sm(mlx_pio, true);
    i2c_program_init(mlx_pio, mlx_sm, offset, PIN_MLX_SDA, PIN_MLX_SCL);

    // Claim two DMA channels (safe alongside any Core 0 DMA peripherals)
    dma_tx_chan = dma_claim_unused_channel(true);
    dma_rx_chan = dma_claim_unused_channel(true);

    // TX DMA: repeatedly feeds tx_cmd_read_ack into PIO TX FIFO
    dma_tx_cfg = dma_channel_get_default_config(dma_tx_chan);
    channel_config_set_transfer_data_size(&dma_tx_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_tx_cfg, false);
    channel_config_set_write_increment(&dma_tx_cfg, false);
    channel_config_set_dreq(&dma_tx_cfg, pio_get_dreq(mlx_pio, mlx_sm, true));

    // RX DMA: drains 16-bit big-endian-packed words from PIO RX FIFO into caller buffer
    dma_rx_cfg = dma_channel_get_default_config(dma_rx_chan);
    channel_config_set_transfer_data_size(&dma_rx_cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&dma_rx_cfg, false);
    channel_config_set_write_increment(&dma_rx_cfg, true);
    channel_config_set_dreq(&dma_rx_cfg, pio_get_dreq(mlx_pio, mlx_sm, false));

    MLX90640_I2CFreqSet(400);
    mlx_initialized = true;
}

void MLX90640_I2CFreqSet(int freq) {
    if (freq <= 0) {
        return;
    }

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
        i2c_recover_bus();
        return -MLX90640_I2C_NACK_ERROR;
    }

    // Phase 2a: Repeated START + read address with autopush still disabled
    // so the echoed address byte never enters the RX DMA stream
    pio_i2c_repstart(mlx_pio, mlx_sm);
    pio_i2c_put_or_err(mlx_pio, mlx_sm, ((uint16_t)slaveAddr << 2) | (1u << MLX_I2C_DATA_LSB) | (1u << MLX_I2C_NAK_LSB));
    i2c_wait_idle(mlx_pio, mlx_sm);

    if (pio_i2c_check_error(mlx_pio, mlx_sm)) {
        i2c_recover_bus();
        return -MLX90640_I2C_NACK_ERROR;
    }

    // Drain any stale RX words and reset ISR shift counter before enabling 16-bit autopush
    while (!pio_sm_is_rx_fifo_empty(mlx_pio, mlx_sm)) {
        (void)pio_i2c_get(mlx_pio, mlx_sm);
    }
    pio_sm_exec_wait_blocking(mlx_pio, mlx_sm, pio_encode_mov(pio_isr, pio_null));
    i2c_set_push_thresh(mlx_pio, mlx_sm, 16u);
    i2c_rx_enable(mlx_pio, mlx_sm, true);

    // Phase 2b: Start RX DMA (nMemAddressRead 16-bit words) and TX DMA (2*n - 1 ACK bytes)
    uint32_t ack_bytes = ((uint32_t)nMemAddressRead * 2u) - 1u;

    dma_channel_configure(
        dma_rx_chan,
        &dma_rx_cfg,
        data,                     // write_addr
        &mlx_pio->rxf[mlx_sm],    // read_addr
        nMemAddressRead,          // transfer_count (16-bit words)
        true                      // start immediately
    );

    dma_channel_configure(
        dma_tx_chan,
        &dma_tx_cfg,
        &mlx_pio->txf[mlx_sm],    // write_addr
        &tx_cmd_read_ack,         // read_addr
        ack_bytes,                // transfer_count (bytes with ACK)
        true                      // start immediately
    );

    bool ok = dma_wait_with_timeout(dma_tx_chan, MLX_DMA_TIMEOUT_US);
    if (ok) {
        // Queue the final byte (Final=1, NAK=1) + STOP while last TX FIFO words clock out
        uint16_t final_cmd = (1u << MLX_I2C_FINAL_LSB) | (0xFFu << MLX_I2C_DATA_LSB) | (1u << MLX_I2C_NAK_LSB);
        pio_i2c_put_or_err(mlx_pio, mlx_sm, final_cmd);
        pio_i2c_stop(mlx_pio, mlx_sm);
        ok = dma_wait_with_timeout(dma_rx_chan, MLX_DMA_TIMEOUT_US);
    }

    if (!ok) {
        dma_channel_abort(dma_tx_chan);
        dma_channel_abort(dma_rx_chan);
        i2c_recover_bus();
        return -MLX90640_I2C_NACK_ERROR;
    }

    i2c_wait_idle(mlx_pio, mlx_sm);
    i2c_rx_enable(mlx_pio, mlx_sm, false);
    i2c_set_push_thresh(mlx_pio, mlx_sm, 8u);

    if (pio_i2c_check_error(mlx_pio, mlx_sm)) {
        i2c_recover_bus();
        return -MLX90640_I2C_NACK_ERROR;
    }

    return MLX90640_NO_ERROR;
}