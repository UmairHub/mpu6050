#include "main.h"
#include <math.h>

/* ---------------------------------------------------------------
 * Nucleo-F446RE, bare-metal, HSI 16 MHz (APB1 = 16 MHz)
 * MPU6050 on I2C1 (PB6 SCL, PB7 SDA), debug on USART2 (PA2/PA3)
 * --------------------------------------------------------------- */

/* MPU6050 I2C addresses */
#define MPU6050_ADDR_WRITE  0xD0    // 0x68 << 1
#define MPU6050_ADDR_READ   0xD1    // (0x68 << 1) | 1

/* MPU6050 registers */
#define WHO_AM_I            0x75
#define PWR_MGMT_1          0x6B
#define ACCEL_XOUT_H        0x3B    // 0x3B..0x48: accel(6) temp(2) gyro(6)

/* Timing */
#define LOOP_PERIOD_MS      5       // 200 Hz sensor loop
#define PRINT_PERIOD_MS     100     // 10 Hz UART output
#define GYRO_CAL_SAMPLES    200
#define I2C_TIMEOUT_MS      3

#define GYRO_LSB_PER_DPS    131.0f  // +-250 dps range

/* ---------------- Prototypes ---------------- */

/* Time base */
static void systick_init(void);
static void delay_ms(uint32_t ms);

/* I2C1 low-level primitives (return 0 = OK, -1 = error/timeout) */
static void I2C1_Init(void);
static void I2C1_Recover(void);
static int  I2C1_WaitSR1(uint32_t flag);
static int  I2C1_Start(void);
static void I2C1_Stop(void);
static int  I2C1_WriteAddr(uint8_t addr);
static int  I2C1_WriteData(uint8_t data);

/* MPU6050 driver */
static int  MPU6050_Init(void);
static int  MPU6050_WriteReg(uint8_t reg, uint8_t value);
static int  MPU6050_ReadReg(uint8_t reg, uint8_t *value);
static int  MPU6050_ReadBurst(uint8_t reg, uint8_t *buf, uint8_t n);
static int  MPU6050_ReadAll(int16_t *ax, int16_t *ay, int16_t *az,
                            int16_t *gx, int16_t *gy, int16_t *gz);

/* UART2 output */
static void UART2_Init(void);
static void UART2_SendChar(char c);
static void UART2_SendString(const char *str);
static void UART2_SendInt(int32_t num);
static void UART2_SendHex8(uint8_t v);
static void UART2_SendFixed2(float v);

/* ---------------- Time base ---------------- */

volatile uint32_t ms_ticks = 0;

/* SysTick_Handler in stm32f4xx_it.c is commented out, so it lives here */
void SysTick_Handler(void)
{
    ms_ticks++;
}

static void systick_init(void)
{
    SysTick->LOAD = 16000 - 1;                  // 16 MHz / 16000 = 1 kHz
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk  // processor clock
                  | SysTick_CTRL_TICKINT_Msk    // interrupt every tick
                  | SysTick_CTRL_ENABLE_Msk;
}

static void delay_ms(uint32_t ms)
{
    uint32_t start = ms_ticks;
    while ((ms_ticks - start) < ms);
}

/* ---------------- main ---------------- */

int main(void)
{
    systick_init();         // first: I2C timeouts use ms_ticks
    UART2_Init();
    I2C1_Init();

    delay_ms(100);          // MPU6050 power-up time

    /* Diagnostics: bus lines and BUSY flag before the first transfer */
    UART2_SendString("SCL=");  UART2_SendInt((GPIOB->IDR >> 6) & 1);
    UART2_SendString(" SDA="); UART2_SendInt((GPIOB->IDR >> 7) & 1);
    UART2_SendString(" BUSY="); UART2_SendInt((I2C1->SR2 & I2C_SR2_BUSY) ? 1 : 0);
    UART2_SendString("\r\n");

    if (MPU6050_Init() != 0)
        UART2_SendString("MPU6050 init failed\r\n");

    uint8_t who = 0;
    if (MPU6050_ReadReg(WHO_AM_I, &who) == 0) {
        UART2_SendString("WHO_AM_I: 0x");
        UART2_SendHex8(who);                    // expect 0x68
        UART2_SendString("\r\n");
    } else {
        UART2_SendString("WHO_AM_I read failed\r\n");
    }

    int16_t ax, ay, az, gx, gy, gz;

    /* Gyro Z bias: keep the board still for ~1 s after reset */
    UART2_SendString("Calibrating gyro, keep still...\r\n");
    int32_t sum = 0;
    int     n   = 0;
    for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
        if (MPU6050_ReadAll(&ax, &ay, &az, &gx, &gy, &gz) == 0) {
            sum += gz;
            n++;
        }
        delay_ms(LOOP_PERIOD_MS);
    }
    float gz_off = (n > 0) ? (float)sum / n : 0.0f;
    UART2_SendString("gz_off: ");
    UART2_SendFixed2(gz_off);
    UART2_SendString("\r\n");

    float    heading    = 0.0f;
    uint32_t i2c_errors = 0;
    uint32_t last       = ms_ticks;
    uint32_t last_print = last;

    while (1)
    {
        /* Fixed-rate loop */
        while ((ms_ticks - last) < LOOP_PERIOD_MS);
        uint32_t now = ms_ticks;

        if (MPU6050_ReadAll(&ax, &ay, &az, &gx, &gy, &gz) != 0) {
            /* Keep 'last' unchanged: the next good sample integrates the gap */
            i2c_errors++;
            I2C1_Recover();
            continue;
        }

        float dt = (now - last) / 1000.0f;      // measured, not assumed
        last = now;

        heading += ((gz - gz_off) / GYRO_LSB_PER_DPS) * dt;

        if ((now - last_print) >= PRINT_PERIOD_MS)
        {
            last_print = now;
            UART2_SendString("ax: ");        UART2_SendInt(ax);
            UART2_SendString("  ay: ");      UART2_SendInt(ay);
            UART2_SendString("  az: ");      UART2_SendInt(az);
            UART2_SendString("  gz: ");      UART2_SendInt(gz);
            UART2_SendString("  heading: "); UART2_SendFixed2(heading);
            UART2_SendString(" deg");
            if (i2c_errors) {
                UART2_SendString("  i2c_err: ");
                UART2_SendInt((int32_t)i2c_errors);
            }
            UART2_SendString("\r\n");
        }
    }
}

/* ---------------- I2C1 low-level primitives ---------------- */

/* Short busy wait for bit-banging, roughly 10 us at 16 MHz (not critical) */
static void i2c_bb_delay(void)
{
    for (volatile int d = 0; d < 15; d++);
}

/* Bus recovery: if a reset hit mid-transfer, the MPU6050 may still hold
 * SDA low waiting for clocks. Bit-bang up to 9 SCL pulses until SDA is
 * released, then send a STOP. Pins are left as GPIO; I2C1_Init() switches
 * them back to AF4. */
static void I2C1_BusClear(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    GPIOB->OTYPER |=  (1U << 6) | (1U << 7);            // open-drain
    GPIOB->PUPDR  &= ~((3U << (6*2)) | (3U << (7*2)));
    GPIOB->PUPDR  |=  ((1U << (6*2)) | (1U << (7*2)));  // pull-up
    GPIOB->BSRR    =  (1U << 6) | (1U << 7);            // release both high
    GPIOB->MODER  &= ~((3U << (6*2)) | (3U << (7*2)));
    GPIOB->MODER  |=  ((1U << (6*2)) | (1U << (7*2)));  // GPIO output
    i2c_bb_delay();

    for (int i = 0; i < 9 && !(GPIOB->IDR & (1U << 7)); i++) {
        GPIOB->BSRR = (1U << (6 + 16));   // SCL low
        i2c_bb_delay();
        GPIOB->BSRR = (1U << 6);          // SCL high
        i2c_bb_delay();
    }

    /* STOP condition: SDA low -> high while SCL high */
    GPIOB->BSRR = (1U << (6 + 16));       // SCL low
    i2c_bb_delay();
    GPIOB->BSRR = (1U << (7 + 16));       // SDA low
    i2c_bb_delay();
    GPIOB->BSRR = (1U << 6);              // SCL high
    i2c_bb_delay();
    GPIOB->BSRR = (1U << 7);              // SDA high
    i2c_bb_delay();
}

static void I2C1_Init(void)
{
    I2C1_BusClear();

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    /* PB6 SCL, PB7 SDA: AF4, open-drain, high speed, pull-up */
    GPIOB->MODER   &= ~((3U << (6*2)) | (3U << (7*2)));
    GPIOB->MODER   |=  ((2U << (6*2)) | (2U << (7*2)));
    GPIOB->OTYPER  |=  (1U << 6) | (1U << 7);
    GPIOB->OSPEEDR |=  (3U << (6*2)) | (3U << (7*2));
    GPIOB->PUPDR   &= ~((3U << (6*2)) | (3U << (7*2)));
    GPIOB->PUPDR   |=  ((1U << (6*2)) | (1U << (7*2)));
    GPIOB->AFR[0]  &= ~((0xFU << (6*4)) | (0xFU << (7*4)));
    GPIOB->AFR[0]  |=  ((4U   << (6*4)) | (4U   << (7*4)));

    /* Software reset clears a stale BUSY flag (also covers F4 errata
     * where the analog filter can latch BUSY) */
    I2C1->CR1 = I2C_CR1_SWRST;
    I2C1->CR1 = 0;                      // out of reset, PE = 0

    I2C1->CR2   = 16;                   // APB1 = 16 MHz
    I2C1->CCR   = 80;                   // 100 kHz: 16 MHz / (2 * 100 kHz)
    I2C1->TRISE = 17;                   // 1000 ns / 62.5 ns + 1

    I2C1->CR1 |= I2C_CR1_PE;
}

/* After a timeout or NACK: bus clear + peripheral reset + MPU wake-up */
static void I2C1_Recover(void)
{
    I2C1_Init();                        // includes bus clear and SWRST
    MPU6050_Init();
}

/* Wait for an SR1 flag; fail on NACK (AF) or timeout */
static int I2C1_WaitSR1(uint32_t flag)
{
    uint32_t t0 = ms_ticks;
    while (!(I2C1->SR1 & flag)) {
        if (I2C1->SR1 & I2C_SR1_AF) {           // NACK from slave
            I2C1->SR1 &= ~I2C_SR1_AF;
            I2C1_Stop();
            return -1;
        }
        if ((ms_ticks - t0) > I2C_TIMEOUT_MS)
            return -1;
    }
    return 0;
}

static int I2C1_Start(void)
{
    /* Wait until a previous STOP has been generated */
    uint32_t t0 = ms_ticks;
    while (I2C1->CR1 & I2C_CR1_STOP) {
        if ((ms_ticks - t0) > I2C_TIMEOUT_MS)
            return -1;
    }

    I2C1->CR1 |= I2C_CR1_START;
    return I2C1_WaitSR1(I2C_SR1_SB);
}

static void I2C1_Stop(void)
{
    I2C1->CR1 |= I2C_CR1_STOP;
}

static int I2C1_WriteAddr(uint8_t addr)
{
    I2C1->DR = addr;
    if (I2C1_WaitSR1(I2C_SR1_ADDR) != 0)
        return -1;
    (void)I2C1->SR1;
    (void)I2C1->SR2;                    // clear ADDR: read SR1 then SR2
    return 0;
}

static int I2C1_WriteData(uint8_t data)
{
    if (I2C1_WaitSR1(I2C_SR1_TXE) != 0)
        return -1;
    I2C1->DR = data;
    return I2C1_WaitSR1(I2C_SR1_BTF);
}

/* ---------------- MPU6050 driver ---------------- */

static int MPU6050_Init(void)
{
    return MPU6050_WriteReg(PWR_MGMT_1, 0x00);  // wake up (clear SLEEP)
}

static int MPU6050_WriteReg(uint8_t reg, uint8_t value)
{
    if (I2C1_Start() != 0)                       goto err;
    if (I2C1_WriteAddr(MPU6050_ADDR_WRITE) != 0) goto err;
    if (I2C1_WriteData(reg) != 0)                goto err;
    if (I2C1_WriteData(value) != 0)              goto err;
    I2C1_Stop();
    return 0;
err:
    I2C1_Stop();
    return -1;
}

/* Single-byte read (RM0390 sequence for N = 1):
 * ACK must be cleared BEFORE ADDR is cleared, STOP set right after. */
static int MPU6050_ReadReg(uint8_t reg, uint8_t *value)
{
    if (I2C1_Start() != 0)                       goto err;
    if (I2C1_WriteAddr(MPU6050_ADDR_WRITE) != 0) goto err;
    if (I2C1_WriteData(reg) != 0)                goto err;

    if (I2C1_Start() != 0)                       goto err;   // repeated start
    I2C1->CR1 &= ~I2C_CR1_ACK;                               // NACK the only byte
    if (I2C1_WriteAddr(MPU6050_ADDR_READ) != 0)  goto err;   // clears ADDR
    I2C1_Stop();

    if (I2C1_WaitSR1(I2C_SR1_RXNE) != 0)         goto err;
    *value = (uint8_t)I2C1->DR;
    return 0;
err:
    I2C1_Stop();
    return -1;
}

/* Burst read, n >= 3 (RM0390 sequence for N > 2):
 * read with ACK until 3 bytes remain, then use BTF to place NACK and
 * STOP exactly on the last byte. The register pointer auto-increments. */
static int MPU6050_ReadBurst(uint8_t reg, uint8_t *buf, uint8_t n)
{
    uint8_t i = 0;

    if (n < 3) return -1;

    if (I2C1_Start() != 0)                       goto err;
    if (I2C1_WriteAddr(MPU6050_ADDR_WRITE) != 0) goto err;
    if (I2C1_WriteData(reg) != 0)                goto err;

    I2C1->CR1 |= I2C_CR1_ACK;                                // ACK data bytes
    if (I2C1_Start() != 0)                       goto err;   // repeated start
    if (I2C1_WriteAddr(MPU6050_ADDR_READ) != 0)  goto err;   // reception starts

    while ((n - i) > 3) {
        if (I2C1_WaitSR1(I2C_SR1_RXNE) != 0)     goto err;
        buf[i++] = (uint8_t)I2C1->DR;
    }

    /* 3 bytes left: N-2 in DR, N-1 in shift register when BTF sets */
    if (I2C1_WaitSR1(I2C_SR1_BTF) != 0)          goto err;
    I2C1->CR1 &= ~I2C_CR1_ACK;                               // NACK the last byte
    buf[i++] = (uint8_t)I2C1->DR;                            // N-2

    /* N-1 in DR, N in shift register */
    if (I2C1_WaitSR1(I2C_SR1_BTF) != 0)          goto err;
    I2C1_Stop();
    buf[i++] = (uint8_t)I2C1->DR;                            // N-1

    if (I2C1_WaitSR1(I2C_SR1_RXNE) != 0)         goto err;
    buf[i++] = (uint8_t)I2C1->DR;                            // N
    return 0;
err:
    I2C1->CR1 &= ~I2C_CR1_ACK;
    I2C1_Stop();
    return -1;
}

/* One 14-byte burst from 0x3B: accel, temp and gyro from the SAME sample */
static int MPU6050_ReadAll(int16_t *ax, int16_t *ay, int16_t *az,
                           int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t b[14];

    if (MPU6050_ReadBurst(ACCEL_XOUT_H, b, 14) != 0)
        return -1;

    *ax = (int16_t)((b[0]  << 8) | b[1]);
    *ay = (int16_t)((b[2]  << 8) | b[3]);
    *az = (int16_t)((b[4]  << 8) | b[5]);
    /* b[6..7] = TEMP_OUT: degC = raw / 340 + 36.53 */
    *gx = (int16_t)((b[8]  << 8) | b[9]);
    *gy = (int16_t)((b[10] << 8) | b[11]);
    *gz = (int16_t)((b[12] << 8) | b[13]);
    return 0;
}

/* ---------------- UART2 output ---------------- */

static void UART2_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /* PA2 TX, PA3 RX: AF7 */
    GPIOA->MODER  &= ~((3U << (2*2)) | (3U << (3*2)));
    GPIOA->MODER  |=  ((2U << (2*2)) | (2U << (3*2)));
    GPIOA->AFR[0] &= ~((0xFU << (2*4)) | (0xFU << (3*4)));
    GPIOA->AFR[0] |=  ((7U   << (2*4)) | (7U   << (3*4)));

    USART2->CR1 = 0;                    // disable while configuring

    /* 115200 baud @ 16 MHz, oversampling 16:
     * USARTDIV = 16e6 / (16 * 115200) = 8.68 -> mantissa 8, fraction 11/16
     * BRR = (8 << 4) | 11 = 0x8B  (actual 115108 baud, -0.08 %) */
    USART2->BRR = 0x8B;
    USART2->CR2 = 0;                    // 1 stop bit
    USART2->CR3 = 0;                    // no flow control
    USART2->CR1 = USART_CR1_TE          // transmitter on, 8N1 (M = 0)
                | USART_CR1_UE;         // USART on
}

static void UART2_SendChar(char c)
{
    while (!(USART2->SR & USART_SR_TXE));
    USART2->DR = (uint8_t)c;
}

static void UART2_SendString(const char *str)
{
    while (*str)
        UART2_SendChar(*str++);
}

static void UART2_SendInt(int32_t num)
{
    char     buf[12];
    int      i = 0;
    uint32_t u;

    if (num < 0) {
        UART2_SendChar('-');
        u = (uint32_t)(-(num + 1)) + 1; // safe for INT32_MIN
    } else {
        u = (uint32_t)num;
    }

    if (u == 0) buf[i++] = '0';
    while (u > 0) {
        buf[i++] = (char)('0' + (u % 10));
        u /= 10;
    }
    while (i > 0)
        UART2_SendChar(buf[--i]);
}

static void UART2_SendHex8(uint8_t v)
{
    const char hex[] = "0123456789ABCDEF";
    UART2_SendChar(hex[v >> 4]);
    UART2_SendChar(hex[v & 0x0F]);
}

/* Prints a float with 2 decimals, correct sign and leading zero
 * (e.g. -0.50, 1.07) */
static void UART2_SendFixed2(float v)
{
    int32_t x = (int32_t)(v * 100.0f + (v >= 0.0f ? 0.5f : -0.5f));
    if (x < 0) {
        UART2_SendChar('-');
        x = -x;
    }
    UART2_SendInt(x / 100);
    UART2_SendChar('.');
    int32_t f = x % 100;
    if (f < 10) UART2_SendChar('0');
    UART2_SendInt(f);
}
