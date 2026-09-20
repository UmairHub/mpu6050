
#include "main.h"

#define MPU6050_ADDR       0xD0   // 0x68 << 1 (write address)
#define MPU6050_ADDR_READ  0xD1   // 0x68 << 1 | 1 (read address)
#define GYRO_XOUT_H  0x43
#define PWR_MGMT_1      0x6B
#define ACCEL_XOUT_H    0x3B
#define WHO_AM_I        0x75

#define MPU6050_ADDR       0xD0   // 0x68 << 1 (write address)
#define MPU6050_ADDR_READ  0xD1   // 0x68 << 1 | 1 (read address)

#define PWR_MGMT_1      0x6B
#define ACCEL_XOUT_H    0x3B
#define WHO_AM_I        0x75



void I2C1_Init(void);
void I2C1_Start(void);
void I2C1_Stop(void);
void I2C1_WriteAddr(uint8_t addr);
uint8_t I2C1_ReadDataAck(void);

void MPU6050_Init(void);
void MPU6050_WriteReg(uint8_t reg, uint8_t value);
uint8_t MPU6050_ReadReg(uint8_t reg);
uint8_t I2C1_ReadDataNack(void);
void MPU6050_ReadAccel(int16_t *ax, int16_t *ay, int16_t *az);
void MPU6050_ReadGyro(int16_t *gx, int16_t *gy, int16_t *gz);
void UART2_Init(void);
void UART2_SendChar(char c);
void UART2_SendString(char *str);
void UART2_SendInt(int32_t num);


int main(void)
{
  I2C1_Init();
  MPU6050_Init();
  UART2_Init();
  int16_t ax, ay, az;
  uint8_t who_am_i = MPU6050_ReadReg(WHO_AM_I);   // should return 0x68

  float heading = 0.0f;
  float dt = 0.05f;    // matches the ~50ms loop delay below

  int16_t gx, gy, gz;
  while (1)
  {
      MPU6050_ReadAccel(&ax, &ay, &az);
      MPU6050_ReadGyro(&gx, &gy, &gz);

      float gyro_z_dps = gz / 131.0f;
      heading += gyro_z_dps * dt;
      int32_t heading_whole = (int32_t)heading;
      float heading_frac_f = heading - heading_whole;
      if (heading_frac_f < 0) heading_frac_f = -heading_frac_f;
      int32_t heading_frac = (int32_t)(heading_frac_f * 100);  // 2 decimal places

      UART2_SendString("ax: ");
      UART2_SendInt(ax);
      UART2_SendString("  ay: ");
      UART2_SendInt(ay);
      UART2_SendString("  az: ");
      UART2_SendInt(az);
      UART2_SendString("  gz: ");
      UART2_SendInt(gz);
      UART2_SendString(" heading: ");
      UART2_SendInt(heading_whole);
      UART2_SendString(".");
      UART2_SendInt(heading_frac);
      UART2_SendString(" deg\r\n");

      for (volatile int d = 0; d < 800000; d++);
  }


}




void I2C1_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // PB6 -> SCL, PB7 -> SDA, Alternate Function, Open-Drain
    GPIOB->MODER &= ~(3 << (6*2)); GPIOB->MODER |= (2 << (6*2));  // AF mode
    GPIOB->MODER &= ~(3 << (7*2)); GPIOB->MODER |= (2 << (7*2));  // AF mode

    GPIOB->OTYPER |= (1 << 6) | (1 << 7);       // Open-drain
    GPIOB->OSPEEDR |= (3 << (6*2)) | (3 << (7*2)); // High speed
    GPIOB->PUPDR |= (1 << (6*2)) | (1 << (7*2));   // Pull-up (internal, backup if module lacks its own)

    GPIOB->AFR[0] &= ~(0xF << (6*4));
    GPIOB->AFR[0] |=  (4   << (6*4));  // AF4 = I2C1 on PB6
    GPIOB->AFR[0] &= ~(0xF << (7*4));
    GPIOB->AFR[0] |=  (4   << (7*4));  // AF4 = I2C1 on PB7

    I2C1->CR1 &= ~I2C_CR1_PE;          // disable I2C before config

    // Assumes APB1 clock = 45MHz (adjust FREQ field if yours differs)
    I2C1->CR2 = 16;                 // peripheral clock in MHz
    I2C1->CCR = 80;                    // 100kHz standard mode: CCR = APB1freq/(2*100000)
    I2C1->TRISE = 17;                   // (APB1freq_MHz + 1)

    I2C1->CR1 |= I2C_CR1_PE;            // enable I2C
}


void I2C1_Start(void)
{
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));
}

void I2C1_Stop(void)
{
    I2C1->CR1 |= I2C_CR1_STOP;
}

void I2C1_WriteAddr(uint8_t addr)
{
    I2C1->DR = addr;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR1;
    (void)I2C1->SR2;   // clear ADDR flag by reading SR1 then SR2
}

void I2C1_WriteData(uint8_t data)
{
    while (!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = data;
    while (!(I2C1->SR1 & I2C_SR1_BTF));
}

uint8_t I2C1_ReadDataAck(void)
{
    I2C1->CR1 |= I2C_CR1_ACK;
    while (!(I2C1->SR1 & I2C_SR1_RXNE));
    return I2C1->DR;
}

uint8_t I2C1_ReadDataNack(void)
{
    I2C1->CR1 &= ~I2C_CR1_ACK;
    I2C1->CR1 |= I2C_CR1_STOP;
    while (!(I2C1->SR1 & I2C_SR1_RXNE));
    return I2C1->DR;
}
void MPU6050_WriteReg(uint8_t reg, uint8_t value)
{
    I2C1_Start();
    I2C1_WriteAddr(MPU6050_ADDR);
    I2C1_WriteData(reg);
    I2C1_WriteData(value);
    I2C1_Stop();
}

uint8_t MPU6050_ReadReg(uint8_t reg)
{
    uint8_t value;
    I2C1_Start();
    I2C1_WriteAddr(MPU6050_ADDR);
    I2C1_WriteData(reg);

    I2C1_Start();               // repeated start
    I2C1_WriteAddr(MPU6050_ADDR_READ);
    value = I2C1_ReadDataNack();
    I2C1_Stop();

    return value;
}

void MPU6050_Init(void)
{
    MPU6050_WriteReg(PWR_MGMT_1, 0x00);   // wake up (clears sleep bit)
}

void MPU6050_ReadAccel(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buf[6];

    I2C1_Start();
    I2C1_WriteAddr(MPU6050_ADDR);
    I2C1_WriteData(ACCEL_XOUT_H);

    I2C1_Start();               // repeated start
    I2C1_WriteAddr(MPU6050_ADDR_READ);

    for (int i = 0; i < 5; i++)
        buf[i] = I2C1_ReadDataAck();
    buf[5] = I2C1_ReadDataNack();   // NACK + STOP on last byte

    *ax = (int16_t)(buf[0] << 8 | buf[1]);
    *ay = (int16_t)(buf[2] << 8 | buf[3]);
    *az = (int16_t)(buf[4] << 8 | buf[5]);
}
void MPU6050_ReadGyro(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[6];

    I2C1_Start();
    I2C1_WriteAddr(MPU6050_ADDR);
    I2C1_WriteData(GYRO_XOUT_H);

    I2C1_Start();
    I2C1_WriteAddr(MPU6050_ADDR_READ);

    for (int i = 0; i < 5; i++)
        buf[i] = I2C1_ReadDataAck();
    buf[5] = I2C1_ReadDataNack();

    *gx = (int16_t)(buf[0] << 8 | buf[1]);
    *gy = (int16_t)(buf[2] << 8 | buf[3]);
    *gz = (int16_t)(buf[4] << 8 | buf[5]);
}
void UART2_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // PA2 = TX, PA3 = RX, Alternate Function
    GPIOA->MODER &= ~(3 << (2*2)); GPIOA->MODER |= (2 << (2*2));
    GPIOA->MODER &= ~(3 << (3*2)); GPIOA->MODER |= (2 << (3*2));

    GPIOA->AFR[0] &= ~(0xF << (2*4));
    GPIOA->AFR[0] |=  (7   << (2*4));   // AF7 = USART2 on PA2
    GPIOA->AFR[0] &= ~(0xF << (3*4));
    GPIOA->AFR[0] |=  (7   << (3*4));   // AF7 = USART2 on PA3

    USART2->CR1 = 0;                     // disable while configuring
    // Assumes APB1 clock = 16MHz
    // Baud = 115200 -> USARTDIV = APB1clk / (16 * baud)
    USART2->BRR = 0x8B;  // 16MHz APB1 clock at 9600 baud
    USART2->CR1 = 0x0008; //enable Tx, 8-bit data
    USART2->CR2 = 0x0;   // 1 stop bit
    USART2->CR3 = 0x0;   //no flow control

    USART2->CR1 |= USART_CR1_TE;         // enable transmitter
    USART2->CR1 |= USART_CR1_UE;         // enable USART
}

void UART2_SendChar(char c)
{
    while (!(USART2->SR & USART_SR_TXE));
    USART2->DR = c;
}

void UART2_SendString(char *str)
{
    while (*str)
        UART2_SendChar(*str++);
}
void UART2_SendInt(int32_t num)
{
    char buf[12];
    int i = 0;
    uint8_t neg = 0;

    if (num < 0) { neg = 1; num = -num; }
    if (num == 0) buf[i++] = '0';

    while (num > 0)
    {
        buf[i++] = '0' + (num % 10);
        num /= 10;
    }
    if (neg) buf[i++] = '-';

    while (i > 0)
        UART2_SendChar(buf[--i]);
}



