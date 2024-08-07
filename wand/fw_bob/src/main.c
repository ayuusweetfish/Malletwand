#include <stm32l0xx_hal.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define BMI_CS_PIN  GPIO_PIN_4
#define BMI_CS_PORT GPIOA

// #define RELEASE
#ifndef RELEASE
#define _release_inline
static uint8_t swv_buf[256];
static size_t swv_buf_ptr = 0;
__attribute__ ((noinline, used))
void swv_trap_line()
{
  *(volatile char *)swv_buf;
}
static inline void swv_putchar(uint8_t c)
{
  // ITM_SendChar(c);
  if (c == '\n') {
    swv_buf[swv_buf_ptr >= sizeof swv_buf ?
      (sizeof swv_buf - 1) : swv_buf_ptr] = '\0';
    swv_trap_line();
    swv_buf_ptr = 0;
  } else if (++swv_buf_ptr <= sizeof swv_buf) {
    swv_buf[swv_buf_ptr - 1] = c;
  }
}
#pragma GCC push_options
#pragma GCC optimize("Os")
static char sbuf[32];
static const int swv_i32toa(uint32_t value, bool sgn, int base, int width, bool zeropad) {
  char *s = &sbuf[32];
  bool neg = false;
  if (sgn && (int32_t)value < 0) {
    neg = true;
    value = (uint32_t)(-(int32_t)value);
  }
  do {
    *(--s) = "0123456789abcdef"[value % base];
    value /= base;
  } while (value != 0);
  if (width > 0 && zeropad) while (32 - (s - sbuf) + (neg ? 1 : 0) < width) *(--s) = '0';
  if (neg) *(--s) = '-';
  if (width > 0 && !zeropad) while (32 - (s - sbuf) < width) *(--s) = ' ';
  return s - sbuf;
}
static inline int swv_vsnprintf(char *restrict buf, size_t bufsz, const char *restrict fmt, va_list va)
{
  size_t bufptr = 0;
  char c;

#define _out(_c) do { \
  if (bufptr < bufsz) buf[bufptr] = (_c); \
  bufptr++; \
} while (0)

  while ((c = *(fmt++)) != '\0') {
    if (c != '%') {
      _out(c);
    } else {
      uint32_t width = 0;
      bool zeropad = false;
      char t = *(fmt++);
      while (t >= '0' && t <= '9') {
        if (t == '0' && width == 0) zeropad = true;
        else width = width * 10 + (t - '0');
        t = *(fmt++);
      }
      if (t == 'd' || t == 'u' || t == 'x') {
        uint32_t n = (uint32_t)va_arg(va, unsigned int);
        int offs = swv_i32toa(n, t == 'd', t == 'x' ? 16 : 10, width, zeropad);
        for (; offs < 32; offs++) _out(sbuf[offs]);
      } else if (t == 'c') {
        char c = (char)va_arg(va, int);
        _out(c);
      }
    }
  }

#undef _out
  return bufptr;
}
static void swv_printf(const char *restrict fmt, ...)
{
  char s[256];
  va_list args;
  va_start(args, fmt);
  int r = swv_vsnprintf(s, sizeof s, fmt, args);
  va_end(args);
  for (int i = 0; i < r && i < sizeof s - 1; i++) swv_putchar(s[i]);
  if (r >= sizeof s) {
    for (int i = 0; i < 3; i++) swv_putchar('.');
    swv_putchar('\n');
  }
}
// swv_printf("%d %3d %8d %08d %04x %3d %03d\n", 3, 4, -7, -7, 26, 12345, -12);
#pragma GCC pop_options
#else
#define _release_inline inline
#define swv_printf(...)
#endif

UART_HandleTypeDef uart2;
SPI_HandleTypeDef spi1;

static inline void spi1_transmit(const uint8_t *data, size_t size)
{
/*
  HAL_SPI_Transmit(&spi1, (uint8_t *)data, size, 1000);
  // Clear Rx FIFO
  while (SPI1->CR2 & SPI_FLAG_RXNE) (void)SPI1->DR;
*/
  __HAL_SPI_DISABLE(&spi1);
  SPI_1LINE_TX(&spi1);
  __HAL_SPI_ENABLE(&spi1);

  for (size_t i = 0; i < size; i++) {
    while (!(SPI1->SR & SPI_SR_TXE)) { }
    SPI1->DR = data[i];
  }

  while (!(SPI1->SR & SPI_SR_TXE)) { }
  while ((SPI1->SR & SPI_SR_BSY)) { }
  // Clear OVR flag
  (void)SPI1->DR;
  (void)SPI1->SR;

  __HAL_SPI_DISABLE(&spi1);
}
static inline void spi1_receive(uint8_t *data, size_t size)
{
/*
  for (int i = 0; i < size; i++) data[i] = 0xAA;
  HAL_SPI_Receive(&spi1, data, size, 1000);
*/
  __HAL_SPI_DISABLE(&spi1);
  SPI_1LINE_RX(&spi1);
  __HAL_SPI_ENABLE(&spi1);

  for (size_t i = 0; i < size; i++) {
    while (!(SPI1->SR & SPI_SR_TXE)) { }
    SPI1->DR = 0;
    while (!(SPI1->SR & SPI_SR_RXNE)) { }
    data[i] = SPI1->DR;
  }

  while (!(SPI1->SR & SPI_SR_TXE)) { }
  while ((SPI1->SR & SPI_SR_BSY)) { }
  // Clear OVR flag
  (void)SPI1->DR;
  (void)SPI1->SR;

  __HAL_SPI_DISABLE(&spi1);
}
static _release_inline uint8_t bmi270_read_reg(uint8_t reg)
{
  uint8_t data[2] = {0x80 | reg};
  HAL_GPIO_WritePin(BMI_CS_PORT, BMI_CS_PIN, 0);
  spi1_transmit(data, 1);
  spi1_receive(data, 2);
  HAL_GPIO_WritePin(BMI_CS_PORT, BMI_CS_PIN, 1);
  // swv_printf("read reg %02x: %02x %02x. error code %u\n", reg, data[0], data[1], spi1.ErrorCode);
  return data[1];
}
static _release_inline void bmi270_write_reg(uint8_t reg, uint8_t value)
{
  uint8_t data[2] = {reg, value};
  HAL_GPIO_WritePin(BMI_CS_PORT, BMI_CS_PIN, 0);
  spi1_transmit(data, 2);
  HAL_GPIO_WritePin(BMI_CS_PORT, BMI_CS_PIN, 1);
  // 2 µs delay
}
static _release_inline void bmi270_read_burst(uint8_t reg, uint8_t *data, uint32_t len)
{
  HAL_GPIO_WritePin(BMI_CS_PORT, BMI_CS_PIN, 0);
  uint8_t data_byte = {0x80 | reg};
  spi1_transmit(&data_byte, 1);

  static uint8_t buf[32];
  spi1_receive(buf, len + 1);
  for (int i = 0; i < len; i++) data[i] = buf[i + 1];
  HAL_GPIO_WritePin(BMI_CS_PORT, BMI_CS_PIN, 1);
}
static _release_inline void bmi270_write_burst(const uint8_t *data, uint32_t len)
{
  HAL_GPIO_WritePin(BMI_CS_PORT, BMI_CS_PIN, 0);
  spi1_transmit(data, len);
  HAL_GPIO_WritePin(BMI_CS_PORT, BMI_CS_PIN, 1);
}
static const uint8_t bmi270_config_file[] = {
  0x5E,
  // bmi270_maximum_fifo_config_file
  0xc8, 0x2e, 0x00, 0x2e, 0x80, 0x2e, 0x1a, 0x00, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00,
  0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0x90, 0x32, 0x21, 0x2e, 0x59, 0xf5,
  0x10, 0x30, 0x21, 0x2e, 0x6a, 0xf5, 0x1a, 0x24, 0x22, 0x00, 0x80, 0x2e, 0x3b, 0x00, 0xc8, 0x2e, 0x44, 0x47, 0x22,
  0x00, 0x37, 0x00, 0xa4, 0x00, 0xff, 0x0f, 0xd1, 0x00, 0x07, 0xad, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
  0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00,
  0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x11, 0x24, 0xfc, 0xf5, 0x80, 0x30, 0x40, 0x42, 0x50, 0x50, 0x00, 0x30, 0x12, 0x24, 0xeb,
  0x00, 0x03, 0x30, 0x00, 0x2e, 0xc1, 0x86, 0x5a, 0x0e, 0xfb, 0x2f, 0x21, 0x2e, 0xfc, 0xf5, 0x13, 0x24, 0x63, 0xf5,
  0xe0, 0x3c, 0x48, 0x00, 0x22, 0x30, 0xf7, 0x80, 0xc2, 0x42, 0xe1, 0x7f, 0x3a, 0x25, 0xfc, 0x86, 0xf0, 0x7f, 0x41,
  0x33, 0x98, 0x2e, 0xc2, 0xc4, 0xd6, 0x6f, 0xf1, 0x30, 0xf1, 0x08, 0xc4, 0x6f, 0x11, 0x24, 0xff, 0x03, 0x12, 0x24,
  0x00, 0xfc, 0x61, 0x09, 0xa2, 0x08, 0x36, 0xbe, 0x2a, 0xb9, 0x13, 0x24, 0x38, 0x00, 0x64, 0xbb, 0xd1, 0xbe, 0x94,
  0x0a, 0x71, 0x08, 0xd5, 0x42, 0x21, 0xbd, 0x91, 0xbc, 0xd2, 0x42, 0xc1, 0x42, 0x00, 0xb2, 0xfe, 0x82, 0x05, 0x2f,
  0x50, 0x30, 0x21, 0x2e, 0x21, 0xf2, 0x00, 0x2e, 0x00, 0x2e, 0xd0, 0x2e, 0xf0, 0x6f, 0x02, 0x30, 0x02, 0x42, 0x20,
  0x26, 0xe0, 0x6f, 0x02, 0x31, 0x03, 0x40, 0x9a, 0x0a, 0x02, 0x42, 0xf0, 0x37, 0x05, 0x2e, 0x5e, 0xf7, 0x10, 0x08,
  0x12, 0x24, 0x1e, 0xf2, 0x80, 0x42, 0x83, 0x84, 0xf1, 0x7f, 0x0a, 0x25, 0x13, 0x30, 0x83, 0x42, 0x3b, 0x82, 0xf0,
  0x6f, 0x00, 0x2e, 0x00, 0x2e, 0xd0, 0x2e, 0x12, 0x40, 0x52, 0x42, 0x00, 0x2e, 0x12, 0x40, 0x52, 0x42, 0x3e, 0x84,
  0x00, 0x40, 0x40, 0x42, 0x7e, 0x82, 0xe1, 0x7f, 0xf2, 0x7f, 0x98, 0x2e, 0x6a, 0xd6, 0x21, 0x30, 0x23, 0x2e, 0x61,
  0xf5, 0xeb, 0x2c, 0xe1, 0x6f
};

static inline int16_t satneg16(int16_t x)
{
  if (x == INT16_MIN) return INT16_MAX;
  return -x;
}

static uint8_t uart2_rx_buf[6] = { 0 };
static bool uart2_rx_flag = false;

int main()
{
  HAL_Init();

  // ======== GPIO ========
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitTypeDef gpio_init;

  // SWD (SWDIO PA13, SWCLK PA14)
  gpio_init.Pin = GPIO_PIN_13 | GPIO_PIN_14;
  gpio_init.Mode = GPIO_MODE_AF_PP; // Pass over control to AF peripheral
  gpio_init.Alternate = GPIO_AF0_SWDIO;
  gpio_init.Pull = GPIO_PULLUP;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio_init);

if (0) {
  // ======== Clocks ========
  RCC_OscInitTypeDef osc_init = { 0 };
  osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  osc_init.HSIState = RCC_HSI_ON;
  osc_init.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  osc_init.PLL.PLLState = RCC_PLL_ON;
  osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  osc_init.PLL.PLLMUL = RCC_PLL_MUL4;
  osc_init.PLL.PLLDIV = RCC_PLL_DIV2;
  HAL_RCC_OscConfig(&osc_init);

  RCC_ClkInitTypeDef clk_init = { 0 };
  clk_init.ClockType =
    RCC_CLOCKTYPE_SYSCLK |
    RCC_CLOCKTYPE_HCLK |
    RCC_CLOCKTYPE_PCLK1;
  clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk_init.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_0);
}

  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  // ======== USART ========
  // USART2_TX (PA9 AF4)
  gpio_init = (GPIO_InitTypeDef){
    .Pin = GPIO_PIN_9,
    .Mode = GPIO_MODE_AF_OD,
    .Alternate = GPIO_AF4_USART2,
    .Speed = GPIO_SPEED_FREQ_HIGH,
  };
  HAL_GPIO_Init(GPIOA, &gpio_init);

  __HAL_RCC_USART2_CLK_ENABLE();
  uart2 = (UART_HandleTypeDef){
    .Instance = USART2,
    .Init = (UART_InitTypeDef){
      .BaudRate = 115200,
      .WordLength = UART_WORDLENGTH_8B,
      .StopBits = UART_STOPBITS_1,
      .Parity = UART_PARITY_NONE,
      .Mode = UART_MODE_TX_RX,
      .HwFlowCtl = UART_HWCONTROL_NONE,
      .OverSampling = UART_OVERSAMPLING_16,
      .OneBitSampling = UART_ONE_BIT_SAMPLE_ENABLE,
    },
  };
  HAL_HalfDuplex_Init(&uart2);

  HAL_NVIC_EnableIRQ(USART2_IRQn);
  HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);

  // ======== SPI ========
  // SPI1
  // SPI1_SCK (PA5 AF0), SPI1_MOSI (PA7 AF0)
  gpio_init = (GPIO_InitTypeDef){
    .Pin = GPIO_PIN_5 | GPIO_PIN_7,
    .Mode = GPIO_MODE_AF_PP,
    .Alternate = GPIO_AF0_SPI1,
    .Pull = GPIO_NOPULL,
    .Speed = GPIO_SPEED_FREQ_HIGH,
  };
  HAL_GPIO_Init(GPIOA, &gpio_init);
  // CS (PA4 GPIO)
  gpio_init = (GPIO_InitTypeDef){
    .Pin = BMI_CS_PIN,
    .Mode = GPIO_MODE_OUTPUT_PP,
    .Pull = GPIO_NOPULL,
    .Speed = GPIO_SPEED_FREQ_HIGH,
  };
  HAL_GPIO_Init(BMI_CS_PORT, &gpio_init);

  __HAL_RCC_SPI1_CLK_ENABLE();
  spi1 = (SPI_HandleTypeDef){
    .Instance = SPI1,
    .Init = (SPI_InitTypeDef){
      .Mode = SPI_MODE_MASTER,
      .Direction = SPI_DIRECTION_1LINE,
      .CLKPolarity = SPI_POLARITY_LOW,  // CPOL = 0
      .CLKPhase = SPI_PHASE_1EDGE,      // CPHA = 0
      .NSS = SPI_NSS_SOFT,
      .FirstBit = SPI_FIRSTBIT_MSB,
      .TIMode = SPI_TIMODE_DISABLE,
      .CRCCalculation = SPI_CRCCALCULATION_DISABLE,
      .DataSize = SPI_DATASIZE_8BIT,
      .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8, // APB / 8 = 2 MHz
    },
  };
  HAL_SPI_Init(&spi1);
  __HAL_SPI_ENABLE(&spi1);

  // BMI270 set-up

  HAL_GPIO_WritePin(BMI_CS_PORT, BMI_CS_PIN, 0); HAL_Delay(1);
  HAL_GPIO_WritePin(BMI_CS_PORT, BMI_CS_PIN, 1); HAL_Delay(1);
  bmi270_read_reg(0x00);
  bmi270_write_reg(0x7E, 0xB6); // Soft reset
  HAL_Delay(1);

  HAL_GPIO_WritePin(BMI_CS_PORT, BMI_CS_PIN, 0); HAL_Delay(1);
  HAL_GPIO_WritePin(BMI_CS_PORT, BMI_CS_PIN, 1); HAL_Delay(1);
  bmi270_read_reg(0x00);
  bmi270_write_reg(0x6B, 0x01); // IF_CONF.spi3 = 1
  HAL_Delay(1);
  uint8_t chip_id = bmi270_read_reg(0x00);
  swv_printf("BMI270 chip ID = 0x%02x\n", (int)chip_id);  // Should read 0x24

  // while (1) { }

  bmi270_write_reg(0x7C, 0x00);
  HAL_Delay(1);
  bmi270_write_reg(0x59, 0x00);
  bmi270_write_burst(bmi270_config_file, sizeof bmi270_config_file);
  bmi270_write_reg(0x59, 0x01);
  HAL_Delay(150);

  // INTERNAL_STATUS
  uint8_t init_status = bmi270_read_reg(0x21) & 0xF;
  if (init_status == 0x1) {
    swv_printf("BMI270 init ok\n");
  } else {
    swv_printf("BMI270 init status 0x%02x\n", (int)init_status);
  }

  // Normal mode
  bmi270_write_reg(0x7D, 0b0110); // PWR_CTRL.gyr_en = 1, .acc_en = 1
  bmi270_write_reg(0x40, 0xA8);   // ACC_CONF
  bmi270_write_reg(0x41, 0x01);   // ACC_RANGE: +/-4g
  bmi270_write_reg(0x42, 0xA9);   // GYR_CONF
  bmi270_write_reg(0x43, 0x00);   // GYR_RANGE: +/-2000dps
  HAL_Delay(10);

  bmi270_write_reg(0x6B, 0x21);   // IF_CONF.aux_en = 1
  bmi270_write_reg(0x7D, 0b0110); // PWR_CTRL.aux_en = 0
  bmi270_write_reg(0x4B, 0x30 << 1);  // AUX_DEV_ID
  bmi270_write_reg(0x4C, 0b10001111);
    // AUX_IF_CONF
    //   .aux_rd_burst = 0x3 (length 8)
    //   .man_rd_burst = 0x3 (length 8)
    //   .aux_manual_en = 1
  // Write to ODR (0x1A), value 100
  bmi270_write_reg(0x4F, 100);
  bmi270_write_reg(0x4E, 0x1A);
  HAL_Delay(1);
  // Write to Internal Control 0 (0x1B), value 0xA1 (Cmm_freq_en, Auto_SR_en, Take_meas_M)
  bmi270_write_reg(0x4F, 0xA1);
  bmi270_write_reg(0x4E, 0x1B);
  HAL_Delay(1);
  // Write to Internal Control 2 (0x1D), value 0x10 (Cmm_en)
  bmi270_write_reg(0x4F, 0x10);
  bmi270_write_reg(0x4E, 0x1D);
  HAL_Delay(1);

  bmi270_write_reg(0x68, 0x02);   // AUX_IF_TRIM (10 kΩ)
  bmi270_write_reg(0x7D, 0b0111); // PWR_CTRL.aux_en = 1
  bmi270_write_reg(0x4D, 0x00);   // AUX_RD_ADDR
  bmi270_write_reg(0x4C, 0b00001111); // AUX_IF_CONF.aux_manual_en = 0
  HAL_Delay(10);

  uart2_rx_flag = true;
  uint16_t rgb_state[3] = { 0 };

  while (1) {
    int16_t mag_out[3], acc_out[3], gyr_out[3];
    uint8_t data[24];
    bmi270_read_burst(0x04, data, 23);
    for (int i = 0; i < 23; i++) swv_printf("%02x%c", (int)data[i], i == 22 ? '\n' : ' ');
    // Assumes little endian
    // TODO: Adapt orientation to updated board
    mag_out[2] = satneg16(((int16_t)((uint16_t)data[0] << 8) | data[1]) + 0x8000);
    mag_out[0] =         (((int16_t)((uint16_t)data[2] << 8) | data[3]) + 0x8000);
    mag_out[1] = satneg16(((int16_t)((uint16_t)data[4] << 8) | data[5]) + 0x8000);
    acc_out[2] =          *( int16_t *)(data +  8);
    acc_out[0] = satneg16(*( int16_t *)(data + 10));
    acc_out[1] = satneg16(*( int16_t *)(data + 12));
    gyr_out[2] =          *( int16_t *)(data + 14);
    gyr_out[0] = satneg16(*( int16_t *)(data + 16));
    gyr_out[1] = satneg16(*( int16_t *)(data + 18));
    data[23] = 0;
    uint32_t time = *(uint32_t *)(data + 20);
    // Gyroscope calibration
    int8_t gyr_cas = ((int8_t)bmi270_read_reg(0x3C) << 1) >> 1;
    gyr_out[0] -= ((uint32_t)gyr_cas * gyr_out[2]) >> 9;
    // XXX: magnetic sensor reads noisy data?
    if (0) swv_printf("time = %6u | acc = %6d %6d %6d | gyr = %6d %6d %6d | mag = %6d %6d %6d\n",
      time,
      (int)acc_out[0], (int)acc_out[1], (int)acc_out[2],
      (int)gyr_out[0], (int)gyr_out[1], (int)gyr_out[2],
      (int)mag_out[0], (int)mag_out[1], (int)mag_out[2]);

    // Transmit through UART
    *(uint16_t *)(data +  0) = mag_out[0];
    *(uint16_t *)(data +  2) = mag_out[1];
    *(uint16_t *)(data +  4) = mag_out[2];
    *(uint16_t *)(data +  6) = acc_out[0];
    *(uint16_t *)(data +  8) = acc_out[1];
    *(uint16_t *)(data + 10) = acc_out[2];
    *(uint16_t *)(data + 12) = gyr_out[0];
    *(uint16_t *)(data + 14) = gyr_out[1];
    *(uint16_t *)(data + 16) = gyr_out[2];
    *(uint16_t *)(data + 18) = rgb_state[0];
    *(uint16_t *)(data + 20) = rgb_state[1];
    *(uint16_t *)(data + 22) = rgb_state[2];

    HAL_HalfDuplex_EnableTransmitter(&uart2);
    HAL_UART_Transmit(&uart2, data, 24, 1000);
    HAL_HalfDuplex_EnableReceiver(&uart2);

    if (uart2_rx_flag) {
      // swv_printf("rx! %02x %02x %02x %02x - ",
      //   uart2_rx_buf[0], uart2_rx_buf[1], uart2_rx_buf[2], uart2_rx_buf[3]);
      rgb_state[0] = *(uint16_t *)(uart2_rx_buf + 0);
      rgb_state[1] = *(uint16_t *)(uart2_rx_buf + 2);
      rgb_state[2] = *(uint16_t *)(uart2_rx_buf + 4);

      uart2_rx_flag = false;
      HAL_UART_Receive_IT(&uart2, uart2_rx_buf, 6);

      static int rx_count = 0;
      if (++rx_count == 10) swv_printf("ok!\n");
    }

    HAL_Delay(10);
  }

  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio_init.Pin = GPIO_PIN_0;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(GPIOA, &gpio_init);
  while (1) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0); HAL_Delay(500);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1); HAL_Delay(500);
  }
}

void SysTick_Handler()
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

void USART2_IRQHandler()
{
  // HAL_UART_IRQHandler(&uart2);
  uint32_t isr = USART2->ISR;
  uint32_t err = (isr &
    (USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE | USART_ISR_RTOF));
  if (err != 0) {
    swv_printf("error! %u\n", uart2.ErrorCode);
  } else {
    if (isr & USART_ISR_RXNE) {
      uart2.RxISR(&uart2);  // UART_RxISR_8BIT
    }
  }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *_uart2)
{
/*
  swv_printf("rx! %02x %02x %02x %02x\n",
    uart2_rx_buf[0], uart2_rx_buf[1], uart2_rx_buf[2], uart2_rx_buf[3]);
*/
  uart2_rx_flag = true;
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *_uart2)
{
  // swv_printf("error! %u\n", uart2.ErrorCode);
}

void NMI_Handler() { while (1) { } }
void HardFault_Handler() { while (1) { } }
void SVC_Handler() { while (1) { } }
void PendSV_Handler() { while (1) { } }
void WWDG_IRQHandler() { while (1) { } }
void PVD_IRQHandler() { while (1) { } }
void RTC_IRQHandler() { while (1) { } }
void FLASH_IRQHandler() { while (1) { } }
void RCC_IRQHandler() { while (1) { } }
void EXTI0_1_IRQHandler() { while (1) { } }
void EXTI2_3_IRQHandler() { while (1) { } }
void EXTI4_15_IRQHandler() { while (1) { } }
void DMA1_Channel1_IRQHandler() { while (1) { } }
void DMA1_Channel2_3_IRQHandler() { while (1) { } }
void DMA1_Channel4_5_IRQHandler() { while (1) { } }
void ADC1_COMP_IRQHandler() { while (1) { } }
void LPTIM1_IRQHandler() { while (1) { } }
void TIM2_IRQHandler() { while (1) { } }
void TIM21_IRQHandler() { while (1) { } }
void I2C1_IRQHandler() { while (1) { } }
void SPI1_IRQHandler() { while (1) { } }
void LPUART1_IRQHandler() { while (1) { } }
