#include <stm32g0xx_hal.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define LED_IND_ACT_PORT  GPIOA
#define LED_IND_ACT_PIN   GPIO_PIN_1

#define LED_OUT_R_PORT  GPIOA
#define LED_OUT_R_PIN   GPIO_PIN_4
#define LED_OUT_G_PORT  GPIOB
#define LED_OUT_G_PIN   GPIO_PIN_6
#define LED_OUT_B_PORT  GPIOA
#define LED_OUT_B_PIN   GPIO_PIN_0

#define DRV_NFAULT_PORT GPIOA
#define DRV_NFAULT_PIN  GPIO_PIN_2
#define DRV_NSLEEP_PORT GPIOA
#define DRV_NSLEEP_PIN  GPIO_PIN_3
#define DRV_EN_PORT   GPIOA
#define DRV_EN_PIN    GPIO_PIN_5
#define DRV_IN_U_PORT GPIOA
#define DRV_IN_U_PIN  GPIO_PIN_6
#define DRV_IN_V_PORT GPIOA
#define DRV_IN_V_PIN  GPIO_PIN_7
#define DRV_IN_W_PORT GPIOB
#define DRV_IN_W_PIN  GPIO_PIN_0

TIM_HandleTypeDef tim14, tim16, tim17, tim3;
I2C_HandleTypeDef i2c1, i2c2;

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
static void swv_printf(const char *restrict fmt, ...)
{
  char s[256];
  va_list args;
  va_start(args, fmt);
  int r = vsnprintf(s, sizeof s, fmt, args);
  for (int i = 0; i < r && i < sizeof s - 1; i++) swv_putchar(s[i]);
  if (r >= sizeof s) {
    for (int i = 0; i < 3; i++) swv_putchar('.');
    swv_putchar('\n');
  }
}

#define PWM_RESOLUTION 1000
// angle: [0, 36000000)
/*
from math import *
N=720
print(', '.join('%d' % round(1000*(1+0.35*sin(i/N*2*pi))/2) for i in range(N)))
*/
const uint16_t sin_lookup[720] = {
500, 502, 503, 505, 506, 508, 509, 511, 512, 514, 515, 517, 518, 520, 521, 523, 524, 526, 527, 529, 530, 532, 533, 535, 536, 538, 539, 541, 542, 544, 545, 547, 548, 550, 551, 553, 554, 556, 557, 558, 560, 561, 563, 564, 566, 567, 568, 570, 571, 573, 574, 575, 577, 578, 579, 581, 582, 584, 585, 586, 588, 589, 590, 591, 593, 594, 595, 597, 598, 599, 600, 602, 603, 604, 605, 607, 608, 609, 610, 611, 612, 614, 615, 616, 617, 618, 619, 620, 622, 623, 624, 625, 626, 627, 628, 629, 630, 631, 632, 633, 634, 635, 636, 637, 638, 639, 640, 641, 642, 642, 643, 644, 645, 646, 647, 648, 648, 649, 650, 651, 652, 652, 653, 654, 655, 655, 656, 657, 657, 658, 659, 659, 660, 660, 661, 662, 662, 663, 663, 664, 664, 665, 665, 666, 666, 667, 667, 668, 668, 669, 669, 669, 670, 670, 671, 671, 671, 671, 672, 672, 672, 673, 673, 673, 673, 674, 674, 674, 674, 674, 674, 674, 675, 675, 675, 675, 675, 675, 675, 675, 675, 675, 675, 675, 675, 675, 675, 675, 675, 674, 674, 674, 674, 674, 674, 674, 673, 673, 673, 673, 672, 672, 672, 671, 671, 671, 671, 670, 670, 669, 669, 669, 668, 668, 667, 667, 666, 666, 665, 665, 664, 664, 663, 663, 662, 662, 661, 660, 660, 659, 659, 658, 657, 657, 656, 655, 655, 654, 653, 652, 652, 651, 650, 649, 648, 648, 647, 646, 645, 644, 643, 642, 642, 641, 640, 639, 638, 637, 636, 635, 634, 633, 632, 631, 630, 629, 628, 627, 626, 625, 624, 623, 622, 620, 619, 618, 617, 616, 615, 614, 612, 611, 610, 609, 608, 607, 605, 604, 603, 602, 600, 599, 598, 597, 595, 594, 593, 591, 590, 589, 588, 586, 585, 584, 582, 581, 579, 578, 577, 575, 574, 573, 571, 570, 568, 567, 566, 564, 563, 561, 560, 558, 557, 556, 554, 553, 551, 550, 548, 547, 545, 544, 542, 541, 539, 538, 536, 535, 533, 532, 530, 529, 527, 526, 524, 523, 521, 520, 518, 517, 515, 514, 512, 511, 509, 508, 506, 505, 503, 502, 500, 498, 497, 495, 494, 492, 491, 489, 488, 486, 485, 483, 482, 480, 479, 477, 476, 474, 473, 471, 470, 468, 467, 465, 464, 462, 461, 459, 458, 456, 455, 453, 452, 450, 449, 447, 446, 444, 443, 442, 440, 439, 437, 436, 434, 433, 432, 430, 429, 427, 426, 425, 423, 422, 421, 419, 418, 416, 415, 414, 412, 411, 410, 409, 407, 406, 405, 403, 402, 401, 400, 398, 397, 396, 395, 393, 392, 391, 390, 389, 388, 386, 385, 384, 383, 382, 381, 380, 378, 377, 376, 375, 374, 373, 372, 371, 370, 369, 368, 367, 366, 365, 364, 363, 362, 361, 360, 359, 358, 358, 357, 356, 355, 354, 353, 352, 352, 351, 350, 349, 348, 348, 347, 346, 345, 345, 344, 343, 343, 342, 341, 341, 340, 340, 339, 338, 338, 337, 337, 336, 336, 335, 335, 334, 334, 333, 333, 332, 332, 331, 331, 331, 330, 330, 329, 329, 329, 329, 328, 328, 328, 327, 327, 327, 327, 326, 326, 326, 326, 326, 326, 326, 325, 325, 325, 325, 325, 325, 325, 325, 325, 325, 325, 325, 325, 325, 325, 325, 325, 326, 326, 326, 326, 326, 326, 326, 327, 327, 327, 327, 328, 328, 328, 329, 329, 329, 329, 330, 330, 331, 331, 331, 332, 332, 333, 333, 334, 334, 335, 335, 336, 336, 337, 337, 338, 338, 339, 340, 340, 341, 341, 342, 343, 343, 344, 345, 345, 346, 347, 348, 348, 349, 350, 351, 352, 352, 353, 354, 355, 356, 357, 358, 358, 359, 360, 361, 362, 363, 364, 365, 366, 367, 368, 369, 370, 371, 372, 373, 374, 375, 376, 377, 378, 380, 381, 382, 383, 384, 385, 386, 388, 389, 390, 391, 392, 393, 395, 396, 397, 398, 400, 401, 402, 403, 405, 406, 407, 409, 410, 411, 412, 414, 415, 416, 418, 419, 421, 422, 423, 425, 426, 427, 429, 430, 432, 433, 434, 436, 437, 439, 440, 442, 443, 444, 446, 447, 449, 450, 452, 453, 455, 456, 458, 459, 461, 462, 464, 465, 467, 468, 470, 471, 473, 474, 476, 477, 479, 480, 482, 483, 485, 486, 488, 489, 491, 492, 494, 495, 497, 498
};

// Increasing values correspond to clockwise rotation
static inline void drive_motor(uint32_t angle)
{
  TIM3->CCR1 = sin_lookup[(angle / 50000 +   0) % 720];
  TIM3->CCR2 = sin_lookup[(angle / 50000 + 240) % 720];
  TIM3->CCR3 = sin_lookup[(angle / 50000 + 480) % 720];
}

int main()
{
  HAL_Init();

  // ======== GPIO ========
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef gpio_init;

  gpio_init.Pin = LED_IND_ACT_PIN;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_PULLDOWN;
  gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_IND_ACT_PORT, &gpio_init);
  HAL_GPIO_WritePin(LED_IND_ACT_PORT, LED_IND_ACT_PIN, GPIO_PIN_SET);

  // SWD (PA13, PA14)
  gpio_init.Pin = GPIO_PIN_13 | GPIO_PIN_14;
  gpio_init.Mode = GPIO_MODE_AF_PP; // Pass over control to AF peripheral
  gpio_init.Alternate = GPIO_AF0_SWJ;
  gpio_init.Pull = GPIO_PULLUP;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio_init);

  // ======== Clocks ========
  RCC_OscInitTypeDef osc_init = { 0 };
  osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  osc_init.HSIState = RCC_HSI_ON;
  osc_init.PLL.PLLState = RCC_PLL_ON;
  osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  osc_init.PLL.PLLM = RCC_PLLM_DIV2;
  osc_init.PLL.PLLN = 8;
  osc_init.PLL.PLLP = RCC_PLLP_DIV2;
  osc_init.PLL.PLLR = RCC_PLLR_DIV2;
  HAL_RCC_OscConfig(&osc_init);

  RCC_ClkInitTypeDef clk_init = { 0 };
  clk_init.ClockType =
    RCC_CLOCKTYPE_SYSCLK |
    RCC_CLOCKTYPE_HCLK |
    RCC_CLOCKTYPE_PCLK1;
  clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk_init.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_2);

  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  // ======== Timer ========
  // APB1 = 64 MHz
  // period = 4 kHz = 16000 cycles

  // LED Red, TIM14
  gpio_init.Pin = LED_OUT_R_PIN;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Alternate = GPIO_AF4_TIM14;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_OUT_R_PORT, &gpio_init);
  __HAL_RCC_TIM14_CLK_ENABLE();
  tim14 = (TIM_HandleTypeDef){
    .Instance = TIM14,
    .Init = {
      .Prescaler = 1 - 1,
      .CounterMode = TIM_COUNTERMODE_UP,
      .Period = 16000 - 1,
      .ClockDivision = TIM_CLOCKDIVISION_DIV1,
      .RepetitionCounter = 0,
    },
  };
  HAL_TIM_PWM_Init(&tim14);
  TIM_OC_InitTypeDef tim14_ch1_oc_init = {
    .OCMode = TIM_OCMODE_PWM2,
    .Pulse = 0, // to be filled
    .OCPolarity = TIM_OCPOLARITY_HIGH,
  };
  HAL_TIM_PWM_ConfigChannel(&tim14, &tim14_ch1_oc_init, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&tim14, TIM_CHANNEL_1);

  // LED Green, TIM16
  gpio_init.Pin = LED_OUT_G_PIN;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Alternate = GPIO_AF2_TIM16;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_OUT_G_PORT, &gpio_init);
  __HAL_RCC_TIM16_CLK_ENABLE();
  tim16 = (TIM_HandleTypeDef){
    .Instance = TIM16,
    .Init = {
      .Prescaler = 1 - 1,
      .CounterMode = TIM_COUNTERMODE_UP,
      .Period = 16000 - 1,
      .ClockDivision = TIM_CLOCKDIVISION_DIV1,
      .RepetitionCounter = 0,
    },
  };
  HAL_TIM_PWM_Init(&tim16);
  TIM_OC_InitTypeDef tim16_ch1_oc_init = {
    .OCMode = TIM_OCMODE_PWM2,
    .Pulse = 0, // to be filled
    .OCNPolarity = TIM_OCNPOLARITY_HIGH,  // Output is TIM16_CH1N
  };
  HAL_TIM_PWM_ConfigChannel(&tim16, &tim16_ch1_oc_init, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&tim16, TIM_CHANNEL_1);

/*
  // LED Blue, TIM17
  gpio_init.Pin = LED_OUT_B_PIN;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Alternate = GPIO_AF2_TIM17;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_OUT_B_PORT, &gpio_init);
  __HAL_RCC_TIM17_CLK_ENABLE();
  tim17 = (TIM_HandleTypeDef){
    .Instance = TIM17,
    .Init = {
      .Prescaler = 1 - 1,
      .CounterMode = TIM_COUNTERMODE_UP,
      .Period = 16000 - 1,
      .ClockDivision = TIM_CLOCKDIVISION_DIV1,
      .RepetitionCounter = 0,
    },
  };
  HAL_TIM_PWM_Init(&tim17);
  TIM_OC_InitTypeDef tim17_ch1_oc_init = {
    .OCMode = TIM_OCMODE_PWM2,
    .Pulse = 0, // to be filled
    .OCNPolarity = TIM_OCNPOLARITY_HIGH,  // Output is TIM17_CH1N
  };
  HAL_TIM_PWM_ConfigChannel(&tim17, &tim17_ch1_oc_init, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&tim17, TIM_CHANNEL_1);
*/

  // Driver PWMs, TIM3
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;

  gpio_init.Pin = DRV_IN_U_PIN;
  gpio_init.Alternate = GPIO_AF1_TIM3;
  HAL_GPIO_Init(DRV_IN_U_PORT, &gpio_init);
  gpio_init.Pin = DRV_IN_V_PIN;
  gpio_init.Alternate = GPIO_AF1_TIM3;
  HAL_GPIO_Init(DRV_IN_V_PORT, &gpio_init);
  gpio_init.Pin = DRV_IN_W_PIN;
  gpio_init.Alternate = GPIO_AF1_TIM3;
  HAL_GPIO_Init(DRV_IN_W_PORT, &gpio_init);

  // Timer TIM3
  // APB1 = 64 MHz
  // Period = 64 MHz / 1000 = 64 kHz
  __HAL_RCC_TIM3_CLK_ENABLE();
  tim3 = (TIM_HandleTypeDef){
    .Instance = TIM3,
    .Init = {
      .Prescaler = 1 - 1,
      // .CounterMode = TIM_COUNTERMODE_UP,
      .CounterMode = TIM_COUNTERMODE_CENTERALIGNED1,
      .Period = PWM_RESOLUTION - 1,
      .ClockDivision = TIM_CLOCKDIVISION_DIV1,
      .RepetitionCounter = 0,
    },
  };
  HAL_TIM_PWM_Init(&tim3);
  TIM_OC_InitTypeDef tim3_ch1_oc_init = {
    .OCMode = TIM_OCMODE_PWM2,
    .Pulse = 0, // to be filled
    .OCPolarity = TIM_OCPOLARITY_HIGH,
  };
  HAL_TIM_PWM_ConfigChannel(&tim3, &tim3_ch1_oc_init, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&tim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&tim3, &tim3_ch1_oc_init, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&tim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&tim3, &tim3_ch1_oc_init, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&tim3, TIM_CHANNEL_3);

  // Driver nFAULT, nSLEEP, and EN pins
  gpio_init.Pin = DRV_NFAULT_PIN;
  gpio_init.Mode = GPIO_MODE_INPUT;
  gpio_init.Pull = GPIO_PULLUP;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DRV_NFAULT_PORT, &gpio_init);

  gpio_init.Pin = DRV_NSLEEP_PIN;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DRV_NSLEEP_PORT, &gpio_init);
  HAL_GPIO_WritePin(DRV_NSLEEP_PORT, DRV_NSLEEP_PIN, 1);

  gpio_init.Pin = DRV_EN_PIN;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DRV_EN_PORT, &gpio_init);
  HAL_GPIO_WritePin(DRV_EN_PORT, DRV_EN_PIN, 1);

  // ======== I2C 1 (unit interface) ========
  gpio_init.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Alternate = GPIO_AF6_I2C1;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio_init);

  __HAL_RCC_I2C1_CLK_ENABLE();
  HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C1);
  i2c1 = (I2C_HandleTypeDef){
    .Instance = I2C1,
    .Init = {
      .Timing = 0xF0110707,
      .OwnAddress1 = 0xAA,
      .AddressingMode = I2C_ADDRESSINGMODE_7BIT,
      .GeneralCallMode = I2C_GENERALCALL_ENABLE,
    },
  };
  HAL_I2C_Init(&i2c1);

  HAL_NVIC_SetPriority(I2C1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(I2C1_IRQn);
  HAL_I2C_EnableListen_IT(&i2c1);

  // ======== I2C 2 (encoder) ========
  gpio_init.Pin = GPIO_PIN_11 | GPIO_PIN_12;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Alternate = GPIO_AF6_I2C2;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio_init);

  __HAL_RCC_I2C2_CLK_ENABLE();
  HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C2);
  i2c2 = (I2C_HandleTypeDef){
    .Instance = I2C2,
    .Init = {
      // AS5600 datasheet p. 12
      // f_SCL <= 1 MHz, t_LOW >= 0.5 us, t_HIGH >= 0.26 us

      // RM0454 Rev 5, pp. 711, 726, 738 (examples), 766
      // APB = 64 MHz
      // PRESC = 3 (1 / (64 MHz / (3+1)) = 0.0625 us)
      // SCLH = SCLL = 7 (0.5 us) -> f_SCL = 1 MHz
      // SCLDEL = 1, SDADEL = 1
      .Timing = 0x30110707,
      .OwnAddress1 = 0x00,
      .AddressingMode = I2C_ADDRESSINGMODE_7BIT,
    },
  };
  HAL_I2C_Init(&i2c2);

  HAL_StatusTypeDef enc_ready = HAL_I2C_IsDeviceReady(&i2c2, 0x36 << 1, 3, 1000);
  swv_printf("encoder ready status: %d\n", (int)enc_ready);

  // Values increase clockwise
  uint16_t read_magenc() {
    // Read registers from AS5600
    uint8_t status = 0, agc = 0;
    uint8_t raw_angle[2];
    for (int attempts = 0; attempts < 5; attempts++) {
      HAL_StatusTypeDef r1 = HAL_I2C_Mem_Read(&i2c2, 0x36 << 1, 0x0B, I2C_MEMADD_SIZE_8BIT, &status, 1, 1000);
      HAL_StatusTypeDef r2 = HAL_I2C_Mem_Read(&i2c2, 0x36 << 1, 0x1A, I2C_MEMADD_SIZE_8BIT, &agc, 1, 1000);
      HAL_StatusTypeDef r3 = HAL_I2C_Mem_Read(&i2c2, 0x36 << 1, 0x0E, I2C_MEMADD_SIZE_8BIT, raw_angle, 2, 1000);
      // status bit 5: MD, bit 4: ML, bit 3: MH
      // HAL_I2C_ERROR_TIMEOUT (32) can arise when SCL/SDA pull-ups are not well soldered
      // Otherwise device NACK should be HAL_I2C_ERROR_AF (4)
      if (r1 == 0 && r2 == 0 && r3 == 0 && i2c2.ErrorCode == 0 && (status & 0x20)) {
        return ((uint16_t)raw_angle[0] << 8) | raw_angle[1];
      }
      for (int i = 0; i < 10000; i++) asm volatile ("nop");
    }
    while (1) {
      // Debug log
      HAL_StatusTypeDef r1 = HAL_I2C_Mem_Read(&i2c2, 0x36 << 1, 0x0B, I2C_MEMADD_SIZE_8BIT, &status, 1, 1000);
      HAL_StatusTypeDef r2 = HAL_I2C_Mem_Read(&i2c2, 0x36 << 1, 0x1A, I2C_MEMADD_SIZE_8BIT, &agc, 1, 1000);
      HAL_StatusTypeDef r3 = HAL_I2C_Mem_Read(&i2c2, 0x36 << 1, 0x0E, I2C_MEMADD_SIZE_8BIT, raw_angle, 2, 1000);
      swv_printf("status = %02x, AGC = %02x, raw angle = %4u, returned status = %d %d %d, error = %d\n",
        status & 0x38, agc, ((uint32_t)raw_angle[0] << 8) | raw_angle[1], r1, r2, r3, (int)i2c2.ErrorCode);
      HAL_GPIO_WritePin(LED_IND_ACT_PORT, LED_IND_ACT_PIN, 1); HAL_Delay(200);
      HAL_GPIO_WritePin(LED_IND_ACT_PORT, LED_IND_ACT_PIN, 0); HAL_Delay(200);
    }
  }

  uint16_t wait_stablize() {
    // Wait for a stable motor position (mallets down)
    static uint16_t angles[200];
    uint16_t angles_ptr = 0;
    while (1) {
      HAL_Delay(10);
      angles[angles_ptr++] = read_magenc();
      if (angles_ptr == 200) {
        // Check whether the motor has stablized
        // Find centroid of records
        uint32_t sum_A = 0, sum_B = 0;
        for (int i = 0; i < 200; i++) {
          sum_A += angles[i];
          sum_B += (angles[i] + 2048) % 4096;
        }
        uint16_t avg_A = sum_A / 200, avg_B = sum_B / 200;
        // 4096 * 4096 * 200 = 3355443200 is within uint32_t range
        uint32_t err_A = 0, err_B = 0;
        for (int i = 0; i < 200; i++) {
        #define sqr(_x) ((_x) * (_x))
          err_A += sqr((int32_t)angles[i] - avg_A);
          err_B += sqr((int32_t)((angles[i] + 2048) % 4096) - avg_B);
        #undef sqr
        }
        /* swv_printf("%d (%d) %d (%d)\n",
          (int)err_A / 200, (int)avg_A,
          (int)err_B / 200, (int)avg_B); */
        if (err_A <= err_B && err_A < 200 * 25) {
          return avg_A;
        } else if (err_B <= err_A && err_B < 200 * 25) {
          return (avg_B + 2048) % 4096;
        }
        // Continue searching
        for (int i = 0; i < 200 - 80; i++) angles[i] = angles[i + 80];
        angles_ptr -= 80;
      }
    }
  }

  uint16_t rest_angle = wait_stablize();
  swv_printf("rest: %d\n", (int)rest_angle);

  for (int i = 0; i < 3600; i++) {
    int angle = (int)(36000000 + i * 10000);
    drive_motor(angle);
    for (int i = 0; i < 1000; i++) asm volatile ("nop");
  }
  uint16_t elec_zero_angle = wait_stablize();
  swv_printf("elec zero: %d\n", (int)elec_zero_angle);
  // elec_zero_angle > rest_angle

  // Going down (counterclockwise)
  for (int i = elec_zero_angle; i != rest_angle; i = (i + 4096 - 1) % 4096) {
    int elec_angle = (int)(36000000 +
      (uint64_t)((i - elec_zero_angle + 4096) % 4096) * 36000000 * 7 / 4096);
    drive_motor(elec_angle);
    for (int i = 0; i < 12000; i++) asm volatile ("nop");
  }

  int idle_elec_angle = (int)(36000000 +
    (uint64_t)((rest_angle - elec_zero_angle + 4096) % 4096) * 36000000 * 7 / 4096);
  for (int i = 0; i < 1800; i++) {
    drive_motor(idle_elec_angle + i * 20000);
    for (int i = 0; i < 1000; i++) asm volatile ("nop");
  }

  while (1) {
    for (int i = 0; i < 10; i++) {
      HAL_GPIO_WritePin(LED_IND_ACT_PORT, LED_IND_ACT_PIN, 1); HAL_Delay(100);
      HAL_GPIO_WritePin(LED_IND_ACT_PORT, LED_IND_ACT_PIN, 0); HAL_Delay(100);
    }
    for (int i = 1700; i >= -100; i--) {
      drive_motor(36000000 + idle_elec_angle + i * 20000);
      for (int i = 0; i < 400; i++) asm volatile ("nop");
    }
    TIM3->CCR1 = TIM3->CCR2 = TIM3->CCR3 = 0;
    HAL_Delay(5);
    for (int i = 0; i < 1800; i++) {
      drive_motor(idle_elec_angle + i * 20000);
      for (int i = 0; i < 1000; i++) asm volatile ("nop");
    }
  }

  while (1) {
    static int chroma = 2;
    static TIM_TypeDef *const chroma_timers[3] = {TIM14, TIM16, TIM17};
    chroma = (chroma + 1) % 3;
    TIM14->CCR1 = 0;
    TIM16->CCR1 = 0;
    TIM17->CCR1 = 0;
    for (int i = 0; i < 1800; i += 1) {
      float t = 1 - cosf((float)i / 1800 * 6.2831853f);
      // angle normalized into [0, 36000000) (36000000 = 1/7 revolution)
      int angle = (int)(0.5f + 72000000 - 18000000 * t);
      drive_motor(angle);
      chroma_timers[chroma]->CCR1 = 4000 * t;
    }
    static int parity = 1;
    HAL_GPIO_WritePin(LED_IND_ACT_PORT, LED_IND_ACT_PIN, parity ^= 1);

  }
}

void SysTick_Handler()
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();

  // if (HAL_GetTick() % 500 == 0)
  //   HAL_GPIO_WritePin(LED_IND_ACT_PORT, LED_IND_ACT_PIN, HAL_GetTick() % 1000 == 0);
}

static uint8_t i2c_rx_buf[2];

void I2C1_IRQHandler()
{
  if (I2C1->ISR & (I2C_FLAG_BERR | I2C_FLAG_ARLO | I2C_FLAG_OVR)) {
    HAL_I2C_ER_IRQHandler(&i2c1);
  } else {
    HAL_I2C_EV_IRQHandler(&i2c1);
  }
}

void NMI_Handler() { while (1) { } }
void HardFault_Handler() { while (1) { } }
void SVC_Handler() { while (1) { } }
void PendSV_Handler() { while (1) { } }
void WWDG_IRQHandler() { while (1) { } }
void RTC_TAMP_IRQHandler() { while (1) { } }
void FLASH_IRQHandler() { while (1) { } }
void RCC_IRQHandler() { while (1) { } }
void EXTI0_1_IRQHandler() { while (1) { } }
void EXTI2_3_IRQHandler() { while (1) { } }
void EXTI4_15_IRQHandler() { while (1) { } }
void DMA1_Channel1_IRQHandler() { while (1) { } }
void DMA1_Channel2_3_IRQHandler() { while (1) { } }
void DMA1_Ch4_5_DMAMUX1_OVR_IRQHandler() { while (1) { } }
void ADC1_IRQHandler() { while (1) { } }
void TIM1_BRK_UP_TRG_COM_IRQHandler() { while (1) { } }
void TIM1_CC_IRQHandler() { while (1) { } }
void TIM3_IRQHandler() { while (1) { } }
void TIM14_IRQHandler() { while (1) { } }
void TIM16_IRQHandler() { while (1) { } }
void TIM17_IRQHandler() { while (1) { } }
void I2C2_IRQHandler() { while (1) { } }
void SPI1_IRQHandler() { while (1) { } }
void SPI2_IRQHandler() { while (1) { } }
void USART1_IRQHandler() { while (1) { } }
void USART2_IRQHandler() { while (1) { } }

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *i2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
  if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
    HAL_I2C_Slave_Seq_Receive_IT(i2c, i2c_rx_buf, 2, I2C_FIRST_AND_LAST_FRAME);
  }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *i2c)
{
  swv_printf("Received %02x %02x\n", i2c_rx_buf[0], i2c_rx_buf[1]);
  int parity = 0;
  for (int i = 0; i < 20; i++) {
    HAL_GPIO_WritePin(LED_IND_ACT_PORT, LED_IND_ACT_PIN, parity ^= 1);
    HAL_Delay(100);
  }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *i2c)
{
  // This can be triggered on device-ready probe (HAL_I2C_IsDeviceReady)
  // since there is no data to be received in this scenario

  // However, the listen cannot be restarted here as i2c->State is still HAL_I2C_STATE_LISTEN
  // See stm32g0xx_hal_i2c.c:5868 <I2C_ITSlaveCplt>
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *i2c)
{
  // Device-ready probes will also invoke this subroutine
  if (i2c == &i2c1) {
    HAL_I2C_EnableListen_IT(i2c);
  }
}
