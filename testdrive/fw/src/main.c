#include <stm32g0xx_hal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define PIN_PWR_LATCH   GPIO_PIN_5
#define PORT_PWR_LATCH  GPIOA

#define PIN_CS_BMI270   GPIO_PIN_7
#define PORT_CS_BMI270  GPIOA

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

SPI_HandleTypeDef spi1 = { 0 }, spi2 = { 0 };
TIM_HandleTypeDef tim14, tim16, tim17;

#pragma GCC optimize("O3")
static inline void spi1_transmit(uint8_t *data, size_t size)
{
  HAL_SPI_Transmit(&spi1, data, size, 1000); return;
/*
  for (int i = 0; i < size; i++) {
    while (!(SPI1->SR & SPI_SR_TXE)) { }
    SPI1->DR = data[i];
  }
  while (!(SPI1->SR & SPI_SR_TXE)) { }
  while ((SPI1->SR & SPI_SR_BSY)) { }
  // Clear OVR flag
  (void)SPI1->DR;
  (void)SPI1->SR;
*/
}

static inline void spi2_transmit(uint8_t *data, size_t size)
{
  HAL_SPI_Transmit(&spi2, data, size, 1000); return;
}
static inline void spi2_receive(uint8_t *data, size_t size)
{
  for (int i = 0; i < size; i++) data[i] = 0xAA;
  HAL_StatusTypeDef result = HAL_SPI_Receive(&spi2, data, size, 1000);
  swv_printf("SPI2 receive %d\n", (int)result);
}

int main()
{
  HAL_Init();

  // ======== GPIO ========
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitTypeDef gpio_init;

  // SWD (PA13, PA14)
  gpio_init.Pin = GPIO_PIN_13 | GPIO_PIN_14;
  gpio_init.Mode = GPIO_MODE_AF_PP; // Pass over control to AF peripheral
  gpio_init.Alternate = GPIO_AF0_SWJ;
  gpio_init.Pull = GPIO_PULLUP;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio_init);

  gpio_init.Pin = PIN_PWR_LATCH;
  gpio_init.Mode = GPIO_MODE_OUTPUT_OD;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(PORT_PWR_LATCH, &gpio_init);
  HAL_GPIO_WritePin(PORT_PWR_LATCH, PIN_PWR_LATCH, GPIO_PIN_RESET);

  // ======== Clocks ========
  RCC_OscInitTypeDef osc_init = { 0 };
  osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  osc_init.HSIState = RCC_HSI_ON;
  osc_init.PLL.PLLState = RCC_PLL_OFF;
  HAL_RCC_OscConfig(&osc_init);

  RCC_ClkInitTypeDef clk_init = { 0 };
  clk_init.ClockType =
    RCC_CLOCKTYPE_SYSCLK |
    RCC_CLOCKTYPE_HCLK |
    RCC_CLOCKTYPE_PCLK1;
  clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_HSI; // 16 MHz
  clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk_init.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_2);

  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  // ======== LED Timers ========
  // APB1 = 16 MHz
  // period = 4 kHz = 4000 cycles

  // LED Blue, TIM14 Ch1, PB1
  gpio_init.Pin = GPIO_PIN_1;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Alternate = GPIO_AF0_TIM14;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio_init);
  __HAL_RCC_TIM14_CLK_ENABLE();
  tim14 = (TIM_HandleTypeDef){
    .Instance = TIM14,
    .Init = {
      .Prescaler = 1 - 1,
      .CounterMode = TIM_COUNTERMODE_UP,
      .Period = 4000 - 1,
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

  // LED Green, TIM16 Ch1N, PB6
  gpio_init.Pin = GPIO_PIN_6;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Alternate = GPIO_AF2_TIM16;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio_init);
  __HAL_RCC_TIM16_CLK_ENABLE();
  tim16 = (TIM_HandleTypeDef){
    .Instance = TIM16,
    .Init = {
      .Prescaler = 1 - 1,
      .CounterMode = TIM_COUNTERMODE_UP,
      .Period = 4000 - 1,
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

  // LED Red, TIM17 Ch1N PB7
  gpio_init.Pin = GPIO_PIN_7;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Alternate = GPIO_AF2_TIM17;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio_init);
  __HAL_RCC_TIM17_CLK_ENABLE();
  tim17 = (TIM_HandleTypeDef){
    .Instance = TIM17,
    .Init = {
      .Prescaler = 1 - 1,
      .CounterMode = TIM_COUNTERMODE_UP,
      .Period = 4000 - 1,
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

/*
from math import *
N=1800
print(', '.join('%d' % round(2000*(1+sin(i/N*2*pi))) for i in range(N)))
*/
#define N 1800
  const uint16_t sin_lut[N] = {
2000, 2007, 2014, 2021, 2028, 2035, 2042, 2049, 2056, 2063, 2070, 2077, 2084, 2091, 2098, 2105, 2112, 2119, 2126, 2133, 2140, 2146, 2153, 2160, 2167, 2174, 2181, 2188, 2195, 2202, 2209, 2216, 2223, 2230, 2237, 2244, 2251, 2258, 2265, 2271, 2278, 2285, 2292, 2299, 2306, 2313, 2320, 2327, 2334, 2340, 2347, 2354, 2361, 2368, 2375, 2382, 2388, 2395, 2402, 2409, 2416, 2423, 2429, 2436, 2443, 2450, 2457, 2463, 2470, 2477, 2484, 2491, 2497, 2504, 2511, 2518, 2524, 2531, 2538, 2545, 2551, 2558, 2565, 2571, 2578, 2585, 2591, 2598, 2605, 2611, 2618, 2625, 2631, 2638, 2645, 2651, 2658, 2664, 2671, 2677, 2684, 2691, 2697, 2704, 2710, 2717, 2723, 2730, 2736, 2743, 2749, 2756, 2762, 2769, 2775, 2781, 2788, 2794, 2801, 2807, 2813, 2820, 2826, 2833, 2839, 2845, 2852, 2858, 2864, 2870, 2877, 2883, 2889, 2896, 2902, 2908, 2914, 2920, 2927, 2933, 2939, 2945, 2951, 2957, 2964, 2970, 2976, 2982, 2988, 2994, 3000, 3006, 3012, 3018, 3024, 3030, 3036, 3042, 3048, 3054, 3060, 3066, 3072, 3078, 3083, 3089, 3095, 3101, 3107, 3113, 3118, 3124, 3130, 3136, 3141, 3147, 3153, 3159, 3164, 3170, 3176, 3181, 3187, 3192, 3198, 3204, 3209, 3215, 3220, 3226, 3231, 3237, 3242, 3248, 3253, 3259, 3264, 3269, 3275, 3280, 3286, 3291, 3296, 3302, 3307, 3312, 3317, 3323, 3328, 3333, 3338, 3343, 3349, 3354, 3359, 3364, 3369, 3374, 3379, 3384, 3389, 3394, 3399, 3404, 3409, 3414, 3419, 3424, 3429, 3434, 3439, 3444, 3448, 3453, 3458, 3463, 3467, 3472, 3477, 3482, 3486, 3491, 3496, 3500, 3505, 3509, 3514, 3519, 3523, 3528, 3532, 3537, 3541, 3545, 3550, 3554, 3559, 3563, 3567, 3572, 3576, 3580, 3585, 3589, 3593, 3597, 3601, 3606, 3610, 3614, 3618, 3622, 3626, 3630, 3634, 3638, 3642, 3646, 3650, 3654, 3658, 3662, 3666, 3670, 3674, 3677, 3681, 3685, 3689, 3692, 3696, 3700, 3703, 3707, 3711, 3714, 3718, 3721, 3725, 3729, 3732, 3736, 3739, 3742, 3746, 3749, 3753, 3756, 3759, 3763, 3766, 3769, 3772, 3776, 3779, 3782, 3785, 3788, 3791, 3795, 3798, 3801, 3804, 3807, 3810, 3813, 3816, 3818, 3821, 3824, 3827, 3830, 3833, 3836, 3838, 3841, 3844, 3846, 3849, 3852, 3854, 3857, 3860, 3862, 3865, 3867, 3870, 3872, 3875, 3877, 3879, 3882, 3884, 3886, 3889, 3891, 3893, 3896, 3898, 3900, 3902, 3904, 3906, 3908, 3911, 3913, 3915, 3917, 3919, 3921, 3923, 3924, 3926, 3928, 3930, 3932, 3934, 3935, 3937, 3939, 3941, 3942, 3944, 3946, 3947, 3949, 3950, 3952, 3953, 3955, 3956, 3958, 3959, 3961, 3962, 3963, 3965, 3966, 3967, 3968, 3970, 3971, 3972, 3973, 3974, 3975, 3976, 3978, 3979, 3980, 3981, 3981, 3982, 3983, 3984, 3985, 3986, 3987, 3988, 3988, 3989, 3990, 3990, 3991, 3992, 3992, 3993, 3994, 3994, 3995, 3995, 3996, 3996, 3996, 3997, 3997, 3998, 3998, 3998, 3999, 3999, 3999, 3999, 3999, 4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000, 3999, 3999, 3999, 3999, 3999, 3998, 3998, 3998, 3997, 3997, 3996, 3996, 3996, 3995, 3995, 3994, 3994, 3993, 3992, 3992, 3991, 3990, 3990, 3989, 3988, 3988, 3987, 3986, 3985, 3984, 3983, 3982, 3981, 3981, 3980, 3979, 3978, 3976, 3975, 3974, 3973, 3972, 3971, 3970, 3968, 3967, 3966, 3965, 3963, 3962, 3961, 3959, 3958, 3956, 3955, 3953, 3952, 3950, 3949, 3947, 3946, 3944, 3942, 3941, 3939, 3937, 3935, 3934, 3932, 3930, 3928, 3926, 3924, 3923, 3921, 3919, 3917, 3915, 3913, 3911, 3908, 3906, 3904, 3902, 3900, 3898, 3896, 3893, 3891, 3889, 3886, 3884, 3882, 3879, 3877, 3875, 3872, 3870, 3867, 3865, 3862, 3860, 3857, 3854, 3852, 3849, 3846, 3844, 3841, 3838, 3836, 3833, 3830, 3827, 3824, 3821, 3818, 3816, 3813, 3810, 3807, 3804, 3801, 3798, 3795, 3791, 3788, 3785, 3782, 3779, 3776, 3772, 3769, 3766, 3763, 3759, 3756, 3753, 3749, 3746, 3742, 3739, 3736, 3732, 3729, 3725, 3721, 3718, 3714, 3711, 3707, 3703, 3700, 3696, 3692, 3689, 3685, 3681, 3677, 3674, 3670, 3666, 3662, 3658, 3654, 3650, 3646, 3642, 3638, 3634, 3630, 3626, 3622, 3618, 3614, 3610, 3606, 3601, 3597, 3593, 3589, 3585, 3580, 3576, 3572, 3567, 3563, 3559, 3554, 3550, 3545, 3541, 3537, 3532, 3528, 3523, 3519, 3514, 3509, 3505, 3500, 3496, 3491, 3486, 3482, 3477, 3472, 3467, 3463, 3458, 3453, 3448, 3444, 3439, 3434, 3429, 3424, 3419, 3414, 3409, 3404, 3399, 3394, 3389, 3384, 3379, 3374, 3369, 3364, 3359, 3354, 3349, 3343, 3338, 3333, 3328, 3323, 3317, 3312, 3307, 3302, 3296, 3291, 3286, 3280, 3275, 3269, 3264, 3259, 3253, 3248, 3242, 3237, 3231, 3226, 3220, 3215, 3209, 3204, 3198, 3192, 3187, 3181, 3176, 3170, 3164, 3159, 3153, 3147, 3141, 3136, 3130, 3124, 3118, 3113, 3107, 3101, 3095, 3089, 3083, 3078, 3072, 3066, 3060, 3054, 3048, 3042, 3036, 3030, 3024, 3018, 3012, 3006, 3000, 2994, 2988, 2982, 2976, 2970, 2964, 2957, 2951, 2945, 2939, 2933, 2927, 2920, 2914, 2908, 2902, 2896, 2889, 2883, 2877, 2870, 2864, 2858, 2852, 2845, 2839, 2833, 2826, 2820, 2813, 2807, 2801, 2794, 2788, 2781, 2775, 2769, 2762, 2756, 2749, 2743, 2736, 2730, 2723, 2717, 2710, 2704, 2697, 2691, 2684, 2677, 2671, 2664, 2658, 2651, 2645, 2638, 2631, 2625, 2618, 2611, 2605, 2598, 2591, 2585, 2578, 2571, 2565, 2558, 2551, 2545, 2538, 2531, 2524, 2518, 2511, 2504, 2497, 2491, 2484, 2477, 2470, 2463, 2457, 2450, 2443, 2436, 2429, 2423, 2416, 2409, 2402, 2395, 2388, 2382, 2375, 2368, 2361, 2354, 2347, 2340, 2334, 2327, 2320, 2313, 2306, 2299, 2292, 2285, 2278, 2271, 2265, 2258, 2251, 2244, 2237, 2230, 2223, 2216, 2209, 2202, 2195, 2188, 2181, 2174, 2167, 2160, 2153, 2146, 2140, 2133, 2126, 2119, 2112, 2105, 2098, 2091, 2084, 2077, 2070, 2063, 2056, 2049, 2042, 2035, 2028, 2021, 2014, 2007, 2000, 1993, 1986, 1979, 1972, 1965, 1958, 1951, 1944, 1937, 1930, 1923, 1916, 1909, 1902, 1895, 1888, 1881, 1874, 1867, 1860, 1854, 1847, 1840, 1833, 1826, 1819, 1812, 1805, 1798, 1791, 1784, 1777, 1770, 1763, 1756, 1749, 1742, 1735, 1729, 1722, 1715, 1708, 1701, 1694, 1687, 1680, 1673, 1666, 1660, 1653, 1646, 1639, 1632, 1625, 1618, 1612, 1605, 1598, 1591, 1584, 1577, 1571, 1564, 1557, 1550, 1543, 1537, 1530, 1523, 1516, 1509, 1503, 1496, 1489, 1482, 1476, 1469, 1462, 1455, 1449, 1442, 1435, 1429, 1422, 1415, 1409, 1402, 1395, 1389, 1382, 1375, 1369, 1362, 1355, 1349, 1342, 1336, 1329, 1323, 1316, 1309, 1303, 1296, 1290, 1283, 1277, 1270, 1264, 1257, 1251, 1244, 1238, 1231, 1225, 1219, 1212, 1206, 1199, 1193, 1187, 1180, 1174, 1167, 1161, 1155, 1148, 1142, 1136, 1130, 1123, 1117, 1111, 1104, 1098, 1092, 1086, 1080, 1073, 1067, 1061, 1055, 1049, 1043, 1036, 1030, 1024, 1018, 1012, 1006, 1000, 994, 988, 982, 976, 970, 964, 958, 952, 946, 940, 934, 928, 922, 917, 911, 905, 899, 893, 887, 882, 876, 870, 864, 859, 853, 847, 841, 836, 830, 824, 819, 813, 808, 802, 796, 791, 785, 780, 774, 769, 763, 758, 752, 747, 741, 736, 731, 725, 720, 714, 709, 704, 698, 693, 688, 683, 677, 672, 667, 662, 657, 651, 646, 641, 636, 631, 626, 621, 616, 611, 606, 601, 596, 591, 586, 581, 576, 571, 566, 561, 556, 552, 547, 542, 537, 533, 528, 523, 518, 514, 509, 504, 500, 495, 491, 486, 481, 477, 472, 468, 463, 459, 455, 450, 446, 441, 437, 433, 428, 424, 420, 415, 411, 407, 403, 399, 394, 390, 386, 382, 378, 374, 370, 366, 362, 358, 354, 350, 346, 342, 338, 334, 330, 326, 323, 319, 315, 311, 308, 304, 300, 297, 293, 289, 286, 282, 279, 275, 271, 268, 264, 261, 258, 254, 251, 247, 244, 241, 237, 234, 231, 228, 224, 221, 218, 215, 212, 209, 205, 202, 199, 196, 193, 190, 187, 184, 182, 179, 176, 173, 170, 167, 164, 162, 159, 156, 154, 151, 148, 146, 143, 140, 138, 135, 133, 130, 128, 125, 123, 121, 118, 116, 114, 111, 109, 107, 104, 102, 100, 98, 96, 94, 92, 89, 87, 85, 83, 81, 79, 77, 76, 74, 72, 70, 68, 66, 65, 63, 61, 59, 58, 56, 54, 53, 51, 50, 48, 47, 45, 44, 42, 41, 39, 38, 37, 35, 34, 33, 32, 30, 29, 28, 27, 26, 25, 24, 22, 21, 20, 19, 19, 18, 17, 16, 15, 14, 13, 12, 12, 11, 10, 10, 9, 8, 8, 7, 6, 6, 5, 5, 4, 4, 4, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 4, 4, 5, 5, 6, 6, 7, 8, 8, 9, 10, 10, 11, 12, 12, 13, 14, 15, 16, 17, 18, 19, 19, 20, 21, 22, 24, 25, 26, 27, 28, 29, 30, 32, 33, 34, 35, 37, 38, 39, 41, 42, 44, 45, 47, 48, 50, 51, 53, 54, 56, 58, 59, 61, 63, 65, 66, 68, 70, 72, 74, 76, 77, 79, 81, 83, 85, 87, 89, 92, 94, 96, 98, 100, 102, 104, 107, 109, 111, 114, 116, 118, 121, 123, 125, 128, 130, 133, 135, 138, 140, 143, 146, 148, 151, 154, 156, 159, 162, 164, 167, 170, 173, 176, 179, 182, 184, 187, 190, 193, 196, 199, 202, 205, 209, 212, 215, 218, 221, 224, 228, 231, 234, 237, 241, 244, 247, 251, 254, 258, 261, 264, 268, 271, 275, 279, 282, 286, 289, 293, 297, 300, 304, 308, 311, 315, 319, 323, 326, 330, 334, 338, 342, 346, 350, 354, 358, 362, 366, 370, 374, 378, 382, 386, 390, 394, 399, 403, 407, 411, 415, 420, 424, 428, 433, 437, 441, 446, 450, 455, 459, 463, 468, 472, 477, 481, 486, 491, 495, 500, 504, 509, 514, 518, 523, 528, 533, 537, 542, 547, 552, 556, 561, 566, 571, 576, 581, 586, 591, 596, 601, 606, 611, 616, 621, 626, 631, 636, 641, 646, 651, 657, 662, 667, 672, 677, 683, 688, 693, 698, 704, 709, 714, 720, 725, 731, 736, 741, 747, 752, 758, 763, 769, 774, 780, 785, 791, 796, 802, 808, 813, 819, 824, 830, 836, 841, 847, 853, 859, 864, 870, 876, 882, 887, 893, 899, 905, 911, 917, 922, 928, 934, 940, 946, 952, 958, 964, 970, 976, 982, 988, 994, 1000, 1006, 1012, 1018, 1024, 1030, 1036, 1043, 1049, 1055, 1061, 1067, 1073, 1080, 1086, 1092, 1098, 1104, 1111, 1117, 1123, 1130, 1136, 1142, 1148, 1155, 1161, 1167, 1174, 1180, 1187, 1193, 1199, 1206, 1212, 1219, 1225, 1231, 1238, 1244, 1251, 1257, 1264, 1270, 1277, 1283, 1290, 1296, 1303, 1309, 1316, 1323, 1329, 1336, 1342, 1349, 1355, 1362, 1369, 1375, 1382, 1389, 1395, 1402, 1409, 1415, 1422, 1429, 1435, 1442, 1449, 1455, 1462, 1469, 1476, 1482, 1489, 1496, 1503, 1509, 1516, 1523, 1530, 1537, 1543, 1550, 1557, 1564, 1571, 1577, 1584, 1591, 1598, 1605, 1612, 1618, 1625, 1632, 1639, 1646, 1653, 1660, 1666, 1673, 1680, 1687, 1694, 1701, 1708, 1715, 1722, 1729, 1735, 1742, 1749, 1756, 1763, 1770, 1777, 1784, 1791, 1798, 1805, 1812, 1819, 1826, 1833, 1840, 1847, 1854, 1860, 1867, 1874, 1881, 1888, 1895, 1902, 1909, 1916, 1923, 1930, 1937, 1944, 1951, 1958, 1965, 1972, 1979, 1986, 1993
  };
  if (1) {
    for (int i = 0; i < N; i++) {
      TIM14->CCR1 = sin_lut[i] / 2;
      TIM16->CCR1 = sin_lut[(i + N / 3) % N] / 2;
      TIM17->CCR1 = sin_lut[(i + N * 2 / 3) % N];
      // HAL_Delay(1);
      for (int i = 0; i < 1000; i++) asm volatile ("nop");
    }
  }

  // ======== SPI ========
  // SPI1
  // SPI1_SCK (PA1), SPI1_MOSI (PA2)
  gpio_init.Pin = GPIO_PIN_1 | GPIO_PIN_2;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Alternate = GPIO_AF0_SPI1;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio_init);

  __HAL_RCC_SPI1_CLK_ENABLE();
  spi1.Instance = SPI1;
  spi1.Init.Mode = SPI_MODE_MASTER;
  spi1.Init.Direction = SPI_DIRECTION_2LINES;
  spi1.Init.CLKPolarity = SPI_POLARITY_LOW; // CPOL = 0
  spi1.Init.CLKPhase = SPI_PHASE_1EDGE;     // CPHA = 0
  spi1.Init.NSS = SPI_NSS_SOFT;
  spi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  spi1.Init.TIMode = SPI_TIMODE_DISABLE;
  spi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  spi1.Init.DataSize = SPI_DATASIZE_8BIT;
  spi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;  // APB / 2 = 8 MHz
  HAL_SPI_Init(&spi1);
  __HAL_SPI_ENABLE(&spi1);

  // SPI2
  // SPI2_SCK (PA0), SPI2_MOSI (PA10)
  gpio_init.Pin = GPIO_PIN_0 | GPIO_PIN_10;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Alternate = GPIO_AF0_SPI2;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio_init);
  // Remap pin 17 (default PA12) to PA10
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  SYSCFG->CFGR1 |= SYSCFG_REMAP_PA12;

  __HAL_RCC_SPI2_CLK_ENABLE();
  spi2.Instance = SPI2;
  spi2.Init.Mode = SPI_MODE_MASTER;
  spi2.Init.Direction = SPI_DIRECTION_1LINE;
  spi2.Init.CLKPolarity = SPI_POLARITY_LOW; // CPOL = 0
  spi2.Init.CLKPhase = SPI_PHASE_1EDGE;     // CPHA = 0
  spi2.Init.NSS = SPI_NSS_SOFT;
  spi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  spi2.Init.TIMode = SPI_TIMODE_DISABLE;
  spi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  spi2.Init.DataSize = SPI_DATASIZE_8BIT;
  spi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;  // APB / 16 = 1 MHz
  HAL_SPI_Init(&spi2);
  __HAL_SPI_ENABLE(&spi2);

  // CS_BMI270
  gpio_init.Pin = PIN_CS_BMI270;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(PORT_CS_BMI270, &gpio_init);
  HAL_GPIO_WritePin(PORT_CS_BMI270, PIN_CS_BMI270, 0);
  HAL_Delay(100);
  HAL_GPIO_WritePin(PORT_CS_BMI270, PIN_CS_BMI270, 1);
  HAL_Delay(100);

  while (1) {
    uint8_t data[6];
    data[0] = 0x80;
    HAL_GPIO_WritePin(PORT_CS_BMI270, PIN_CS_BMI270, 0);
    spi2_transmit(data, 1);
    spi2_receive(data, 2);
    HAL_GPIO_WritePin(PORT_CS_BMI270, PIN_CS_BMI270, 1);
    swv_printf("data A = %02x\n", data[1]); // (ignored)
    // HAL_Delay(10);

    data[0] = 0x80;
    HAL_GPIO_WritePin(PORT_CS_BMI270, PIN_CS_BMI270, 0);
    spi2_transmit(data, 1);
    spi2_receive(data, 2);
    HAL_GPIO_WritePin(PORT_CS_BMI270, PIN_CS_BMI270, 1);
    swv_printf("data B = %02x\n", data[1]); // 0x24 (already correct?)
    // HAL_Delay(10);

    data[0] = 0x6B; data[1] = 0x01; // IF_CONF.spi3 = 1
    HAL_GPIO_WritePin(PORT_CS_BMI270, PIN_CS_BMI270, 0);
    spi2_transmit(data, 2);
    HAL_GPIO_WritePin(PORT_CS_BMI270, PIN_CS_BMI270, 1);
    // HAL_Delay(10);

    data[0] = 0x80;
    HAL_GPIO_WritePin(PORT_CS_BMI270, PIN_CS_BMI270, 0);
    spi2_transmit(data, 1);
    spi2_receive(data, 2);
    HAL_GPIO_WritePin(PORT_CS_BMI270, PIN_CS_BMI270, 1);
    swv_printf("data C = %02x\n", data[1]);
    // HAL_Delay(100);
  }

  TIM14->CCR1 = 0;
  TIM16->CCR1 = 0;
  for (int i = 0; i < 10; i++) {
    swv_printf("blink!\n");
    TIM17->CCR1 = 8000;
    HAL_Delay(200);
    TIM17->CCR1 = 0;
    HAL_Delay(200);
  }
  while (1) { }
}

void SysTick_Handler()
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}
