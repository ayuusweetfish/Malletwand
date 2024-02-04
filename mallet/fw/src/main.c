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
print(', '.join('%d' % round(1000*(1+0.45*sin(i/N*2*pi))/2) for i in range(N)))
*/
const uint16_t sin_lookup[720] = {
500, 502, 504, 506, 508, 510, 512, 514, 516, 518, 520, 522, 524, 525, 527, 529, 531, 533, 535, 537, 539, 541, 543, 545, 547, 549, 551, 553, 554, 556, 558, 560, 562, 564, 566, 568, 570, 571, 573, 575, 577, 579, 581, 582, 584, 586, 588, 590, 592, 593, 595, 597, 599, 600, 602, 604, 606, 607, 609, 611, 612, 614, 616, 618, 619, 621, 623, 624, 626, 627, 629, 631, 632, 634, 635, 637, 639, 640, 642, 643, 645, 646, 648, 649, 651, 652, 653, 655, 656, 658, 659, 660, 662, 663, 665, 666, 667, 669, 670, 671, 672, 674, 675, 676, 677, 679, 680, 681, 682, 683, 684, 685, 687, 688, 689, 690, 691, 692, 693, 694, 695, 696, 697, 698, 699, 700, 700, 701, 702, 703, 704, 705, 706, 706, 707, 708, 709, 709, 710, 711, 711, 712, 713, 713, 714, 715, 715, 716, 716, 717, 717, 718, 718, 719, 719, 720, 720, 720, 721, 721, 722, 722, 722, 723, 723, 723, 723, 724, 724, 724, 724, 724, 724, 725, 725, 725, 725, 725, 725, 725, 725, 725, 725, 725, 725, 725, 725, 725, 724, 724, 724, 724, 724, 724, 723, 723, 723, 723, 722, 722, 722, 721, 721, 720, 720, 720, 719, 719, 718, 718, 717, 717, 716, 716, 715, 715, 714, 713, 713, 712, 711, 711, 710, 709, 709, 708, 707, 706, 706, 705, 704, 703, 702, 701, 700, 700, 699, 698, 697, 696, 695, 694, 693, 692, 691, 690, 689, 688, 687, 685, 684, 683, 682, 681, 680, 679, 677, 676, 675, 674, 672, 671, 670, 669, 667, 666, 665, 663, 662, 660, 659, 658, 656, 655, 653, 652, 651, 649, 648, 646, 645, 643, 642, 640, 639, 637, 635, 634, 632, 631, 629, 627, 626, 624, 623, 621, 619, 618, 616, 614, 612, 611, 609, 607, 606, 604, 602, 600, 599, 597, 595, 593, 592, 590, 588, 586, 584, 582, 581, 579, 577, 575, 573, 571, 570, 568, 566, 564, 562, 560, 558, 556, 554, 553, 551, 549, 547, 545, 543, 541, 539, 537, 535, 533, 531, 529, 527, 525, 524, 522, 520, 518, 516, 514, 512, 510, 508, 506, 504, 502, 500, 498, 496, 494, 492, 490, 488, 486, 484, 482, 480, 478, 476, 475, 473, 471, 469, 467, 465, 463, 461, 459, 457, 455, 453, 451, 449, 447, 446, 444, 442, 440, 438, 436, 434, 432, 430, 429, 427, 425, 423, 421, 419, 418, 416, 414, 412, 410, 408, 407, 405, 403, 401, 400, 398, 396, 394, 393, 391, 389, 387, 386, 384, 382, 381, 379, 377, 376, 374, 373, 371, 369, 368, 366, 365, 363, 361, 360, 358, 357, 355, 354, 352, 351, 349, 348, 347, 345, 344, 342, 341, 340, 338, 337, 335, 334, 333, 331, 330, 329, 328, 326, 325, 324, 323, 321, 320, 319, 318, 317, 316, 315, 313, 312, 311, 310, 309, 308, 307, 306, 305, 304, 303, 302, 301, 300, 300, 299, 298, 297, 296, 295, 294, 294, 293, 292, 291, 291, 290, 289, 289, 288, 287, 287, 286, 285, 285, 284, 284, 283, 283, 282, 282, 281, 281, 280, 280, 280, 279, 279, 278, 278, 278, 277, 277, 277, 277, 276, 276, 276, 276, 276, 276, 275, 275, 275, 275, 275, 275, 275, 275, 275, 275, 275, 275, 275, 275, 275, 276, 276, 276, 276, 276, 276, 277, 277, 277, 277, 278, 278, 278, 279, 279, 280, 280, 280, 281, 281, 282, 282, 283, 283, 284, 284, 285, 285, 286, 287, 287, 288, 289, 289, 290, 291, 291, 292, 293, 294, 294, 295, 296, 297, 298, 299, 300, 300, 301, 302, 303, 304, 305, 306, 307, 308, 309, 310, 311, 312, 313, 315, 316, 317, 318, 319, 320, 321, 323, 324, 325, 326, 328, 329, 330, 331, 333, 334, 335, 337, 338, 340, 341, 342, 344, 345, 347, 348, 349, 351, 352, 354, 355, 357, 358, 360, 361, 363, 365, 366, 368, 369, 371, 373, 374, 376, 377, 379, 381, 382, 384, 386, 387, 389, 391, 393, 394, 396, 398, 400, 401, 403, 405, 407, 408, 410, 412, 414, 416, 418, 419, 421, 423, 425, 427, 429, 430, 432, 434, 436, 438, 440, 442, 444, 446, 447, 449, 451, 453, 455, 457, 459, 461, 463, 465, 467, 469, 471, 473, 475, 476, 478, 480, 482, 484, 486, 488, 490, 492, 494, 496, 498
};
/*
from math import *
N=720
print(', '.join('%d' % round(1000*(1+0.65*sin(i/N*2*pi))/2) for i in range(N)))
*/
// TODO: Use proper FOC algorithm
const uint16_t sin_lookup_high_power[720] = {
500, 503, 506, 509, 511, 514, 517, 520, 523, 525, 528, 531, 534, 537, 540, 542, 545, 548, 551, 554, 556, 559, 562, 565, 568, 570, 573, 576, 579, 581, 584, 587, 590, 592, 595, 598, 600, 603, 606, 608, 611, 614, 616, 619, 622, 624, 627, 630, 632, 635, 637, 640, 642, 645, 648, 650, 653, 655, 658, 660, 662, 665, 667, 670, 672, 675, 677, 679, 682, 684, 686, 689, 691, 693, 696, 698, 700, 702, 705, 707, 709, 711, 713, 715, 717, 720, 722, 724, 726, 728, 730, 732, 734, 736, 738, 740, 742, 743, 745, 747, 749, 751, 753, 754, 756, 758, 760, 761, 763, 765, 766, 768, 769, 771, 773, 774, 776, 777, 779, 780, 781, 783, 784, 786, 787, 788, 790, 791, 792, 793, 795, 796, 797, 798, 799, 800, 801, 802, 803, 804, 805, 806, 807, 808, 809, 810, 811, 812, 812, 813, 814, 815, 815, 816, 817, 817, 818, 818, 819, 820, 820, 821, 821, 821, 822, 822, 823, 823, 823, 824, 824, 824, 824, 824, 825, 825, 825, 825, 825, 825, 825, 825, 825, 825, 825, 825, 825, 824, 824, 824, 824, 824, 823, 823, 823, 822, 822, 821, 821, 821, 820, 820, 819, 818, 818, 817, 817, 816, 815, 815, 814, 813, 812, 812, 811, 810, 809, 808, 807, 806, 805, 804, 803, 802, 801, 800, 799, 798, 797, 796, 795, 793, 792, 791, 790, 788, 787, 786, 784, 783, 781, 780, 779, 777, 776, 774, 773, 771, 769, 768, 766, 765, 763, 761, 760, 758, 756, 754, 753, 751, 749, 747, 745, 743, 742, 740, 738, 736, 734, 732, 730, 728, 726, 724, 722, 720, 717, 715, 713, 711, 709, 707, 705, 702, 700, 698, 696, 693, 691, 689, 686, 684, 682, 679, 677, 675, 672, 670, 667, 665, 662, 660, 658, 655, 653, 650, 648, 645, 642, 640, 637, 635, 632, 630, 627, 624, 622, 619, 616, 614, 611, 608, 606, 603, 600, 598, 595, 592, 590, 587, 584, 581, 579, 576, 573, 570, 568, 565, 562, 559, 556, 554, 551, 548, 545, 542, 540, 537, 534, 531, 528, 525, 523, 520, 517, 514, 511, 509, 506, 503, 500, 497, 494, 491, 489, 486, 483, 480, 477, 475, 472, 469, 466, 463, 460, 458, 455, 452, 449, 446, 444, 441, 438, 435, 432, 430, 427, 424, 421, 419, 416, 413, 410, 408, 405, 402, 400, 397, 394, 392, 389, 386, 384, 381, 378, 376, 373, 370, 368, 365, 363, 360, 358, 355, 352, 350, 347, 345, 342, 340, 337, 335, 333, 330, 328, 325, 323, 321, 318, 316, 314, 311, 309, 307, 304, 302, 300, 298, 295, 293, 291, 289, 287, 285, 283, 280, 278, 276, 274, 272, 270, 268, 266, 264, 262, 260, 258, 257, 255, 253, 251, 249, 247, 246, 244, 242, 240, 239, 237, 235, 234, 232, 231, 229, 227, 226, 224, 223, 221, 220, 219, 217, 216, 214, 213, 212, 210, 209, 208, 207, 205, 204, 203, 202, 201, 200, 199, 198, 197, 196, 195, 194, 193, 192, 191, 190, 189, 188, 188, 187, 186, 185, 185, 184, 183, 183, 182, 182, 181, 180, 180, 179, 179, 179, 178, 178, 177, 177, 177, 176, 176, 176, 176, 176, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 176, 176, 176, 176, 176, 177, 177, 177, 178, 178, 179, 179, 179, 180, 180, 181, 182, 182, 183, 183, 184, 185, 185, 186, 187, 188, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 207, 208, 209, 210, 212, 213, 214, 216, 217, 219, 220, 221, 223, 224, 226, 227, 229, 231, 232, 234, 235, 237, 239, 240, 242, 244, 246, 247, 249, 251, 253, 255, 257, 258, 260, 262, 264, 266, 268, 270, 272, 274, 276, 278, 280, 283, 285, 287, 289, 291, 293, 295, 298, 300, 302, 304, 307, 309, 311, 314, 316, 318, 321, 323, 325, 328, 330, 333, 335, 337, 340, 342, 345, 347, 350, 352, 355, 358, 360, 363, 365, 368, 370, 373, 376, 378, 381, 384, 386, 389, 392, 394, 397, 400, 402, 405, 408, 410, 413, 416, 419, 421, 424, 427, 430, 432, 435, 438, 441, 444, 446, 449, 452, 455, 458, 460, 463, 466, 469, 472, 475, 477, 480, 483, 486, 489, 491, 494, 497
};
/*
from math import *
N=720
print(', '.join('%d' % round(1000*(1+0.15*sin(i/N*2*pi))/2) for i in range(N)))
*/
// TODO: Automatically calibrate this
const uint16_t sin_lookup_low_power[720] = {
500, 501, 501, 502, 503, 503, 504, 505, 505, 506, 507, 507, 508, 508, 509, 510, 510, 511, 512, 512, 513, 514, 514, 515, 516, 516, 517, 518, 518, 519, 519, 520, 521, 521, 522, 523, 523, 524, 524, 525, 526, 526, 527, 527, 528, 529, 529, 530, 531, 531, 532, 532, 533, 533, 534, 535, 535, 536, 536, 537, 538, 538, 539, 539, 540, 540, 541, 541, 542, 542, 543, 544, 544, 545, 545, 546, 546, 547, 547, 548, 548, 549, 549, 550, 550, 551, 551, 552, 552, 553, 553, 553, 554, 554, 555, 555, 556, 556, 557, 557, 557, 558, 558, 559, 559, 560, 560, 560, 561, 561, 561, 562, 562, 563, 563, 563, 564, 564, 564, 565, 565, 565, 566, 566, 566, 567, 567, 567, 567, 568, 568, 568, 569, 569, 569, 569, 570, 570, 570, 570, 570, 571, 571, 571, 571, 572, 572, 572, 572, 572, 572, 573, 573, 573, 573, 573, 573, 573, 574, 574, 574, 574, 574, 574, 574, 574, 574, 575, 575, 575, 575, 575, 575, 575, 575, 575, 575, 575, 575, 575, 575, 575, 575, 575, 575, 575, 575, 575, 575, 575, 575, 575, 575, 575, 574, 574, 574, 574, 574, 574, 574, 574, 574, 573, 573, 573, 573, 573, 573, 573, 572, 572, 572, 572, 572, 572, 571, 571, 571, 571, 570, 570, 570, 570, 570, 569, 569, 569, 569, 568, 568, 568, 567, 567, 567, 567, 566, 566, 566, 565, 565, 565, 564, 564, 564, 563, 563, 563, 562, 562, 561, 561, 561, 560, 560, 560, 559, 559, 558, 558, 557, 557, 557, 556, 556, 555, 555, 554, 554, 553, 553, 553, 552, 552, 551, 551, 550, 550, 549, 549, 548, 548, 547, 547, 546, 546, 545, 545, 544, 544, 543, 542, 542, 541, 541, 540, 540, 539, 539, 538, 538, 537, 536, 536, 535, 535, 534, 533, 533, 532, 532, 531, 531, 530, 529, 529, 528, 527, 527, 526, 526, 525, 524, 524, 523, 523, 522, 521, 521, 520, 519, 519, 518, 518, 517, 516, 516, 515, 514, 514, 513, 512, 512, 511, 510, 510, 509, 508, 508, 507, 507, 506, 505, 505, 504, 503, 503, 502, 501, 501, 500, 499, 499, 498, 497, 497, 496, 495, 495, 494, 493, 493, 492, 492, 491, 490, 490, 489, 488, 488, 487, 486, 486, 485, 484, 484, 483, 482, 482, 481, 481, 480, 479, 479, 478, 477, 477, 476, 476, 475, 474, 474, 473, 473, 472, 471, 471, 470, 469, 469, 468, 468, 467, 467, 466, 465, 465, 464, 464, 463, 462, 462, 461, 461, 460, 460, 459, 459, 458, 458, 457, 456, 456, 455, 455, 454, 454, 453, 453, 452, 452, 451, 451, 450, 450, 449, 449, 448, 448, 447, 447, 447, 446, 446, 445, 445, 444, 444, 443, 443, 443, 442, 442, 441, 441, 440, 440, 440, 439, 439, 439, 438, 438, 437, 437, 437, 436, 436, 436, 435, 435, 435, 434, 434, 434, 433, 433, 433, 433, 432, 432, 432, 431, 431, 431, 431, 430, 430, 430, 430, 430, 429, 429, 429, 429, 428, 428, 428, 428, 428, 428, 427, 427, 427, 427, 427, 427, 427, 426, 426, 426, 426, 426, 426, 426, 426, 426, 425, 425, 425, 425, 425, 425, 425, 425, 425, 425, 425, 425, 425, 425, 425, 425, 425, 425, 425, 425, 425, 425, 425, 425, 425, 425, 425, 426, 426, 426, 426, 426, 426, 426, 426, 426, 427, 427, 427, 427, 427, 427, 427, 428, 428, 428, 428, 428, 428, 429, 429, 429, 429, 430, 430, 430, 430, 430, 431, 431, 431, 431, 432, 432, 432, 433, 433, 433, 433, 434, 434, 434, 435, 435, 435, 436, 436, 436, 437, 437, 437, 438, 438, 439, 439, 439, 440, 440, 440, 441, 441, 442, 442, 443, 443, 443, 444, 444, 445, 445, 446, 446, 447, 447, 447, 448, 448, 449, 449, 450, 450, 451, 451, 452, 452, 453, 453, 454, 454, 455, 455, 456, 456, 457, 458, 458, 459, 459, 460, 460, 461, 461, 462, 462, 463, 464, 464, 465, 465, 466, 467, 467, 468, 468, 469, 469, 470, 471, 471, 472, 473, 473, 474, 474, 475, 476, 476, 477, 477, 478, 479, 479, 480, 481, 481, 482, 482, 483, 484, 484, 485, 486, 486, 487, 488, 488, 489, 490, 490, 491, 492, 492, 493, 493, 494, 495, 495, 496, 497, 497, 498, 499, 499
};

// Increasing values correspond to clockwise rotation
static inline void drive_motor(uint32_t angle)
{
  TIM3->CCR1 = sin_lookup[(angle / 50000 +   0) % 720];
  TIM3->CCR2 = sin_lookup[(angle / 50000 + 240) % 720];
  TIM3->CCR3 = sin_lookup[(angle / 50000 + 480) % 720];
}

static inline void drive_motor_high_power(uint32_t angle)
{
  TIM3->CCR1 = sin_lookup_high_power[(angle / 50000 +   0) % 720];
  TIM3->CCR2 = sin_lookup_high_power[(angle / 50000 + 240) % 720];
  TIM3->CCR3 = sin_lookup_high_power[(angle / 50000 + 480) % 720];
}

static inline void drive_motor_low_power(uint32_t angle)
{
  TIM3->CCR1 = sin_lookup_low_power[(angle / 50000 +   0) % 720];
  TIM3->CCR2 = sin_lookup_low_power[(angle / 50000 + 240) % 720];
  TIM3->CCR3 = sin_lookup_low_power[(angle / 50000 + 480) % 720];
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

  // LED Blue, TIM17
  gpio_init.Pin = LED_OUT_B_PIN;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Alternate = GPIO_AF2_TIM17;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_OUT_B_PORT, &gpio_init);
  HAL_GPIO_WritePin(LED_OUT_B_PORT, LED_OUT_B_PIN, 1);
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
  TIM17->CCR1 = 0;
  HAL_TIM_OC_Init(&tim17);
  HAL_TIM_OC_Start_IT(&tim17, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&tim17);
  HAL_NVIC_SetPriority(TIM17_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM17_IRQn);

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
      .CounterMode = TIM_COUNTERMODE_UP,
      // .CounterMode = TIM_COUNTERMODE_CENTERALIGNED1,
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

  HAL_NVIC_SetPriority(I2C1_IRQn, 2, 0);
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

#define TESTRUN 0

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
    #if TESTRUN
      if (!(r1 == 0 && r2 == 0 && r3 == 0 && i2c2.ErrorCode == 0))
        for (int i = 0; i < 20; i++) {
          HAL_GPIO_WritePin(LED_IND_ACT_PORT, LED_IND_ACT_PIN, 1); HAL_Delay(50);
          HAL_GPIO_WritePin(LED_IND_ACT_PORT, LED_IND_ACT_PIN, 0); HAL_Delay(50);
        }
      return 0;
    #endif
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

#if TESTRUN
  // TIM17->CCR1 = 10; HAL_Delay(500);
  while (1) {
    uint16_t mag_value = read_magenc();
    swv_printf("mag encoder value = %u\n", (unsigned)mag_value);
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
#endif

  while (0) {
    for (int i = 0; i < 300; i++) {
      TIM17->CCR1 = i;
      HAL_Delay(1);
    }
    HAL_GPIO_WritePin(LED_IND_ACT_PORT, LED_IND_ACT_PIN, 1); HAL_Delay(200);
    HAL_GPIO_WritePin(LED_IND_ACT_PORT, LED_IND_ACT_PIN, 0);
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

  int rest_elec_angle = (int)(36000000 +
    (uint64_t)((rest_angle - elec_zero_angle + 4096) % 4096) * 36000000 * 7 / 4096);
  for (int i = 0; i < 1800; i++) {
    drive_motor(rest_elec_angle + i * 20000);
    for (int i = 0; i < 500; i++) asm volatile ("nop");
  }

  drive_motor_low_power(rest_elec_angle + 36000000);
  // TODO: Automatically calibrate minimum torque for low power drive

#define LED_TIMER_1     TIM17
#define LED_MAX_DUTY_1  5000
#define LED_TIMER_2     TIM14
#define LED_MAX_DUTY_2  0
#if 0
#define LED_TIMER_1     TIM14
#define LED_MAX_DUTY_1  10000
#define LED_TIMER_2     TIM16
#define LED_MAX_DUTY_2  5000
#endif

  while (1) {
    for (int i = 0; i < 5; i++) {
      HAL_GPIO_WritePin(LED_IND_ACT_PORT, LED_IND_ACT_PIN, 1); HAL_Delay(100);
      HAL_GPIO_WritePin(LED_IND_ACT_PORT, LED_IND_ACT_PIN, 0); HAL_Delay(100);
    }
    TIM3->CCR1 = TIM3->CCR2 = TIM3->CCR3 = 0;
    for (int i = 1800; i >= 0; i -= 2) {
      float x = i / 1800.f;
      x = (1 - (1 - x) * (1 - x) * (1 - x)) * 1800;
      drive_motor(36000000 + rest_elec_angle + x * 20000);
      for (int i = 0; i < 100; i++) asm volatile ("nop");
      x = x / 1800;
      uint32_t duty = (1 - (x < 0.25 ? cbrtf(x * 4) : 1)) * LED_MAX_DUTY_1;
      LED_TIMER_1->CCR1 = duty;
      LED_TIMER_2->CCR1 = duty * LED_MAX_DUTY_2 / LED_MAX_DUTY_1;
    }
    uint32_t wait_start_tick = HAL_GetTick();
    // 30 counts = 360 / 4096 * 30 deg = 2.64 deg
    while (((int)read_magenc() - rest_angle - 30 + 8192) % 4096 < 2048
      && HAL_GetTick() - wait_start_tick < 200
    ) {
      // Wait while position is greater (higher, more clockwise) than rest position
      // nop
    }
    uint32_t hit_tick = HAL_GetTick();
    LED_TIMER_1->CCR1 = LED_MAX_DUTY_1;
    // Raise after the free fall
    HAL_Delay(3);
    int cur_pos = read_magenc();
    // delta = rest_angle + 4096 / 7 - cur_pos
    int delta = (rest_angle + 4096 / 7 - cur_pos + 2048 + 4096) % 4096 - 2048;
    for (int i = 0; i < 500; i++) {
      float progress = (float)(i + 1) / 500;
      progress = 1 - (1 - progress) * (1 - progress) * (1 - progress);
      uint16_t target_angle = cur_pos + delta * progress;
      int elec_angle = (int)(36000000 +
        (uint64_t)((target_angle - elec_zero_angle + 4096) % 4096) * 36000000 * 7 / 4096);
      drive_motor(elec_angle);
      for (int i = 0; i < 1000; i++) asm volatile ("nop");
      // Fading LED
      uint32_t since_hit = HAL_GetTick() - hit_tick;
      if (since_hit < 250) LED_TIMER_1->CCR1 = LED_MAX_DUTY_1 - since_hit * LED_MAX_DUTY_1 / 250;
      else LED_TIMER_1->CCR1 = 0;
      if (since_hit < 250) LED_TIMER_2->CCR1 = LED_MAX_DUTY_2 - since_hit * LED_MAX_DUTY_2 / 250;
      else LED_TIMER_2->CCR1 = 0;
    }
    drive_motor_high_power(rest_elec_angle + 36000000);
    // HAL_Delay(300);
    uint32_t delay_start_tick = HAL_GetTick();
    while (HAL_GetTick() - delay_start_tick < 300) {
      uint32_t since_hit = HAL_GetTick() - hit_tick;
      if (since_hit < 250) LED_TIMER_1->CCR1 = LED_MAX_DUTY_1 - since_hit * LED_MAX_DUTY_1 / 250;
      else LED_TIMER_1->CCR1 = 0;
      if (since_hit < 250) LED_TIMER_2->CCR1 = LED_MAX_DUTY_2 - since_hit * LED_MAX_DUTY_2 / 250;
      else LED_TIMER_2->CCR1 = 0;
    }
    LED_TIMER_1->CCR1 = 0;
    drive_motor_low_power(rest_elec_angle + 36000000);
  }
}

void SysTick_Handler()
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();

  // if (HAL_GetTick() % 500 == 0)
  //   HAL_GPIO_WritePin(LED_IND_ACT_PORT, LED_IND_ACT_PIN, HAL_GetTick() % 1000 == 0);
}

// Issue: when OC pulse is too small, interrupts triggered by a period-elapse
// might fall into `HAL_TIM_OC_DelayElapsedCallback` first before `HAL_TIM_PeriodElapsedCallback`,
// creating a nearly-always-on PWM
// Here we reverse the order
// Macros `__HAL_TIM_GET_FLAG` and `__HAL_TIM_CLEAR_FLAG` are expanded
// to avoid the `tim17.Instance` indirection

void TIM17_IRQHandler()
{
  // HAL_TIM_IRQHandler(&tim17);
  uint32_t clear_mask = 0xffffffffu;
  bool on = false;
  // Period elapsed, and OC not triggered too fast (within a few cycles)?
  if (TIM17->SR & TIM_IT_UPDATE) {
    clear_mask &= ~TIM_IT_UPDATE;
    on = true;
  }
  // Output-compare delay elapsed?
  if (TIM17->SR & TIM_FLAG_CC1) {
    clear_mask &= ~(TIM_FLAG_CC1 | TIM_IT_UPDATE);
    on = false;
  }
  LED_OUT_B_PORT->BSRR = (uint32_t)LED_OUT_B_PIN << (on ? 16 : 0);
  TIM17->SR = clear_mask;
}

/*
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *tim)
{
  // assert(tim == &tim17);
  // HAL_GPIO_WritePin(LED_OUT_B_PORT, LED_OUT_B_PIN, TIM17->CCR1 == 0 ? 1 : 0);
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *tim)
{
  // assert(tim == &tim17);
  // HAL_GPIO_WritePin(LED_OUT_B_PORT, LED_OUT_B_PIN, 1);
}
*/

static uint8_t i2c_rx_buf[2];

void I2C1_IRQHandler()
{
  if (I2C1->ISR & (I2C_FLAG_BERR | I2C_FLAG_ARLO | I2C_FLAG_OVR)) {
    HAL_I2C_ER_IRQHandler(&i2c1);
  } else {
    HAL_I2C_EV_IRQHandler(&i2c1);
  }
}

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
void I2C2_IRQHandler() { while (1) { } }
void SPI1_IRQHandler() { while (1) { } }
void SPI2_IRQHandler() { while (1) { } }
void USART1_IRQHandler() { while (1) { } }
void USART2_IRQHandler() { while (1) { } }
