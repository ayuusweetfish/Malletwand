#include <math.h>
#include <stdint.h>
#include <stdio.h>

struct filter {
  float i1, i2;
};

static inline float filter_update(
  struct filter *f,
  float th_a, float dth_g
) {
  float x1 = f->i2 - th_a;
  f->i1 += x1 * 1.f/100;
  const float omega = 3.0f;
  float x2 = 1.0f * omega * f->i1 + 1.414f * omega * x1;
  float x3 = dth_g - x2;
  f->i2 += x3 * 1.f/100;
  return f->i2;
}

static inline uint32_t myrand()
{
  static uint32_t seed = 240109;
  return (seed = (seed * 1103515245 + 12345) & 0x7fffffff);
}

int main()
{
  struct filter f = { 0 };
  for (int i = 0; i < 5000; i++) {
    float t = (float)i / 100;
    float freq_Hz = 0.1f;
    // 30% amplitude high-frequency white noise
    float anoise = (float)((int32_t)myrand() % 88888 - 44444) / 44444 * 3.0;
    // Low-frequency noise
    static float gnoise_arr[1000] = { 0 }, gnoise_sum = 0;
    static int gnoise_ptr = 0;
    gnoise_sum -= gnoise_arr[gnoise_ptr];
    gnoise_arr[gnoise_ptr] = (float)((int32_t)myrand() % 88888 - 44444) / 44444 * 3.0;
    gnoise_sum += gnoise_arr[gnoise_ptr];
    gnoise_ptr = (gnoise_ptr + 1) % 1000;
    float gnoise = gnoise_sum / 300 * 10;
    float a = 9.99 * sinf(t * 6.2832 * freq_Hz) + anoise;
    float g = 9.99 * cosf(t * 6.2832 * freq_Hz) * (6.2832 * freq_Hz) + gnoise;
    float th = filter_update(&f, a, g);
    printf("%4d %9.5f | %9.5f (%8.5f) %9.5f (%8.5f) | %8.5f\n", i, th,
      a, anoise, g, gnoise, 9.99 * sinf(t * 6.2832 * freq_Hz));
  }

  return 0;
}
