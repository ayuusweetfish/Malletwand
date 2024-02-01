// c++ % -ISimpleBLE/simpleble/include -Ibuild_simpleble/export build_simpleble/lib/libsimpleble.a -Iraylib-5.0_macos/include raylib-5.0_macos/lib/libraylib.a -std=c++17 -framework CoreBluetooth -framework Foundation -framework CoreGraphics -framework IOKit -framework AppKit

#include "geom.h"
#include "elli_fit.h"
#include "ekf.h"

#include "simpleble/SimpleBLE.h"
namespace rl {
  #include "raylib.h"
}

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdint>
#if defined(WIN32) || defined(_WIN32)
#include <windows.h>
#define sleep(x) Sleep(1000 * (x))
#else
#include <unistd.h>
#endif

#include <mutex>
#include <queue>
#include <set>

std::mutex readings_mutex;
float music_phase, uncertainty;
float omega;

#define BTEX_HIST_SIZE 64
struct beat_extrapolator {
  // Circular queue
  float history[BTEX_HIST_SIZE][2];
  int hist_tail, hist_head;
};
static inline void btex_update(beat_extrapolator *btex, float time, float value)
{
  btex->history[btex->hist_head][0] = time;
  btex->history[btex->hist_head][1] = value;
  btex->hist_head = (btex->hist_head + 1) % BTEX_HIST_SIZE;
  if (btex->hist_head == btex->hist_tail)
    btex->hist_tail = (btex->hist_tail + 1) % BTEX_HIST_SIZE;
}
static inline float btex_estimate(beat_extrapolator *btex, float time)
{
  int last = (btex->hist_head + BTEX_HIST_SIZE - 1) % BTEX_HIST_SIZE;
  float last_time = btex->history[last][0] - 0.1f;
  float last_value = btex->history[last][1];
  // Remove history records that are more than 1 second old
  while (btex->hist_head != btex->hist_tail
      && btex->history[btex->hist_tail][0] < last_time - 1.0f)
    btex->hist_tail = (btex->hist_tail + 1) % BTEX_HIST_SIZE;
  // Weight least squares
  const float scale = sqrtf(0.9f);
  float w;
  int i;
  float sum_aa = 0, sum_aw = 0, sum_ww = 0;
  w = 1;
  i = btex->hist_head;
  while (i != btex->hist_tail) {
    i = (i + BTEX_HIST_SIZE - 1) % BTEX_HIST_SIZE;
    float a = btex->history[i][0] * w;
    sum_aa += a * a;
    sum_aw += a * w;
    sum_ww += w * w;
    w *= scale;
  }
  // inv(A' A) = [sum_ww, -sum_aw; -sum_aw, sum_aa] / denom
  float denom = -sum_aw * sum_aw + sum_aa * sum_ww;
  // inv(A' A) A' y
  float b1 = 0, b2 = 0;
  w = 1;
  i = btex->hist_head;
  while (i != btex->hist_tail) {
    i = (i + BTEX_HIST_SIZE - 1) % BTEX_HIST_SIZE;
    float a = btex->history[i][0] * w;
    float y = btex->history[i][1] * w;
    b1 += y * (a *  sum_ww + w * -sum_aw);
    b2 += y * (a * -sum_aw + w *  sum_aa);
    w *= scale;
  }
  float estimated = (b1 * time + b2) / denom;
  // Transform to suppress far-stretching estimations
  float x = estimated - last_value;
  if (x > 0.5f) x = 1 - 0.5f * expf(1 - 2 * x);
  return x + btex->history[last][1];
}

int main() {
  if (!SimpleBLE::Adapter::bluetooth_enabled()) {
    printf("Bluetooth not enabled\n");
    puts("Press Enter to exit"); getchar();
    return 1;
  }

  auto adapters = SimpleBLE::Adapter::get_adapters();
  if (adapters.empty()) {
    printf("No Bluetooth adapter found\n");
    puts("Press Enter to exit"); getchar();
    return 1;
  }
  auto adapter = adapters[0];

  struct hitChecker {
    int16_t count;
    uint8_t last;
    hitChecker() : count(0), last(0) { }
    // -1: invalid; 0: pending; 1: accepted
    inline int check(std::map<uint16_t, SimpleBLE::ByteArray> mfr_data) {
      if (count == -1) return -1;
      if (mfr_data.size() != 1 /* || mfr_data.begin()->second.length() != 11 - 2 */) {
        count = -1;
        return -1;
      }
      SimpleBLE::ByteArray buf = mfr_data.begin()->second;
      uint8_t c = buf[buf.length() - 1];
      if (c == 'M' || c == 'l' || c == 't') {
        if (c != last) {
          count += 1;
          if (count >= 5) return 1;
          last = c;
        }
        return 0;
      } else {
        count = -1;
        return -1;
      }
    }
  };
  std::map<SimpleBLE::BluetoothAddress, hitChecker> addrCheckers;
  std::map<SimpleBLE::BluetoothAddress, int> identifiedAddrs;

  struct beat_extrapolator btex = { 0 };

  auto upd_print = [&addrCheckers, &identifiedAddrs] (SimpleBLE::Peripheral p) {
    SimpleBLE::BluetoothAddress addr = p.address();
    auto mfr_data = p.manufacturer_data();
    int recordedIdentification = identifiedAddrs[addr];
    if (recordedIdentification == 0) {
      int result = addrCheckers[addr].check(mfr_data);
      if (result == -1) {
        identifiedAddrs[addr] = -1;
        addrCheckers.erase(addr);
      } else if (result == 1) {
        printf("Identified device: %s\n", addr.c_str());
        recordedIdentification = identifiedAddrs[addr] = 1;
      }
    }
    if (recordedIdentification == 1) {
      auto [x, y] = *mfr_data.begin();
      uint8_t payload[18];
      payload[0] = x & 0xff;
      payload[1] = (x >> 8) & 0xff;
      for (int i = 0; i < y.length(); i++) payload[i + 2] = (uint8_t)y[i];
      // for (int i = 0; i < y.length() + 2; i++) printf(" %02x", payload[i]); putchar('\n');
      readings_mutex.lock();
      music_phase = (float)(
        ((uint32_t)payload[0] << 24) |
        ((uint32_t)payload[1] << 16) |
        ((uint32_t)payload[2] <<  8) |
        ((uint32_t)payload[3] <<  0)
      ) / 1000000;
      uncertainty = (float)(
        ((uint32_t)payload[4] << 24) |
        ((uint32_t)payload[5] << 16) |
        ((uint32_t)payload[6] <<  8) |
        ((uint32_t)payload[7] <<  0)
      ) / 1000000;
      omega = (float)(
        ((uint32_t)payload[8] << 24) |
        ((uint32_t)payload[9] << 16) |
        ((uint32_t)payload[10] <<  8) |
        ((uint32_t)payload[11] <<  0)
      ) / 1000000;
      readings_mutex.unlock();
    }
  };
  adapter.set_callback_on_scan_found(upd_print);
  adapter.set_callback_on_scan_updated(upd_print);

  printf("Starting scan\n");
  adapter.scan_start();

  bool disp_verbose = true;

  const int W = 1200;
  const int H = 640;
  rl::InitWindow(W, H, "Malletwand test");
  rl::SetTargetFPS(60);

  while (!rl::WindowShouldClose()) {
    rl::BeginDrawing();
      rl::ClearBackground((rl::Color){255, 254, 253, 255});

      readings_mutex.lock();
      float R = 100;
      rl::DrawLineEx(
        (rl::Vector2){W/2, H/2},
        (rl::Vector2){W/2 + R * cosf(music_phase), H/2 + R * sinf(music_phase)},
        5,
        (rl::Color){64, 64, 64, 255}
      );
      float L = 50;
      for (int i = 0; i < 5; i++) {
        float y = H*0.7 - i * L;
        rl::DrawLineEx(
          (rl::Vector2){W*0.65 - 20, y},
          (rl::Vector2){W*0.65 + 20, y},
          3,
          (rl::Color){192, 192, 192, 255}
        );
      }
      rl::DrawLineEx(
        (rl::Vector2){W*0.65, H*0.7f},
        (rl::Vector2){W*0.65, H*0.7f - uncertainty * L},
        5,
        (rl::Color){64, 64, 64, 255}
      );
      printf("%.7f\n", omega);
      readings_mutex.unlock();
    rl::EndDrawing();
  }

  rl::CloseWindow();
  adapter.scan_stop();

  return 0;
}
