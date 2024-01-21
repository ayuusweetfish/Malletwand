// c++ % -ISimpleBLE/simpleble/include -Ibuild_simpleble/export build_simpleble/lib/libsimpleble.a -Iraylib-5.0_macos/include raylib-5.0_macos/lib/libraylib.a -std=c++17 -framework CoreBluetooth -framework Foundation -framework CoreGraphics -framework IOKit -framework AppKit

#include "geom.h"
#include "elli_fit.h"

#include "simpleble/SimpleBLE.h"
namespace rl {
  #include "raylib.h"
}

#include <cassert>
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
int16_t mag_out[3], acc_out[3], gyr_out[3];
bool readings_updated = false;

int main() {
/*
  float A[3][3], c[3];
  vec3 x[] = {
    (vec3){-0.048768226,  0.506737186, -1.114967264},
    (vec3){-0.366845495,  0.752516022, -0.122585893},
    (vec3){ 0.071589973,  1.440691850, -0.870344969},
    (vec3){ 1.200876474,  0.776673265, -0.213629012},
    (vec3){-0.251914644,  0.337953796,  0.196076337},
    (vec3){ 1.170570723, -0.060363077,  0.435995064},
    (vec3){ 0.666124827,  0.707259657,  0.609291333},
    (vec3){ 0.613461620, -0.333568854,  0.593126059},
    (vec3){ 0.022064863,  0.593658273,  0.321310612},
    (vec3){-0.630838399,  0.761593886, -1.133984329},
  };
  elli_fit(sizeof x / sizeof x[0], x, 0.01, A, c);
  return 0;
*/
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
      if (mfr_data.size() != 1 || mfr_data.begin()->second.length() != 17) {
        count = -1;
        return -1;
      }
      SimpleBLE::ByteArray buf = mfr_data.begin()->second;
      uint8_t c = buf[buf.length() - 1];
      if (c == 'M' || c == 'l' || c == 't') {
        if (c != last) {
          count += 1;
          if (count >= 7) return true;
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

  auto upd_print = [&addrCheckers, &identifiedAddrs] (SimpleBLE::Peripheral p) {
    SimpleBLE::BluetoothAddress addr = p.address();
    auto mfr_data = p.manufacturer_data();
    int recordedIdentification = identifiedAddrs[addr];
    if (recordedIdentification == 0) {
      int result = addrCheckers[addr].check(mfr_data);
      if (result == -1) {
        identifiedAddrs[addr] = -1;
        addrCheckers.erase(addr);
      } else {
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
      for (int i = 0; i < y.length() + 2; i++) printf(" %02x", payload[i]); putchar('\n');
      readings_mutex.lock();
      mag_out[0] = ((int16_t)payload[ 0] << 8) | payload[ 1];
      mag_out[1] = ((int16_t)payload[ 2] << 8) | payload[ 3];
      mag_out[2] = ((int16_t)payload[ 4] << 8) | payload[ 5];
      acc_out[0] = ((int16_t)payload[ 6] << 8) | payload[ 7];
      acc_out[1] = ((int16_t)payload[ 8] << 8) | payload[ 9];
      acc_out[2] = ((int16_t)payload[10] << 8) | payload[11];
      gyr_out[0] = ((int16_t)payload[12] << 8) | payload[13];
      gyr_out[1] = ((int16_t)payload[14] << 8) | payload[15];
      gyr_out[2] = ((int16_t)payload[16] << 8) | payload[17];
      readings_updated = true;
      readings_mutex.unlock();
    }
  };
  adapter.set_callback_on_scan_found(upd_print);
  adapter.set_callback_on_scan_updated(upd_print);

  printf("Starting scan\n");
  adapter.scan_start();

  std::deque<vec3> mag_history;
  quat q_ref = (quat){0, 0, 0, 1};
  float m_tfm[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  vec3 m_cen = (vec3){0, 0, 0};

  const int W = 1200;
  const int H = 640;
  rl::InitWindow(W, H, "Malletwand test");
  rl::SetTargetFPS(60);

  rl::Camera3D camera = (rl::Camera3D){
    .position = (rl::Vector3){2, 1, 3},
    .target = (rl::Vector3){0, 0, 0},
    .up = (rl::Vector3){0, 1, 0},
    .fovy = 45,
    .projection = rl::CAMERA_PERSPECTIVE,
  };
  while (!rl::WindowShouldClose()) {
    rl::BeginDrawing();
      rl::ClearBackground((rl::Color){255, 254, 253, 255});

      if (rl::IsKeyDown(rl::KEY_LEFT_SHIFT) || rl::IsMouseButtonDown(rl::MOUSE_BUTTON_LEFT))
        rl::UpdateCamera(&camera, rl::CAMERA_THIRD_PERSON);

      readings_mutex.lock();
      vec3 acc = (vec3){(float)acc_out[0], (float)acc_out[1], (float)acc_out[2]};
      vec3 gyr = (vec3){(float)gyr_out[0], (float)gyr_out[1], (float)gyr_out[2]};
      vec3 mag = (vec3){(float)mag_out[0], (float)mag_out[1], (float)mag_out[2]};
      // vec3 mag = (vec3){(float)mag_out[0]+1234, (float)mag_out[1]*1.5f, (float)mag_out[2]};
      bool cur_readings_updated = readings_updated;
      readings_updated = false;
      readings_mutex.unlock();

      float magScale = 2.f / 1024;  // unit = 0.5 G = 0.05 mT ≈ geomagnetic field
      mag = vec3_scale(mag, magScale);

      vec3 acc_raw = acc;
      vec3 gyr_raw = gyr;
      vec3 mag_raw = mag;

      // Accelerometer and magnetometer calibration are disjoint
      // and can be carried out in any order
      if (rl::IsKeyPressed(rl::KEY_SPACE)) {
        q_ref = rot_from_endpoints(acc, (vec3){0, 0, 1});
      }
      if (rl::IsKeyPressed(rl::KEY_M)) {
        // Fit an ellipsoid
        vec3 scaled_mag[mag_history.size()];
        for (int i = 0; i < mag_history.size(); i++)
          scaled_mag[i] = quat_rot(q_ref, mag_history[i]);
        printf("Calibrating from %zu point(s)\n", mag_history.size());
        float c[3];
        elli_fit(mag_history.size(), scaled_mag, 0.01, m_tfm, c);
        m_cen = (vec3){c[0], c[1], c[2]};
        mag_history.clear();
      }
      acc = quat_rot(q_ref, acc_raw);
      gyr = quat_rot(q_ref, gyr_raw);
      mag = quat_rot(q_ref, mag_raw);
      mag = vec3_transform(m_tfm, vec3_diff(mag, m_cen));

      rl::BeginMode3D(camera);
        // maths (x, y, z) -> screen (x, -z, y)
        // screen (x, y, z) -> maths (x, z, -y)
        const float L = 3;
        rl::DrawLine3D((rl::Vector3){-L, 0, 0}, (rl::Vector3){L, 0, 0}, (rl::Color){180, 128, 128, 255});
        rl::DrawLine3D((rl::Vector3){0, 0, L}, (rl::Vector3){0, 0, -L}, (rl::Color){128, 180, 128, 255});
        rl::DrawLine3D((rl::Vector3){0, -L, 0}, (rl::Vector3){0, L, 0}, (rl::Color){128, 128, 180, 255});
        rl::DrawSphere((rl::Vector3){1, 0, 0}, 0.015, (rl::Color){180, 128, 128, 255});
        rl::DrawSphere((rl::Vector3){0, 0, -1}, 0.015, (rl::Color){128, 180, 128, 255});
        rl::DrawSphere((rl::Vector3){0, 1, 0}, 0.015, (rl::Color){128, 128, 180, 255});
        rl::DrawCube((rl::Vector3){-1, 0, 0}, 0.005, 0.012, 0.012, (rl::Color){180, 128, 128, 255});
        rl::DrawCube((rl::Vector3){0, 0, 1}, 0.012, 0.012, 0.005, (rl::Color){128, 180, 128, 255});
        rl::DrawCube((rl::Vector3){0, -1, 0}, 0.012, 0.005, 0.012, (rl::Color){128, 128, 180, 255});
        // rl::DrawSphereWires(
        //   (rl::Vector3){0, 0, 0}, 1,
        //   24, 24, (rl::Color){255, 0, 0, 128});
        // rl::DrawGrid(10, 1);

        auto plotPoint = [] (vec3 v, rl::Color c, bool ref = true) -> void {
          float x = v.x, y = v.y, z = v.z;
          rl::Vector3 p = (rl::Vector3){x, z, -y};
          rl::DrawSphere(p, 0.04, c);
          if (ref) {
            rl::Vector3 px = (rl::Vector3){x, 0, 0};
            rl::Vector3 py = (rl::Vector3){0, z, 0};
            rl::Vector3 pz = (rl::Vector3){0, 0, -y};
            rl::DrawSphere(px, 0.02, c);
            rl::DrawSphere(py, 0.02, c);
            rl::DrawSphere(pz, 0.02, c);
            rl::DrawCubeWiresV((rl::Vector3){p.x/2, p.y/2, p.z/2}, p, c);
          }
        };

        float accScale = 1.f / 8192;  // unit = 1g
        plotPoint(
          vec3_scale(acc, accScale),
          (rl::Color){120, 60, 0, 255});

        float gyrScale = 1.f / (16.384 * 360);  // unit = 1 rps
        plotPoint(
          vec3_scale(gyr, gyrScale),
          (rl::Color){150, 20, 210, 255});

        if (cur_readings_updated) mag_history.push_back(mag_raw);
        if (mag_history.size() >= 500) mag_history.pop_front();
        if (0) for (const vec3 p : mag_history) {
          plotPoint(
            vec3_transform(m_tfm, vec3_diff(quat_rot(q_ref, p), m_cen)),
            (rl::Color){24, 20, 180, 255}, false);
        }
        plotPoint(
          mag,
          (rl::Color){24, 20, 180, 255});
      rl::EndMode3D();
    rl::EndDrawing();
  }

  // while (1) sleep(1);
  rl::CloseWindow();
  adapter.scan_stop();

  return 0;
}
