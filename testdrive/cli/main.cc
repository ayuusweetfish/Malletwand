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
int16_t mag_out[3], acc_out[3], gyr_out[3];
bool readings_updated = false;

struct filter {
  float i1, i2;
};

static inline float filter_update(
  struct filter *f,
  float th, float thd
) {
  if (th < f->i2 - M_PI) th += M_PI * 2;
  if (th > f->i2 + M_PI) th -= M_PI * 2;
  float x1 = f->i2 - th;
  f->i1 += x1 * 1.f/100;
  const float omega = 24.0f;
  float x2 = 1.0f * omega * f->i1 + 1.414f * omega * x1;
  float x3 = thd - x2;
  f->i2 += x3 * 1.f/100;
  if (f->i2 >  M_PI) f->i2 -= M_PI * 2;
  if (f->i2 < -M_PI) f->i2 += M_PI * 2;
  return f->i2;
}

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
  // Remove history records that are more than 1 second old
  while (btex->hist_head != btex->hist_tail
      && btex->history[btex->hist_tail][0] < time - 1)
    btex->hist_tail = (btex->hist_tail + 1) % BTEX_HIST_SIZE;
  int last = (btex->hist_head + BTEX_HIST_SIZE - 1) % BTEX_HIST_SIZE;
  // Ordinary least squares fit, Ak = y => k = A'y/A'A
  float AdotA = 0, AdotY = 0;
  for (int i = btex->hist_tail; i != btex->hist_head; i = (i + 1) % BTEX_HIST_SIZE) {
    float a = btex->history[i][0] - btex->history[last][0];
    float y = btex->history[i][1] - btex->history[last][1];
    AdotA += a * a;
    AdotY += a * y;
  }
  if (AdotA == 0) return btex->history[last][1];
  float k = AdotY / AdotA;
  return k * (time - btex->history[last][0]) + btex->history[last][1];
}

int main() {
  struct beat_extrapolator btex = { 0 };
  const float records[][2] = {
    {5.5, 0.40},
    {5.7, 0.50},
    {5.9, 0.70},
    {6.1, 0.66},
    {6.3, 0.82},
    {6.5, 0.91},
    {6.7, 1.03},
    {6.9, 1.12},
    {7.1, 1.22},
    {7.3, 1.35},
    {7.5, 1.44},
    {7.7, 1.51},
    {7.9, 1.60},
    {8.1, 1.70},
    {8.3, 1.75},
    {8.5, 1.87},
  };
  for (int i = 0; i < sizeof records / sizeof records[0]; i++) {
    btex_update(&btex, records[i][0], records[i][1]);
    printf("upd %.2f: %.2f -> extrap %.2f: %.5f\n",
      records[i][0], records[i][1],
      records[i][0] + 0.2, btex_estimate(&btex, records[i][0] + 0.2));
  }
  return 0;
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
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      printf("%9.5f%c", A[i][j], j == 2 ? '\n' : ' ');
  for (int i = 0; i < 3; i++) printf("%9.5f%c", c[i], i == 2 ? '\n' : ' ');
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
          if (count >= 5) return true;
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
      // for (int i = 0; i < y.length() + 2; i++) printf(" %02x", payload[i]); putchar('\n');
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
      if ((mag_out[0] != 0x7fff || mag_out[1] != 0x7fff || mag_out[2] != 0x7fff)
        && (acc_out[0] != 0x0000 || acc_out[1] != 0x0000 || acc_out[2] != 0x0000
         || gyr_out[0] != 0x0000 || gyr_out[1] != 0x0000 || gyr_out[2] != 0x0000)
      )
        readings_updated = true;
      else puts("Ignoring invalid data at startup");
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

  float ekf_x[5] = {40, 1, 0, 1, 0};
  float ekf_P[5][5] = {
    {30, 0, 0, 0, 0},
    {0, 1, 0, 0, 0},
    {0, 0, 4, 0, 0},
    {0, 0, 0, 1, 0},
    {0, 0, 0, 0, 4},
  };

  bool disp_verbose = true;

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

      if (rl::IsKeyPressed(rl::KEY_TAB))
        disp_verbose ^= 1;

      readings_mutex.lock();
      vec3 gyr = (vec3){(float)gyr_out[0], (float)gyr_out[1], (float)gyr_out[2]};
    #ifndef DIRECT_OUTPUT
      vec3 acc = (vec3){(float)acc_out[0], (float)acc_out[1], (float)acc_out[2]};
      vec3 mag = (vec3){(float)mag_out[0], (float)mag_out[1], (float)mag_out[2]};
    #else
      vec3 acc = (vec3){0, 0, 100};
      vec3 mag = (vec3){100, 0, 0};
    #endif
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

      vec3 mag_upright_g = quat_rot(rot_from_endpoints(acc, (vec3){0, 0, 1}), mag);

      static float xy_ori_filtered = 0;
      if (cur_readings_updated) {
        float xy_ori_d = gyr.z / (16.384 * 180) * M_PI;
        float xy_ori = -atan2f(mag_upright_g.y, mag_upright_g.x);
        static struct filter xy_f;
        xy_ori_filtered = filter_update(&xy_f, xy_ori, xy_ori_d);

        xy_ori_filtered = 0;
        vec3 gyr_calibrated =
          vec3_scale(vec_rot(gyr, (vec3){0, 0, 1}, xy_ori_filtered), 1.f / (16.384 * 360));
        float z[2] = {gyr_calibrated.x, gyr_calibrated.y};
        ekf_step(ekf_x, ekf_P, z);
        /* printf("EKF step %.6f %.6f | gyr %.6f %.6f\n",
          gyr_calibrated.x, gyr_calibrated.y,
          gyr.x / (16.384 * 360), gyr.y / (16.384 * 360)); */
        // printf("%.7f\n", ekf_x[2]);
      }

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
        auto plotLine = [] (vec3 a, vec3 b, rl::Color c) -> void {
          rl::Vector3 pa = (rl::Vector3){a.x, a.z, -a.y};
          rl::Vector3 pb = (rl::Vector3){b.x, b.z, -b.y};
          rl::DrawLine3D(pa, pb, c);
        };

        float accScale = 1.f / 8192;  // unit = 1g
        /*
        plotPoint(
          vec3_scale(acc, accScale),
          (rl::Color){120, 60, 0, 255});
        */

        float gyrScale = 1.f / (16.384 * 360);  // unit = 1 rps
        if (disp_verbose) {
          plotPoint(
            vec3_scale(gyr, gyrScale),
            (rl::Color){150, 20, 210, 255});
        }

        if (cur_readings_updated) mag_history.push_back(mag_raw);
        if (mag_history.size() >= 500) mag_history.pop_front();
        if (0) for (const vec3 p : mag_history) {
          plotPoint(
            vec3_transform(m_tfm, vec3_diff(quat_rot(q_ref, p), m_cen)),
            (rl::Color){24, 20, 180, 255}, false);
        }
        if (disp_verbose) {
          /*
          plotPoint(
            mag,
            (rl::Color){24, 20, 180, 255});
          plotPoint(
            mag_upright_g,
            (rl::Color){24, 60, 150, 255});
          */
          plotLine(
            (vec3){0, 0, 0},
            (vec3){cosf(xy_ori_filtered), sinf(xy_ori_filtered), 0},
            (rl::Color){24, 20, 180, 255});
        }

        float P_F = 0;
        for (int i = 0; i < 5; i++)
          for (int j = 0; j < 5; j++)
            P_F += ekf_P[i][j] * ekf_P[i][j];
        // printf("%10.7f ", sqrtf(P_F));

        float ω = ekf_x[0];
        float A = ekf_x[1];
        float θ = ekf_x[2];
        float B = ekf_x[3];
        float ϕ = ekf_x[4];
        float δ = atan2(A*A + B*B * cosf(2*ϕ), -B*B * sinf(2*ϕ));
        float cen_phase = -0.5 * δ + 0.25 * M_PI;
        // printf("%.7f %.7f %.7f\n", θ, cen_phase, P_F);
      #ifndef DIRECT_OUTPUT
        printf("%10.7f\n", cen_phase);
      #else
        float music_phase =
          (float)(((int32_t)mag_out[0] << 16) | (int32_t)(uint16_t)mag_out[1]) / 1000000;
          // (float)mag_out[0] * 65536 / 1000000;
        float raw_B = 0;  // (float)mag_out[1] / 10000;
        float raw_A = (float)mag_out[2] / 10000;
        float raw_omega = (float)acc_out[0] / 1000;
        float raw_theta = (float)acc_out[1] / 1000;
        float raw_phi = (float)acc_out[2] / 1000;
        printf("(Device) %10.7f %10.7f | omega=%7.4f A=%9.6f theta=%7.4f B=%9.6f phi=%7.4f\n",
          fmodf(cen_phase - θ + M_PI * 4, M_PI * 2),
          fmodf(music_phase + M_PI * 4, M_PI * 2),
          raw_omega,
          raw_A,
          raw_theta,
          raw_B,
          raw_phi
        );
        printf("(Local)                        | omega=%7.4f A=%9.6f theta=%7.4f B=%9.6f phi=%7.4f\n",
          ω, A, θ, B, ϕ);
      #endif
        /*
        plotLine(
          (vec3){0, 0, 0},
          (vec3){cosf(θ) * 2, sinf(θ) * 2, 0},
          (rl::Color){220, 70, 40, 255});
        plotLine(
          (vec3){cosf(cen_phase) *-2, sinf(cen_phase) *-2, 0},
          (vec3){cosf(cen_phase) * 2, sinf(cen_phase) * 2, 0},
          (rl::Color){160, 40, 40, 255});
        */
        plotPoint((vec3){ 1.2, 0, 0}, (rl::Color){160, 160, 160, 255}, false);
        plotPoint((vec3){-1.2, 0, 0}, (rl::Color){160, 160, 160, 255}, false);
        plotLine(
          (vec3){cosf(cen_phase - θ) *-2, sinf(cen_phase - θ) *-2, 0},
          (vec3){cosf(cen_phase - θ) * 2, sinf(cen_phase - θ) * 2, 0},
      #ifndef DIRECT_OUTPUT
          (rl::Color){160, 40, 40, 255});
        plotPoint(
          (vec3){cosf(cen_phase - θ) * 1.2f, sinf(cen_phase - θ) * 1.2f, 0},
          (rl::Color){160, 40, 40, 255}, false);
      #else
          (rl::Color){160, 40, 40, 20});
        plotLine(
          (vec3){cosf(music_phase) *-2, sinf(music_phase) *-2, 0},
          (vec3){cosf(music_phase) * 2, sinf(music_phase) * 2, 0},
          (rl::Color){180, 80, 80, 255});
        plotPoint(
          (vec3){raw_A * cosf(raw_theta), raw_B * cosf(raw_theta + raw_phi), 0},
          (rl::Color){0, 80, 255, 255});
      #endif

        if (disp_verbose) {
          plotPoint(
            (vec3){A * cosf(θ), B * cosf(θ + ϕ), 0},
            (rl::Color){255, 0, 255, 255});

          plotPoint(
            vec3_scale(vec_rot(acc, (vec3){0, 0, 1}, xy_ori_filtered), accScale),
            (rl::Color){230, 120, 60, 255});
          plotPoint(
            vec3_scale(vec_rot(gyr, (vec3){0, 0, 1}, xy_ori_filtered), gyrScale),
            (rl::Color){200, 70, 240, 255});
          if (0 && cur_readings_updated) {
            vec3 gyr_calibrated =
              vec3_scale(vec_rot(gyr, (vec3){0, 0, 1}, xy_ori_filtered), gyrScale);
            printf("%.8f %.8f %.8f\n",
              vec3_scale(vec_rot(acc, (vec3){0, 0, 1}, xy_ori_filtered), accScale).z,
              gyr_calibrated.x, gyr_calibrated.y
            );
          }
        }
      rl::EndMode3D();
    rl::EndDrawing();
  }

  // while (1) sleep(1);
  rl::CloseWindow();
  adapter.scan_stop();

  return 0;
}
