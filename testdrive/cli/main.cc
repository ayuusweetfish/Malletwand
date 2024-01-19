// c++ % -ISimpleBLE/simpleble/include -Ibuild_simpleble/export build_simpleble/lib/libsimpleble.a -Iraylib-5.0_macos/include raylib-5.0_macos/lib/libraylib.a -std=c++17 -framework CoreBluetooth -framework Foundation -framework CoreGraphics -framework IOKit -framework AppKit

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

std::mutex readings_mutex;
int16_t mag[3], acc[3], gyr[3];

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

  auto upd_print = [] (SimpleBLE::Peripheral p) {
    if (p.identifier() == "Mlt") {
      auto mfr_data = p.manufacturer_data();
      for (auto &[x, y] : mfr_data) {
        uint8_t payload[16];
        payload[0] = x & 0xff;
        payload[1] = (x >> 8) & 0xff;
        for (int i = 0; i < y.length(); i++) payload[i + 2] = (uint8_t)y[i];
        for (int i = 0; i < y.length() + 2; i++) printf(" %02x", payload[i]); putchar('\n');
        readings_mutex.lock();
        mag[0] = ((int16_t)payload[ 0] << 8) | payload[ 1];
        mag[1] = ((int16_t)payload[ 2] << 8) | payload[ 3];
        mag[2] = ((int16_t)payload[ 4] << 8) | payload[ 5];
        acc[0] = ((int16_t)payload[ 6] << 8) | payload[ 7];
        acc[1] = ((int16_t)payload[ 8] << 8) | payload[ 9];
        acc[2] = ((int16_t)payload[10] << 8) | payload[11];
        readings_mutex.unlock();
      }
    }
  };
  adapter.set_callback_on_scan_found(upd_print);
  adapter.set_callback_on_scan_updated(upd_print);

  printf("Starting scan\n");
  adapter.scan_start();

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

        auto plotPoint = [] (float x, float y, float z, rl::Color c) -> void {
          rl::Vector3 p = (rl::Vector3){x, -z, y};
          rl::Vector3 px = (rl::Vector3){x, 0, 0};
          rl::Vector3 py = (rl::Vector3){0, -z, 0};
          rl::Vector3 pz = (rl::Vector3){0, 0, y};
          rl::DrawSphere(p, 0.04, c);
          rl::DrawSphere(px, 0.02, c);
          rl::DrawSphere(py, 0.02, c);
          rl::DrawSphere(pz, 0.02, c);
          // rl::Vector3 pyz = (rl::Vector3){0, -z, y};
          // rl::Vector3 pzx = (rl::Vector3){x, 0, y};
          // rl::Vector3 pxy = (rl::Vector3){x, -z, 0};
          // rl::DrawLine3D(p, pxy, c);
          rl::DrawCubeWiresV((rl::Vector3){p.x/2, p.y/2, p.z/2}, p, c);
        };

        float accScale = 1.f / 8192;
        plotPoint(
          (float)acc[0] * accScale, (float)acc[1] * accScale, (float)acc[2] * accScale,
          (rl::Color){120, 60, 0, 255});

        float magScale = 1.f / 512;
        plotPoint(
          (float)mag[0] * magScale, (float)mag[1] * magScale, (float)mag[2] * magScale,
          (rl::Color){24, 20, 180, 255});
      rl::EndMode3D();
    rl::EndDrawing();
  }

  // while (1) sleep(1);
  rl::CloseWindow();
  adapter.scan_stop();

  return 0;
}
