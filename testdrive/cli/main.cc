// c++ % -ISimpleBLE/simpleble/include -Ibuild_simpleble/export build_simpleble/lib/libsimpleble.a -std=c++17 -framework CoreBluetooth -framework Foundation

#include "simpleble/SimpleBLE.h"

#include <cassert>
#include <cstdio>
#if defined(WIN32) || defined(_WIN32)
#include <windows.h>
#define sleep(x) Sleep(1000 * (x))
#else
#include <unistd.h>
#endif

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
      }
    }
  };
  adapter.set_callback_on_scan_found(upd_print);
  adapter.set_callback_on_scan_updated(upd_print);

  printf("Starting scan\n");
  adapter.scan_start();

  while (1) sleep(1);

  return 0;
}
