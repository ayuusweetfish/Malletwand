# ~/.platformio/packages/tool-openocd/bin/openocd -f interface/stlink.cfg -f target/stm32l0.cfg -d1
# (cd ~/Downloads/stlink; build/Release/bin/st-info --probe)

# ~/.platformio/packages/tool-openocd/bin/openocd -f interface/stlink.cfg -f target/stm32l0.cfg -c 'init; reset halt; stm32l0x unlock 0; reset halt; exit'
# https://github.com/stlink-org/stlink/issues/705

cat >debug/gdbinit <<EOF
define hook-quit
  set confirm off
end
define hook-run
  set confirm off
end
define hookpost-run
  set confirm on
end
set pagination off
target extended-remote localhost:3333

b swv_trap_line
commands
  silent
  printf "%s\n", (char *)swv_buf
  c
end
r
EOF

~/.platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-gdb .pio/build/dev/firmware.elf -x debug/gdbinit
rm debug/gdbinit
