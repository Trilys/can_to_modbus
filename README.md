# can_to_modbus
Simple interface CAN to modbus for linux OS.

First step, you need to install libmodbus (https://github.com/stephane/libmodbus). You also may need to install can-utils (https://github.com/linux-can/can-utils).

If you want to compile can_to_modbus : 
  gcc can_min.c lib_can.c can_to_modbus.c -o can_to_modbus -lpthread `pkg-config --libs --cflags libmodbus`

You can add debug with -D DEBUG option and use virtual CAN with -D VCAN option in the gcc command.
