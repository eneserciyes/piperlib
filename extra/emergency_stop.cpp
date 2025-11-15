#include "piper_interface.h"

int main() {
  PiperInterface piper_interface("can0");
  piper_interface.set_emergency_stop(EmergencyStop::STOP);
  return 0;
}