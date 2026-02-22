#include <Arduino.h>
#include "UartRemote.h"

std::vector<MRArg>
app_handler(const String& cmd,
            const std::vector<MRArg>& args)
{
  if (cmd == "ping") {
    return { UartRemote::MR_int(millis()) };
  }

  if (cmd == "add") {
    if (args.size() >= 2) {
      int a = UartRemote::to_int(args[0]);
      int b = UartRemote::to_int(args[1]);
      return { UartRemote::MR_int(a + b) };
    }
  }

  if (cmd == "led") {
    if (args.size() >= 1) {
      digitalWrite(PB2,
                   UartRemote::to_bool(args[0]));
      return { UartRemote::MR_int(1) };
    }
  }

  return { UartRemote::MR_int(-0x3fffffff) };
}