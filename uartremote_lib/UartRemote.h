#pragma once
#include <Arduino.h>
#include <vector>
#include <deque>

/* =========================================================
   Types
   ========================================================= */

enum MRType {
  MR_INT = 78,
  MR_BYTES = 65,
  MR_STR = 83,
  MR_BOOL = 66
};

struct MRArg {
  MRType type;
  std::vector<uint8_t> data;
};

struct MRPacket {
  String cmd;
  std::vector<MRArg> args;
};

/* =========================================================
   Callback Type
   ========================================================= */

using MRHandler =
  std::vector<MRArg> (*)(const String &,
                         const std::vector<MRArg> &);

/* =========================================================
   UartRemote
   ========================================================= */

class UartRemote {
public:
  UartRemote(Stream &s, MRHandler cb);

  void process();
  void queue(const char *cmd,
             const std::vector<MRArg> &args);

  static MRArg MR_int(int v);
  static int to_int(const MRArg &a);
  static bool to_bool(const MRArg &a);

private:
  Stream &io;
  MRHandler handler_cb;

  enum State {
    WAIT_LEN,
    WAIT_PREAMBLE,
    WAIT_PAYLOAD
  };

  State state = WAIT_LEN;
  uint8_t expected_len = 0;
  uint8_t pre_idx = 0;

  std::vector<uint8_t> buf;
  std::deque<MRPacket> rxq;
  std::deque<std::vector<uint8_t>> txq;

  void pollRX();
  void pollTX();
  void decode(const std::vector<uint8_t> &in);
  std::vector<uint8_t> encode(const char *cmd,
                              const std::vector<MRArg> &args);
};