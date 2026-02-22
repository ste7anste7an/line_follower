#include "UartRemote.h"

#define PREAMBLE0 '<'
#define PREAMBLE1 '$'
#define PREAMBLE2 'M'
#define PREAMBLE3 'U'

/* =========================================================
   Constructor
   ========================================================= */

UartRemote::UartRemote(Stream &s, MRHandler cb)
  : io(s), handler_cb(cb) {
}

/* =========================================================
   Utility Functions
   ========================================================= */

MRArg UartRemote::MR_int(int v) {
  MRArg a;
  a.type = MR_INT;

  char buf[16];
  itoa(v, buf, 10);

  a.data.assign(buf, buf + strlen(buf));
  return a;
}

int UartRemote::to_int(const MRArg &a) {
  int value = 0;
  bool neg = false;

  for (size_t i = 0; i < a.data.size(); i++) {
    char c = a.data[i];

    if (i == 0 && c == '-') {
      neg = true;
      continue;
    }

    if (c < '0' || c > '9')
      break;

    value = value * 10 + (c - '0');
  }

  return neg ? -value : value;
}

bool UartRemote::to_bool(const MRArg &a) {
  return (a.data.size() > 0) && (a.data[0] != 0);
}

/* =========================================================
   RX State Machine
   ========================================================= */

void UartRemote::pollRX() {
  while (io.available() > 0) {
    uint8_t b = io.read();

    switch (state) {
      case WAIT_LEN:
        expected_len = b;
        buf.clear();
        pre_idx = 0;
        state = WAIT_PREAMBLE;
        break;

      case WAIT_PREAMBLE:
        if ((pre_idx == 0 && b != PREAMBLE0) || (pre_idx == 1 && b != PREAMBLE1) || (pre_idx == 2 && b != PREAMBLE2) || (pre_idx == 3 && b != PREAMBLE3)) {
          state = WAIT_LEN;
          break;
        }

        pre_idx++;

        if (pre_idx == 4)
          state = WAIT_PAYLOAD;

        break;

      case WAIT_PAYLOAD:
        buf.push_back(b);

        if (buf.size() == expected_len - 4) {
          decode(buf);
          state = WAIT_LEN;
        }
        break;
    }
  }
} 

/* =========================================================
   TX
   ========================================================= */

void UartRemote::pollTX() {
  if (txq.empty())
    return;

  auto &frame = txq.front();

  io.write(frame.data(), frame.size());

  txq.pop_front();
}

/* =========================================================
   Decode Packet
   ========================================================= */

void UartRemote::decode(const std::vector<uint8_t> &in) {
  if (in.empty())
    return;

  MRPacket p;

  int cmd_len = in[0];

  if ((size_t)(1 + cmd_len) > in.size())
    return;  // malformed

  p.cmd = String((char *)&in[1]).substring(0, cmd_len);

  int i = 1 + cmd_len;

  while (i < (int)in.size()) {
    if (i + 2 > (int)in.size())
      break;

    MRArg a;

    a.type = (MRType)in[i++];
    int l = in[i++];

    if (i + l > (int)in.size())
      break;

    a.data.insert(a.data.end(),
                  in.begin() + i,
                  in.begin() + i + l);

    i += l;

    p.args.push_back(a);
  }

  rxq.push_back(p);
}

/* =========================================================
   Encode Packet
   ========================================================= */

std::vector<uint8_t>
UartRemote::encode(const char *cmd,
                   const std::vector<MRArg> &args) {
  std::vector<uint8_t> out;

  uint8_t l = strlen(cmd);

  out.push_back(l);
  out.insert(out.end(), cmd, cmd + l);

  for (auto &a : args) {
    out.push_back(a.type);
    out.push_back(a.data.size());
    out.insert(out.end(),
               a.data.begin(),
               a.data.end());
  }

  return out;
}

void UartRemote::queue(const char *cmd,
                       const std::vector<MRArg> &args) {
  auto payload = encode(cmd, args);

  std::vector<uint8_t> frame;

  frame.push_back(payload.size() + 4);
  frame.push_back(PREAMBLE0);
  frame.push_back(PREAMBLE1);
  frame.push_back(PREAMBLE2);
  frame.push_back(PREAMBLE3);

  frame.insert(frame.end(),
               payload.begin(),
               payload.end());

  txq.push_back(frame);
}

/* =========================================================
   Main Process
   ========================================================= */

void UartRemote::process() {
  pollRX();

  if (!rxq.empty()) {
    auto packet = rxq.front();
    rxq.pop_front();

    std::vector<MRArg> resp;

    if (handler_cb)
      resp = handler_cb(packet.cmd,
                        packet.args);
    String ack;
    if (UartRemote::to_int(resp[0])==-0x3fffffff) {
      ack = "!Error";
    } else
      ack = packet.cmd + "_ack";

    queue(ack.c_str(), resp);
  }

  pollTX();
}