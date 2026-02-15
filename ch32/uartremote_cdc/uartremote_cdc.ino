#include <Arduino.h>
#include <vector>
#include <deque>
#include "Adafruit_TinyUSB.h"
#include <Wire.h>
#define MY_I2C_ADDRESS 0x33

extern "C" {
#include "ch32v20x.h"
}

enum MRType { MR_INT = 78,
              MR_BYTES = 65,
              MR_STR = 83,
              MR_BOOL = 66 };

struct MRArg {
  MRType type;
  std::vector<uint8_t> data;
};

struct MRPacket {
  String cmd;
  std::vector<MRArg> args;
};

// forward declare handler
std::vector<MRArg> handler(String cmd, const std::vector<MRArg> &args);

MRArg MR_int(int v) {
  MRArg a;
  a.type = MR_INT;
  char buf[16];
  itoa(v, buf, 10);
  a.data.assign(buf, buf + strlen(buf));
  return a;
}

int to_int(const MRArg &a) {
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

bool to_bool(const MRArg &a) {
  return a.data[0] != 0;
}


const char *MRTypeName(MRType t) {
  switch (t) {
    case MR_INT: return "INT";
    case MR_BYTES: return "BYTES";
    case MR_STR: return "STR";
    case MR_BOOL: return "BOOL";
    default: return "UNKNOWN";
  }
}

void printMRArg(const MRArg &a) {
  SerialTinyUSB.print("Type=");
  SerialTinyUSB.print(MRTypeName(a.type));
  SerialTinyUSB.print("  Len=");
  SerialTinyUSB.print(a.data.size());
  SerialTinyUSB.print("  Value=");

  switch (a.type) {
    case MR_INT:
      SerialTinyUSB.print(to_int(a));
      break;

    case MR_BOOL:
      SerialTinyUSB.print(a.data[0] ? "true" : "false");
      break;

    case MR_STR:
      SerialTinyUSB.write(a.data.data(), a.data.size());
      break;

    case MR_BYTES:
    default:
      for (uint8_t b : a.data) {
        if (b < 16) SerialTinyUSB.print('0');
        SerialTinyUSB.print(b, HEX);
        SerialTinyUSB.print(' ');
      }
      break;
  }

  SerialTinyUSB.println();
}

/* =========================================================
   LOW LEVEL USART1 DRIVER (PA9/PA10)
   ========================================================= */

#define UART1_BUF 256
volatile uint8_t uart1_rxbuf[UART1_BUF];
volatile uint16_t uart1_head = 0;
volatile uint16_t uart1_tail = 0;

static inline void uart1_push(uint8_t c) {
  uint16_t n = (uart1_head + 1) % UART1_BUF;
  if (n != uart1_tail) {
    uart1_rxbuf[uart1_head] = c;
    uart1_head = n;
  }
}

extern "C" void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART1_IRQHandler(void) {
  if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
    uart1_push(USART_ReceiveData(USART1));
    USART_ClearITPendingBit(USART1, USART_IT_RXNE);
  }
}

void uart1_begin(uint32_t baud) {
  GPIO_InitTypeDef gpio;
  USART_InitTypeDef us;
  NVIC_InitTypeDef nvic;

  // clocks
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO | RCC_APB2Periph_USART1, ENABLE);

  // TX PA9
  gpio.GPIO_Pin = GPIO_Pin_9;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  gpio.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &gpio);

  // RX PA10
  gpio.GPIO_Pin = GPIO_Pin_10;
  gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &gpio);

  USART_StructInit(&us);
  us.USART_BaudRate = baud;
  us.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &us);

  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

  nvic.NVIC_IRQChannel = USART1_IRQn;
  nvic.NVIC_IRQChannelPreemptionPriority = 1;
  nvic.NVIC_IRQChannelSubPriority = 1;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);

  USART_Cmd(USART1, ENABLE);
}

/* =========================================================
   STREAM WRAPPER FOR USART1
   ========================================================= */

class USART1Stream : public Stream {
public:
  void begin(uint32_t baud) {
    uart1_begin(baud);
  }

  int available() override {
    return (UART1_BUF + uart1_head - uart1_tail) % UART1_BUF;
  }

  int read() override {
    if (uart1_head == uart1_tail) return -1;
    uint8_t c = uart1_rxbuf[uart1_tail];
    uart1_tail = (uart1_tail + 1) % UART1_BUF;
    return c;
  }

  int peek() override {
    if (uart1_head == uart1_tail) return -1;
    return uart1_rxbuf[uart1_tail];
  }

  void flush() override {}

  size_t write(uint8_t c) override {
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
      ;
    USART_SendData(USART1, c);
    return 1;
  }

  size_t write(const uint8_t *buf, size_t size) override {
    for (size_t i = 0; i < size; i++) write(buf[i]);
    return size;
  }
};

USART1Stream SerialA;

/* =========================================================
   MICROREMOTE PROTOCOL
   ========================================================= */

#define PREAMBLE0 '<'
#define PREAMBLE1 '$'
#define PREAMBLE2 'M'
#define PREAMBLE3 'U'

class MicroRemote {
public:
  Stream &io;
  MicroRemote(Stream &s)
    : io(s) {}
  enum State { WAIT_LEN,
               WAIT_PREAMBLE,
               WAIT_PAYLOAD };
  State state = WAIT_LEN;
  uint8_t expected_len = 0;
  uint8_t pre_idx = 0;
  std::vector<uint8_t> buf;
  std::deque<MRPacket> rxq;
  std::deque<std::vector<uint8_t>> txq;

  void pollRX() {
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
          if (pre_idx == 4) state = WAIT_PAYLOAD;
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

  void decode(const std::vector<uint8_t> &in) {
    MRPacket p;
    int cmd_len = in[0];
    p.cmd = String((char *)&in[1]).substring(0, cmd_len);
    int i = 1 + cmd_len;
    while (i < in.size()) {
      MRArg a;
      a.type = (MRType)in[i++];
      int l = in[i++];
      a.data.insert(a.data.end(), in.begin() + i, in.begin() + i + l);
      i += l;
      p.args.push_back(a);
    }
    rxq.push_back(p);
  }

  std::vector<uint8_t> encode(const char *cmd, const std::vector<MRArg> &args) {
    std::vector<uint8_t> out;
    uint8_t l = strlen(cmd);
    out.push_back(l);
    out.insert(out.end(), cmd, cmd + l);
    for (auto &a : args) {
      out.push_back(a.type);
      out.push_back(a.data.size());
      out.insert(out.end(), a.data.begin(), a.data.end());
    }
    return out;
  }

  void queue(const char *cmd, const std::vector<MRArg> &args) {
    auto p = encode(cmd, args);
    std::vector<uint8_t> f;
    f.push_back(p.size() + 4);
    f.insert(f.end(), { '<', '$', 'M', 'U' });
    f.insert(f.end(), p.begin(), p.end());
    txq.push_back(f);
  }

  void pollTX() {
    if (txq.empty()) return;
    auto &f = txq.front();
    io.write(f.data(), f.size());
    txq.pop_front();
  }

  void process() {
    pollRX();
    if (!rxq.empty()) {
      auto p = rxq.front();
      rxq.pop_front();
      auto resp = handler(p.cmd, p.args);
      String ack = p.cmd + "_ack";
      queue(ack.c_str(), resp);
    }
    pollTX();
  }
};

/* =========================================================
   HANDLER
   ========================================================= */

std::vector<MRArg> handler(String cmd, const std::vector<MRArg> &args) {
  if (cmd == "ping") {
    SerialTinyUSB.println("ping");
    return { MR_int(millis()) };
  }
  if (cmd == "add") {
    //   SerialTinyUSB.print("receive add ");
    //     printMRArg(args[0]);
    //     printMRArg(args[1]);
    //     char buf[100];

    //   sprintf(buf,"converted %d %d\r\n",to_int(args[0]),to_int(args[1]),MR_int((to_int(args[0])+to_int(args[1]))));
    //   SerialTinyUSB.print("add ");
    //   SerialTinyUSB.println(buf);
    //   SerialTinyUSB.print("result is:");

    //   printMRArg(MR_int(to_int(args[0])+to_int(args[1])));
    return { MR_int(to_int(args[0]) + to_int(args[1])) };
  }
  if (cmd == "led") {
    digitalWrite(PB2, to_bool(args[0]));
    return { MR_int(1) };
  }
  if (cmd == "str") {
    SerialTinyUSB.print("str ");
    printMRArg(args[0]);
    return { MR_int(1) };
  }
  return { MR_int(-1) };
}


void ReceiveEvent(int nBytes) {
  uint8_t command;
  //LOG(LOG_DEBUG, "received I2C msg");
  if (nBytes > 0) {
    command = Wire.read();
    nBytes--;
    //LOG(LOG_DEBUG, "received command: %d\r\n", command);
  }
  if (command < 10) {  // valid command
    // read remaining bytes
    //LOG(LOG_DEBUG,"entering command=%d command MAX_CMDS=%d\r",command,MAX_CMDS);
    switch (command) {
      case 1:
        SerialTinyUSB.println("I2C cmd  =1");
        for (int i = 0; i < 4; i++) {
          uint8_t v = Wire.read();
          SerialTinyUSB.print(v, HEX);
        }
        SerialTinyUSB.println("");

        break;
    }
    while (nBytes > 0) {
      uint8_t val = Wire.read();
      nBytes--;
    }
  }
}

void RequestEvent() {

  uint8_t buf[] = { 1, 2, 3, 4, 5, 6, 7, 8 };
  Wire.write(buf, 8);
}


/* ========================================================= */

MicroRemote uart(SerialA);

void setup() {
  // if (!TinyUSBDevice.isInitialized()) TinyUSBDevice.begin(0);
 // no remapping
 // SDA = PB7, SCL = PB6
 // remapping:
  //Wire.setSDA(PB9);
  //Wire.setSCL(PB8);

  Wire.begin(MY_I2C_ADDRESS);
  // Note: enable I2C slave functionality in /libraries/Wire/src/utility/twi.h to prevent error message for next two lines
  Wire.onReceive(ReceiveEvent);
  Wire.onRequest(RequestEvent);


  pinMode(PB2, OUTPUT);
  SerialA.begin(115200);
}

void loop() {
  // #ifdef TINYUSB_NEED_POLLING_TASK
  // TinyUSBDevice.task();
  // #endif

  uart.process();
}
