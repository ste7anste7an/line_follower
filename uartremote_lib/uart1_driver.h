#pragma once
#include <Arduino.h>

extern "C" {
#include "ch32v20x.h"
}

class USART1Stream : public Stream {
public:
  void begin(uint32_t baud);

  int available() override;
  int read() override;
  int peek() override;
  void flush() override;

  size_t write(uint8_t c) override;
  size_t write(const uint8_t *buf, size_t size) override;
};