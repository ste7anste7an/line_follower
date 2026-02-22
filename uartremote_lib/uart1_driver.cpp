#include "uart1_driver.h"

/* =========================================================
   LOW LEVEL USART1 DRIVER (PA9/PA10)
   ========================================================= */

#define UART1_BUF 256

static volatile uint8_t uart1_rxbuf[UART1_BUF];
static volatile uint16_t uart1_head = 0;
static volatile uint16_t uart1_tail = 0;

static inline void uart1_push(uint8_t c) {
  uint16_t n = (uart1_head + 1) % UART1_BUF;
  if (n != uart1_tail) {
    uart1_rxbuf[uart1_head] = c;
    uart1_head = n;
  }
}

extern "C" void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
extern "C" void USART1_IRQHandler(void) {
  if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
    uart1_push(USART_ReceiveData(USART1));
    USART_ClearITPendingBit(USART1, USART_IT_RXNE);
  }
}

static void uart1_begin_hw(uint32_t baud) {
  GPIO_InitTypeDef gpio;
  USART_InitTypeDef us;
  NVIC_InitTypeDef nvic;

  RCC_APB2PeriphClockCmd(
    RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO | RCC_APB2Periph_USART1,
    ENABLE);

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
   USART1Stream Implementation
   ========================================================= */

void USART1Stream::begin(uint32_t baud) {
  uart1_begin_hw(baud);
}

int USART1Stream::available() {
  return (UART1_BUF + uart1_head - uart1_tail) % UART1_BUF;
}

int USART1Stream::read() {
  if (uart1_head == uart1_tail) return -1;
  uint8_t c = uart1_rxbuf[uart1_tail];
  uart1_tail = (uart1_tail + 1) % UART1_BUF;
  return c;
}

int USART1Stream::peek() {
  if (uart1_head == uart1_tail) return -1;
  return uart1_rxbuf[uart1_tail];
}

void USART1Stream::flush() {}

size_t USART1Stream::write(uint8_t c) {
  while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    ;
  USART_SendData(USART1, c);
  return 1;
}

size_t USART1Stream::write(const uint8_t *buf, size_t size) {
  for (size_t i = 0; i < size; i++)
    write(buf[i]);
  return size;
}