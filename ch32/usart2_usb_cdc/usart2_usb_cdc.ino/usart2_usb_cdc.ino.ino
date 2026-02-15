extern "C" {
#include "ch32v20x.h"
}

#include "Adafruit_TinyUSB.h"

/* ===================== RING BUFFER ===================== */

#define UART_BUF 128

volatile uint8_t rxbuf[UART_BUF];
volatile uint16_t rx_head = 0;
volatile uint16_t rx_tail = 0;

static inline void rb_push(uint8_t c)
{
    uint16_t next = (rx_head + 1) % UART_BUF;
    if(next != rx_tail) {   // drop if full
        rxbuf[rx_head] = c;
        rx_head = next;
    }
}

int usart2_available()
{
    return (UART_BUF + rx_head - rx_tail) % UART_BUF;
}

int usart2_read()
{
    if(rx_head == rx_tail) return -1;
    uint8_t c = rxbuf[rx_tail];
    rx_tail = (rx_tail + 1) % UART_BUF;
    return c;
}

/* ===================== TX ===================== */

void usart2_write(uint8_t c)
{
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    USART_SendData(USART2, c);
}

/* ===================== INTERRUPT ===================== */

extern "C" void USART2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void USART2_IRQHandler(void)
{
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        uint8_t c = USART_ReceiveData(USART2);
        rb_push(c);
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

/* ===================== INIT ===================== */

void usart2_init(uint32_t baud)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    /* clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    /* TX PA2 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* RX PA3 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_DeInit(USART2);

    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.USART_BaudRate = baud;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART2, &USART_InitStructure);

    /* Enable RX interrupt */
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    /* NVIC */
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART2, ENABLE);
}

/* ===================== ARDUINO ===================== */

void setup()
{
   if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  }

    usart2_init(115200);

    const char *msg = "USART2 interrupt echo ready\r\n";
    while(*msg) usart2_write(*msg++);
}

void loop()
{

    #ifdef TINYUSB_NEED_POLLING_TASK
  // Manual call tud_task since it isn't called by Core's background
  TinyUSBDevice.task();
  #endif
  uint8_t buf[64];
  uint32_t count = 0;
  while (SerialTinyUSB.available()>0) {
    buf[count++] = (uint8_t) toupper(SerialTinyUSB.read());
  }

  if (count) {
    SerialTinyUSB.write(buf, count);
  }
    while(usart2_available())
    {
        int c = usart2_read();
        usart2_write(c);   // echo
    }
}
