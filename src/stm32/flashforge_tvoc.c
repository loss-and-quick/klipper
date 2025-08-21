#include "autoconf.h"         // Main Klipper configuration
#include "board/armcm_boot.h" // MCU specific boot functions
#include "board/irq.h"        // Interrupt handling
#include "board/misc.h"       // Misc utilities like enable_pclock
#include "command.h"  // Klipper command declarations (DECL_COMMAND, etc.)
#include "internal.h" // Klipper internals
#include "sched.h"    // Klipper scheduler
#include <stdint.h>   // Standard integer types
#include <string.h>   // String functions (memset, strncpy, strstr)

#define UARTx UART4
#define UARTx_IRQn UART4_IRQn

#define BAUDRATE 9600U
#define RXBUF_SIZE 128
#define TVOC_PACKET_SIZE 9

#define TVOC_HEADER_1 0xFF
#define TVOC_HEADER_2 0x18

struct tvoc_state {
  uint8_t rxbuf[RXBUF_SIZE];
  volatile uint16_t rx_head, rx_tail;
  
  volatile uint8_t packet_ready;
  volatile uint8_t rx_overflow;
  
  uint16_t last_tvoc_value;
};

static struct tvoc_state tvoc;
static struct task_wake tvoc_wake;

// Calculate checksum for TVOC packet
static uint8_t calculate_tvoc_checksum(uint8_t *packet) {
  uint8_t sum = 0;
  // Sum bytes 1-7 (skip byte 0 which is 0xFF)
  for (int i = 1; i < 8; i++) {
    sum += packet[i];
  }
  // Invert all bits and add 1
  return (~sum) + 1;
}

// Validate and parse TVOC packet
static int parse_tvoc_packet(uint8_t *packet, uint16_t *tvoc_value) {
  // Check header
  if (packet[0] != TVOC_HEADER_1 || packet[1] != TVOC_HEADER_2) {
    return 0;
  }
  
  // Verify checksum
  uint8_t calculated_checksum = calculate_tvoc_checksum(packet);
  if (packet[8] != calculated_checksum) {
    return 0;
  }
  
  // Extract TVOC value (bytes 4-5, big-endian)
  *tvoc_value = (packet[4] << 8) | packet[5];
  return 1;
}

static void flashforge_tvoc_response_send(uint16_t tvoc_value, const char *status) {
  sendf("flashforge_tvoc_response tvoc=%u status=%s", tvoc_value, status);
}

static void process_tvoc_packet(void) {
  irq_disable();
  uint16_t head = tvoc.rx_head;
  uint16_t tail = tvoc.rx_tail;
  tvoc.packet_ready = 0;
  irq_enable();
  
  while ((head - tail + RXBUF_SIZE) % RXBUF_SIZE >= TVOC_PACKET_SIZE) {
    uint8_t packet[TVOC_PACKET_SIZE];
    
    uint16_t search_pos = tail;
    int found_header = 0;
    
    while (search_pos != head) {
      if (tvoc.rxbuf[search_pos] == TVOC_HEADER_1) {
        uint16_t next_pos = (search_pos + 1) % RXBUF_SIZE;
        if (next_pos != head && tvoc.rxbuf[next_pos] == TVOC_HEADER_2) {
          // Found header, check if we have enough bytes for full packet
          uint16_t remaining = (head - search_pos + RXBUF_SIZE) % RXBUF_SIZE;
          if (remaining >= TVOC_PACKET_SIZE) {
            // Extract packet
            for (int i = 0; i < TVOC_PACKET_SIZE; i++) {
              packet[i] = tvoc.rxbuf[(search_pos + i) % RXBUF_SIZE];
            }
            found_header = 1;
            tail = (search_pos + TVOC_PACKET_SIZE) % RXBUF_SIZE;
            break;
          }
        }
      }
      search_pos = (search_pos + 1) % RXBUF_SIZE;
    }
    
    if (!found_header) {
      if (tail != head) {
        tail = (tail + 1) % RXBUF_SIZE;
      } else {
        break;
      }
      continue;
    }
    
    uint16_t tvoc_value;
    if (parse_tvoc_packet(packet, &tvoc_value)) {
      tvoc.last_tvoc_value = tvoc_value;
      flashforge_tvoc_response_send(tvoc_value, "ok");
    } else {
      flashforge_tvoc_response_send(0, "checksum_error");
    }
    
    irq_disable();
    tvoc.rx_tail = tail;
    irq_enable();
  }
}

void TVOC_UARTx_IRQHandler(void) {
  uint32_t sr = UARTx->SR;
  
  if (sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) {
    if (sr & USART_SR_ORE) {
      (void)UARTx->DR;
    }
    UARTx->SR &= ~(USART_SR_NE | USART_SR_FE | USART_SR_PE);
  }
  
  if (sr & USART_SR_RXNE) {
    uint8_t data = UARTx->DR;
    uint16_t next = (tvoc.rx_head + 1) % RXBUF_SIZE;
    
    if (next != tvoc.rx_tail) {
      tvoc.rxbuf[tvoc.rx_head] = data;
      tvoc.rx_head = next;
      tvoc.packet_ready = 1;
      sched_wake_task(&tvoc_wake);
    } else {
      tvoc.rx_overflow = 1;
      sched_wake_task(&tvoc_wake);
    }
  }
}

void flashforge_tvoc_task(void) {
  if (tvoc.rx_overflow) {
    tvoc.rx_overflow = 0;
    irq_disable();
    tvoc.rx_head = tvoc.rx_tail;
    irq_enable();
    flashforge_tvoc_response_send(0, "rx_overflow");
  }
  
  if (!sched_check_wake(&tvoc_wake))
    return;
    
  if (tvoc.packet_ready) {
    process_tvoc_packet();
  }
}
DECL_TASK(flashforge_tvoc_task);

void flashforge_tvoc_init(void) {
  memset(&tvoc, 0, sizeof(tvoc));
  
  enable_pclock((uint32_t)UARTx);
  
  gpio_clock_enable(GPIOC);
  
  // PC11 (RX) - Floating input
  GPIOC->CRH &= ~(0xF << 12);
  GPIOC->CRH |= (0x4 << 12); // 0100 = Floating input

  UARTx->CR1 = 0;
  UARTx->CR2 = 0;
  UARTx->CR3 = 0;
  
  uint32_t pclk = get_pclock_frequency((uint32_t)UARTx);
  UARTx->BRR = DIV_ROUND_CLOSEST(pclk, BAUDRATE);
  
  // We don't need TX for this sensor, only RX
  UARTx->CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_RXNEIE;
  
  armcm_enable_irq(TVOC_UARTx_IRQHandler, UARTx_IRQn, 1);
}
DECL_INIT(flashforge_tvoc_init);

void flashforge_tvoc_shutdown(void) {
  UARTx->CR1 &= ~USART_CR1_UE;
}
DECL_SHUTDOWN(flashforge_tvoc_shutdown);