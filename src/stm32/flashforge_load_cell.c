#include "autoconf.h"         // Main Klipper configuration
#include "board/armcm_boot.h" // MCU specific boot functions
#include "board/irq.h"        // Interrupt handling
#include "board/misc.h"       // Misc utilities like enable_pclock
#include "command.h"  // Klipper command declarations (DECL_COMMAND, etc.)
#include "internal.h" // Klipper internals
#include "sched.h"    // Klipper scheduler
#include <stdint.h>   // Standard integer types
#include <string.h>   // String functions (memset, strncpy, strstr)

#define UARTx UART5
#define UARTx_IRQn UART5_IRQn

#define BAUDRATE 9600U
#define RXBUF_SIZE 64
#define TXBUF_SIZE 64

#define FLASHFORGE_CMD_TIMEOUT timer_from_us(500000) // 500 ms
#define CMD_QUEUE_SIZE 6

static const char CMD_H1[] = "H1 \0\0\0\0\0\0\0";
static const char CMD_H7[] = "H7 \0\0\0\0\0\0\0";
static const char CMD_H3_S200[] = "H3 S200 \0\0";

struct queued_cmd {
  char cmd_name[16];
  char cmd_data[32];
  size_t cmd_len;
};

static struct queued_cmd cmd_queue[CMD_QUEUE_SIZE];

static volatile uint8_t cmdq_head = 0, cmdq_tail = 0;

enum flashforge_state { FLASHFORGE_IDLE, FLASHFORGE_CMD_SENT };

struct flashforge_bridge_state {
  uint8_t rxbuf[RXBUF_SIZE];
  volatile uint16_t rx_head, rx_tail;
  uint8_t txbuf[TXBUF_SIZE];
  volatile uint16_t tx_head, tx_tail;

  enum flashforge_state state;
  char last_cmd_name[16];
  uint32_t cmd_sent_time;

  volatile uint8_t line_ready;
  volatile uint8_t rx_overflow;
};

static struct flashforge_bridge_state bridge;

static struct task_wake loadcell_wake;

static void flashforge_loadcell_response_send(const char *status,
                                              const char *command,
                                              int32_t value,
                                              const char *raw_response) {
  sendf("flashforge_loadcell_response status=%s command=%s value=%i "
        "raw_response=%s",
        status, command, value, raw_response);
}

static void flashforge_loadcell_send(const char *data, size_t len) {
  irq_disable();
  for (size_t i = 0; i < len; i++) {
    uint16_t next = (bridge.tx_head + 1) % TXBUF_SIZE;
    if (next != bridge.tx_tail) {
      bridge.txbuf[bridge.tx_head] = data[i];
      bridge.tx_head = next;
    } else {
      break;
    }
  }
  if (bridge.tx_head != bridge.tx_tail) {
    UARTx->CR1 |= USART_CR1_TXEIE;
  }
  irq_enable();
}

static void try_send_next_queued_command(void) {
  if (bridge.state == FLASHFORGE_IDLE && cmdq_tail != cmdq_head) {
    struct queued_cmd *slot = &cmd_queue[cmdq_tail];
    bridge.state = FLASHFORGE_CMD_SENT;
    strncpy(bridge.last_cmd_name, slot->cmd_name,
            sizeof(bridge.last_cmd_name) - 1);
    bridge.last_cmd_name[sizeof(bridge.last_cmd_name) - 1] = '\0';
    bridge.cmd_sent_time = timer_read_time();
    flashforge_loadcell_send(slot->cmd_data, slot->cmd_len);
    cmdq_tail = (cmdq_tail + 1) % CMD_QUEUE_SIZE;
  }
}

void UARTx_IRQHandler(void) {
  uint32_t sr = UARTx->SR;

  if (sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) {
    if (sr & USART_SR_ORE) {
      (void)UARTx->DR;
    }
    UARTx->SR &= ~(USART_SR_NE | USART_SR_FE | USART_SR_PE);
  }

  if (sr & USART_SR_RXNE) {
    char d = UARTx->DR;
    uint16_t next = (bridge.rx_head + 1) % RXBUF_SIZE;
    if (next != bridge.rx_tail) {
      bridge.rxbuf[bridge.rx_head] = d;
      bridge.rx_head = next;
      if (d == '\n') {
        bridge.line_ready = 1;
        sched_wake_task(&loadcell_wake);
      }
    } else {
      bridge.rx_overflow = 1;
      sched_wake_task(&loadcell_wake);
    }
  }

  if ((sr & USART_SR_TXE) && (UARTx->CR1 & USART_CR1_TXEIE)) {
    if (bridge.tx_tail != bridge.tx_head) {
      UARTx->DR = bridge.txbuf[bridge.tx_tail];
      bridge.tx_tail = (bridge.tx_tail + 1) % TXBUF_SIZE;
    } else {
      UARTx->CR1 &= ~USART_CR1_TXEIE;
    }
  }
}

static int32_t parse_weight_from_response(char *line) {
  char buf[RXBUF_SIZE];
  strncpy(buf, line, sizeof(buf));
  buf[sizeof(buf) - 1] = '\0';

  char *saveptr;
  char *token;
  token = strtok_r(buf, " ", &saveptr);
  for (int idx = 0; idx < 4 && token; idx++) {
    token = strtok_r(NULL, " ", &saveptr);
  }
  if (!token) {
    return 0;
  }
  char *p = token;
  int sign = 1;
  if (*p == '-') {
    sign = -1;
    p++;
  } else if (*p == '+') {
    p++;
  }
  if (!(*p >= '0' && *p <= '9')) {
    return 0;
  }
  int32_t val = 0;
  while (*p >= '0' && *p <= '9') {
    val = val * 10 + (*p - '0');
    p++;
  }
  return sign * val;
}

static void process_received_line(void) {
  irq_disable();
  uint16_t head = bridge.rx_head;
  uint16_t tail = bridge.rx_tail;
  bridge.line_ready = 0;
  irq_enable();

  if (tail == head || bridge.state != FLASHFORGE_CMD_SENT) {
    if (tail != head) {
      bridge.rx_tail = head;
    }
    return;
  }

  char line[RXBUF_SIZE];
  size_t len = 0;
  uint16_t pos = tail;
  while (pos != head && len < sizeof(line) - 1) {
    char c = bridge.rxbuf[pos];
    if (c != '\r' && c != '\n') {
      line[len++] = c;
    }
    pos = (pos + 1) % RXBUF_SIZE;
  }
  line[len] = '\0';

  bridge.rx_tail = head;

  const char *status = "error";
  if (strstr(line, "ok.")) {
    status = "ok";
  }

  int32_t weight_value = 0;
  if (strcmp(bridge.last_cmd_name, "H7") == 0 && strcmp(status, "ok") == 0) {
    weight_value = parse_weight_from_response(line);
  }
  flashforge_loadcell_response_send(status, bridge.last_cmd_name, weight_value,
                                    line);

  bridge.state = FLASHFORGE_IDLE;
  try_send_next_queued_command();
}

void flashforge_loadcell_task(void) {
  if (bridge.rx_overflow) {
    bridge.rx_overflow = 0;
    irq_disable();
    bridge.rx_head = bridge.rx_tail;
    irq_enable();
    if (bridge.state == FLASHFORGE_CMD_SENT) {
      flashforge_loadcell_response_send("error", bridge.last_cmd_name, 0,
                                        "RX buffer overflow");
      bridge.state = FLASHFORGE_IDLE;
      try_send_next_queued_command();
    }
  }

  if (!sched_check_wake(&loadcell_wake))
    return;

  if (bridge.line_ready) {
    process_received_line();
  }

  if (bridge.state == FLASHFORGE_CMD_SENT &&
      timer_is_before(bridge.cmd_sent_time + FLASHFORGE_CMD_TIMEOUT,
                      timer_read_time())) {
    flashforge_loadcell_response_send("timeout", bridge.last_cmd_name, 0, "");
    bridge.state = FLASHFORGE_IDLE;
    try_send_next_queued_command();
  }
}
DECL_TASK(flashforge_loadcell_task);

static void enqueue_flashforge_command(const char *cmd_name,
                                       const char *cmd_data, size_t cmd_len) {
  irq_disable();
  uint8_t next = (cmdq_head + 1) % CMD_QUEUE_SIZE;
  if (next == cmdq_tail) {
    irq_enable();
    flashforge_loadcell_response_send("error", cmd_name, 0,
                                      "MCU command queue overflow");
    return;
  }
  struct queued_cmd *slot = &cmd_queue[cmdq_head];
  strncpy(slot->cmd_name, cmd_name, sizeof(slot->cmd_name) - 1);
  slot->cmd_name[sizeof(slot->cmd_name) - 1] = '\0';
  memcpy(slot->cmd_data, cmd_data, cmd_len);
  slot->cmd_len = cmd_len;
  cmdq_head = next;
  irq_enable();
}

static void send_flashforge_command(const char *cmd_name, const char *cmd_data,
                                    size_t cmd_len) {
  enqueue_flashforge_command(cmd_name, cmd_data, cmd_len);
  try_send_next_queued_command();
}

// Command H1: Tare
void command_flashforge_loadcell_h1(uint32_t *args) {
  send_flashforge_command("H1", CMD_H1, sizeof(CMD_H1) - 1);
}
DECL_COMMAND(command_flashforge_loadcell_h1, "flashforge_loadcell_h1");

// Command H2: Calibrate by known weight
void command_flashforge_loadcell_h2(uint32_t *args) {
  uint32_t weight = args[0];
  char cmd_buf[32];
  char *p = cmd_buf;

  *p++ = 'H';
  *p++ = '2';
  *p++ = ' ';
  *p++ = 'S';

  char *num_start = p;
  int num_len = 0;
  unsigned int n = weight;

  if (n == 0) {
    *p++ = '0';
    num_len = 1;
  } else {
    while (n > 0 && (p - cmd_buf) < sizeof(cmd_buf) - 1) {
      *p++ = (n % 10) + '0';
      n /= 10;
      num_len++;
    }

    char *start = num_start;
    char *end = p - 1;
    while (start < end) {
      char tmp = *start;
      *start = *end;
      *end = tmp;
      start++;
      end--;
    }
  }

  int total_len = (p - cmd_buf);
  send_flashforge_command("H2", cmd_buf, total_len);
}
DECL_COMMAND(command_flashforge_loadcell_h2,
             "flashforge_loadcell_h2 weight=%u");

// Command H3: Save calibration
void command_flashforge_loadcell_h3(uint32_t *args) {
  send_flashforge_command("H3", CMD_H3_S200, sizeof(CMD_H3_S200) - 1);
}
DECL_COMMAND(command_flashforge_loadcell_h3, "flashforge_loadcell_h3");

// Command H7: Get current weight
void command_flashforge_loadcell_h7(uint32_t *args) {
  send_flashforge_command("H7", CMD_H7, sizeof(CMD_H7) - 1);
}
DECL_COMMAND(command_flashforge_loadcell_h7, "flashforge_loadcell_h7");

void command_flashforge_loadcell_test_cmd(uint32_t *args) {
  uint32_t length = args[0];
  char *cmd_str = (char *)command_decode_ptr(args[1]);
  send_flashforge_command("TEST", cmd_str, length);
}
DECL_COMMAND(command_flashforge_loadcell_test_cmd,
             "flashforge_loadcell_test_cmd cmd=%*s");

void flashforge_loadcell_init(void) {
  memset(&bridge, 0, sizeof(bridge));
  bridge.state = FLASHFORGE_IDLE;

  enable_pclock((uint32_t)UARTx);
  gpio_clock_enable(GPIOC);
  gpio_clock_enable(GPIOD);

  // PC12 (TX) - AF output push-pull
  GPIOC->CRH &= ~(0xF << 16);
  GPIOC->CRH |= (0x9 << 16); // 1001 = AF PP, 10MHz

  // PD2 (RX) - Floating input
  GPIOD->CRL &= ~(0xF << 8);
  GPIOD->CRL |= (0x4 << 8); // 0100 = Floating input

  UARTx->CR1 = 0;
  UARTx->CR2 = 0;
  UARTx->CR3 = 0;
  uint32_t pclk = get_pclock_frequency((uint32_t)UARTx);
  UARTx->BRR = DIV_ROUND_CLOSEST(pclk, BAUDRATE);

  UARTx->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

  armcm_enable_irq(UARTx_IRQHandler, UARTx_IRQn, 1);
}
DECL_INIT(flashforge_loadcell_init);

void flashforge_loadcell_shutdown(void) { UARTx->CR1 &= ~USART_CR1_UE; }
DECL_SHUTDOWN(flashforge_loadcell_shutdown);