#include "mcb.h"
#include "config.h"

void MCB::init(UART_HandleTypeDef* huart) {
  huart_ = huart;
}

bool MCB::sendPing() {
  const char* pingMessage = "ping\n";
  HAL_UART_Transmit(huart_, (uint8_t*)pingMessage, strlen(pingMessage), UART_TX_TIMEOUT_MS);
  return false; // reply is not read in this build
}

void MCB::sendForwardCommand(float speed) {
  char command[50];
  snprintf(command, sizeof(command), "forward%.2f\n", speed);
  HAL_UART_Transmit(huart_, (uint8_t*)command, strlen(command), UART_TX_TIMEOUT_MS);
}

void MCB::sendReverseCommand(float speed) {
  char command[50];
  snprintf(command, sizeof(command), "reverse%.2f\n", speed);
  HAL_UART_Transmit(huart_, (uint8_t*)command, strlen(command), UART_TX_TIMEOUT_MS);
}

void MCB::sendIdle(void) {
  const char* command = "idle\n";
  HAL_UART_Transmit(huart_, (uint8_t*)command, strlen(command), UART_TX_TIMEOUT_MS);
}
