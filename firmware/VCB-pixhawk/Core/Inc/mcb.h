#ifndef MCB_H
#define MCB_H

#include "main.h"

class MCB {
  public:
    void init(
      UART_HandleTypeDef* huart
    );

    bool sendPing(void);
    
    void sendForwardCommand(float speed);
    void sendReverseCommand(float speed);

    void sendIdle(void);

  private:
    UART_HandleTypeDef* huart_;

};

#endif /* MCB_H */
