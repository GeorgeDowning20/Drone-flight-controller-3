#define IBUS_BUFFER_LEN 32
 uint16_t ibus[IBUS_BUFFER_LEN];

float InputR,InputP,InputY,InputT;
void ibus_callback(void);
void configure_ibus(UART_HandleTypeDef *huart);



