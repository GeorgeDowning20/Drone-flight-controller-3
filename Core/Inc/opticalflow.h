

void enable_opticalflow(UART_HandleTypeDef *huart);
void optical_flow_callback(UART_HandleTypeDef *huart);

typedef struct _message
{
	uint32_t start;
	uint8_t type;
	uint8_t random_byte;
	uint8_t length;
}_message;

typedef struct _flow
{
	uint8_t start;
	uint8_t quality;
	int16_t vx;
	uint16_t another_few_random_bytes;
	int16_t	vy;
	uint16_t check_sum;
}_flow;

typedef struct _lidar
{
		uint8_t start;
		uint8_t another_random_byte;
		int16_t distance;
		uint16_t quality;
		uint16_t check_sum;
		uint8_t buffer_size;
}_lidar;


typedef volatile struct _opticalflow
{
	 _message message;
	 _flow flow;
	 _lidar lidar;
} _opticalflow;


_opticalflow opticalflow;

