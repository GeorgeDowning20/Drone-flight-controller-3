



	float  c0;
	float  c1;
	float  c00;
	float  c10;
	float  c01;
	float  c11;
	float  c20;
	float  c21;
	float  c30;

	float pressure;
	float temperature;
	float inital_pressure;


	float baro_alt;
	//float halfex, halfey, halfez;

	float Praw_sc;
	float Traw_sc;

	#define I2C_write			1


	#define DPS310_I2C_ADDR             0x76<<1

	#define DPS310_REG_PSR_B2           0x00
	#define DPS310_REG_PSR_B1           0x01
	#define DPS310_REG_PSR_B0           0x02
	#define DPS310_REG_TMP_B2           0x03
	#define DPS310_REG_TMP_B1           0x04
	#define DPS310_REG_TMP_B0           0x05
	#define DPS310_REG_PRS_CFG          0x06
	#define DPS310_REG_TMP_CFG          0x07
	#define DPS310_REG_MEAS_CFG         0x08
	#define DPS310_REG_CFG_REG          0x09
	#define DPS310_REG_RESET            0x0C
	#define DPS310_REG_ID               0x0D
	#define DPS310_REG_COEF             0x10
	#define DPS310_REG_COEF_SRCE        0x28
	#define DPS310_ID_REV_AND_PROD_ID       (0x10)

	#define DPS310_RESET_BIT_SOFT_RST       (0x09)    // 0b1001

	#define DPS310_MEAS_CFG_COEF_RDY        (1 << 7)
	#define DPS310_MEAS_CFG_SENSOR_RDY      (1 << 6)
	#define DPS310_MEAS_CFG_TMP_RDY         (1 << 5)
	#define DPS310_MEAS_CFG_PRS_RDY         (1 << 4)
	#define DPS310_MEAS_CFG_MEAS_CTRL_CONT  (0x7)

	#define DPS310_PRS_CFG_BIT_PM_RATE_32HZ (0x50)      //  101 - 32 measurements pr. sec.
	#define DPS310_PRS_CFG_BIT_PM_PRC_16    (0x04)      // 0100 - 16 times (Standard).

	#define DPS310_TMP_CFG_BIT_TMP_EXT          (0x80)  //
	#define DPS310_TMP_CFG_BIT_TMP_RATE_32HZ    (0x50)  //  101 - 32 measurements pr. sec.
	#define DPS310_TMP_CFG_BIT_TMP_PRC_16       (0x04)  // 0100 - 16 times (Standard).

	#define DPS310_CFG_REG_BIT_P_SHIFT          (0x04)
	#define DPS310_CFG_REG_BIT_T_SHIFT          (0x08)

	#define DPS310_COEF_SRCE_BIT_TMP_COEF_SRCE  (0x80)


	uint8_t DPS310_rx_buffer[20];
	uint8_t DPS310_tx_buffer[20];


	int32_t getTwosComplement(uint32_t raw, uint8_t length);
	void configure_baro (I2C_HandleTypeDef *hi2c);
	void read_baro (void);

	#define COEFFICIENT_LENGTH 18
	uint8_t coef[COEFFICIENT_LENGTH];


