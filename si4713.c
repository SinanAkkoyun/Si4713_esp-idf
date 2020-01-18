#include "si4713.h"

i2c_cmd_handle_t cmd;

void initGPIOSI4713() {
	gpio_pad_select_gpio(RST_PIN);
	gpio_set_direction(RST_PIN, GPIO_MODE_OUTPUT);
	
	i2c_config_t i2c_config = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = SDA_PIN,
		.scl_io_num = SCL_PIN,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 1000000
	};
	i2c_param_config(I2C_NUM_0, &i2c_config);
	i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

/*void readASQ() {
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (SI4710_ADDR1 << 1) | I2C_MASTER_READ, true);
	i2c_master_read(cmd, 5)
}*/

esp_err_t readI2C(uint8_t* data, uint8_t data_length, uint8_t* read, uint8_t length) {
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (SI4710_ADDR1 << 1) | I2C_MASTER_WRITE, true);
	if(data_length >= 1)
		i2c_master_write(cmd, data, length, true);
	
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (SI4710_ADDR1 << 1) | I2C_MASTER_READ, true);
	
	if (length > 1)
		i2c_master_read(cmd, read, length - 1, I2C_MASTER_ACK);
	
	i2c_master_read_byte(cmd, read + length - 1, I2C_MASTER_NACK);
	i2c_master_stop(cmd);
	
	esp_err_t espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
	if (espRc == ESP_OK) {
		ESP_LOGI("I2C_R", "OK");
	} else {
		ESP_LOGI("I2C_R", "ERR");
	}
	i2c_cmd_link_delete(cmd);
	
	return espRc;
}

uint8_t getRev() {
	uint8_t pn, fw, val0, patch, val1, cmp, val2, chiprev;
	uint8_t req[] = {SI4710_CMD_GET_REV, 0};
	uint8_t read[] = {2,3,0,0,0,0,0,0,0};
	
	readI2C(req, 1, read, 9);
	
	for (int i = 0; i < 9; i++) {
		ESP_LOGI("READ", "%u", read[i]);
	}
	
	//read[0]
	pn = read[1];
	fw = read[2];
	val0 = read[3];
	patch = read[4];
	val1 = read[5];
	cmp = read[6];
	val2 = read[7];
	chiprev = read[8];
	
	fw <<= 8;
	fw |= val0;
	patch <<= 8;
	patch |= val1;
	cmp <<= 8;
	cmp |= val2;
	
	return pn;
}

bool beginSI4713() {
	initGPIOSI4713();
	resetSI4713();
	powerUpSI4713();
	
	if (getRev() != 13)
		return false;
	
	return true;
}

esp_err_t cmdI2C(uint8_t* data, uint8_t length) {
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (SI4710_ADDR1 << 1) | I2C_MASTER_WRITE, false);
	i2c_master_write(cmd, data, length, true);
	i2c_master_stop(cmd);
	esp_err_t espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	
	uint8_t status = 0;
	while (!(status & SI4710_STATUS_CTS)) {
		//uint8_t write[] = {SI4710_CMD_GET_INT_STATUS};
		uint8_t read[] = {0};
		readI2C(0, 0, read, 1);
		status = read[0];
	}
	
	return espRc;
}

uint8_t getStatus() {
	uint8_t write[] = {SI4710_CMD_GET_INT_STATUS};
	uint8_t read[] = {0};
	readI2C(write, 1, read, 1);
	
	ESP_LOGI("READ", "%u", read[0]);
	
	return read[0];
}

void setPropertySI4713(uint16_t property, uint16_t value) {
	uint8_t data[] = {SI4710_CMD_SET_PROPERTY, 0, (uint8_t)(property >> 8), (uint8_t)(property & 0xFF), (uint8_t)(value >> 8), (uint8_t)(value & 0xFF)};
	cmdI2C(data, 6);
}

void resetSI4713() {
	gpio_set_level(RST_PIN, 1);
	vTaskDelay(10 / portTICK_PERIOD_MS);
	gpio_set_level(RST_PIN, 0);
	vTaskDelay(10 / portTICK_PERIOD_MS);
	gpio_set_level(RST_PIN, 1);
}

void powerUpSI4713() {
	uint8_t data[] = {SI4710_CMD_POWER_UP, 0x12, 0x50};
	cmdI2C(data, 3);
	
	setPropertySI4713(SI4713_PROP_REFCLK_FREQ, 32768);   // crystal is 32.768
	setPropertySI4713(SI4713_PROP_TX_PREEMPHASIS, 0);    // 74uS pre-emph (USA std)
	setPropertySI4713(SI4713_PROP_TX_ACOMP_GAIN, 10);    // max gain?
//	setPropertySI4713(SI4713_PROP_TX_ACOMP_ENABLE, 0x02); // turn on limiter, but no
//	dynamic ranging
	setPropertySI4713(SI4713_PROP_TX_ACOMP_ENABLE, 0x0);   // turn on limiter and AGC
}

void setTXPowerSI4713(uint8_t pwr, uint8_t antcap) {	//antcap = 0
	uint8_t data[] = {SI4710_CMD_TX_TUNE_POWER, 0, 0, pwr, antcap};
	cmdI2C(data, 5);
}

void tuneFMSI4713(uint16_t freqKHz) {
	uint8_t data[] = {SI4710_CMD_TX_TUNE_FREQ, 0, (uint8_t)(freqKHz >> 8), (uint8_t)freqKHz};
	cmdI2C(data, 4);
	
	while ((getStatus() & 0x81) != 0x81) {
		vTaskDelay(10 / portTICK_RATE_MS);
		ESP_LOGI("Lo", "op");
	}
}

void beginRDSSI4713(uint16_t programID) {	//0xADAF
	// 66.25KHz (default is 68.25)
	setPropertySI4713(SI4713_PROP_TX_AUDIO_DEVIATION, 6625);
	// 2KHz (default)
	setPropertySI4713(SI4713_PROP_TX_RDS_DEVIATION, 200);
	// RDS IRQ
	setPropertySI4713(SI4713_PROP_TX_RDS_INTERRUPT_SOURCE, 0x0001);
	// program identifier
	setPropertySI4713(SI4713_PROP_TX_RDS_PI, programID);
	// 50% mix (default)
	setPropertySI4713(SI4713_PROP_TX_RDS_PS_MIX, 0x03);
	// RDSD0 & RDSMS (default)
	setPropertySI4713(SI4713_PROP_TX_RDS_PS_MISC, 0x1808);
	// 3 repeats (default)
	setPropertySI4713(SI4713_PROP_TX_RDS_PS_REPEAT_COUNT, 3);
	setPropertySI4713(SI4713_PROP_TX_RDS_MESSAGE_COUNT, 1);
	setPropertySI4713(SI4713_PROP_TX_RDS_PS_AF, 0xE0E0);   // no AF
	setPropertySI4713(SI4713_PROP_TX_RDS_FIFO_SIZE, 0);
	setPropertySI4713(SI4713_PROP_TX_COMPONENT_ENABLE, 0x0007);
}

void setRDSStationSI4713(char* s) {
	uint8_t i, len = strlen(s);
	uint8_t slots = (len + 3) / 4;

	for (uint8_t i = 0; i < slots; i++) {
		uint8_t data[7];
		memset(data, ' ', 6);
		memcpy(data + 2, s, ((int)strlen(s) > 4 ? 4 : (int)strlen(s)));
		s += 4;
		data[6] = 0;
		data[0] = SI4710_CMD_TX_RDS_PS;
		data[1] = i;
		cmdI2C(data, 7);
	}
}

void setRDSBufferSI4713(char* s) {
	uint8_t i, len = strlen(s);
	uint8_t slots = (len + 3) / 4;
	char slot[5];

	for (uint8_t i = 0; i < slots; i++) {
		uint8_t data[9];
		memset(data, ' ', 8);
		memcpy(data + 4, s, ((int)strlen(s) > 4 ? 4 : (int)strlen(s)));
		s += 4;
		data[8] = 0;
		data[0] = SI4710_CMD_TX_RDS_BUFF;
		data[1] = ((i == 0) ? 0x06 : 0x04);
		data[2] = 0x20;
		data[3] = i;
		cmdI2C(data, 9);
	}
	setPropertySI4713(SI4713_PROP_TX_COMPONENT_ENABLE, 0x0007);   // stereo, pilot+rds
}