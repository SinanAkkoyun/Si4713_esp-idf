#include "si4713.h"

i2c_cmd_handle_t cmd;
bool isRDSinit = false;

esp_err_t SI4713_readI2C(uint8_t* data, uint8_t data_length, uint8_t* read, uint8_t length) {
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
	i2c_cmd_link_delete(cmd);
	
	return espRc;
}

esp_err_t SI4713_writeI2C(uint8_t* data, uint8_t length) {
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (SI4710_ADDR1 << 1) | I2C_MASTER_WRITE, false);
	i2c_master_write(cmd, data, length, true);
	i2c_master_stop(cmd);
	esp_err_t espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	
	uint8_t status = 0;
	while (!(status & SI4710_STATUS_CTS)) {
		uint8_t read[] = {0};
		SI4713_readI2C(0, 0, read, 1);
		status = read[0];
	}
	
	return espRc;
}

uint8_t SI4713_getRev() {
	uint8_t pn, fw, val0, patch, val1, cmp, val2, chiprev;
	uint8_t req[] = {SI4710_CMD_GET_REV, 0};
	uint8_t read[] = {2,3,0,0,0,0,0,0,0};
	
	SI4713_readI2C(req, 1, read, 9);
	
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

bool SI4713_begin() {
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
	
	SI4713_reset();
	SI4713_powerUp();
	
	if (SI4713_getRev() != 13)
		return false;
	
	return true;
}

uint8_t SI4713_getStatus() {
	uint8_t write[] = {SI4710_CMD_GET_INT_STATUS};
	uint8_t read[] = {0};
	SI4713_readI2C(write, 1, read, 1);
	
	return read[0];
}

void SI4713_setProperty(uint16_t property, uint16_t value) {
	uint8_t data[] = {SI4710_CMD_SET_PROPERTY, 0, (uint8_t)(property >> 8), (uint8_t)(property & 0xFF), (uint8_t)(value >> 8), (uint8_t)(value & 0xFF)};
	SI4713_writeI2C(data, 6);
}

void SI4713_reset() {
	gpio_set_level(RST_PIN, 1);
	vTaskDelay(10 / portTICK_PERIOD_MS);
	gpio_set_level(RST_PIN, 0);
	vTaskDelay(10 / portTICK_PERIOD_MS);
	gpio_set_level(RST_PIN, 1);
	
	isRDSinit = false;
}

void SI4713_powerUp() {
	uint8_t data[] = {SI4710_CMD_POWER_UP, 0x12, 0x50};
	SI4713_writeI2C(data, 3);
	
	SI4713_setProperty(SI4713_PROP_REFCLK_FREQ, 32768);   // crystal is 32.768
	SI4713_setProperty(SI4713_PROP_TX_PREEMPHASIS, 0);    // 74uS pre-emph (USA std)
	SI4713_setProperty(SI4713_PROP_TX_ACOMP_GAIN, 10);    // max gain?
//	SI4713_setProperty(SI4713_PROP_TX_ACOMP_ENABLE, 0x02); // turn on limiter, but no
//	dynamic ranging
	SI4713_setProperty(SI4713_PROP_TX_ACOMP_ENABLE, 0x0);   // turn on limiter and AGC
	
	isRDSinit = false;
}

void SI4713_setTXPower(uint8_t pwr, uint8_t antcap) {
		//antcap = 0
	uint8_t data[] = {SI4710_CMD_TX_TUNE_POWER, 0, 0, pwr, antcap};
	SI4713_writeI2C(data, 5);
}

void SI4713_tune(uint16_t freqKHz) {
	uint8_t data[] = {SI4710_CMD_TX_TUNE_FREQ, 0, (uint8_t)(freqKHz >> 8), (uint8_t)freqKHz};
	SI4713_writeI2C(data, 4);
	
	while ((SI4713_getStatus() & 0x81) != 0x81) {
		vTaskDelay(10 / portTICK_RATE_MS);
	}
}

void SI4713_beginRDS() {
	uint16_t programID = 0xADAF; //0xADAF
	
	SI4713_setProperty(SI4713_PROP_TX_AUDIO_DEVIATION, 6625);// 66.25KHz (default is 68.25)
	SI4713_setProperty(SI4713_PROP_TX_RDS_DEVIATION, 200);// 2KHz (default)
	SI4713_setProperty(SI4713_PROP_TX_RDS_INTERRUPT_SOURCE, 0x0001);// RDS IRQ
	SI4713_setProperty(SI4713_PROP_TX_RDS_PI, programID);// program identifier
	SI4713_setProperty(SI4713_PROP_TX_RDS_PS_MIX, 0x03);// 50% mix (default)
	SI4713_setProperty(SI4713_PROP_TX_RDS_PS_MISC, 0x1808);// RDSD0 & RDSMS (default)
	SI4713_setProperty(SI4713_PROP_TX_RDS_PS_REPEAT_COUNT, 3);// 3 repeats (default)
	SI4713_setProperty(SI4713_PROP_TX_RDS_MESSAGE_COUNT, 1);
	SI4713_setProperty(SI4713_PROP_TX_RDS_PS_AF, 0xE0E0);   // no AF
	SI4713_setProperty(SI4713_PROP_TX_RDS_FIFO_SIZE, 0);
	SI4713_setProperty(SI4713_PROP_TX_COMPONENT_ENABLE, 0x0007);
	
	isRDSinit = true;
}

void SI4713_setRDSStation(char* s) {
	uint8_t len = strlen(s);
	uint8_t slots = (len + 3) / 4;

	for (uint8_t i = 0; i < slots; i++) {
		uint8_t data[7];
		memset(data, ' ', 6);
		memcpy(data + 2, s, ((int)strlen(s) > 4 ? 4 : (int)strlen(s)));
		s += 4;
		data[6] = 0;
		data[0] = SI4710_CMD_TX_RDS_PS;
		data[1] = i;
		SI4713_writeI2C(data, 7);
	}
}

void SI4713_setRDSBuffer(char* s) {
	uint8_t len = strlen(s);
	uint8_t slots = (len + 3) / 4;

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
		SI4713_writeI2C(data, 9);
	}
	SI4713_setProperty(SI4713_PROP_TX_COMPONENT_ENABLE, 0x0007);   // stereo, pilot+rds
}

uint8_t SI4713_readASQ() {
	uint8_t write[] = {SI4710_CMD_TX_ASQ_STATUS, 0x1};
	uint8_t read[5];
	SI4713_readI2C(write, 2, read, 5);
	return read[1];
}

uint8_t SI4713_readIN() {
	uint8_t write[] = {SI4710_CMD_TX_ASQ_STATUS, 0x1};
	uint8_t read[5];
	SI4713_readI2C(write, 2, read, 5);
	
	return read[4];
}

uint8_t SI4713_readFrequency() {
	uint8_t write[] = {SI4710_CMD_TX_TUNE_STATUS, 0x1};
	uint8_t read[8];
	uint8_t freq;
	SI4713_readI2C(write, 2, read, 8);
	
	freq = read[2];
	freq <<= 8;
	freq |= read[3];
	
	return freq;
}

uint8_t SI4713_readBuV() {
	uint8_t write[] = {SI4710_CMD_TX_TUNE_STATUS, 0x1};
	uint8_t read[8];
	SI4713_readI2C(write, 2, read, 8);
	
	return read[5];
}

uint8_t SI4713_readAntCap() {
	uint8_t write[] = {SI4710_CMD_TX_TUNE_STATUS, 0x1};
	uint8_t read[8];
	SI4713_readI2C(write, 2, read, 8);
	
	return read[6];
}

uint8_t SI4713_readNoiseLevel() {
	uint8_t write[] = {SI4710_CMD_TX_TUNE_STATUS, 0x1};
	uint8_t read[8];
	SI4713_readI2C(write, 2, read, 8);
	
	return read[7];
}

void SI4713_readTuneMeasure(uint16_t freq) {
	if (freq % 5 != 0) {
		freq -= (freq % 5);
	}
	uint8_t write[] = {SI4710_CMD_TX_TUNE_MEASURE, 0, freq >> 8, freq, 0};
	SI4713_writeI2C(write, 4);
	
	while (SI4713_getStatus() != 0x81)
		vTaskDelay(10 / portTICK_RATE_MS);
}

void SI4713_setRDSMessage(char* station, char* buffer) {
	if (isRDSinit)
		SI4713_beginRDS();
	SI4713_setRDSStation(station);
	SI4713_setRDSBuffer(buffer);
}

void SI4713_setGPIODirection(uint8_t dir) {
	uint8_t write[] = {SI4710_CMD_GPO_CTL, dir};
	SI4713_writeI2C(write, 2);
}

void SI4713_setGPIOLevel(uint8_t level) {
	uint8_t write[] = {SI4710_CMD_GPO_SET, level};
	SI4713_writeI2C(write, 2);
}
