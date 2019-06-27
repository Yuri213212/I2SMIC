//device 0x78	SSD1306		128*64 OLED dot matrix screen

#include <stdint.h>

#define I2CCMD_DATA		0x00
#define I2CCMD_END		0x10
#define I2CCMD_RESTART	0x20

struct i2ctransaction{
	uint8_t state;		//loaded,phase(7)
	uint8_t cmdi;		//set 0 on start
	uint8_t count;		//set 0 on start/restart
	uint8_t ini;		//set 0 on start/restart
	uint8_t const *cmd;	//{addr(e),ctrl,[max,[data],ctrl]},{addr(o),max,ctrl}	//ctrl:0x20=restart,0x10=stop,0x00=data//max:datalen-1
	uint8_t *in;
	struct i2ctransaction *next;
};

const uint8_t i2coledinit[]={0x78,I2CCMD_DATA,5,0x00,0xD5,0xF0,0x8D,0x14,0xAF,I2CCMD_END};
