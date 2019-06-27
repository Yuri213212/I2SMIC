/***

I2S MIC ver 0.1 alpha
Copyright (C) 2019 Yuri213212
Site:https://github.com/Yuri213212/I2SMIC
Email: yuri213212@vip.qq.com
License: CC BY-NC-SA 4.0
https://creativecommons.org/licenses/by-nc-sa/4.0/

***/

#pragma arm section zidata="non_initialized"

#include "LPC1100L.h"
#include "i2c.h"
#include "oledchar.h"

#define P1_LEDr			0x200

#define ADCBIAS			0x1A00
#define MINTHRESHOLD	0x1C00
#define MAXTHRESHOLD	0xED00

#define chbias			128

#define wallclock (int16_t)TMR16B0TC

#define refreshwait		2500	//66.67ms(15fps)
#define i2cresetwait	375		//10ms

#define I2C_MAX			0
#define I2C_OLED		0

int16_t refreshwaitto;
uint16_t minbuf,maxbuf,min,max;

int8_t i2cbusy,i2coledrefresh,i2coledmax,i2coledi,i2coledfont;
uint8_t i2coledfont8;
uint8_t i2coledoutbuf[129],i2coledout[21];
struct i2ctransaction i2ccontrol[I2C_MAX+1],*i2ccp;
uint32_t i2cerrc;

__inline uint8_t toHex(register uint8_t x){
	return x<10?x+'0':x+55;
}

void str2buf(uint8_t *buf,char const *s){
	register int i;

	for (i=0;s[i];++i){
		buf[i]=(uint8_t)s[i];
	}
	buf[i]=0;
}

//0x78,I2CCMD_DATA,6,0x80,<0x00>,0x80,<0x10>,0x80,<0xB0>,0x40,I2CCMD_DATA,7,<0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77>,I2CCMD_END	//0xE3=NOP
void oledprintdata(int8_t pos,uint8_t const *data){
	register int i;

	if (pos>=0){
		i2coledout[4]=(pos<<3)&0x08;
		i2coledout[6]=((pos>>1)&0x07)|0x10;
		i2coledout[8]=((pos>>4)&0x07)|0xB0;
	}else{
		i2coledout[4]=0xE3;
		i2coledout[6]=0xE3;
		i2coledout[8]=0xE3;
	}
	for (i=0;i<8;++i){
		i2coledout[i+12]=data[i];
	}
}

//0x78,I2CCMD_DATA,6,0x80,<0x00>,0x80,<0x10>,0x80,<0xB0>,0x40,I2CCMD_DATA,7,<0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77>,I2CCMD_END	//0xE3=NOP
void oledprinthex(int8_t pos,uint8_t c){
	register int i;

	if (pos>=0){
		i2coledout[4]=((pos<<3)&0x08);
		i2coledout[6]=((pos>>1)&0x07)|0x10;
		i2coledout[8]=((pos>>4)&0x07)|0xB0;
	}else{
		i2coledout[4]=0xE3;
		i2coledout[6]=0xE3;
		i2coledout[8]=0xE3;
	}
	for (i=0;i<4;++i){
		i2coledout[i+12]=oledhex8[(c>>4<<2)|i];
	}
	for (i=0;i<4;++i){
		i2coledout[i+16]=oledhex8[((c&0x0F)<<2)|64|i];
	}
}

void Reset_Handler(){
//clock settings
	/*comment out following codes if no crystal is attached*/
/*from here
	PDRUNCFG&=~0x20;			//XTAL power-on
	SYSPLLCLKSEL=0x01;			//select PLL clock source to XTAL
	SYSPLLCLKUEN=0x01;
	SYSPLLCLKUEN=0x00;
	SYSPLLCLKUEN=0x01;			//update PLL clock source
to here*/
	SYSPLLCTRL=0x23;			//set PLL to 48MHz(12MHz*4)
	PDRUNCFG&=~0x80;			//system PLL power-on
	while (!(SYSPLLSTAT&0x01));	//wait until PLL lock
	MAINCLKSEL=0x03;			//select main clock source to PLL output
	MAINCLKUEN=0x01;
	MAINCLKUEN=0x00;
	MAINCLKUEN=0x01;			//update main clock source
	SYSAHBCLKDIV=0x01;			//set system clock to 48MHz(48MHz/1)
	SYSAHBCLKCTRL=0x1AFFF;		//enable clock for peripherals (using GPIO,WDT,timers,I2C,SPI,ADC)
	WDTCLKSEL=0x01;				//select WDT clock source to main clock
	WDTCLKUEN=0x01;
	WDTCLKUEN=0x00;
	WDTCLKUEN=0x01;				//update WDT clock source
	WDTCLKDIV=0x01;				//set WDT clock to 12MHz(48MHz/4/1)
	SSP0CLKDIV=0x01;			//set SPI clock to 48MHz(48MHz/1)
	PRESETCTRL=0x03;			//clear reset for I2C,SPI
	PDRUNCFG&=~0x10;			//ADC power-on
//IO port settings
	//IOCON_RESET_PIO0_0=0xD0;	//pulled up input for reset
	IOCON_PIO0_1=0xF0;			//pulled up input for button 1
	IOCON_PIO0_2=0xF1;			//pulled up input for SPI_SS
	IOCON_PIO0_3=0xF0;			//pulled up input for button 2
	IOCON_PIO0_4=0x201;			//open drain io for I2C_SCL
	IOCON_PIO0_5=0x201;			//open drain io for I2C_SDA
	//IOCON_PIO0_6=0xD0;		//reserved
	//IOCON_PIO0_7=0xD0;		//reserved
	IOCON_PIO0_8=0xC1;			//standard output for SPI_MISO
	//IOCON_PIO0_9=0xD0;		//reserved
	IOCON_SWCLK_PIO0_10=0xD2;	//pulled up input for SPI_SCK
	//IOCON_R_PIO0_11=0xD0;		//reserved
	IOCON_R_PIO1_0=0x42;		//analog input for AD1
	//IOCON_R_PIO1_1=0xD0;		//reserved
	IOCON_R_PIO1_2=0xC3;		//standard output for PWMch1
	IOCON_SWDIO_PIO1_3=0xC3;	//standard output for PWMch2
	IOCON_PIO1_4=0xC2;			//standard output for PWMch3
	//IOCON_PIO1_5=0xD0;		//reserved
	IOCON_PIO1_6=0xD1;			//pulled up input for UART_RXD
	IOCON_PIO1_7=0xC1;			//standard output for UART_TXD
	//IOCON_PIO1_8=0xD0;		//reserved
	IOCON_PIO1_9=0x4C0;			//open drain output for red LED
	GPIO0DIR=0x3D0;
	GPIO1DIR=0x2BC;				//set IO direction
	GPIO0DATA[0x3D0]=0x350;
	GPIO1DATA[0x2BC]=0x29C;		//init output
//previous error handling
	if (SYSRSTSTAT&0x04){		//reset by WDT
		GPIO1DATA[P1_LEDr]=0;	//red LED on
		for (;;);				//halt
	}
//WDT settings
	WDTC=1199999;				//time-out after 0.1s
	WDMOD=0x03;					//enanble WDT (start on first feed)
//wall clock settings
	TMR16B0PR=1279;				//set period 1280(37500Hz)
	TMR16B0TCR=0x01;			//start counter
//interrupt settings
	ISER=0x800F8000;			//enable interrupts (for GPIO0,timers,I2C)
	IPR3=0x80000000;			//set priority (I2C=-2)
	IPR4=0x40000000;			//set priority (T4=-1)
//GPIO interrupt settings
	GPIO0IEV=0x08;				//set trigger at rising edge for IO3
	GPIO0IE=0x08;				//enable interrupt for IO3
//timer settings
	TMR32B1MR0=1279;			//set period 1280(37500Hz)
	TMR32B1MCR=0x03;			//set interrupt and reset on match #0
	TMR32B1PWMC=0x0E;			//enable PWM on ch1~3
	TMR32B1MR1=640;				//initial data 640
	TMR32B1MR2=640;				//initial data 640
	TMR32B1MR3=640;				//initial data 640
	TMR32B1TCR=0x01;			//start counter
//I2C settings
	I2C0SCLH=45;
	I2C0SCLL=75;				//set SCL to 400KHz(48MHz/(45+75))
	//I2C0CONCLR=0x6C;			//reset state
	I2C0CONSET=0x40;			//enable I2C
//SPI settings
	SSP0CR0=0x8F;				//set SPI data length to 16bit at mode 2
	SSP0CR1=0x06;				//enable SPI as slave
//ADC settings
	AD0INTEN=0x00;				//disable interrupt
	AD0CR=0x15902;				//set ADC to 10bit at 48484Hz(48MHz/90/11/1) for ch1

/*** idle process start ***/

	refreshwaitto=refreshwait;
	minbuf=0x8000;
	maxbuf=0x8000;
	min=0xFFFF;
	max=0x0000;

	i2cbusy=-1;
	i2coledrefresh=-1;
	i2coledout[0]=0x78;
	i2coledout[1]=I2CCMD_DATA;
	i2coledout[2]=6;
	i2coledout[3]=0x80;
	i2coledout[5]=0x80;
	i2coledout[7]=0x80;
	i2coledout[9]=0x40;
	i2coledout[10]=I2CCMD_DATA;
	i2coledout[11]=7;
	i2coledout[20]=I2CCMD_END;
	str2buf(i2coledoutbuf,"MIC test        min=0x????      max=0x????      I2CERRC=????????");
	i2coledmax=-1;
	i2coledi=0;
	i2coledfont8=0;
	i2coledfont=2;
	i2ccontrol[I2C_OLED].state=0x81;
	i2ccontrol[I2C_OLED].cmd=i2coledinit;
	i2ccontrol[I2C_OLED].next=&i2ccontrol[I2C_OLED];
	i2ccp=&i2ccontrol[I2C_MAX];
	i2cerrc=0;

	for (;;){//main loop
		register int temp;

		WDFEED=0xAA;
		WDFEED=0x55;			//feed watch dog

		if ((int16_t)(refreshwaitto-wallclock)<=0){
			refreshwaitto=wallclock+refreshwait;
			minbuf=min;
			maxbuf=max;
			if (min<=MINTHRESHOLD||max>=MAXTHRESHOLD){
				GPIO1DATA[P1_LEDr]=0;//red LED on
			}else{
				GPIO1DATA[P1_LEDr]=P1_LEDr;//red LED off
			}
			min=0xFFFF;
			max=0x0000;
			i2coledrefresh=1;
		}

		switch (i2ccontrol[I2C_OLED].state){
		case 0x01:
			i2ccontrol[I2C_OLED].cmd=i2coledout;
			i2ccontrol[I2C_OLED].state=0x00;
		case 0x00:
			if (i2coledi>=0){
				if (i2coledmax>=0&&i2coledmax<i2coledi){
					i2coledi=-1;
					break;
				}
				if (i2coledfont&0x2){//0=half height,2=fullheight
					temp=i2coledoutbuf[i2coledi>>1];
				}else{
					temp=i2coledoutbuf[i2coledi];
				}
				if (!temp&&i2coledmax<0){
					i2coledi=-1;
					break;
				}
				switch (i2coledfont){//0=char,1=hex
				case 0:
					oledprintdata(i2coledi,&oledchar8[temp<<3]);
					break;
				case 1:
					oledprinthex(i2coledi,temp);
					break;
				case 2:
					oledprintdata((i2coledi&~0x1F)+((i2coledi&1)<<4)+((i2coledi>>1)&0x0F),&oledchar16[i2coledi&0x01][temp<<3]);
					break;
				case 3:
					oledprintdata((i2coledi&~0x1F)+((i2coledi&1)<<4)+((i2coledi>>1)&0x0F),&oledhex16[(i2coledi&1?(temp&0x0F)|0x10:temp>>4)<<3]);
				}
				++i2coledi;
				i2ccontrol[I2C_OLED].state=0x80;
			}else if (i2coledrefresh){
				i2coledoutbuf[0x16]=toHex(minbuf>>12);
				i2coledoutbuf[0x17]=toHex((minbuf>>8)&0x0F);
				i2coledoutbuf[0x18]=toHex((minbuf>>4)&0x0F);
				i2coledoutbuf[0x19]=toHex(minbuf&0x0F);
				i2coledoutbuf[0x26]=toHex(maxbuf>>12);
				i2coledoutbuf[0x27]=toHex((maxbuf>>8)&0x0F);
				i2coledoutbuf[0x28]=toHex((maxbuf>>4)&0x0F);
				i2coledoutbuf[0x29]=toHex(maxbuf&0x0F);
				i2coledoutbuf[0x38]=toHex(i2cerrc>>28);
				i2coledoutbuf[0x39]=toHex((i2cerrc>>24)&0x0F);
				i2coledoutbuf[0x3A]=toHex((i2cerrc>>20)&0x0F);
				i2coledoutbuf[0x3B]=toHex((i2cerrc>>16)&0x0F);
				i2coledoutbuf[0x3C]=toHex((i2cerrc>>12)&0x0F);
				i2coledoutbuf[0x3D]=toHex((i2cerrc>>8)&0x0F);
				i2coledoutbuf[0x3E]=toHex((i2cerrc>>4)&0x0F);
				i2coledoutbuf[0x3F]=toHex(i2cerrc&0x0F);
				i2coledmax=0x7F;
				i2coledi=0x16;
				i2coledrefresh=0;
			}
		}

		if (i2cbusy<0&&(int16_t)(i2cresetwait-wallclock)<=0){
			i2cbusy=0;
		}
		if (!i2cbusy){
			//for (temp=I2C_MAX;temp;--temp){
				i2ccp=i2ccp->next;
				if (i2ccp->state&0x80){
					i2cbusy=1;
					i2ccp->cmdi=0;
					I2C0CONSET=I2CCMD_RESTART;	//start I2C
					//break;
				}
			//}
		}
	}
}

void I2C_IRQHandler(){
	register int c;

	switch (I2C0STAT){
	default:	//error
		I2C0CONSET=I2CCMD_END;
		i2cbusy=0;
		++i2cerrc;
		break;
	case 0x08:	//start
	case 0x10:	//restart
		i2ccp->count=0;
		i2ccp->ini=0;
		I2C0DAT=i2ccp->cmd[i2ccp->cmdi++];
		I2C0CONCLR=I2CCMD_RESTART;
		break;
	case 0x18:	//acceptW
	case 0x28:	//dataW
		if (!i2ccp->count){
			c=i2ccp->cmd[i2ccp->cmdi++];
			if (c){
				if (c==I2CCMD_END){
					i2ccp->state&=~0x80;
					i2cbusy=0;
				}
				I2C0CONSET=c;
			}else{
				i2ccp->count=i2ccp->cmd[i2ccp->cmdi++];
				I2C0DAT=i2ccp->cmd[i2ccp->cmdi++];
			}
		}else{
			I2C0DAT=i2ccp->cmd[i2ccp->cmdi++];
			--i2ccp->count;
		}
		break;
	case 0x40:	//acceptR
		i2ccp->count=i2ccp->cmd[i2ccp->cmdi++];
		if (i2ccp->count){
			I2C0CONSET=0x04;
		}
		break;
	case 0x50:	//dataR
		i2ccp->in[i2ccp->ini++]=I2C0DATA_BUFFER;
		if (!(--i2ccp->count)){
			I2C0CONCLR=0x04;
		}
		break;
	case 0x58:	//disconnectR
		i2ccp->in[i2ccp->ini++]=I2C0DATA_BUFFER;
		c=i2ccp->cmd[i2ccp->cmdi++];
		if (c==I2CCMD_END){
			i2ccp->state&=~0x80;
			i2cbusy=0;
		}
		I2C0CONSET=c;
	}
	I2C0CONCLR=0x08;			//acknoledgement
}

void PIOINT0_IRQHandler(){
	register int temp;

	GPIO0IC=GPIO0MIS;			//acknowledgement
	temp=(AD0DR1&0xFFFF)-0x8000+ADCBIAS;
	if (temp>32767){
		temp=32767;
	}
	if (temp<-32768){
		temp=-32768;
	}
	if (0x10&~SSP0SR){//SPI idle
		SSP0DR=(uint16_t)temp;
	}else{
		GPIO1DATA[P1_LEDr]=0;//red LED on
	}
}

void TIMER32_1_IRQHandler(){
	register int temp;

	TMR32B1IR=TMR32B1IR;		//acknowledgement
	temp=(AD0DR1&0xFFFF)+ADCBIAS;
	if (temp<0){
		temp=0;
	}
	if (temp>0xFFFF){
		temp=0xFFFF;
	}
	TMR32B1MR1=(temp>>6)+chbias;//echo
	TMR32B1MR2=(temp>>6)+chbias;//echo
	TMR32B1MR3=(temp>>6)+chbias;//echo
	if (temp<min){
		min=temp;
	}
	if (temp>max){
		max=temp;
	}
}
