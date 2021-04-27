
#include "hdr.h"

#ifdef SET_KBD

#include "main.h"
#include "mpr121.h"


//------------------------------------------------------------------------

//******************************************************************************************
/*
void kbdReadBuf(uint8_t from, uint8_t *buf, size_t len)
{
	HAL_I2C_Master_Transmit(portKBD, kbdAddr << 1, &from, 1, min_wait_ms);

	HAL_I2C_Master_Receive(portKBD, kbdAddr << 1, buf, len, max_wait_ms);
}
*/
//-------------------------------------------------------------------------------------------
void kbdWriteRegs(uint8_t reg, uint8_t *data, size_t len)
{
	if (HAL_I2C_Mem_Write(portKBD, kbdAddr << 1, reg, sizeof(reg), data, len, min_wait_ms) != HAL_OK) {
		devError |= devI2C;
		//cnt_err++;
	} else {
		devError &= ~devI2C;
	}
}
//-------------------------------------------------------------------------------------------
/*
uint8_t kbdReadReg(uint8_t reg)
{
	uint8_t ret = 0;

	if (HAL_I2C_Mem_Read(portKBD, kbdAddr << 1, reg, 1, &ret, 1, min_wait_ms) != HAL_OK) {
		devError |= devI2C;
		//cnt_err++;
	} else {
		devError &= ~devI2C;
	}

	return ret;
}
*/
//-------------------------------------------------------------------------------------------
void kbdReadRegs(uint8_t reg, uint8_t *data, size_t len)
{
	if (HAL_I2C_Mem_Read(portKBD, kbdAddr << 1, reg, 1, data, len, max_wait_ms) != HAL_OK) {
		devError |= devI2C;
		//cnt_err++;
	} else {
		devError &= ~devI2C;
	}
}
//-------------------------------------------------------------------------------------------
bool KBD_getAddr(int16_t *addr)
{
	int16_t i = 0, adr = KBD_ADDR1;
	uint8_t byte = 0x63;
	for (i = 0; i < 4; i++) {
		adr += i;
		if (HAL_I2C_Mem_Write(portKBD, adr << 1, SRST, 1, &byte, 1, max_wait_ms) == HAL_OK) {
			*addr = adr;
			return true;
		}
	}
    return false;
}
//-------------------------------------------------------------------------------------------
bool kbdInit()
{
	bool success = true;
	uint8_t ec;
  	uint8_t reg_value = 0;
  	uint8_t byte;
/*
void mpr121QuickConfig(void)
{

	 mpr121_irqInit();//interrupt set
  // Section A
  // This group controls filtering when data is > baseline.
  mpr121Write(MHD_R, 0x01);//0x2B
  mpr121Write(NHD_R, 0x01);//
  mpr121Write(NCL_R, 0x00);//
  mpr121Write(FDL_R, 0x00);//

  // Section B
  // This group controls filtering when data is < baseline.
  mpr121Write(MHD_F, 0x01);
  mpr121Write(NHD_F, 0x01);
  mpr121Write(NCL_F, 0xFF);
  mpr121Write(FDL_F, 0x02);//0x32

  // Section C
  // This group sets touch and release thresholds for each electrode
  mpr121Write(ELE0_T, TOU_THRESH);
  mpr121Write(ELE0_R, REL_THRESH);
  mpr121Write(ELE1_T, TOU_THRESH);
  mpr121Write(ELE1_R, REL_THRESH);
  mpr121Write(ELE2_T, TOU_THRESH);
  mpr121Write(ELE2_R, REL_THRESH);
  mpr121Write(ELE3_T, TOU_THRESH);
  mpr121Write(ELE3_R, REL_THRESH);
  mpr121Write(ELE4_T, TOU_THRESH);
  mpr121Write(ELE4_R, REL_THRESH);
  mpr121Write(ELE5_T, TOU_THRESH);
  mpr121Write(ELE5_R, REL_THRESH);
  mpr121Write(ELE6_T, TOU_THRESH);
  mpr121Write(ELE6_R, REL_THRESH);
  mpr121Write(ELE7_T, TOU_THRESH);
  mpr121Write(ELE7_R, REL_THRESH);
  mpr121Write(ELE8_T, TOU_THRESH);
  mpr121Write(ELE8_R, REL_THRESH);
  mpr121Write(ELE9_T, TOU_THRESH);
  mpr121Write(ELE9_R, REL_THRESH);
  mpr121Write(ELE10_T, TOU_THRESH);
  mpr121Write(ELE10_R, REL_THRESH);
  mpr121Write(ELE11_T, TOU_THRESH);
  mpr121Write(ELE11_R, REL_THRESH);

  // Section D
  // Set the Filter Configuration
  // Set ESI2
  mpr121Write(FIL_CFG, 0x04);//0x5D

  // Section E
  // Electrode Configuration
  // Enable 6 Electrodes and set to run mode
  // Set ELE_CFG to 0x00 to return to standby mode
  mpr121Write(ELE_CFG, 0x0C);//0x5E	// Enables all 12 Electrodes
  //mpr121Write(ELE_CFG, 0x06);//0x5E		// Enable first 6 electrodes

  // Section F
  // Enable Auto Config and auto Reconfig
  ////mpr121Write(ATO_CFG0, 0x0B);
  ////mpr121Write(ATO_CFGU, 0xC9);	// USL = (Vdd-0.7)/vdd*256 = 0xC9 @3.3V   mpr121Write(ATO_CFGL, 0x82);	// LSL = 0.65*USL = 0x82 @3.3V
  ////mpr121Write(ATO_CFGT, 0xB5);	// Target = 0.9*USL = 0xB5 @3.3V

}
*/

	// soft reset
	//write_register(SRST, 0x63);
	//byte = 0x63;
	//kbdWriteRegs(SRST, &byte, 1);
	//HAL_Delay(1);
/*
	// read AFE Configuration 2
	//read_register(AFE2, &reg_value);
	kbdReadRegs(AFE2, &reg_value, 1);
	// check default value
	if (reg_value != 0x24) {
		//reg_value = 0x24;
		//kbdWriteRegs(AFE2, &reg_value, 1);
		return false;
	}
*/
	// read Touch Status register
	//read_register(TS2, &reg_value);
	kbdReadRegs(TS2, &reg_value, 1);
	if (reg_value & 0x80) {//clear OVCF
		reg_value = 0x80;
		kbdWriteRegs(TS2, &reg_value, 1);

		kbdReadRegs(TS2, &reg_value, 1);
		if (reg_value & 0x80) {
			return false;
		}
	}

	// if no previous error
	//if (success) {
		byte = 0;
		kbdWriteRegs(ECR, &byte, 1);// turn off all electrodes to stop

		uint8_t data[] = {
			1, 1, 0x10, 0x20, 1, 1, 0x10, 0x20,
			1, 0x10, 0xFF, 0x0F, 0x0F, 0, 0, 1,
			1, 0xFF, 0xFF, 0, 0, 0
		};
		kbdWriteRegs(MHDR, data, sizeof(data));//0x2B ... 0x40

		data[0] = 0x11;
		data[1] = 0xD0;//0x5C - 16uA
		data[2] = 0x14;//0x5D - Period set to 16 ms (Default)
		kbdWriteRegs(DTR, data, 3);//0x5B ... 0x5D

		memset(data, 0, 5);
		kbdWriteRegs(ACCR0, data, 5);//0x7B ... 0x7F

		byte = 0xCC;
		kbdWriteRegs(ECR, &byte, 1);

		// apply next setting for all electrodes
		data[0] = 40;//5..0x30
		data[1] = 20;
		byte = E0TTH;
		for (ec = 0; ec < NUM_OF_ELECTRODES; ec++) {
			kbdWriteRegs(byte, data, 2);
			byte += 2;
		}

		byte = 0x10;
		kbdWriteRegs(ECR, &byte, 1);// enable electrodes and set the current to 16uA
	//}

	return success;
}
//-------------------------------------------------------------------------------------------
uint16_t kbd_get_touch()// get touch status
{
	uint8_t data[2] = {0};
	kbdReadRegs(TS1, data, 2);
	/*if (data[1] & 0x80) {
		uint8_t byte = 0x80;
		kbdWriteRegs(TS2, &byte, 1);
	}*/
	uint16_t ret = data[1];
	ret <<= 8;
	ret |= data[0];

	return ret;

/*
	//int tn = 0;
	//for (int j = 0; j < NUM_OF_ELECTRODES - 1; j++) if ((ret & (1 << j))) tn++;

	uint16_t key = 0;
	//if (tn == 1) {
		     if (ret & (1 << STAR))  key = 0x2a;//'*';
		else if (ret & (1 << SEVEN)) key = 0x37;//'7';
		else if (ret & (1 << FOUR))  key = 0x34;//'4';
		else if (ret & (1 << ONE))   key = 0x31;//'1';
		else if (ret & (1 << ZERO))  key = 0x30;//'0';
		else if (ret & (1 << EIGHT)) key = 0x38;//'8';
		else if (ret & (1 << FIVE))  key = 0x35;//'5';
		else if (ret & (1 << TWO))   key = 0x32;//'2';
		else if (ret & (1 << POUND)) key = 0x23;//'#';
		else if (ret & (1 << NINE))  key = 0x39;//'9';
		else if (ret & (1 << SIX))   key = 0x36;//'6';
		else if (ret & (1 << THREE)) key = 0x33;//'3';
	//}

	return key;
*/
}

/*
char getPhoneNumber()
{
  int touchNumber;
	int j;
  uint16_t touchstatus;
	char key=-1;
  //Serial.println("Please Enter a phone number...");

    //while(key_pressed);//用while读取会阻塞程序运行
	if(key_pressed==0)//非阻塞方式
	{
	key_pressed=1;
    touchNumber = 0;

    touchstatus = mpr121Read(0x01) << 8;
    touchstatus |= mpr121Read(0x00);

    for (j=0; j<12; j++)  // Check how many electrodes were pressed
    {
      if ((touchstatus & (1<<j)))
        touchNumber++;
    }

    if (touchNumber == 1)
    {
      if (touchstatus & (1<<STAR))
        key = '*';
      else if (touchstatus & (1<<SEVEN))
        key = '7';
      else if (touchstatus & (1<<FOUR))
        key= '4';
      else if (touchstatus & (1<<ONE))
        key = '1';
      else if (touchstatus & (1<<ZERO))
        key= '0';
      else if (touchstatus & (1<<EIGHT))
        key = '8';
      else if (touchstatus & (1<<FIVE))
        key = '5';
      else if (touchstatus & (1<<TWO))
        key = '2';
      else if (touchstatus & (1<<POUND))
        key = '#';
      else if (touchstatus & (1<<NINE))
        key = '9';
      else if (touchstatus & (1<<SIX))
        key = '6';
      else if (touchstatus & (1<<THREE))
        key = '3';

      //Serial.print(key[i]);

    }
    else if (touchNumber == 0);
    else;
      //Serial.println("Only touch ONE button!");
	}
		return key;
}
*/
//-----------------------------------------------------------------------------------------

//******************************************************************************************

#endif

