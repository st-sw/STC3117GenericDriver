#define STC311x_SLAVE_ADDRESS            0xE0   /* STC31xx 8-bit address byte */
#define STC311x_7BIT_SLAVE_ADDRESS       0x70   /* STC31xx 7-bit address byte */


int I2C_WriteBytes(int I2cSlaveAddr, int RegAddress, unsigned char * TxBuffer, int NumberOfBytes);
int I2C_ReadBytes(int I2cSlaveAddr, int RegAddress, unsigned char * RxBuffer, int NumberOfBytes);
