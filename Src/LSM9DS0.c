/**
  ******************************************************************************
  * @file    LSM9DS0.c
  * @author  George Vigelette
  * @version V1.0.0
  * @date    28-April-2015
  * @brief   This file includes the 9-Axis Mems driver.
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "LSM9DS0.h"
#include "cmsis_os.h"

#include <math.h>

#define I2C_COMMS
/* Private typedef -----------------------------------------------------------*/

#ifdef I2C_COMMS
extern I2C_HandleTypeDef hi2c1;
#else
extern SPI_HandleTypeDef hspi1;
#endif

/* Private define ------------------------------------------------------------*/

//#define 		LSM_GYRO_ENABLE()  					HAL_GPIO_WritePin(GPIOI, GPIO_PIN_11, GPIO_PIN_RESET)
//#define 		LSM_GYRO_DISABLE()  				HAL_GPIO_WritePin(GPIOI, GPIO_PIN_11, GPIO_PIN_SET)

#define 		LSM_GYRO_CS_LOW()  					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET)
#define 		LSM_GYRO_CS_HIGH()  				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET)

#define 		LSM_XM_CS_LOW()  						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)
#define 		LSM_XM_CS_HIGH()  					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)


#define 		LSM_READ_MODIFIER 					0x80 
#define 		LSM_AUTO_INCR_MODIFIER			0x40	 

#define 		TEMP_DECIMAL_DIGITS             (2)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static uint8_t TEMP_Initialized = 0;
static uint8_t MAG_Initialized = 0;
static uint8_t ACC_Initialized = 0;
static uint8_t GYRO_Initialized = 0;

static uint8_t MAG_Scale;
static uint8_t ACC_Scale;
static uint8_t GYRO_Scale;

static float MAG_Sensitivity;
static float ACC_Sensitivity;
static float GYRO_Sensitivity;

/* Private function prototypes -----------------------------------------------*/
static void LSM_Error_Handler(void);
static uint8_t LSM_ReadRegister(uint8_t reg);
static void LSM_WriteRegister(uint8_t reg, uint8_t val);
static void LSM_MultiByte_ReadRegister(uint8_t reg, uint8_t* pBuffer, uint8_t bufferSize);


/* Private functions ---------------------------------------------------------*/

/******************************************************************************
*	LSM9DS0 Low Level Read/Write function definitions
* 
*******************************************************************************/

#ifdef I2C_COMMS
static uint8_t i2c_xmReadByte(uint8_t address){
	
	uint8_t val = 0x00;
	
	while(HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)LSM9_XM_ADDRESS_WR, &address, 1, 100)!= HAL_OK){
		/* Error_Handler() function is called when Timout error occurs.
       When Acknowledge failure ocucurs (Slave don't acknowledge it's address)
       Master restarts communication */
    if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    }
	}
	
	while(HAL_I2C_Master_Receive(&hi2c1, (uint16_t)LSM9_XM_ADDRESS_RD, &val, 1, 100) != HAL_OK){
		/* Error_Handler() function is called when Timout error occurs.
       When Acknowledge failure ocucurs (Slave don't acknowledge it's address)
       Master restarts communication */
    if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    }
	}
	
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	return val;
}

static void i2c_xmWriteByte(uint8_t address, uint8_t val){
	
	uint8_t buffer[2];
	buffer[0] = address;
	buffer[1] = val;
	
	while(HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)LSM9_XM_ADDRESS_WR, buffer, 2, 100)!= HAL_OK){
		/* Error_Handler() function is called when Timout error occurs.
       When Acknowledge failure ocucurs (Slave don't acknowledge it's address)
       Master restarts communication */
    if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    }
	}
	
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}


static uint8_t i2c_gReadByte(uint8_t address){
	
	uint8_t val = 0x00;
	
	while(HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)LSM9_GYRO_ADDRESS_WR, &address, 1, 100)!= HAL_OK){
		/* Error_Handler() function is called when Timout error occurs.
       When Acknowledge failure ocucurs (Slave don't acknowledge it's address)
       Master restarts communication */
    if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    }
	}
	
	while(HAL_I2C_Master_Receive(&hi2c1, (uint16_t)LSM9_GYRO_ADDRESS_RD, &val, 1, 100) != HAL_OK){
		/* Error_Handler() function is called when Timout error occurs.
       When Acknowledge failure ocucurs (Slave don't acknowledge it's address)
       Master restarts communication */
    if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    }
	}
	
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	return val;
}

static void i2c_gWriteByte(uint8_t address, uint8_t val){
	
	uint8_t buffer[2];
	buffer[0] = address;
	buffer[1] = val;
	
	while(HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)LSM9_GYRO_ADDRESS_WR, buffer, 2, 100)!= HAL_OK){
		/* Error_Handler() function is called when Timout error occurs.
       When Acknowledge failure ocucurs (Slave don't acknowledge it's address)
       Master restarts communication */
    if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    }
	}
	
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}
#else
static uint8_t LSM_ReadRegister(uint8_t reg){
	uint8_t val = 0xFF;
	uint8_t mreg = reg | LSM_READ_MODIFIER;
	
	/* Wait for SPI1 Tx buffer empty */ 
  while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
	
  /* Send SPI1 data */   
  if(HAL_SPI_Transmit(&hspi1,(uint8_t *)&mreg,1,100) != HAL_OK){
		LSM_Error_Handler();		
	}
	
	/* Wait for SPI1 Tx buffer empty */ 
  while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
		
	if(HAL_SPI_Receive(&hspi1, &val, 1, 500) != HAL_OK){
		LSM_Error_Handler();		
	}
	
	/* Wait for SPI1 Rx buffer empty */ 
  while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_RX);
	
	return val;
}

static void LSM_MultiByte_ReadRegister(uint8_t reg, uint8_t* pBuffer, uint8_t bufferSize){
	uint8_t mreg = reg | LSM_READ_MODIFIER | LSM_AUTO_INCR_MODIFIER;
	
	/* Wait for SPI1 Tx buffer empty */ 
  while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
	
  /* Send SPI1 data */   
  if(HAL_SPI_Transmit(&hspi1,(uint8_t *)&mreg,1,100) != HAL_OK)
	{
		LSM_Error_Handler();		
	}
	
	/* Wait for SPI1 Tx buffer empty */ 
  while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
		
	if(HAL_SPI_Receive(&hspi1, pBuffer, bufferSize, 500) != HAL_OK){
		LSM_Error_Handler();		
	}
	
	/* Wait for SPI1 Rx buffer empty */ 
  while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_RX);
}

void LSM_WriteRegister(uint8_t reg, uint8_t val){
	uint8_t mreg = reg;
	
	/* Wait for SPI1 Tx buffer empty */ 
  while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
	
  /* Send SPI1 data */   
  if(HAL_SPI_Transmit(&hspi1,(uint8_t *)&mreg,1,100) != HAL_OK){
		LSM_Error_Handler();		
	}
	
	/* Wait for SPI1 Tx buffer empty */ 
  while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
		
  /* Send SPI1 data */   
  if(HAL_SPI_Transmit(&hspi1,(uint8_t *)&val,1,100) != HAL_OK){	
		LSM_Error_Handler();
	}
	
	/* Wait for SPI1 Tx buffer empty */ 
  while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
}

#endif
/******************************************************************************
*	LSM9DS0 GYRO Low Level Read/Write function definitions
* 
*******************************************************************************/

uint8_t LSM_GYRO_ReadRegister(uint8_t reg){	
	
	uint8_t ret = 0xFF;
#ifdef I2C_COMMS
	LSM_GYRO_CS_HIGH();
	LSM_XM_CS_HIGH();
	ret = i2c_gReadByte(reg);
#else
	LSM_GYRO_CS_LOW();
	ret = LSM_ReadRegister(reg);	
	LSM_GYRO_CS_HIGH();
#endif
	return ret;
}

void LSM_GYRO_WriteRegister(uint8_t reg, uint8_t val){
#ifdef I2C_COMMS
	LSM_GYRO_CS_HIGH();
	LSM_XM_CS_HIGH();
	i2c_gWriteByte(reg, val);
#else
	LSM_GYRO_CS_LOW();	
	LSM_WriteRegister(reg, val);
	LSM_GYRO_CS_HIGH();	
#endif
}

/******************************************************************************
*	LSM9DS0 XM Low Level Read/Write function definitions
* 
*******************************************************************************/

uint8_t LSM_XM_ReadRegister(uint8_t reg){	
	uint8_t ret = 0xFF;
#ifdef I2C_COMMS
	LSM_GYRO_CS_HIGH();
	LSM_XM_CS_HIGH();
	ret = i2c_xmReadByte(reg);
#else
	LSM_XM_CS_LOW();
	ret = LSM_ReadRegister(reg);	
	LSM_XM_CS_HIGH();
#endif
	return ret;	
}

void LSM_XM_MultiByte_ReadData(uint8_t reg, uint8_t* pBuffer, uint8_t bufferSize){		
	
	
#ifdef I2C_COMMS
	
#else
	LSM_XM_CS_LOW();	
	LSM_MultiByte_ReadRegister(reg, pBuffer, bufferSize);
	LSM_XM_CS_HIGH();	
#endif
}

void LSM_XM_WriteRegister(uint8_t reg, uint8_t val){	
#ifdef I2C_COMMS
	LSM_GYRO_CS_HIGH();
	LSM_XM_CS_HIGH();
	i2c_xmWriteByte(reg, val);
#else
	LSM_XM_CS_LOW();	
	LSM_WriteRegister(reg, val);
	LSM_XM_CS_HIGH();
#endif
}

static void LSM_Error_Handler(void)
{
  /* Toggle LED4 on */
  while(1)
  {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    osDelay(100);
  }
}


/* Exported functions --------------------------------------------------------*/

/******************************************************************************
*	LSM9DS0 Who AM I function definitions
* 
*******************************************************************************/
void LSM_InitChip(void){
	LSM_GYRO_CS_HIGH();
	LSM_XM_CS_HIGH();
#ifdef I2C_COMMS
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
#endif
	osDelay(25);
}

uint8_t LSM_GYRO_Whoami(void){
	return LSM_GYRO_ReadRegister(LSM9_WHO_AM_I_G);	
}

uint8_t LSM_XM_Whoami(void){
	return LSM_XM_ReadRegister(LSM9_WHO_AM_I_XM);	
}

/******************************************************************************
*	LSM9DS0 Temperature sensor function definitions
* 
*******************************************************************************/

uint8_t LSM_TEMP_isInitialized(void){	
	return TEMP_Initialized;
}

LSM_BSP_StatusTypeDef LSM_TEMP_Init(void){
	LSM_TEMP_Enable();
	return LSM_OK;
}

void LSM_TEMP_Enable(void){	
	uint8_t bwr = 0x00;
	uint8_t ctrl_reg =0x00;
	ctrl_reg	= LSM_XM_ReadRegister(LSM9_CTRL_REG5_XM);
	
	// check TEMP Enable Bit
	if((ctrl_reg & LSM9_XM_TEMP_ENABLE) != LSM9_XM_TEMP_ENABLE)
	{
		bwr = 0x01;
		ctrl_reg |= LSM9_XM_TEMP_ENABLE;		
	}
	
	// check ODR
	if(((ctrl_reg & LSM9_M_ODR_MASK) != LSM9_M_ODR_RES1) || ((ctrl_reg & LSM9_M_ODR_MASK) != LSM9_M_ODR_RES2)) 
	{
		bwr = 0x01;
		ctrl_reg &= ~(LSM9_M_ODR_MASK);		// clear ODR bits
		ctrl_reg |= LSM9_M_ODR_50Hz;		// default to 50hz
	}
	
	if(bwr){
		LSM_XM_WriteRegister(LSM9_CTRL_REG5_XM, ctrl_reg);	
	}
	
	TEMP_Initialized = 0x01;
}

void LSM_TEMP_Disable(void){	
	uint8_t bwr = 0x00;
	uint8_t ctrl_reg =0x00;
	ctrl_reg	= LSM_XM_ReadRegister(LSM9_CTRL_REG5_XM);
	
	// check TEMP Enable Bit
	if((ctrl_reg & LSM9_XM_TEMP_ENABLE) == LSM9_XM_TEMP_ENABLE)
	{
		bwr = 0x01;
		ctrl_reg &= ~(LSM9_XM_TEMP_ENABLE);	// clear temp enable bit	
	}
	
	if(bwr){
		LSM_XM_WriteRegister(LSM9_CTRL_REG5_XM, ctrl_reg);	
	}
	
	TEMP_Initialized = 0x00;
}

int16_t LSM_TEMP_Read(void){	
	int16_t T_out;
  uint8_t tempReg[2] = {0,0};
	
	//LSM_XM_MultiByte_ReadData(LSM9_OUT_TEMP_L_XM, tempReg, 2);	
	T_out  = (((int16_t) tempReg[1] << 12) | tempReg[0] << 4 ) >> 4;;
	
	return T_out;
}


/******************************************************************************
*	LSM9DS0 Magnetometer sensor function definitions
* 
*******************************************************************************/

uint8_t LSM_Mag_isInitialized(void){
	return MAG_Initialized;
}

void LSM_Mag_calcRes(void)
{
    uint8_t tempReg = 0x00;
		tempReg = LSM_XM_ReadRegister(LSM9_CTRL_REG6_XM);
		tempReg &= LSM9_M_FS_MASK;
	
		switch(tempReg)
    {
      case LSM9_M_FS_2:
        MAG_Sensitivity = 0.08;
        break;
      case LSM9_M_FS_4:
        MAG_Sensitivity = 0.16;
        break;
      case LSM9_M_FS_8:
        MAG_Sensitivity = 0.32;
        break;
			case LSM9_M_FS_12:
        MAG_Sensitivity = 0.48;
        break;
    }
}

float LSM_Mag_Get_Sensitivity(void){
	return MAG_Sensitivity;
}

float LSM_Mag_Get_Scaled(int16_t raw_data){
	return ((float)raw_data * MAG_Sensitivity);
}

float LSM_Mag_Get_Measurement(int16_t raw_data){
	return ((float)raw_data * MAG_Sensitivity);
}

LSM_BSP_StatusTypeDef LSM_Mag_Init(void){	
	
	uint8_t tmp_ctrl_reg =0x00;	
	
	// LSM9_CTRL_REG5_XM (odr and resolution)
	
	tmp_ctrl_reg	= LSM_XM_ReadRegister(LSM9_CTRL_REG5_XM);
		
	tmp_ctrl_reg &= ~(LSM9_M_ODR_MASK);		// clear ODR bits
	tmp_ctrl_reg |= LSM9_M_ODR_50Hz;		  // default to 50hz
	tmp_ctrl_reg |= LSM9_XM_TEMP_ENABLE;   // enable temp sensor
	
	LSM_XM_WriteRegister(LSM9_CTRL_REG5_XM, tmp_ctrl_reg);		
	
	// LSM9_CTRL_REG6_XM (Full Scale)
	tmp_ctrl_reg = LSM_XM_ReadRegister(LSM9_CTRL_REG6_XM);
	tmp_ctrl_reg &= ~(LSM9_M_FS_MASK);
	tmp_ctrl_reg |= LSM9_M_FS_2;
	LSM_XM_WriteRegister(LSM9_CTRL_REG6_XM, tmp_ctrl_reg);  // this register is only used for MAG so just write it Mag scale to +/- 2GS
	
	// LSM9_CTRL_REG7_XM (Sensor Mode)
	tmp_ctrl_reg	= LSM_XM_ReadRegister(LSM9_CTRL_REG7_XM);
	tmp_ctrl_reg = LSM9_CTRL_REG7_XM_DEFAULT;  // mode will be set to power down
  tmp_ctrl_reg &=	~(LSM9_M_MODE_MASK);		// clear mode
	tmp_ctrl_reg |=	LSM9_M_MODE_CONTINUOUS;		// set mode to continuous
	LSM_XM_WriteRegister(LSM9_CTRL_REG7_XM, tmp_ctrl_reg); // write register
	
	// calculate scale
	LSM_Mag_calcRes();
	
	// set up interrupts
	
	MAG_Initialized = 0x01;
	return LSM_OK;
}

LSM_BSP_StatusTypeDef LSM_Mag_Set_Hires(void){
	
	uint8_t ctrl_reg =0x00;	
	
	if(MAG_Initialized){		
		ctrl_reg	= LSM_XM_ReadRegister(LSM9_CTRL_REG5_XM);
		ctrl_reg |= LSM9_M_RES_MASK;		// set both bits
		LSM_XM_WriteRegister(LSM9_CTRL_REG5_XM, ctrl_reg);			
		return LSM_OK;
	}
	
	return LSM_ERROR;
	
}

LSM_BSP_StatusTypeDef LSM_MagSet_Lores(void){
	
	uint8_t ctrl_reg =0x00;	
	
	if(MAG_Initialized){		
		ctrl_reg	= LSM_XM_ReadRegister(LSM9_CTRL_REG5_XM);
		ctrl_reg &= ~(LSM9_M_RES_MASK);		// clear 
		LSM_XM_WriteRegister(LSM9_CTRL_REG5_XM, ctrl_reg);			
		return LSM_OK;
	}
	
	return LSM_ERROR;
	
}

LSM_BSP_StatusTypeDef LSM_Mag_Set_Scale(uint8_t scale){	
	//MAG_Scale = scale;
	//LSM_MAG_calcRes();
	return LSM_NOT_IMPLEMENTED;
}

LSM_BSP_StatusTypeDef LSM_Mag_Set_Mode(uint8_t mode){
	return LSM_NOT_IMPLEMENTED;
}

uint8_t LSM_Mag_Get_Status(void){
	return LSM_XM_ReadRegister(LSM9_STATUS_REG_M);
}

void LSM_Mag_ReadRaw(int16_t* pData){
	uint8_t loByte = 0x00;
	uint8_t hiByte = 0x00;
	uint8_t status = 0x00;
	
	status = LSM_Mag_Get_Status();
	
	// get X
	loByte = LSM_XM_ReadRegister(LSM9_OUT_X_L_M);	
  hiByte = LSM_XM_ReadRegister(LSM9_OUT_X_H_M);
  pData[0]  = ((((int16_t)hiByte) << 8)+(int16_t)loByte);
	
	// get Y
	loByte = LSM_XM_ReadRegister(LSM9_OUT_Y_L_M);	
	hiByte = LSM_XM_ReadRegister(LSM9_OUT_Y_H_M);
  pData[1] = ((((int16_t)hiByte) << 8)+(int16_t)loByte);
		
	// get Z	
	loByte = LSM_XM_ReadRegister(LSM9_OUT_Z_L_M);	
  hiByte = LSM_XM_ReadRegister(LSM9_OUT_Z_H_M);
  pData[2]  = ((((int16_t)hiByte) << 8)+(int16_t)loByte);
}

/******************************************************************************
*	LSM9DS0 Accel function declarations
* 
*******************************************************************************/

uint8_t LSM_Acc_isInitialized(void){
	return ACC_Initialized;
}

void LSM_Acc_calcRes(void)
{
    uint8_t tempReg = 0x00;
		tempReg = LSM_XM_ReadRegister(LSM9_CTRL_REG2_XM);
		tempReg &= LSM9_A_FULL_SCALE_MASK;
	
		switch(tempReg)
    {
      case LSM9_A_FS_2:
        ACC_Sensitivity = 0.061;
        break;
      case LSM9_A_FS_4:
        ACC_Sensitivity = 0.122;
        break;
      case LSM9_A_FS_6:
        ACC_Sensitivity = 0.183;
        break;
			case LSM9_A_FS_8:
        ACC_Sensitivity = 0.244;
        break;
			case LSM9_A_FS_16:
        ACC_Sensitivity = 0.732;
        break;
    }
}

float LSM_Acc_Get_Measurement(int16_t raw_data){
	return ((float)raw_data * ACC_Sensitivity)*9.8f/1000.f;
}

float LSM_Acc_Get_Scaled(int16_t raw_data){
	return ((float)raw_data * ACC_Sensitivity);
}

float LSM_Acc_Get_Sensitivity(void){
	return ACC_Sensitivity;
}

LSM_BSP_StatusTypeDef LSM_Acc_Init(void){
	
	uint8_t tmp_ctrl_reg =0x00;	
	
	
	// LSM9_CTRL_REG0_XM (filter and fifo)
	
	
	// LSM9_CTRL_REG1_XM (odr and axis)
	tmp_ctrl_reg	= LSM_XM_ReadRegister(LSM9_CTRL_REG1_XM);
	tmp_ctrl_reg &=	~(LSM9_A_ODR_MASK);		// clear odr
	tmp_ctrl_reg &=	~(LSM9_A_XYZ_ENABLE_MASK);		// AXIS Enable bits
	tmp_ctrl_reg &=	~(LSM9_XM_BDU_BIT_MASK);		// clear BDU bit
	
	tmp_ctrl_reg |= LSM9_A_ODR_50Hz;		// set odr to 50hz
	tmp_ctrl_reg |= LSM9_XM_BDU_ENABLE;		// set BDU
	tmp_ctrl_reg |= LSM9_A_AXIS_ENABLE_ALL;		// set all axis to enable
	
	LSM_XM_WriteRegister(LSM9_CTRL_REG1_XM, tmp_ctrl_reg);	// should be 0x5F
	
	
	// LSM9_CTRL_REG2_XM (FS and self test)
	tmp_ctrl_reg =0x00;	
	tmp_ctrl_reg	= LSM_XM_ReadRegister(LSM9_CTRL_REG2_XM);
	
	tmp_ctrl_reg &=	~(LSM9_A_BANDWIDTH_MASK);		// clear BW filter
	tmp_ctrl_reg &=	~(LSM9_A_FULL_SCALE_MASK);		// clear full scale bits
	tmp_ctrl_reg &=	~(LSM9_A_SELF_TEST_MASK);		// clear self test bits
		
	tmp_ctrl_reg |= LSM9_A_FS_2;		// set full scale
	tmp_ctrl_reg |= LSM9_A_ST_NORMAL;		// set all axis to enable
	//tmp_ctrl_reg |= LSM9_A_ST_POSITIVE;		// set positve selftest
	//tmp_ctrl_reg |= LSM9_A_ST_NEGATIVE;		// set negative selftest
	
	LSM_XM_WriteRegister(LSM9_CTRL_REG2_XM, tmp_ctrl_reg);	// should be 0x00
	
	// LSM9_CTRL_REG3_XM (interrupts)
	// LSM9_CTRL_REG4_XM (interrupts)
	
	// calculate scale
	LSM_Acc_calcRes();
	
	ACC_Initialized = 0x01;
	return LSM_OK; 
}

uint8_t LSM_Acc_Get_Status(void){
	return LSM_XM_ReadRegister(LSM9_STATUS_REG_A);	
}

void LSM_Acc_ReadRaw(int16_t* pData){
	uint8_t loByte = 0x00;
	uint8_t hiByte = 0x00;
	uint8_t	status = 0x00;
	
	status = LSM_Acc_Get_Status();
	
	// get X
	loByte = LSM_XM_ReadRegister(LSM9_OUT_X_L_A);	
  hiByte = LSM_XM_ReadRegister(LSM9_OUT_X_H_A);
  pData[0] = ((((int16_t)hiByte) << 8)+(int16_t)loByte);
	
	// get Y
	loByte = LSM_XM_ReadRegister(LSM9_OUT_Y_L_A);	
	hiByte = LSM_XM_ReadRegister(LSM9_OUT_Y_H_A);
  pData[1]  = ((((int16_t)hiByte) << 8)+(int16_t)loByte);
		
	// get Z	
	loByte = LSM_XM_ReadRegister(LSM9_OUT_Z_L_A);	
  hiByte = LSM_XM_ReadRegister(LSM9_OUT_Z_H_A);
  pData[2]  = ((((int16_t)hiByte) << 8)+(int16_t)loByte);
}

/******************************************************************************
*	LSM9DS0 Gyro function declarations
* 
*******************************************************************************/

uint8_t LSM_Gyro_isInitialized(void){
	return GYRO_Initialized;
}

void LSM_Gyro_calcRes(void)
{
    uint8_t tempReg = 0x00;
		tempReg = LSM_GYRO_ReadRegister(LSM9_CTRL_REG4_G);
		tempReg &= LSM9_G_FULL_SCALE_MASK;
	
		switch(tempReg)
    {
      case LSM9_G_FS_245DPS:
        GYRO_Sensitivity = 8.75;
        break;
      case LSM9_G_FS_500DPS:
        GYRO_Sensitivity = 17.50;
        break;
      case LSM9_G_FS_2000DPS:
        GYRO_Sensitivity = 70;
        break;
			default:
        GYRO_Sensitivity = 70;
        break;
    }
}

float LSM_Gyro_Get_Scaled(int16_t raw_data){	
	return ((float)raw_data * GYRO_Sensitivity);
}


float LSM_Gyro_Get_Sensitivity(void){
	return GYRO_Sensitivity;
}

LSM_BSP_StatusTypeDef LSM_Gyro_Init(void){
		
	uint8_t tmp_ctrl_reg =0x00;	
	
		
	// LSM9_CTRL_REG1_G (odr, power, and axis)
	tmp_ctrl_reg	= LSM9_CTRL_REG1_G_DEFAULT;
	tmp_ctrl_reg &=	~(LSM9_G_ODR_MASK);		// clear odr
	tmp_ctrl_reg &=	~(LSM9_G_BANDWIDTH_MASK);		// clear bw
	tmp_ctrl_reg &=	~(LSM9_G_XYZ_ENABLE_MASK);		// AXIS Enable bits
	tmp_ctrl_reg &=	~(LSM9_G_POWERDOWN_ENABLE_MASK);		// power down
	
	tmp_ctrl_reg |= LSM9_G_ODR_95Hz;		// set odr to 50hz
	tmp_ctrl_reg |= LSM9_G_POWERDOWN_ENABLE_MASK;		// set PD to normal mode
	tmp_ctrl_reg |= LSM9_G_AXIS_ENABLE_ALL;		// set all axis to enable
	
	LSM_GYRO_WriteRegister(LSM9_CTRL_REG1_G, tmp_ctrl_reg);	// should be 0x5F
	
	
	// LSM9_CTRL_REG2_G (High Pass Filter)
	
	// LSM9_CTRL_REG3_G (interrupt)
	
	// LSM9_CTRL_REG4_G (Full Scale, Block update, and Self Test)
	tmp_ctrl_reg	= LSM9_CTRL_REG4_G_DEFAULT;
			
	tmp_ctrl_reg |= LSM9_G_FS_245DPS;		// set full scale
	tmp_ctrl_reg |= LSM9_G_BDU_ENABLE;		// set BDU
	tmp_ctrl_reg |= LSM9_G_ST_NORMAL;
	//tmp_ctrl_reg |= LSM9_G_ST_POSITIVE;
	//tmp_ctrl_reg |= LSM9_G_ST_NEGATIVE;
	
	LSM_GYRO_WriteRegister(LSM9_CTRL_REG4_G, tmp_ctrl_reg);	// should be 0x00
	
	// LSM9_CTRL_REG5_G (Fifo and itnerrupt)
	LSM_Gyro_calcRes();
	GYRO_Initialized = 0x01;
	return LSM_OK; 
}

uint8_t LSM_Gyro_Get_Status(void){
	return LSM_GYRO_ReadRegister(LSM9_STATUS_REG_G);		
}

void LSM_Gyro_ReadRaw(int16_t* pData){
	uint8_t loByte = 0x00;
	uint8_t hiByte = 0x00;
	
	uint8_t status = 0x00;
	
	status = LSM_Gyro_Get_Status();
	
	// get X
	loByte = LSM_GYRO_ReadRegister(LSM9_OUT_X_L_G);	
  hiByte = LSM_GYRO_ReadRegister(LSM9_OUT_X_H_G);
  pData[0]  = ((((int16_t)hiByte) << 8)+(int16_t)loByte);
	
	// get Y
	loByte = LSM_GYRO_ReadRegister(LSM9_OUT_Y_L_G);	
	hiByte = LSM_GYRO_ReadRegister(LSM9_OUT_Y_H_G);
  pData[1]  = ((((int16_t)hiByte) << 8)+(int16_t)loByte);
		
	// get Z	
	loByte = LSM_GYRO_ReadRegister(LSM9_OUT_Z_L_G);	
  hiByte = LSM_GYRO_ReadRegister(LSM9_OUT_Z_H_G);
  pData[2] = ((((int16_t)hiByte) << 8)+(int16_t)loByte);	
}


/************************ END OF FILE *****************************************/

