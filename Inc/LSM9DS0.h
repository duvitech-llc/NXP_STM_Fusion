/**
  ******************************************************************************
  * @file    LSM9DSO.h
  * @author  George Vigelette
  * @version V1.0.0
  * @date    28-April-2015
  * @brief   This file contains all the functions prototypes for the 9-Axis Mems driver.
  ******************************************************************************	
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LSM9DS0_H
#define __LSM9DS0_H

#ifdef __cplusplus
 extern "C" {
#endif 

	 
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
	 
/* Defines -------------------------------------------------------------------*/	  

#define BIT(x) ( 1<<(x) )

#define LSM9_XM_ADDRESS_WR	                  0x3A   // 0x3C if SA0 is low
#define LSM9_XM_ADDRESS_RD  	                0x3B   // 0x3D if SA0 is low
#define LSM9_GYRO_ADDRESS_WR  	              0xD6   // 0xD4 if SA0 is low
#define LSM9_GYRO_ADDRESS_RD                 	0xD7   // 0xD5 if SA0 is low
#define LSM9_WHO_AM_I_G		 									 	0x0F
#define LSM9_WHO_AM_I_XM		 									0x0F

// Gyro Control Registers
#define LSM9_CTRL_REG1_G										0x20
#define LSM9_CTRL_REG1_G_DEFAULT						0x07
#define LSM9_CTRL_REG2_G										0x21
#define LSM9_CTRL_REG2_G_DEFAULT						0x00
#define LSM9_CTRL_REG3_G										0x22
#define LSM9_CTRL_REG3_G_DEFAULT						0x00
#define LSM9_CTRL_REG4_G										0x23
#define LSM9_CTRL_REG4_G_DEFAULT						0x00
#define LSM9_CTRL_REG5_G										0x24
#define LSM9_CTRL_REG5_G_DEFAULT						0x00
#define LSM9_REFERENCE_G										0x25
#define LSM9_REFERENCE_G_DEFAULT						0x00

#define LSM9_OUT_X_L_G												0x28
#define LSM9_OUT_X_H_G												0x29
#define LSM9_OUT_Y_L_G												0x2A
#define LSM9_OUT_Y_H_G												0x2B
#define LSM9_OUT_Z_L_G												0x2C
#define LSM9_OUT_Z_H_G												0x2D


#define LSM9_FIFO_CTRL_REG_G									0x2E
#define LSM9_FIFO_CTRL_REG_G_DEFAULT					0x00
#define LSM9_FIFO_SRC_REG_G										0x2F

#define LSM9_INT1_CFG_G												0x30
#define LSM9_INT1_CFG_G_DEFAULT								0x00
#define LSM9_INT1_SRC_G												0x31

#define LSM9_INT1_TSH_XH_G									0x32
#define LSM9_INT1_TSH_XH_G_DEFAULT					0x00
#define LSM9_INT1_TSH_XL_G									0x33
#define LSM9_INT1_TSH_XL_G_DEFAULT					0x00
#define LSM9_INT1_TSH_YH_G									0x34
#define LSM9_INT1_TSH_YH_G_DEFAULT					0x00
#define LSM9_INT1_TSH_YL_G									0x35
#define LSM9_INT1_TSH_YL_G_DEFAULT					0x00
#define LSM9_INT1_TSH_ZH_G									0x36
#define LSM9_INT1_TSH_ZH_G_DEFAULT					0x00
#define LSM9_INT1_TSH_ZL_G									0x37
#define LSM9_INT1_TSH_ZL_G_DEFAULT					0x00
#define LSM9_INT1_DURATION_G								0x38
#define LSM9_INT1_DURATION_G_DEFAULT				0x00

// Mag Control Registers

#define LSM9_INT_CTRL_REG_M									0x12
#define LSM9_INT_CTRL_REG_M_DEFAULT					0xE8
#define LSM9_INT_SRC_REG_M									0x13
#define LSM9_INT_THS_L_M										0x14
#define LSM9_INT_THS_L_M_DEFAULT						0x00
#define LSM9_INT_THS_H_M										0x15
#define LSM9_INT_THS_H_M_DEFAULT						0x00

#define LSM9_OFFSET_X_L_M										0x16
#define LSM9_OFFSET_X_L_M_DEFAULT						0x00
#define LSM9_OFFSET_X_H_M										0x17
#define LSM9_OFFSET_X_H_M_DEFAULT						0x00
#define LSM9_OFFSET_Y_L_M										0x18
#define LSM9_OFFSET_Y_L_M_DEFAULT						0x00
#define LSM9_OFFSET_Y_H_M										0x19
#define LSM9_OFFSET_Y_H_M_DEFAULT						0x00
#define LSM9_OFFSET_Z_L_M										0x1A
#define LSM9_OFFSET_Z_L_M_DEFAULT						0x00
#define LSM9_OFFSET_Z_H_M										0x1B
#define LSM9_OFFSET_Z_H_M_DEFAULT						0x00

#define LSM9_REFERENCE_X										0x1C
#define LSM9_REFERENCE_X_DEFAULT						0x00
#define LSM9_REFERENCE_Y										0x1D
#define LSM9_REFERENCE_Y_DEFAULT						0x00
#define LSM9_REFERENCE_Z										0x1E
#define LSM9_REFERENCE_Z_DEFAULT						0x00

// XM Control Registers

#define LSM9_CTRL_REG0_XM  									0x1F
#define LSM9_CTRL_REG0_XM_DEFAULT						0x00

#define LSM9_CTRL_REG1_XM  									0x20
#define LSM9_CTRL_REG1_XM_DEFAULT						0x07

#define LSM9_CTRL_REG2_XM  									0x21
#define LSM9_CTRL_REG2_XM_DEFAULT						0x00

#define LSM9_CTRL_REG3_XM  									0x22
#define LSM9_CTRL_REG3_XM_DEFAULT						0x00

#define LSM9_CTRL_REG4_XM  									0x23
#define LSM9_CTRL_REG4_XM_DEFAULT						0x00

#define LSM9_CTRL_REG5_XM  									0x24
#define LSM9_CTRL_REG5_XM_DEFAULT						0x18

#define LSM9_CTRL_REG6_XM  									0x25
#define LSM9_CTRL_REG6_XM_DEFAULT						0x20

#define LSM9_CTRL_REG7_XM  									0x26
#define LSM9_CTRL_REG7_XM_DEFAULT						0x02

#define LSM9_FIFO_CTRL_REG									0x2E
#define LSM9_FIFO_CTRL_REG_DEFAULT					0x00
#define LSM9_FIFO_SRC_REG										0x2F

#define LSM9_INT_GEN_1_REG									0x30
#define LSM9_INT_GEN_1_REG_DEFAULT					0x00
#define LSM9_INT_GEN_1_SRC									0x31
#define LSM9_INT_GEN_1_THS									0x32
#define LSM9_INT_GEN_1_THS_DEFAULT					0x00
#define LSM9_INT_GEN_1_DURATION							0x33
#define LSM9_INT_GEN_1_DURATION_DEFAULT			0x00

#define LSM9_INT_GEN_2_REG									0x34
#define LSM9_INT_GEN_2_REG_DEFAULT					0x00
#define LSM9_INT_GEN_2_SRC									0x35
#define LSM9_INT_GEN_2_THS									0x36
#define LSM9_INT_GEN_2_THS_DEFAULT					0x00
#define LSM9_INT_GEN_2_DURATION							0x37
#define LSM9_INT_GEN_2_DURATION_DEFAULT			0x00

#define LSM9_CLICK_CFG									0x38
#define LSM9_CLICK_CFG_DEFAULT					0x00
#define LSM9_CLICK_SRC									0x39
#define LSM9_CLICK_THS									0x3A
#define LSM9_CLICK_THS_DEFAULT					0x00

#define LSM9_TIME_LIMIT									0x3B
#define LSM9_TIME_LIMIT_DEFAULT					0x00

#define LSM9_TIME_LATENCY									0x3C
#define LSM9_TIME_LATENCY_DEFAULT					0x00

#define LSM9_TIME_WINDOW									0x3D
#define LSM9_TIME_WINDOW_DEFAULT					0x00


#define LSM9_ACT_THS									0x3E
#define LSM9_ACT_THS_DEFAULT					0x00

#define LSM9_ACT_DUR									0x3F
#define LSM9_ACT_DUR_DEFAULT					0x00


//		STATUS REGISTER

#define LSM9_STATUS_REG_M										0x07
#define LSM9_STATUS_REG_A										0x27
#define LSM9_STATUS_REG_G										0x27


typedef enum {
  LSM9_A_ODR_POWER_DOWN = 0x00,
  LSM9_A_ODR_3_125Hz	 = 0x10,
  LSM9_A_ODR_6_25Hz = 0x20,
  LSM9_A_ODR_12_5Hz = 0x30,
  LSM9_A_ODR_25Hz = 0x40,
  LSM9_A_ODR_50Hz = 0x50,
	LSM9_A_ODR_100Hz = 0x60,
	LSM9_A_ODR_200Hz = 0x70,
	LSM9_A_ODR_400Hz = 0x80,
	LSM9_A_ODR_800Hz = 0x90,
	LSM9_A_ODR_1600Hz = 0xA0
} LSM9_A_ODRTypeDef;

typedef enum {
  LSM9_A_AXIS_DISABLE_ALL = 0x00,
  LSM9_A_AXIS_X_ENABLE = 0x01,
  LSM9_A_AXIS_Y_ENABLE = 0x02,
  LSM9_A_AXIS_Z_ENABLE = 0x04,
  LSM9_A_AXIS_ENABLE_ALL = 0x07
} LSM9_A_AXIS_ENABLETypeDef;

typedef enum {
	LSM9_A_FS_2 = 0x00, /*!< Full scale: +-2 guass */
  LSM9_A_FS_4 = 0x08, /*!< Full scale: +-4 gauss */
  LSM9_A_FS_6 = 0x10, /*!< Full scale: +-6 gauss */
  LSM9_A_FS_8 = 0x18, /*!< Full scale: +-8 gauss */
  LSM9_A_FS_16 = 0x20 /*!< Full scale: +-16 gauss */
} LSM9_A_FULL_SCALETypeDef;

typedef enum {
	LSM9_A_ST_NORMAL = 0x00, /*!< Full scale: +-2 guass */
  LSM9_A_ST_POSITIVE = 0x02, /*!< Full scale: +-4 gauss */
  LSM9_A_ST_NEGATIVE = 0x04, /*!< Full scale: +-6 gauss */
} LSM9_A_SELF_TESTTypeDef;

typedef enum {
	LSM9_M_FS_2 = 0x00, /*!< Full scale: +-2 guass */
	LSM9_M_FS_4 = 0x20, /*!< Full scale: +-4 gauss */
	LSM9_M_FS_8 = 0x40, /*!< Full scale: +-8 gauss */
	LSM9_M_FS_12 = 0x60, /*!< Full scale: +-12 gauss */
} LSM9_M_FULL_SCALETypeDef;

typedef enum {  
  LSM9_M_ODR_3_125Hz	 = 0x00,
  LSM9_M_ODR_6_25Hz = 0x04,
  LSM9_M_ODR_12_5Hz = 0x08,
  LSM9_M_ODR_25Hz = 0x0C,
  LSM9_M_ODR_50Hz = 0x10,
	LSM9_M_ODR_100Hz = 0x14,
	LSM9_M_ODR_RES1 = 0x18,
	LSM9_M_ODR_RES2 = 0x1C
} LSM9_M_ODRTypeDef;

#define LSM9_M_ODR_MASK												0x1C	
#define LSM9_M_FS_MASK                 				0x60 
#define LSM9_XM_SIM_MASK											0x01
#define LSM9_A_BANDWIDTH_MASK									0xC0
#define LSM9_A_SELF_TEST_MASK									0x06
#define LSM9_A_FULL_SCALE_MASK								0x38
#define LSM9_XM_BDU_ENABLE										0x08
#define LSM9_A_ODR_MASK												0xF0	
#define LSM9_XM_BDU_BIT_MASK									0x08
#define LSM9_A_XYZ_ENABLE_MASK								0x07



#define LSM9_XM_TEMP_ENABLE									0x80

#define LSM9_OUT_TEMP_L_XM                  	0x05
#define LSM9_OUT_TEMP_H_XM                  	0x06

typedef enum { 
	LSM9_M_RES_LOW = 0x00, /*!< Low Resolution  */
	LSM9_M_RES_HI = 0x60 /*!< High Resolution */
} LSM9_M_RESOLUTIONTypeDef;

#define LSM9_M_RES_MASK                 		0x60 

typedef enum { 
	LSM9_M_MODE_CONTINUOUS = 0x00,  /*!< Operating mode: Continuous-conversion mode */
	LSM9_M_MODE_SINGLE	= 0x01,  /*!< Operating mode: Continuous-conversion mode */
	LSM9_M_MODE_POWER_DOWN	= 0x02  /*!< Operating mode: Continuous-conversion mode */
}LSM9_M_MODETypeDef;

#define LSM9_M_MODE_MASK                     0x03

typedef enum {  
  LSM9_G_ODR_95Hz	 = 0x00,
  LSM9_G_ODR_190Hz = 0x40,
  LSM9_G_ODR_380Hz = 0x80,
  LSM9_G_ODR_760Hz = 0x0C	
} LSM9_G_ODRTypeDef;


typedef enum {
  LSM9_G_AXIS_DISABLE_ALL = 0x00,
  LSM9_G_AXIS_X_ENABLE = 0x02,
  LSM9_G_AXIS_Y_ENABLE = 0x01,
  LSM9_G_AXIS_Z_ENABLE = 0x04,
  LSM9_G_AXIS_ENABLE_ALL = 0x07
} LSM9_G_AXIS_ENABLETypeDef;

typedef enum {
	LSM9_G_FS_245DPS = 0x00, /*!< Full scale: +- 245DPS */
	LSM9_G_FS_500DPS = 0x10, /*!< Full scale: +- 500DPS */
	LSM9_G_FS_2000DPS = 0x30  /*!< Full scale: +- 2000DPS */
} LSM9_G_FULL_SCALETypeDef;

typedef enum {
	LSM9_G_ST_NORMAL = 0x00, 
  LSM9_G_ST_POSITIVE = 0x02, // X positve Y & Z negative
  LSM9_G_ST_NEGATIVE = 0x06, // X negative Y & Z positive
} LSM9_G_SELF_TESTTypeDef;

#define LSM9_G_FULL_SCALE_MASK						0x30
#define LSM9_G_ODR_MASK										0xC0
#define LSM9_G_BANDWIDTH_MASK							0xC0
#define LSM9_G_POWERDOWN_ENABLE_MASK			0x08
#define LSM9_G_XYZ_ENABLE_MASK						0x07
#define LSM9_G_BDU_ENABLE									0x80
#define LSM9_G_SELF_TEST_MASK							0x06

#define LSM9_MAG_LP_ENABLE                   0x00 /*!< Low-power mode: magnetic data rate is configured by
                                                                                            the DO bits in the CTRL_REG */
#define LSM9_MAG_LP_DISABLE                  0x40  /*!< Low-power mode: the DO bits is set to 3.25 Hz */
#define LSM9_MAG_LP_MASK                     0x40


						

#define LSM9_MAG_STATUS_REG                  0x07
#define LSM9_MAG_DATAREADY_BIT               0

#define LSM9_MAG_MR_REG_M                    0x02
#define LSM9_MAG_CFG_REGA 		                0x24

#define LSM9_OUT_X_L_M												0x08
#define LSM9_OUT_X_H_M												0x09
#define LSM9_OUT_Y_L_M												0x0A
#define LSM9_OUT_Y_H_M												0x0B
#define LSM9_OUT_Z_L_M												0x0C
#define LSM9_OUT_Z_H_M												0x0D

#define LSM9_OUT_X_L_A												0x28
#define LSM9_OUT_X_H_A												0x29
#define LSM9_OUT_Y_L_A												0x2A
#define LSM9_OUT_Y_H_A												0x2B
#define LSM9_OUT_Z_L_A												0x2C
#define LSM9_OUT_Z_H_A												0x2D

/* Exported types ------------------------------------------------------------*/
/**
* @brief  LSM status enumerator definition
*/
typedef enum {
    LSM_OK = 0,
    LSM_ERROR = 1,
    LSM_TIMEOUT = 2,
    LSM_INVALID_PARAMS = 3,
    LSM_NOT_IMPLEMENTED = 4
} LSM_BSP_StatusTypeDef;

typedef float iNEMO_QUAT[4];

typedef struct
{
  unsigned int m_nCount;  /*!< It is used to perform correction at different frequencies */
  float   m_fDeltaTime;   /*!< It should trace the time difference */

  float m_fAcc[3];		/*!< The Acceleration Measurement */
  float m_fGyro[3];		/*!< The Gyroscope Measurement */
  float m_fMag[3];		/*!< The Magnitude Measurement */

  float m_fScaleFactor[9];      /*!< The Scale Factor Vector for each Measurement */
  float m_fOffset[9];		/*!< The Offset Vector for each Measurement */
  
  float m_fAccRef[3];           /*!< The gravitational vector refer */
  float m_fMagRef[3];           /*!< The magnetic vector refer */

  float m_fVarAcc;              /*!< The accelerometer variance */
  float m_fVarMag;              /*!< The magnetometer variance */
  float m_fVarGyro;             /*!< The gyroscope variance */
  
} iNEMO_SENSORDATA;

/**
 * @struct iNEMO_EULER_ANGLES
 * @brief iNEMO Euler Angle Struct
 */

typedef struct
{
  float m_fRoll;          /*!< Roll Angle */ 
  float m_fPitch;         /*!< Pitch Angle */  
  float m_fYaw;           /*!< Yaw Angle */ 
} iNEMO_EULER_ANGLES;

/**
 * @struct iNemoData
 * @brief iNemo data structure.
 */
typedef struct
{

  int16_t pnAcc[3];             /*!< Accelerometer XYZ data (raw) */
  int16_t pnGyro[3];            /*!< Gyroscope XYZ data (raw) */
  int16_t pnMag[3];             /*!< Magnetometer XYZ data (raw) */

  int32_t lPress;               /*!< Pressure sensor data (raw) */
  int16_t nTemp;                /*!< Temperature sensor data (raw) */
  
  float fSensitivity[11];       /*!< Sensitivities. Divide the raw in order to have a non-rated measurement.
                                        <br> The order of sensitivities in the array is {Acc[XYZ] - Gyro[XYZ] - Magn[XYZ] - Press - Temp} */
  
  float fScaleFactor[11];       /*!< Scale Factor. The calibrated data is obtained as ((raw_data/sensitivity[i])-nOffset[i])/fScaleFactor[i].
                                        <br> The order of scale factor in the array is {Acc[XYZ] - Gyro[XYZ] - Magn[XYZ] - Press - Temp} */
  int16_t nOffset[11];          /*!< Offset. The calibrated data is obtained as ((raw_data/sensitivity[i])-nOffset[i])/fScaleFactor[i].
                                        <br> The order of offset in the array is {Acc[XYZ] - Gyro[XYZ] - Magn[XYZ] - Press - Temp} */


	iNEMO_QUAT          xQuat;         /*!< Quaternions data */
  float               fHeading;      /*!< Heading angle data (rad) */

  iNEMO_SENSORDATA    xSensorData;   /*!< Sensor data to be elaborated by an algorithm. In this structure the user can decide to store preprocessed data (example: exchange axis or multiply by a constant) before an algorithm will process it (i.e. AHRS, COMPASS). */
  iNEMO_EULER_ANGLES  xEulerAngles;  /*!< Euler angles data (rad) */
  
} iNemoData;


/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/	 
/* Exported functions --------------------------------------------------------*/


/******************************************************************************
*	LSM9DS0 GYRO Low Level Read/Write function declarations
* 
*******************************************************************************/

uint8_t LSM_GYRO_ReadRegister(uint8_t reg);
void LSM_GYRO_WriteRegister(uint8_t reg, uint8_t val);

/******************************************************************************
*	LSM9DS0 XM Low Level Read/Write function declarations
* 
*******************************************************************************/

uint8_t LSM_XM_ReadRegister(uint8_t reg);
void LSM_XM_MultiByte_ReadData(uint8_t reg, uint8_t* pBuffer, uint8_t bufferSize);
void LSM_XM_WriteRegister(uint8_t reg, uint8_t val);

/******************************************************************************
*	LSM9DS0 Who Am I function declarations
* 
*******************************************************************************/

uint8_t LSM_GYRO_Whoami(void);
uint8_t LSM_XM_Whoami(void);

void LSM_InitChip(void);

/******************************************************************************
*	LSM9DS0 Temp function declarations
* 
*******************************************************************************/

uint8_t LSM_TEMP_isInitialized(void);
LSM_BSP_StatusTypeDef LSM_TEMP_Init(void);
void LSM_TEMP_Enable(void);
void LSM_TEMP_Disable(void);
int16_t LSM_TEMP_Read(void);
	
/******************************************************************************
*	LSM9DS0 MAG function declarations
* 
*******************************************************************************/

uint8_t LSM_Mag_isInitialized(void);
LSM_BSP_StatusTypeDef LSM_Mag_Init(void);
LSM_BSP_StatusTypeDef LSM_Mag_Set_Hires(void);
LSM_BSP_StatusTypeDef LSM_Mag_Set_Lores(void);
LSM_BSP_StatusTypeDef LSM_Mag_Set_Scale(uint8_t scale);
LSM_BSP_StatusTypeDef LSM_Mag_Set_Mode(uint8_t mode);
float LSM_Mag_Get_Measurement(int16_t raw_data);
float LSM_Mag_Get_Scaled(int16_t raw_data);
float LSM_Mag_Get_Sensitivity(void);

uint8_t LSM_Mag_Get_Status(void);
void LSM_Mag_ReadRaw(int16_t* pData);

/******************************************************************************
*	LSM9DS0 Accel function declarations
* 
*******************************************************************************/

uint8_t LSM_Acc_isInitialized(void);
LSM_BSP_StatusTypeDef LSM_Acc_Init(void);
uint8_t LSM_Acc_Get_Status(void);
void LSM_Acc_ReadRaw(int16_t* pData);
float LSM_Acc_Get_Scaled(int16_t raw_data);
float LSM_Acc_Get_Measurement(int16_t raw_data);
float LSM_Acc_Get_Sensitivity(void);

/******************************************************************************
*	LSM9DS0 Gyro function declarations
* 
*******************************************************************************/

uint8_t LSM_Gyro_isInitialized(void);
LSM_BSP_StatusTypeDef LSM_Gyro_Init(void);
uint8_t LSM_Gyro_Get_Status(void);
void LSM_Gyro_ReadRaw(int16_t* pData);
float LSM_Gyro_Get_Scaled(int16_t raw_data);
float LSM_Gyro_Get_Sensitivity(void);

#ifdef __cplusplus
}
#endif

#endif /* __LSM9DS0_H */
