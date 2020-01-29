/****************************************************************************
* Title                    :   Battery Pack MCU
* File name                :   bat_soc.c
* Author name[ID]          :   Abhinav Choudhary
* Origin Date(DD/MM/YYYY)  :   14/11/2019 
* Version                  :   2.0.0
* Compiler                 :   Keil IDE
* Hardware                 :   Refer to documentation
* Target                   :   STM32F446 Nucleo
* Notes                    :   
* Copy-Rights
*/

/**
 *  @file  bat_soc.c
 *  @brief I2C Communication with Bq34z100 SoC Meter
 */

/*******************************************************************************
*Includes
*******************************************************************************/
#include "bat_soc.h"
#include "bat_can.h"

/*******************************************************************************
*Module Preprocessor Constants
*******************************************************************************/


/*******************************************************************************
*Module Preprocessor Macros
*******************************************************************************/
#define I2C1_SCL_Pin GPIO_PIN_6
#define I2C1_SCL_GPIO_Port GPIOB
#define I2C1_SDA_Pin GPIO_PIN_7
#define I2C1_SDA_GPIO_Port GPIOB

/*******************************************************************************
*Module Typedefs
*******************************************************************************/


/*******************************************************************************
*Module Variable Definitions
*******************************************************************************/
I2C_HandleTypeDef hi2c1;
Gauge_Param_t Gauge_Param;
DataBlock_config_t Block;                     // Data Block config structure to read SoC-meter data in blocks format
GaugeData1_t can_msg_1;
GaugeData2_t can_msg_2;
GaugeData3_t can_msg_3;

uint8_t *pGaugeData = NULL;
uint32_t CAN_BaseID;

/*******************************************************************************
*Function Definitions
*******************************************************************************/
extern void Error_Handler(void);
/* Private function declerations*/
static uint8_t  Config_DataBlock (uint8_t block_num);
static void     I2C_Read_Data_Block (uint8_t block_num);
static uint16_t Byte_Swap (uint16_t msb, uint16_t lsb);
static int32_t  map_gauge_data (uint8_t block_num);

/**
  * @brief  Configure Data Block parameters 
  * @param  block_num Block Number
  * @retval arr_len Length of the array to be defined to read data
  */
static uint8_t Config_DataBlock(uint8_t block_num)
{

  #ifdef BLOCK0      /* Block: 0 */
    if (block_num == 0)
    {
      Block.Addr  = 0x02;
      Block.Len   = 18;
    }
  #endif
  #ifdef BLOCK1      /* Block: 1 */
    if (block_num == 1)
    {
      Block.Addr  = 0x18;
      Block.Len   = 8;
    }
  #endif
  #ifdef BLOCK2      /* Block: 2 */
    if  (block_num == 2)
    {
      Block.Addr  = 0x24;
      Block.Len   = 16;
    }
  #endif
  #ifdef BLOCK3      /* Block: 3 */
    if (block_num == 3)
    {
      Block.Addr  = 0x3A;
      Block.Len   = 60;
    }
  #endif
    return Block.Len;
}

/**
  * @brief  Read Gauge data blocks into array
  * @param  block_num : Block Number
  * @retval None  
  */
static void I2C_Read_Data_Block (uint8_t block_num)
{
  
  Config_DataBlock(block_num);    /* Configure blocks first */
  
  pGaugeData = malloc(Block.Len);    /* Allocate the memory bytes to array : pGaugeData */
  /* Master write address to Slave */
  if (HAL_I2C_Master_Transmit (&hi2c1, BQ_WR_ADDR, &Block.Addr, sizeof(Block.Addr), 1000)!=HAL_OK)
  {
    Error_Handler();
  }
  /* Master request Slave for data */
  if (HAL_I2C_Master_Receive (&hi2c1, BQ_RD_ADDR, &pGaugeData[0], Block.Len, 1000)!=HAL_OK)
  {
    Error_Handler();
  }
 /* Map array data to structure */
  map_gauge_data(block_num);
  /* Release memory allocated to PGaugeData */
  free(pGaugeData); 

}

/**
  * @brief  Swap MSB and LSB 
  * @param  msb: Most Significant Byte
  * @param  lsb: Least Significant Byte
  * @retval swap_byte: Bytes after swapping  
  */
static uint16_t Byte_Swap (uint16_t msb, uint16_t lsb)
{
  uint16_t swap_data;
  swap_data = ((lsb << 8) & (0xFF00))| ((msb) & (0x00FF)) ; 
  return swap_data;
}

/**
  * @brief  Map array data into Gauge_Param_t 
  * @param  block_num Block Number
  * @retval None  
  */
static int32_t map_gauge_data(uint8_t block_num)
{
  #ifdef BLOCK0
    if (block_num == 0)
    {
//    pGauge_Param.Control              = Byte_Swap(pGaugeData[0], pGaugeData[1]);
      Gauge_Param.StateOfCharge        = pGaugeData[0];
      Gauge_Param.MaxError             = pGaugeData[1];
      Gauge_Param.RemainingCapacity    = Byte_Swap(pGaugeData[2], pGaugeData[3]);
      Gauge_Param.FullChargeCapacity   = Byte_Swap(pGaugeData[4], pGaugeData[5]);
      Gauge_Param.Voltage              = Byte_Swap(pGaugeData[6], pGaugeData[7]);
      Gauge_Param.AvgCurrent           = Byte_Swap(pGaugeData[8], pGaugeData[9]);
      Gauge_Param.Temperature          = Byte_Swap(pGaugeData[10], pGaugeData[11]);
      Gauge_Param.Flags                = Byte_Swap(pGaugeData[12], pGaugeData[13]);
      Gauge_Param.Current              = Byte_Swap(pGaugeData[14], pGaugeData[15]);
      Gauge_Param.FlagsB               = Byte_Swap(pGaugeData[16], pGaugeData[17]);
    }
  #endif
  #ifdef BLOCK1
    if (block_num == 1)
    {
      Gauge_Param.AvgTimeToEmpty       = Byte_Swap(pGaugeData[0], pGaugeData[1]);
      Gauge_Param.AvgTimeToFull        = Byte_Swap(pGaugeData[2], pGaugeData[3]);
      Gauge_Param.PassedCharge         = Byte_Swap(pGaugeData[4], pGaugeData[5]);                          
      Gauge_Param.DoD0Time             = Byte_Swap(pGaugeData[6], pGaugeData[7]);
    }
  #endif
  #ifdef BLOCK2
    if (block_num == 2)
    {
      Gauge_Param.AvailableEnergy      = Byte_Swap(pGaugeData[0], pGaugeData[1]);
      Gauge_Param.AvgPower             = Byte_Swap(pGaugeData[2], pGaugeData[3]);
      Gauge_Param.SerialNumber         = Byte_Swap(pGaugeData[4], pGaugeData[5]);
      Gauge_Param.InternalTemperature  = Byte_Swap(pGaugeData[6], pGaugeData[7]);
      Gauge_Param.CycleCount           = Byte_Swap(pGaugeData[8], pGaugeData[9]);
      Gauge_Param.StateOfHealth        = Byte_Swap(pGaugeData[10], pGaugeData[11]);
      Gauge_Param.ChargeVoltage        = Byte_Swap(pGaugeData[12], pGaugeData[13]);
      Gauge_Param.ChargeCurrent        = Byte_Swap(pGaugeData[14], pGaugeData[15]);
    }
  #endif
  #ifdef BLOCK3
    if (block_num == 3)
    {
      Gauge_Param.PackConfiguration    = Byte_Swap(pGaugeData[0], pGaugeData[1]);
      Gauge_Param.DesignCapacity       = Byte_Swap(pGaugeData[2], pGaugeData[3]);
      Gauge_Param.DataFlashClass       = pGaugeData[4];
      Gauge_Param.DataFlashBlock       = pGaugeData[5];
      Gauge_Param.AuthenticateData0    = pGaugeData[6];
      Gauge_Param.AuthenticateData1    = pGaugeData[7];
      Gauge_Param.AuthenticateData2    = pGaugeData[8];
      Gauge_Param.AuthenticateData3    = pGaugeData[9];
      Gauge_Param.AuthenticateData4    = pGaugeData[10];
      Gauge_Param.AuthenticateData5    = pGaugeData[11];
      Gauge_Param.AuthenticateData6    = pGaugeData[12];
      Gauge_Param.AuthenticateData7    = pGaugeData[13];
      Gauge_Param.AuthenticateData8    = pGaugeData[14];
      Gauge_Param.AuthenticateData9    = pGaugeData[15];
      Gauge_Param.AuthenticateData10   = pGaugeData[16];
      Gauge_Param.AuthenticateData11   = pGaugeData[17];
      Gauge_Param.AuthenticateData12   = pGaugeData[18];
      Gauge_Param.AuthenticateData13   = pGaugeData[19];
      Gauge_Param.AuthenticateData14   = pGaugeData[20];
      Gauge_Param.AuthenticateData15   = pGaugeData[21];
      Gauge_Param.AuthenticateData16   = pGaugeData[22];
      Gauge_Param.AuthenticateData17   = pGaugeData[23];
      Gauge_Param.AuthenticateData18   = pGaugeData[24];
      Gauge_Param.AuthenticateData19   = pGaugeData[25];
      Gauge_Param.AuthenticateChecksum = pGaugeData[26];
      Gauge_Param.BlockData0           = pGaugeData[27];
      Gauge_Param.BlockData1           = pGaugeData[28];
      Gauge_Param.BlockData2           = pGaugeData[29];
      Gauge_Param.BlockData3           = pGaugeData[30];
      Gauge_Param.BlockData4           = pGaugeData[31];
      Gauge_Param.BlockData5           = pGaugeData[32];
      Gauge_Param.BlockData6           = pGaugeData[33];
      Gauge_Param.BlockData7           = pGaugeData[34];
      Gauge_Param.BlockData8           = pGaugeData[35];
      Gauge_Param.BlockData9           = pGaugeData[36];
      Gauge_Param.BlockData10          = pGaugeData[37];
      Gauge_Param.BlockDataChecksum    = pGaugeData[38];
      Gauge_Param.BlockDataControl     = pGaugeData[39];
      Gauge_Param.GridNumber           = pGaugeData[40];
      Gauge_Param.LearnedStatus        = pGaugeData[41];
      Gauge_Param.DoDatEoC             = Byte_Swap(pGaugeData[42], pGaugeData[43]);
      Gauge_Param.QStart               = Byte_Swap(pGaugeData[44], pGaugeData[45]);
      Gauge_Param.TrueRC               = Byte_Swap(pGaugeData[46], pGaugeData[47]);
      Gauge_Param.TrueFCC              = Byte_Swap(pGaugeData[48], pGaugeData[49]);
      Gauge_Param.StateTime            = Byte_Swap(pGaugeData[50], pGaugeData[51]);
      Gauge_Param.QMaxPassedQ          = Byte_Swap(pGaugeData[52], pGaugeData[53]);
      Gauge_Param.DOD0                 = Byte_Swap(pGaugeData[54], pGaugeData[55]);
      Gauge_Param.QMaxDOD0             = Byte_Swap(pGaugeData[56], pGaugeData[57]);
      Gauge_Param.QMaxTime             = Byte_Swap(pGaugeData[58], pGaugeData[59]); 
    }
  #endif
  return 0;
}
/**
* @brief I2C MSP Initialization
* This function configures the hardware resources used in this module
* @param hi2c: I2C handle pointer
* @retval None
*/
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hi2c->Instance==I2C1)
  {  
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /**I2C1 GPIO Configuration    
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA 
    */
    GPIO_InitStruct.Pin     = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode    = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed   = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Pull    = GPIO_PULLUP;    
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    /* Peripheral clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
    
  }
}

/**
* @brief I2C MSP De-Initialization
* This function freeze the hardware resources used in this module
* @param hi2c: I2C handle pointer
* @retval None
*/
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{
  if(hi2c->Instance==I2C1)
  {
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();
  
    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);
  }
}

/**
  * @brief  Configure and Initialise I2C1 Peripheral
  * @param  None
  * @retval None  
  */
void Gauge_I2C_Init(void)
{
  hi2c1.Instance             = I2C1;
  hi2c1.Init.ClockSpeed      = 100000;                               /* 100 KHz */
  hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1     = 0;
  hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2     = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  Read Gauge parameters
  * This function reads all the gauge parameter  @Config_DataBlock
  * @param  None
  * @retval 0: if success
           -1: if failed
  */
int32_t Read_Gauge_Param (void)
{
  if ( sizeof(pGaugeData) != NULL)
  {
    free(pGaugeData);
  }

  /* Read data block 0 from bq34z100 */
  I2C_Read_Data_Block(0);
  can_msg_1.SoC = Gauge_Param.StateOfCharge ;
  can_msg_1.Bat_Volt = Gauge_Param.Voltage;
	can_msg_1.Bat_Temp = Gauge_Param.Temperature;
	can_msg_1.Bat_Current = Gauge_Param.Current;
	can_msg_2.Rem_Capacity = Gauge_Param.RemainingCapacity;
	can_msg_3.FullCap = Gauge_Param.FullChargeCapacity;
	
  /* Read data block 2 from bq34z100 */
  I2C_Read_Data_Block(2);
	can_msg_2.Avl_Energy = Gauge_Param.AvailableEnergy;
	can_msg_3.Cycle_Count = Gauge_Param.CycleCount;
	can_msg_3.SoH = Gauge_Param.StateOfHealth;
	
  return 0;
}

/**
  * @brief  Fetch CAN identifier value stored in 
  * This function reads all the gauge parameter  @Config_DataBlock
  * @param  None
  * @retval 0: if success
           -1: if failed
  */
void CAN_GetBaseID(void)
{
  uint32_t  DataFlash = 0x00003A3E;
  uint16_t DataCommand = 0x3A00;
  uint8_t  CAN_Id_Addr = 0x5C;
  if (HAL_I2C_Master_Transmit (&hi2c1, BQ_WR_ADDR, (uint8_t*)&DataFlash , 3*sizeof(uint8_t), 1000)!=HAL_OK)
  {
      Error_Handler();
  }
  HAL_Delay(10);
  if (HAL_I2C_Master_Transmit (&hi2c1, BQ_WR_ADDR, &CAN_Id_Addr, sizeof(CAN_Id_Addr), 1000)!=HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2C_Master_Receive (&hi2c1, BQ_RD_ADDR, (uint8_t*)&CAN_BaseID, 0x04 , 1000)!=HAL_OK)
  {
    Error_Handler();
  }
  CAN_BaseID = ((CAN_BaseID&0x000000FF)<<0x18)|((CAN_BaseID&0x0000FF00)<<0x08)
              |((CAN_BaseID&0x00FF0000)>>0x08)|((CAN_BaseID&0xFF000000)>>0x18);
}




/*****************************END OF FILE**************************************/
