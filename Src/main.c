#include "main.h"
#include "usb_device.h"
#include "UsbSerial.h"
#include "DataPacker2.h"
#include "UpbitCounter.h"
#include "Schedule.h"
#include <stdlib.h>
#include <ctype.h>
#include "Trackbar.h"
#include "PID.h"
#include "Accel.h"
#include <math.h>

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void setMotorPwm(double pwm);

TIM_HandleTypeDef htim4;

int32_t encoder16Count = 0;
int32_t encoder32Count = 0;
bool plotEnable = 0;
uint32_t lastPlot_ms = 0;

double currentPosition, outPosition, setPosition;
double currentAngle, outAngle, setAngle, dAngle;
Tunings positionTunings = {70, 5, 1};
Tunings homeTunings = {-0.6, -0.04, -10};
Tunings angleTunings = {4000, 0, -200000};

double setAccel;
//bo dieu khien van toc (dau ra la vi tri)
VelocityStepper velocityHandler(setPosition);
//bo dieu khien gia toc
Accel accelHandler(velocityHandler, setAccel);

// toa do can bang
double zero = 0;

PID pidMotor(currentPosition, outPosition, setPosition, positionTunings);
PID pidHome(currentPosition, outAngle, zero, homeTunings);

uint32_t getUs() {
  uint32_t usTicks = HAL_RCC_GetSysClockFreq() / 1000000;
  register uint32_t ms, cycle_cnt;
  do {
    ms = HAL_GetTick();
    cycle_cnt = SysTick->VAL;
  } while (ms != HAL_GetTick());
  return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

void delayUs(uint16_t micros) {
  uint32_t start = getUs();
  while (getUs()-start < (uint32_t) micros) {
    asm("nop");
  }
}

Trackbar tbMotorKp("MP", positionTunings.kp), tbMotorKi("MI", positionTunings.ki), tbMotorKd("MD", positionTunings.kd);
Trackbar tbTargetAngleKp("TP", homeTunings.kp), tbTargetAngleKi("TI", homeTunings.ki), tbTargetAngleKd("TD", homeTunings.kd);
Trackbar tbLargeAngleKp("LAP", angleTunings.kp), tbLargeAngleKi("LAI", angleTunings.ki), tbLargeAngleKd("LAD", angleTunings.kd);
Trackbar tbAngleMargin("AM", zero);

void hwEncoderInit(){
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();

  // TIMER 2: 32bit, PA0+PA1
  GPIO_InitTypeDef  GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // TIMER 3: 16bit, PB5+PC6
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  TIM2->CR1   = 0x0001;     // CEN(Counter ENable)='1'     < TIM control register 1
  TIM2->SMCR  = 0x0003;     // SMS='011' (Encoder mode 3)  < TIM slave mode control register
  TIM2->CCMR1 = 0x0101;     // CC1S='01' CC2S='01'         < TIM capture/compare mode register 1
  TIM2->CCMR2 = 0x0000;     //                             < TIM capture/compare mode register 2
  TIM2->CCER  = 0x0011;     // CC1P CC2P                   < TIM capture/compare enable register
  TIM2->PSC   = 0x0000;     // Prescaler = (0+1)           < TIM prescaler
  TIM2->ARR   = 0xffffffff; // reload at 0xfffffff         < TIM auto-reload register
  TIM2->CNT = 0x0000;       // reset the counter

  TIM3->CR1   = 0x0001;     // CEN(Counter ENable)='1'     < TIM control register 1
  TIM3->SMCR  = 0x0003;     // SMS='011' (Encoder mode 3)  < TIM slave mode control register
  TIM3->CCMR1 = 0x0101;     // CC1S='01' CC2S='01'         < TIM capture/compare mode register 1
  TIM3->CCMR2 = 0x0000;     //                             < TIM capture/compare mode register 2
  TIM3->CCER  = 0x0011;     // CC1P CC2P                   < TIM capture/compare enable register
  TIM3->PSC   = 0x0000;     // Prescaler = (0+1)           < TIM prescaler
  TIM3->ARR   = 0xffff;     // reload at 0xffff         < TIM auto-reload register
  TIM3->CNT = 0x0000;       // reset the counter
}

UpbitCounter<uint16_t, int32_t> angleEncoderCounter;
char message[32];
bool positionOutOfRange = false;
bool motorEnable = true;

// string to upper case
void strupr(char * str){
  char *s = str;
  while (*s) {
    *s = toupper((unsigned char) *s);
    s++;
  }
}

void handleGUI(){
  Serial.readBytes((uint8_t*)message, sizeof(message));

  strupr(message);

  char rw = message[0];
  char cmdName[16];
  float value;

  if(rw != 'R' && rw != 'W') { Serial.println("ERROR"); return; }

  sscanf(message, "%c %s %f", &rw, cmdName, &value);
  bool foundTrackbar = false;

  // bat tat plotter
  if(strcmp(cmdName, "PLOT") == 0){
    plotEnable = ((int)value == 1) ? true : false;
    return;
  }

  // REL angle
  if(strcmp(cmdName, "RELA") == 0){
    TIM3->CR1   = 0x0000;
    TIM3->CNT = 0;
    angleEncoderCounter.reset();
    TIM3->CNT = 0;
    TIM3->CR1   = 0x0001;
    return;
  }

  // REL position
  if(strcmp(cmdName, "RELP") == 0){
    currentPosition = 0;
    TIM2->CR1   = 0x0000;
    TIM2->CNT = 0;
    TIM2->CR1   = 0x0001;
    return;
  }
  // bat tat dong co
  if(strcmp(cmdName, "MOTOREN") == 0){
    motorEnable = ((int)value == 1) ? true : false;
    return;
  }

  // thay doi trackbar
  for(uint8_t i = 0; i < Trackbar::instancesCount; ++i) {
    if(strcmp(cmdName, Trackbar::instances[i]->key) == 0) {
      if(rw == 'R')
        value = Trackbar::instances[i]->value;
      if(rw == 'W')
        Trackbar::instances[i]->value = value;

      foundTrackbar = true;
      break;
    }
  }

  if(!foundTrackbar){
    Serial.println("NOT FOUND");
    return;
  }

  snprintf(message, sizeof(message), "%s %9.4f", cmdName, value);
  Serial.println(message);
}

void setup(){
  Serial.begin();
  hwEncoderInit();

	//dat thoi gian lay mau PID motor, 100us
	pidMotor.SetSampleTime(100);
	//gioi han dau ra PWM cua PID motor
	pidMotor.SetOutputLimit(-1000, 1000);

  pidHome.SetSampleTime(1000);
  pidHome.SetOutputLimit(-5000, 5000); //+- 5degree
}

void loop() {
  const uint32_t current_ms = HAL_GetTick();
  if(Serial.available() > 0) handleGUI();
  static double lastAngle = 0;
  static uint32_t lastMeasureAngle_ms = 0;
  static double angleFiltered = 0;

  angleEncoderCounter.update(TIM3->CNT);
  currentAngle = (double)angleEncoderCounter.getCount() * 360.0/ 2400.0 + setAngle;
  currentPosition = (int32_t)TIM2->CNT;

  if(current_ms > lastMeasureAngle_ms + 5){
    lastMeasureAngle_ms = current_ms;
    angleFiltered = 0.5 * angleFiltered + 0.5 * currentAngle;
    double newError = angleFiltered - lastAngle;

    lastAngle = angleFiltered;
    setAccel = angleTunings.kd * newError + angleTunings.kp * (outAngle / 1000.0 - currentAngle);
  }

  if(setPosition > -9000 && setPosition < 9000)
    pidHome.Update();
  accelHandler.Update();

  setPosition = constrain(setPosition, MIN_POSITION, MAX_POSITION);

  pidMotor.Update();

  setMotorPwm(motorEnable ? outPosition : 0);

  static uint32_t lastInc = 0;
  if(current_ms > lastInc + 200){
    lastInc = current_ms;
  }

  if(plotEnable && current_ms > lastPlot_ms + 10){
    lastPlot_ms = current_ms;
    char buffer[50];
    snprintf(buffer, sizeof(buffer), "PLOT %10.2f %10.2f",
             currentAngle, outAngle/1000);
    Serial.println(buffer);
  }
}

int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_TIM4_Init();

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

  setup();
  while (1)
    loop();

  //return 0;
}

// PWM range: -1000 -> +1000
void setMotorPwm(double pwm){
  if(pwm > 1000) pwm = 1000;
  if(pwm < -1000) pwm = -1000;

  if(pwm >= 0){
    HAL_GPIO_WritePin(Motor_INB_GPIO_Port, Motor_INB_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Motor_INA_GPIO_Port, Motor_INA_Pin, GPIO_PIN_SET);
  }else{
    HAL_GPIO_WritePin(Motor_INB_GPIO_Port, Motor_INB_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Motor_INA_GPIO_Port, Motor_INA_Pin, GPIO_PIN_RESET);
  }

  uint32_t duty = (uint32_t)(pwm >= 0 ? pwm : -pwm);
  if(pwm < 20) pwm = 0;
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, duty);
}
/**
* @brief System Clock Configuration
* @retval None
*/
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
    |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
* @brief GPIO Initialization Function
* @param None
* @retval None
*/
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD3_Pin|LD5_Pin|LD6_Pin
                    |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DATA_Ready_Pin */
  GPIO_InitStruct.Pin = DATA_Ready_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DATA_Ready_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INT1_Pin INT2_Pin MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = INT1_Pin|INT2_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CLK_IN_Pin PB12 */
  GPIO_InitStruct.Pin = CLK_IN_Pin|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Motor_INB_Pin Motor_INA_Pin LD3_Pin LD5_Pin
  LD6_Pin Audio_RST_Pin */
  GPIO_InitStruct.Pin = Motor_INB_Pin|Motor_INA_Pin|LD3_Pin|LD5_Pin
    |LD6_Pin|Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}
/**
* @brief TIM4 Initialization Function
* @param None
* @retval None
*/
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 4;    // 21kHz
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);
}

void Error_Handler(void)
{
  //  Serial.println("Error occured");
  HAL_Delay(5);
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  char buffer[256];
  snprintf(buffer, sizeof(buffer), "Wrong parameters value: file %s on line %d\r\n", file, line);
  //  Serial.println(buffer);
  HAL_Delay(5);
}
#endif