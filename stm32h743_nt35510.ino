// demo stm32h743 with nt35510 3.97" display
// jef collin 2022



#include <SPI.h>
#include <lvgl.h>
#include <Wire.h>
#include <Eeprom24C32_64.h>

#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0

#include "STM32TimerInterrupt.h"
#include "STM32_ISR_Timer.h"

#define TIMER_INTERVAL_MS         5
#define HW_TIMER_INTERVAL_MS      3




// LCD driver start

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef __IO uint32_t  vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;

#define ON  1
#define OFF 0

static u32 fac_us = 0;

typedef struct
{
  u16 width;
  u16 height;
  u16 id;
  u8  dir;
  u16  wramcmd;
  u16  rramcmd;
  u16  setxcmd;
  u16  setycmd;
} _lcd_dev;

//#define USE_HORIZONTAL      1
#define LCD_H 480
#define LCD_W 800

#define LCD_REGION_NUMBER   MPU_REGION_NUMBER0
#define LCD_ADDRESS_START   (0X60000000)
#define LCD_REGION_SIZE     MPU_REGION_SIZE_256MB
#define LCD_RST   8         //         PB8
#define LCD_RST_SET() HAL_GPIO_WritePin(GPIOB,1<<LCD_RST ,GPIO_PIN_SET)//GPIO_TYPE->BSRRL=1<<LCD_LED                      
#define LCD_RST_CLR() HAL_GPIO_WritePin(GPIOB,1<<LCD_RST,GPIO_PIN_RESET)

typedef struct
{
  vu16 LCD_REG;
  vu16 LCD_RAM;
} LCD_TypeDef;

#define LCD_BASE        ((u32)(0x60000000 | 0x0007FFFE))
#define LCD             ((LCD_TypeDef *) LCD_BASE)

#define WHITE       0xFFFF
#define BLACK       0x0000
#define BLUE        0x001F
#define BRED        0XF81F
#define GRED        0XFFE0
#define GBLUE       0X07FF
#define RED         0xF800
#define MAGENTA     0xF81F
#define GREEN       0x07E0
#define CYAN        0x7FFF
#define YELLOW      0xFFE0
#define BROWN       0XBC40
#define BRRED       0XFC07
#define GRAY        0X8430
#define DARKBLUE         0X01CF
#define LIGHTBLUE        0X7D7C
#define GRAYBLUE         0X5458
#define LIGHTGREEN      0X841F
#define LIGHTGRAY     0XEF5B
#define LGRAY           0XC618
#define LGRAYBLUE       0XA651
#define LBBLUE          0X2B12

SRAM_HandleTypeDef SRAM_Handler;
_lcd_dev lcddev;

u16 POINT_COLOR = 0x0000, BACK_COLOR = 0xFFFF;
u16 DeviceCode;

// LCD driver end

// touch driver start
// Initialise with example calibration values so processor does not crash if setTouch() not called in setup()
uint16_t touchCalibration_x0 = 300, touchCalibration_x1 = 3600, touchCalibration_y0 = 300, touchCalibration_y1 = 3600;
uint8_t  touchCalibration_rotate = 1, touchCalibration_invert_x = 2, touchCalibration_invert_y = 0;
uint32_t _pressTime;        // Press and hold time-out
uint16_t _pressX, _pressY;  // For future use (last sampled calibrated coordinates)

// touch driver end


// PA4 CS
#define TOUCH_CS PA4     // Chip select pin (T_CS) of touch screen

#define LVGL_TICK_PERIOD 2


// general

unsigned long loop_timer;
static Eeprom24C32_64 eeprom(0x50);


// touch screen calib
uint16_t calibrationData[5];
uint8_t calDataOK = 0;


static inline lv_color_t lv_color_blue(void) {
  return lv_color_make(0x00, 0x00, 0xff);
}

// timer related
// Init STM32 timer TIM1
STM32Timer ITimer(TIM1);

// Init STM32_ISR_Timer
// Each STM32_ISR_Timer can service 16 different ISR-based timers
STM32_ISR_Timer ISR_Timer;

void TimerHandler()
{
  ISR_Timer.run();
}

void handle_timer_int()
{
  lv_tick_inc(LVGL_TICK_PERIOD);
}


void my_disp_flush(lv_disp_drv_t * disp, const lv_area_t * area, lv_color_t * color_p)
{
  uint32_t w = (area->x2 - area->x1);
  uint32_t h = (area->y2 - area->y1);
   LCD_SetWindows(area->x1, area->y1, area->x2, area->y2);
  int32_t x;
  int32_t y;
  for (y = area->y1; y <= area->y2; y++) {
    for (x = area->x1; x <= area->x2; x++) {
      LCD->LCD_RAM = color_p->full;
      color_p++;
    }
  }
  lv_disp_flush_ready(disp);
}

void my_touchpad_read(lv_indev_drv_t * indev, lv_indev_data_t * data)
{
  uint16_t touchX, touchY;
  bool touched = getTouch(&touchX, &touchY, 400); // was 600
  if (!touched)
  {
    data->state = LV_INDEV_STATE_REL;
  }
  else
  {
    data->state = LV_INDEV_STATE_PR;
  }
  if (touchX > 800 || touchY > 480)
  {
  }
  else
  {
    /*Set the coordinates*/
    data->point.x = touchX;
    data->point.y = touchY;
  }
  
  return;
}


// pointer to update
static lv_obj_t *tabview;

void setup() {
  HAL_Init();
  Stm32_Clock_Init(120, 1, 2, 2);
  delay_init(400);
  LCD_Init();

  SPI.setMOSI(PB5);
  SPI.setMISO(PA6);
  SPI.setSCLK(PA5);
  pinMode(TOUCH_CS, OUTPUT);
  digitalWrite(TOUCH_CS, HIGH);
  SPI.begin();
  SPI.setClockDivider(128);

  lv_init();

  static lv_disp_draw_buf_t draw_buf;
  static lv_color_t buf1[800 * 480 / 10];                        /*Declare a buffer for 1/10 screen size*/
  lv_disp_draw_buf_init(&draw_buf, buf1, NULL, (800 * 480 / 10));  /*Initialize the display buffer.*/

  eeprom.initialize();

  // check for setup mode forced, if so calibrate display
  if (0) {
    calibrateTouch(calibrationData, 15);
  }
  readtouch(calibrationData);
  setTouch(calibrationData);

  static lv_disp_drv_t disp_drv;        /*Descriptor of a display driver*/
  lv_disp_drv_init(&disp_drv);          /*Basic initialization*/
  disp_drv.flush_cb = my_disp_flush;    /*Set your driver function*/
  disp_drv.draw_buf = &draw_buf;          /*Assign the buffer to the display*/
  disp_drv.hor_res = 800;   /*Set the horizontal resolution of the display*/
  disp_drv.ver_res = 480;   /*Set the verizontal resolution of the display*/
  lv_disp_drv_register(&disp_drv);      /*Finally register the driver*/

  static lv_indev_drv_t indev_drv;           /*Descriptor of a input device driver*/
  lv_indev_drv_init(&indev_drv);             /*Basic initialization*/
  indev_drv.type = LV_INDEV_TYPE_POINTER;    /*Touch pad is a pointer-like device*/
  indev_drv.read_cb = my_touchpad_read;      /*Set your driver function*/
  lv_indev_drv_register(&indev_drv);         /*Finally register the driver*/

  /*Initialize the graphics library's tick*/
  //  tick.attach_ms(LVGL_TICK_PERIOD, lv_tick_handler);

  // Interval in microsecs
  ITimer.attachInterruptInterval(HW_TIMER_INTERVAL_MS * 1000, TimerHandler);
  // interval msec
  ISR_Timer.setInterval(3,  handle_timer_int);

  setupscreens();

  __HAL_RCC_D2SRAM1_CLK_ENABLE();
  __HAL_RCC_D2SRAM2_CLK_ENABLE();
  __HAL_RCC_D2SRAM3_CLK_ENABLE();

  loop_timer = millis();

  lv_tabview_set_act(tabview, 0, LV_ANIM_OFF);

}

void loop() {




  if (millis() - loop_timer >= 5) {
    lv_tick_inc(1);
    lv_task_handler(); /* let the GUI do its work */
    loop_timer = millis();
  }
}


void setupscreens(void) {
  lv_obj_t * label;

  // tabs
  tabview = lv_tabview_create(lv_scr_act(), LV_DIR_TOP, 0);
  lv_obj_t *tabmain = lv_tabview_add_tab(tabview, "");

  lv_obj_set_style_bg_color (tabmain, lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT );

  lv_obj_set_style_bg_opa( tabmain, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);

  lv_obj_clear_flag(tabview, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tabmain, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_set_scrollbar_mode(tabview, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_scrollbar_mode(tabmain, LV_SCROLLBAR_MODE_OFF);

  static lv_style_t style12;
  lv_style_set_text_color(&style12,  lv_color_white());
  lv_style_set_text_font(&style12,  &lv_font_montserrat_28);


  // main screen

  lv_obj_t * btn1 = lv_btn_create(tabmain);
  lv_obj_align(btn1, LV_ALIGN_TOP_LEFT, 10, 0);
  label = lv_label_create(btn1);
  lv_label_set_text(label, "Test button");
  lv_obj_add_style(label, &style12, 0);
  lv_obj_center(label);
  lv_obj_set_width(btn1, 200);
  lv_obj_set_height(btn1, 110);

  lv_obj_t * btn9 = lv_btn_create(tabmain);
  lv_obj_align(btn9, LV_ALIGN_BOTTOM_RIGHT, -10, 0);
  label = lv_label_create(btn9);
  lv_label_set_text(label, LV_SYMBOL_SETTINGS);
  lv_obj_center(label);
  lv_obj_set_width(btn9, 200);
  lv_obj_set_height(btn9, 110);

}










// LCD routines start

void Stm32_Clock_Init(u32 plln, u32 pllm, u32 pllp, u32 pllq)
{
  HAL_StatusTypeDef ret = HAL_OK;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  MODIFY_REG(PWR->CR3, PWR_CR3_SCUEN, 0);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while ((PWR->D3CR & (PWR_D3CR_VOSRDY)) != PWR_D3CR_VOSRDY) {}

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.CSIState = RCC_CSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

  RCC_OscInitStruct.PLL.PLLN = plln;
  RCC_OscInitStruct.PLL.PLLM = pllm;
  RCC_OscInitStruct.PLL.PLLP = pllp;
  RCC_OscInitStruct.PLL.PLLQ = pllq;

  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  // nog debuggen
  //  if (ret != HAL_OK) while (1);

  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | \
                                 RCC_CLOCKTYPE_HCLK | \
                                 RCC_CLOCKTYPE_D1PCLK1 | \
                                 RCC_CLOCKTYPE_PCLK1 | \
                                 RCC_CLOCKTYPE_PCLK2 | \
                                 RCC_CLOCKTYPE_D3PCLK1);

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV4;
  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
  if (ret != HAL_OK) while (1);

  __HAL_RCC_CSI_ENABLE() ;
  __HAL_RCC_SYSCFG_CLK_ENABLE() ;
  HAL_EnableCompensationCell();

  // test voor adc

  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 1;
  PeriphClkInitStruct.PLL2.PLL2N = 18;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 6144;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    error1();
  }


}

void delay_init(u16 SYSCLK)
{
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  fac_us = SYSCLK;
}

void delay_us(u32 nus)
{
  u32 ticks;
  u32 told, tnow, tcnt = 0;
  u32 reload = SysTick->LOAD;
  ticks = nus * fac_us;
  told = SysTick->VAL;
  while (1)
  {
    tnow = SysTick->VAL;
    if (tnow != told)
    {
      if (tnow < told)tcnt += told - tnow;
      else tcnt += reload - tnow + told;
      told = tnow;
      if (tcnt >= ticks)break;
    }
  };
}

void delay_ms(u16 nms)
{
  u32 i;
  for (i = 0; i < nms; i++) delay_us(1000);
}

u16 LCD_read(void)
{
  vu16 data;
  data = LCD->LCD_RAM;
  return data;
}

/*****************************************************************************
   @name       :void LCD_WR_REG(u16 data)
    @function   :Write an 16-bit command to the LCD screen
   @parameters :data:Command value to be written
   @retvalue   :None
******************************************************************************/
void LCD_WR_REG(vu16 data)
{
  data = data;
  LCD->LCD_REG = data;
}

/*****************************************************************************
   @name       :void LCD_WR_DATA(u16 data)
   @function   :Write an 16-bit data to the LCD screen
   @parameters :data:data value to be written
   @retvalue   :None
******************************************************************************/
void LCD_WR_DATA(vu16 data)
{
  data = data;
  LCD->LCD_RAM = data;
}

/*****************************************************************************
   @name       :u16 LCD_RD_DATA(void)
   @function   :Read an 16-bit value from the LCD screen
   @parameters :None
   @retvalue   :read value
******************************************************************************/
u16 LCD_RD_DATA(void)
{
  return LCD_read();
}

/*****************************************************************************
   @name       :void LCD_WriteReg(u16 LCD_Reg, u16 LCD_RegValue)
   @function   :Write data into registers
   @parameters :LCD_Reg:Register address
                LCD_RegValue:Data to be written
   @retvalue   :None
******************************************************************************/
void LCD_WriteReg(u16 LCD_Reg, u16 LCD_RegValue)
{
  LCD->LCD_REG = LCD_Reg;
  LCD->LCD_RAM = LCD_RegValue;
}

/*****************************************************************************
   @name       :void LCD_ReadReg(u16 LCD_Reg,u16 *Rval,int n)
   @function   :read value from specially registers
   @parameters :LCD_Reg:Register address
   @retvalue   :read value
******************************************************************************/
void LCD_ReadReg(u16 LCD_Reg, u16 * Rval, int n)
{
  LCD_WR_REG(LCD_Reg);
  while (n--)
  {
    *(Rval++) = LCD_RD_DATA();
    delay_us(5);
  }
}

/*****************************************************************************
   @name       :void LCD_WriteRAM_Prepare(void)
   @function   :Write GRAM
   @parameters :None
   @retvalue   :None
******************************************************************************/
void LCD_WriteRAM_Prepare(void)
{
  LCD_WR_REG(lcddev.wramcmd);
}

/*****************************************************************************
   @name       :void LCD_ReadRAM_Prepare(void)
   @function   :Read GRAM
   @parameters :None
   @retvalue   :None
******************************************************************************/
void LCD_ReadRAM_Prepare(void)
{
  LCD_WR_REG(lcddev.rramcmd);
}

/*****************************************************************************
   @name       :void Lcd_WriteData_16Bit(u16 Data)
   @function   :Write an 16-bit command to the LCD screen
   @parameters :Data:Data to be written
   @retvalue   :None
******************************************************************************/
void Lcd_WriteData_16Bit(u16 Data)
{
  LCD->LCD_RAM = Data;
}

u16 Color_To_565(u8 r, u8 g, u8 b)
{
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3);
}

/*****************************************************************************
   @name       :u16 Lcd_ReadData_16Bit(void)
   @function   :Read an 16-bit value from the LCD screen
   @parameters :None
   @retvalue   :read value
******************************************************************************/
u16 Lcd_ReadData_16Bit(void)
{
  u16 r, g, b;
  //dummy data
  r = LCD_RD_DATA();
  delay_us(1);
  //16bit:red and green data
  r = LCD_RD_DATA();
  delay_us(1);
  //16bit:blue data
  g = LCD_RD_DATA();
  b = g >> 8;
  g = r & 0xFF;
  r = r >> 8;
  return Color_To_565(r, g, b);
}

/*****************************************************************************
   @name       :void LCD_DrawPoint(u16 x,u16 y)
    @function   :Write a pixel data at a specified location
   @parameters :x:the x coordinate of the pixel
                y:the y coordinate of the pixel
   @retvalue   :None
******************************************************************************/
void LCD_DrawPoint(u16 x, u16 y)
{
  LCD_SetCursor(x, y);
  Lcd_WriteData_16Bit(POINT_COLOR);
}

/*****************************************************************************
   @name       :u16 LCD_ReadPoint(u16 x,u16 y)
   @function   :Read a pixel color value at a specified location
   @parameters :x:the x coordinate of the pixel
                y:the y coordinate of the pixel
   @retvalue   :the read color value
******************************************************************************/
u16 LCD_ReadPoint(u16 x, u16 y)
{
  u16 color;
  if (x >= lcddev.width || y >= lcddev.height)
  {
    return 0;
  }
  LCD_SetCursor(x, y);
  LCD_ReadRAM_Prepare();
  color = Lcd_ReadData_16Bit();
  return color;
}

/*****************************************************************************
   @name       :void LCD_Clear(u16 Color)
   @function   :Full screen filled LCD screen
   @parameters :color:Filled color
   @retvalue   :None
******************************************************************************/
void LCD_Clear(u16 Color)
{
  unsigned int i;
  u32 total_point = lcddev.width * lcddev.height;
  LCD_SetWindows(0, 0, lcddev.width - 1, lcddev.height - 1);
  for (i = 0; i < total_point; i++)
  {
    LCD->LCD_RAM = Color;
  }
}

/*****************************************************************************
   @name       :void LCD_MPU_Config(void)
   @function   :Configure the region of the MPU, and configure
                the external SRAM area to be in write-through mode.
   @parameters :None
   @retvalue   :None
******************************************************************************/
void LCD_MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_Initure;

  HAL_MPU_Disable();
  MPU_Initure.Enable = MPU_REGION_ENABLE;
  MPU_Initure.Number = LCD_REGION_NUMBER;
  MPU_Initure.BaseAddress = LCD_ADDRESS_START;
  MPU_Initure.Size = LCD_REGION_SIZE;
  MPU_Initure.SubRegionDisable = 0X00;
  MPU_Initure.TypeExtField = MPU_TEX_LEVEL0;
  MPU_Initure.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_Initure.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_Initure.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_Initure.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_Initure.IsBufferable = MPU_ACCESS_BUFFERABLE;
  HAL_MPU_ConfigRegion(&MPU_Initure);
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/*****************************************************************************
   @name       :void HAL_SRAM_MspInit(SRAM_HandleTypeDef *hsram)
   @function   :SRAM underlying driver, clock enable, pin assignment
                This function will be called by HAL_SRAM_Init ()
   @parameters :hsram:SRAM handle
   @retvalue   :None
******************************************************************************/
void HAL_SRAM_MspInit(SRAM_HandleTypeDef * hsram)
{
  GPIO_InitTypeDef GPIO_Initure;

  __HAL_RCC_FMC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  //PD0,1,4,5,7,8,9,10,13,14,15
  GPIO_Initure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7 | GPIO_PIN_8 | \
                     GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_Initure.Mode = GPIO_MODE_AF_PP;  //¸´ÓÃ
  GPIO_Initure.Pull = GPIO_PULLUP;    //ÉÏÀ­
  GPIO_Initure.Speed = GPIO_SPEED_FREQ_VERY_HIGH; //¸ßËÙ
  GPIO_Initure.Alternate = GPIO_AF12_FMC; //¸´ÓÃÎªFMC
  HAL_GPIO_Init(GPIOD, &GPIO_Initure);

  //PE7,8,9,10,11,12,13,14,15
  GPIO_Initure.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | \
                     GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOE, &GPIO_Initure);
}

/*****************************************************************************
   @name       :void LCD_GPIOInit(void)
   @date       :2018-08-09
   @function   :Initialization LCD screen GPIO
   @parameters :None
   @retvalue   :None
******************************************************************************/
void LCD_GPIOInit(FMC_NORSRAM_TimingTypeDef FWT)
{
  GPIO_InitTypeDef GPIO_Initure;
  FMC_NORSRAM_TimingTypeDef FSMC_ReadWriteTim;
  //    FMC_NORSRAM_TimingTypeDef FSMC_WriteTim;
  __HAL_RCC_GPIOB_CLK_ENABLE();     //¿ªÆôGPIOBÊ±ÖÓ
  GPIO_Initure.Pin = GPIO_PIN_5 | GPIO_PIN_8;        //PB5,±³¹â¿ØÖÆ PB8,¸´Î»
  GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP; //ÍÆÍìÊä³ö
  GPIO_Initure.Pull = GPIO_PULLUP;        //ÉÏÀ­
  GPIO_Initure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;  //¸ßËÙ
  HAL_GPIO_Init(GPIOB, &GPIO_Initure);

  LCD_MPU_Config();
  SRAM_Handler.Instance = FMC_NORSRAM_DEVICE; //BANK1
  SRAM_Handler.Extended = FMC_NORSRAM_EXTENDED_DEVICE;

  SRAM_Handler.Init.NSBank = FMC_NORSRAM_BANK1;   //Ê¹ÓÃNE1
  SRAM_Handler.Init.DataAddressMux = FMC_DATA_ADDRESS_MUX_DISABLE; //²»¸´     ÓÃÊý¾ÝÏß
  SRAM_Handler.Init.MemoryType = FMC_MEMORY_TYPE_SRAM;            //SRAM
  SRAM_Handler.Init.MemoryDataWidth = FMC_NORSRAM_MEM_BUS_WIDTH_16; //16Î»Êý¾Ý¿í¶È
  SRAM_Handler.Init.BurstAccessMode = FMC_BURST_ACCESS_MODE_DISABLE; //ÊÇ·ñÊ¹ÄÜÍ»·¢·ÃÎÊ,½ö¶ÔÍ¬²½Í»·¢´æ´¢Æ÷ÓÐÐ§,´Ë´¦Î´ÓÃµ½
  SRAM_Handler.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW; //µÈ´ýÐÅºÅµÄ¼«ÐÔ,½öÔÚÍ»·¢Ä£Ê½·ÃÎÊÏÂÓÐÓÃ
  SRAM_Handler.Init.WaitSignalActive = FMC_WAIT_TIMING_BEFORE_WS; //´æ´¢Æ÷ÊÇÔÚµÈ´ýÖÜÆÚÖ®Ç°µÄÒ»¸öÊ±ÖÓÖÜÆÚ»¹ÊÇµÈ´ýÖÜÆÚÆÚ¼äÊ¹ÄÜNWAIT
  SRAM_Handler.Init.WriteOperation = FMC_WRITE_OPERATION_ENABLE;  //´æ´¢Æ÷Ð´Ê¹ÄÜ
  SRAM_Handler.Init.WaitSignal = FMC_WAIT_SIGNAL_DISABLE;         //µÈ´ýÊ¹ÄÜÎ»,´Ë´¦Î´ÓÃµ½
  SRAM_Handler.Init.ExtendedMode = FMC_EXTENDED_MODE_ENABLE;      //¶ÁÐ´Ê¹ÓÃ²»Í¬µÄÊ±Ðò
  SRAM_Handler.Init.AsynchronousWait = FMC_ASYNCHRONOUS_WAIT_DISABLE; //ÊÇ·ñÊ¹ÄÜÍ¬²½´«ÊäÄ£Ê½ÏÂµÄµÈ´ýÐÅºÅ,´Ë´¦Î´ÓÃµ½
  SRAM_Handler.Init.WriteBurst = FMC_WRITE_BURST_DISABLE;         //½ûÖ¹Í»·¢Ð´
  SRAM_Handler.Init.ContinuousClock = FMC_CONTINUOUS_CLOCK_SYNC_ASYNC;

  //FSMC
  //FSMC_ReadWriteTim.AddressSetupTime = 0x11;  //µØÖ·½¨Á¢Ê±¼ä(ADDSET)Îª17¸öHCLK 1/216M=4.6ns*17=78ns
  FSMC_ReadWriteTim.AddressSetupTime = 10;

  FSMC_ReadWriteTim.AddressHoldTime = 0x00;
  //FSMC_ReadWriteTim.DataSetupTime = 0x55;     //Êý¾Ý±£´æÊ±¼ä(DATAST)Îª85¸öHCLK  =4.6*85=391ns
  FSMC_ReadWriteTim.DataSetupTime = 100;


  FSMC_ReadWriteTim.AccessMode = FMC_ACCESS_MODE_A; //Ä£Ê½A
  //FSMC
  //FWT.AddressSetupTime = 0x15;
  FWT.AddressSetupTime = 10;

  FWT.AddressHoldTime = 0x00;
  //FWT.DataSetupTime = 0x15;
  FWT.DataSetupTime = 20;

  FWT.AccessMode = FMC_ACCESS_MODE_A;   //Ä£Ê½A

  HAL_SRAM_Init(&SRAM_Handler, &FSMC_ReadWriteTim, &FWT);

}

void LCD_FSMC_Write_Time_Set(FMC_NORSRAM_TimingTypeDef FWT, uint32_t AST, uint32_t DST)
{
  //  FMC_NORSRAM_TimingTypeDef FSMC_WriteTim;
  FWT.AddressSetupTime = AST;
  FWT.DataSetupTime = DST;
  FMC_NORSRAM_Extended_Timing_Init(SRAM_Handler.Extended, &FWT, SRAM_Handler.Init.NSBank, SRAM_Handler.Init.ExtendedMode);
}

/*****************************************************************************
   @name       :void LCD_RESET(void)
   @function   :Reset LCD screen
   @parameters :None
   @retvalue   :None
******************************************************************************/
void LCD_RESET(void)
{
  LCD_RST_CLR();
  delay_ms(100);
  LCD_RST_SET();
  delay_ms(50);
}

/*****************************************************************************
   @name       :void LCD_Init(void)
    @function   :Initialization LCD screen
   @parameters :None
   @retvalue   :None
******************************************************************************/
void LCD_Init(void)
{
  FMC_NORSRAM_TimingTypeDef FSMC_WriteTim;
  LCD_GPIOInit(FSMC_WriteTim);//LCD GPIO³õÊ¼»¯
  delay_ms(50);
  //LCD_RESET();
  //************* NT35510**********//
  LCD_WR_REG(0xF000); LCD_WR_DATA(0x55);
  LCD_WR_REG(0xF001); LCD_WR_DATA(0xAA);
  LCD_WR_REG(0xF002); LCD_WR_DATA(0x52);
  LCD_WR_REG(0xF003); LCD_WR_DATA(0x08);
  LCD_WR_REG(0xF004); LCD_WR_DATA(0x01);
  //# AVDD: manual); LCD_WR_DATA(
  LCD_WR_REG(0xB600); LCD_WR_DATA(0x34);
  LCD_WR_REG(0xB601); LCD_WR_DATA(0x34);
  LCD_WR_REG(0xB602); LCD_WR_DATA(0x34);

  LCD_WR_REG(0xB000); LCD_WR_DATA(0x0D);//09
  LCD_WR_REG(0xB001); LCD_WR_DATA(0x0D);
  LCD_WR_REG(0xB002); LCD_WR_DATA(0x0D);
  //# AVEE: manual); LCD_WR_DATA( -6V
  LCD_WR_REG(0xB700); LCD_WR_DATA(0x24);
  LCD_WR_REG(0xB701); LCD_WR_DATA(0x24);
  LCD_WR_REG(0xB702); LCD_WR_DATA(0x24);

  LCD_WR_REG(0xB100); LCD_WR_DATA(0x0D);
  LCD_WR_REG(0xB101); LCD_WR_DATA(0x0D);
  LCD_WR_REG(0xB102); LCD_WR_DATA(0x0D);
  //#Power Control for
  //VCL
  LCD_WR_REG(0xB800); LCD_WR_DATA(0x24);
  LCD_WR_REG(0xB801); LCD_WR_DATA(0x24);
  LCD_WR_REG(0xB802); LCD_WR_DATA(0x24);

  LCD_WR_REG(0xB200); LCD_WR_DATA(0x00);

  //# VGH: Clamp Enable); LCD_WR_DATA(
  LCD_WR_REG(0xB900); LCD_WR_DATA(0x24);
  LCD_WR_REG(0xB901); LCD_WR_DATA(0x24);
  LCD_WR_REG(0xB902); LCD_WR_DATA(0x24);

  LCD_WR_REG(0xB300); LCD_WR_DATA(0x05);
  LCD_WR_REG(0xB301); LCD_WR_DATA(0x05);
  LCD_WR_REG(0xB302); LCD_WR_DATA(0x05);

  ///LCD_WR_REG(0xBF00); LCD_WR_DATA(0x01);

  //# VGL(LVGL):
  LCD_WR_REG(0xBA00); LCD_WR_DATA(0x34);
  LCD_WR_REG(0xBA01); LCD_WR_DATA(0x34);
  LCD_WR_REG(0xBA02); LCD_WR_DATA(0x34);
  //# VGL_REG(VGLO)
  LCD_WR_REG(0xB500); LCD_WR_DATA(0x0B);
  LCD_WR_REG(0xB501); LCD_WR_DATA(0x0B);
  LCD_WR_REG(0xB502); LCD_WR_DATA(0x0B);
  //# VGMP/VGSP:
  LCD_WR_REG(0xBC00); LCD_WR_DATA(0X00);
  LCD_WR_REG(0xBC01); LCD_WR_DATA(0xA3);
  LCD_WR_REG(0xBC02); LCD_WR_DATA(0X00);
  //# VGMN/VGSN
  LCD_WR_REG(0xBD00); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xBD01); LCD_WR_DATA(0xA3);
  LCD_WR_REG(0xBD02); LCD_WR_DATA(0x00);
  //# VCOM=-0.1
  LCD_WR_REG(0xBE00); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xBE01); LCD_WR_DATA(0x63);
  //  VCOMH+0x01;
  //#R+
  LCD_WR_REG(0xD100); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD101); LCD_WR_DATA(0x37);
  LCD_WR_REG(0xD102); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD103); LCD_WR_DATA(0x52);
  LCD_WR_REG(0xD104); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD105); LCD_WR_DATA(0x7B);
  LCD_WR_REG(0xD106); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD107); LCD_WR_DATA(0x99);
  LCD_WR_REG(0xD108); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD109); LCD_WR_DATA(0xB1);
  LCD_WR_REG(0xD10A); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD10B); LCD_WR_DATA(0xD2);
  LCD_WR_REG(0xD10C); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD10D); LCD_WR_DATA(0xF6);
  LCD_WR_REG(0xD10E); LCD_WR_DATA(0x01);
  LCD_WR_REG(0xD10F); LCD_WR_DATA(0x27);
  LCD_WR_REG(0xD110); LCD_WR_DATA(0x01);
  LCD_WR_REG(0xD111); LCD_WR_DATA(0x4E);
  LCD_WR_REG(0xD112); LCD_WR_DATA(0x01);
  LCD_WR_REG(0xD113); LCD_WR_DATA(0x8C);
  LCD_WR_REG(0xD114); LCD_WR_DATA(0x01);
  LCD_WR_REG(0xD115); LCD_WR_DATA(0xBE);
  LCD_WR_REG(0xD116); LCD_WR_DATA(0x02);
  LCD_WR_REG(0xD117); LCD_WR_DATA(0x0B);
  LCD_WR_REG(0xD118); LCD_WR_DATA(0x02);
  LCD_WR_REG(0xD119); LCD_WR_DATA(0x48);
  LCD_WR_REG(0xD11A); LCD_WR_DATA(0x02);
  LCD_WR_REG(0xD11B); LCD_WR_DATA(0x4A);
  LCD_WR_REG(0xD11C); LCD_WR_DATA(0x02);
  LCD_WR_REG(0xD11D); LCD_WR_DATA(0x7E);
  LCD_WR_REG(0xD11E); LCD_WR_DATA(0x02);
  LCD_WR_REG(0xD11F); LCD_WR_DATA(0xBC);
  LCD_WR_REG(0xD120); LCD_WR_DATA(0x02);
  LCD_WR_REG(0xD121); LCD_WR_DATA(0xE1);
  LCD_WR_REG(0xD122); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD123); LCD_WR_DATA(0x10);
  LCD_WR_REG(0xD124); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD125); LCD_WR_DATA(0x31);
  LCD_WR_REG(0xD126); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD127); LCD_WR_DATA(0x5A);
  LCD_WR_REG(0xD128); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD129); LCD_WR_DATA(0x73);
  LCD_WR_REG(0xD12A); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD12B); LCD_WR_DATA(0x94);
  LCD_WR_REG(0xD12C); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD12D); LCD_WR_DATA(0x9F);
  LCD_WR_REG(0xD12E); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD12F); LCD_WR_DATA(0xB3);
  LCD_WR_REG(0xD130); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD131); LCD_WR_DATA(0xB9);
  LCD_WR_REG(0xD132); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD133); LCD_WR_DATA(0xC1);
  //#G+
  LCD_WR_REG(0xD200); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD201); LCD_WR_DATA(0x37);
  LCD_WR_REG(0xD202); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD203); LCD_WR_DATA(0x52);
  LCD_WR_REG(0xD204); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD205); LCD_WR_DATA(0x7B);
  LCD_WR_REG(0xD206); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD207); LCD_WR_DATA(0x99);
  LCD_WR_REG(0xD208); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD209); LCD_WR_DATA(0xB1);
  LCD_WR_REG(0xD20A); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD20B); LCD_WR_DATA(0xD2);
  LCD_WR_REG(0xD20C); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD20D); LCD_WR_DATA(0xF6);
  LCD_WR_REG(0xD20E); LCD_WR_DATA(0x01);
  LCD_WR_REG(0xD20F); LCD_WR_DATA(0x27);
  LCD_WR_REG(0xD210); LCD_WR_DATA(0x01);
  LCD_WR_REG(0xD211); LCD_WR_DATA(0x4E);
  LCD_WR_REG(0xD212); LCD_WR_DATA(0x01);
  LCD_WR_REG(0xD213); LCD_WR_DATA(0x8C);
  LCD_WR_REG(0xD214); LCD_WR_DATA(0x01);
  LCD_WR_REG(0xD215); LCD_WR_DATA(0xBE);
  LCD_WR_REG(0xD216); LCD_WR_DATA(0x02);
  LCD_WR_REG(0xD217); LCD_WR_DATA(0x0B);
  LCD_WR_REG(0xD218); LCD_WR_DATA(0x02);
  LCD_WR_REG(0xD219); LCD_WR_DATA(0x48);
  LCD_WR_REG(0xD21A); LCD_WR_DATA(0x02);
  LCD_WR_REG(0xD21B); LCD_WR_DATA(0x4A);
  LCD_WR_REG(0xD21C); LCD_WR_DATA(0x02);
  LCD_WR_REG(0xD21D); LCD_WR_DATA(0x7E);
  LCD_WR_REG(0xD21E); LCD_WR_DATA(0x02);
  LCD_WR_REG(0xD21F); LCD_WR_DATA(0xBC);
  LCD_WR_REG(0xD220); LCD_WR_DATA(0x02);
  LCD_WR_REG(0xD221); LCD_WR_DATA(0xE1);
  LCD_WR_REG(0xD222); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD223); LCD_WR_DATA(0x10);
  LCD_WR_REG(0xD224); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD225); LCD_WR_DATA(0x31);
  LCD_WR_REG(0xD226); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD227); LCD_WR_DATA(0x5A);
  LCD_WR_REG(0xD228); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD229); LCD_WR_DATA(0x73);
  LCD_WR_REG(0xD22A); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD22B); LCD_WR_DATA(0x94);
  LCD_WR_REG(0xD22C); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD22D); LCD_WR_DATA(0x9F);
  LCD_WR_REG(0xD22E); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD22F); LCD_WR_DATA(0xB3);
  LCD_WR_REG(0xD230); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD231); LCD_WR_DATA(0xB9);
  LCD_WR_REG(0xD232); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD233); LCD_WR_DATA(0xC1);
  //#B+
  LCD_WR_REG(0xD300); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD301); LCD_WR_DATA(0x37);
  LCD_WR_REG(0xD302); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD303); LCD_WR_DATA(0x52);
  LCD_WR_REG(0xD304); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD305); LCD_WR_DATA(0x7B);
  LCD_WR_REG(0xD306); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD307); LCD_WR_DATA(0x99);
  LCD_WR_REG(0xD308); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD309); LCD_WR_DATA(0xB1);
  LCD_WR_REG(0xD30A); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD30B); LCD_WR_DATA(0xD2);
  LCD_WR_REG(0xD30C); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD30D); LCD_WR_DATA(0xF6);
  LCD_WR_REG(0xD30E); LCD_WR_DATA(0x01);
  LCD_WR_REG(0xD30F); LCD_WR_DATA(0x27);
  LCD_WR_REG(0xD310); LCD_WR_DATA(0x01);
  LCD_WR_REG(0xD311); LCD_WR_DATA(0x4E);
  LCD_WR_REG(0xD312); LCD_WR_DATA(0x01);
  LCD_WR_REG(0xD313); LCD_WR_DATA(0x8C);
  LCD_WR_REG(0xD314); LCD_WR_DATA(0x01);
  LCD_WR_REG(0xD315); LCD_WR_DATA(0xBE);
  LCD_WR_REG(0xD316); LCD_WR_DATA(0x02);
  LCD_WR_REG(0xD317); LCD_WR_DATA(0x0B);
  LCD_WR_REG(0xD318); LCD_WR_DATA(0x02);
  LCD_WR_REG(0xD319); LCD_WR_DATA(0x48);
  LCD_WR_REG(0xD31A); LCD_WR_DATA(0x02);
  LCD_WR_REG(0xD31B); LCD_WR_DATA(0x4A);
  LCD_WR_REG(0xD31C); LCD_WR_DATA(0x02);
  LCD_WR_REG(0xD31D); LCD_WR_DATA(0x7E);
  LCD_WR_REG(0xD31E); LCD_WR_DATA(0x02);
  LCD_WR_REG(0xD31F); LCD_WR_DATA(0xBC);
  LCD_WR_REG(0xD320); LCD_WR_DATA(0x02);
  LCD_WR_REG(0xD321); LCD_WR_DATA(0xE1);
  LCD_WR_REG(0xD322); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD323); LCD_WR_DATA(0x10);
  LCD_WR_REG(0xD324); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD325); LCD_WR_DATA(0x31);
  LCD_WR_REG(0xD326); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD327); LCD_WR_DATA(0x5A);
  LCD_WR_REG(0xD328); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD329); LCD_WR_DATA(0x73);
  LCD_WR_REG(0xD32A); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD32B); LCD_WR_DATA(0x94);
  LCD_WR_REG(0xD32C); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD32D); LCD_WR_DATA(0x9F);
  LCD_WR_REG(0xD32E); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD32F); LCD_WR_DATA(0xB3);
  LCD_WR_REG(0xD330); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD331); LCD_WR_DATA(0xB9);
  LCD_WR_REG(0xD332); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD333); LCD_WR_DATA(0xC1);

  //#R-///////////////////////////////////////////
  LCD_WR_REG(0xD400); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD401); LCD_WR_DATA(0x37);
  LCD_WR_REG(0xD402); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD403); LCD_WR_DATA(0x52);
  LCD_WR_REG(0xD404); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD405); LCD_WR_DATA(0x7B);
  LCD_WR_REG(0xD406); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD407); LCD_WR_DATA(0x99);
  LCD_WR_REG(0xD408); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD409); LCD_WR_DATA(0xB1);
  LCD_WR_REG(0xD40A); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD40B); LCD_WR_DATA(0xD2);
  LCD_WR_REG(0xD40C); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD40D); LCD_WR_DATA(0xF6);
  LCD_WR_REG(0xD40E); LCD_WR_DATA(0x01);
  LCD_WR_REG(0xD40F); LCD_WR_DATA(0x27);
  LCD_WR_REG(0xD410); LCD_WR_DATA(0x01);
  LCD_WR_REG(0xD411); LCD_WR_DATA(0x4E);
  LCD_WR_REG(0xD412); LCD_WR_DATA(0x01);
  LCD_WR_REG(0xD413); LCD_WR_DATA(0x8C);
  LCD_WR_REG(0xD414); LCD_WR_DATA(0x01);
  LCD_WR_REG(0xD415); LCD_WR_DATA(0xBE);
  LCD_WR_REG(0xD416); LCD_WR_DATA(0x02);
  LCD_WR_REG(0xD417); LCD_WR_DATA(0x0B);
  LCD_WR_REG(0xD418); LCD_WR_DATA(0x02);
  LCD_WR_REG(0xD419); LCD_WR_DATA(0x48);
  LCD_WR_REG(0xD41A); LCD_WR_DATA(0x02);
  LCD_WR_REG(0xD41B); LCD_WR_DATA(0x4A);
  LCD_WR_REG(0xD41C); LCD_WR_DATA(0x02);
  LCD_WR_REG(0xD41D); LCD_WR_DATA(0x7E);
  LCD_WR_REG(0xD41E); LCD_WR_DATA(0x02);
  LCD_WR_REG(0xD41F); LCD_WR_DATA(0xBC);
  LCD_WR_REG(0xD420); LCD_WR_DATA(0x02);
  LCD_WR_REG(0xD421); LCD_WR_DATA(0xE1);
  LCD_WR_REG(0xD422); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD423); LCD_WR_DATA(0x10);
  LCD_WR_REG(0xD424); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD425); LCD_WR_DATA(0x31);
  LCD_WR_REG(0xD426); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD427); LCD_WR_DATA(0x5A);
  LCD_WR_REG(0xD428); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD429); LCD_WR_DATA(0x73);
  LCD_WR_REG(0xD42A); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD42B); LCD_WR_DATA(0x94);
  LCD_WR_REG(0xD42C); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD42D); LCD_WR_DATA(0x9F);
  LCD_WR_REG(0xD42E); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD42F); LCD_WR_DATA(0xB3);
  LCD_WR_REG(0xD430); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD431); LCD_WR_DATA(0xB9);
  LCD_WR_REG(0xD432); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD433); LCD_WR_DATA(0xC1);

  //#G-//////////////////////////////////////////////
  LCD_WR_REG(0xD500); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD501); LCD_WR_DATA(0x37);
  LCD_WR_REG(0xD502); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD503); LCD_WR_DATA(0x52);
  LCD_WR_REG(0xD504); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD505); LCD_WR_DATA(0x7B);
  LCD_WR_REG(0xD506); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD507); LCD_WR_DATA(0x99);
  LCD_WR_REG(0xD508); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD509); LCD_WR_DATA(0xB1);
  LCD_WR_REG(0xD50A); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD50B); LCD_WR_DATA(0xD2);
  LCD_WR_REG(0xD50C); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD50D); LCD_WR_DATA(0xF6);
  LCD_WR_REG(0xD50E); LCD_WR_DATA(0x01);
  LCD_WR_REG(0xD50F); LCD_WR_DATA(0x27);
  LCD_WR_REG(0xD510); LCD_WR_DATA(0x01);
  LCD_WR_REG(0xD511); LCD_WR_DATA(0x4E);
  LCD_WR_REG(0xD512); LCD_WR_DATA(0x01);
  LCD_WR_REG(0xD513); LCD_WR_DATA(0x8C);
  LCD_WR_REG(0xD514); LCD_WR_DATA(0x01);
  LCD_WR_REG(0xD515); LCD_WR_DATA(0xBE);
  LCD_WR_REG(0xD516); LCD_WR_DATA(0x02);
  LCD_WR_REG(0xD517); LCD_WR_DATA(0x0B);
  LCD_WR_REG(0xD518); LCD_WR_DATA(0x02);
  LCD_WR_REG(0xD519); LCD_WR_DATA(0x48);
  LCD_WR_REG(0xD51A); LCD_WR_DATA(0x02);
  LCD_WR_REG(0xD51B); LCD_WR_DATA(0x4A);
  LCD_WR_REG(0xD51C); LCD_WR_DATA(0x02);
  LCD_WR_REG(0xD51D); LCD_WR_DATA(0x7E);
  LCD_WR_REG(0xD51E); LCD_WR_DATA(0x02);
  LCD_WR_REG(0xD51F); LCD_WR_DATA(0xBC);
  LCD_WR_REG(0xD520); LCD_WR_DATA(0x02);
  LCD_WR_REG(0xD521); LCD_WR_DATA(0xE1);
  LCD_WR_REG(0xD522); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD523); LCD_WR_DATA(0x10);
  LCD_WR_REG(0xD524); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD525); LCD_WR_DATA(0x31);
  LCD_WR_REG(0xD526); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD527); LCD_WR_DATA(0x5A);
  LCD_WR_REG(0xD528); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD529); LCD_WR_DATA(0x73);
  LCD_WR_REG(0xD52A); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD52B); LCD_WR_DATA(0x94);
  LCD_WR_REG(0xD52C); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD52D); LCD_WR_DATA(0x9F);
  LCD_WR_REG(0xD52E); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD52F); LCD_WR_DATA(0xB3);
  LCD_WR_REG(0xD530); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD531); LCD_WR_DATA(0xB9);
  LCD_WR_REG(0xD532); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD533); LCD_WR_DATA(0xC1);
  //#B-///////////////////////////////
  LCD_WR_REG(0xD600); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD601); LCD_WR_DATA(0x37);
  LCD_WR_REG(0xD602); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD603); LCD_WR_DATA(0x52);
  LCD_WR_REG(0xD604); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD605); LCD_WR_DATA(0x7B);
  LCD_WR_REG(0xD606); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD607); LCD_WR_DATA(0x99);
  LCD_WR_REG(0xD608); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD609); LCD_WR_DATA(0xB1);
  LCD_WR_REG(0xD60A); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD60B); LCD_WR_DATA(0xD2);
  LCD_WR_REG(0xD60C); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xD60D); LCD_WR_DATA(0xF6);
  LCD_WR_REG(0xD60E); LCD_WR_DATA(0x01);
  LCD_WR_REG(0xD60F); LCD_WR_DATA(0x27);
  LCD_WR_REG(0xD610); LCD_WR_DATA(0x01);
  LCD_WR_REG(0xD611); LCD_WR_DATA(0x4E);
  LCD_WR_REG(0xD612); LCD_WR_DATA(0x01);
  LCD_WR_REG(0xD613); LCD_WR_DATA(0x8C);
  LCD_WR_REG(0xD614); LCD_WR_DATA(0x01);
  LCD_WR_REG(0xD615); LCD_WR_DATA(0xBE);
  LCD_WR_REG(0xD616); LCD_WR_DATA(0x02);
  LCD_WR_REG(0xD617); LCD_WR_DATA(0x0B);
  LCD_WR_REG(0xD618); LCD_WR_DATA(0x02);
  LCD_WR_REG(0xD619); LCD_WR_DATA(0x48);
  LCD_WR_REG(0xD61A); LCD_WR_DATA(0x02);
  LCD_WR_REG(0xD61B); LCD_WR_DATA(0x4A);
  LCD_WR_REG(0xD61C); LCD_WR_DATA(0x02);
  LCD_WR_REG(0xD61D); LCD_WR_DATA(0x7E);
  LCD_WR_REG(0xD61E); LCD_WR_DATA(0x02);
  LCD_WR_REG(0xD61F); LCD_WR_DATA(0xBC);
  LCD_WR_REG(0xD620); LCD_WR_DATA(0x02);
  LCD_WR_REG(0xD621); LCD_WR_DATA(0xE1);
  LCD_WR_REG(0xD622); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD623); LCD_WR_DATA(0x10);
  LCD_WR_REG(0xD624); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD625); LCD_WR_DATA(0x31);
  LCD_WR_REG(0xD626); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD627); LCD_WR_DATA(0x5A);
  LCD_WR_REG(0xD628); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD629); LCD_WR_DATA(0x73);
  LCD_WR_REG(0xD62A); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD62B); LCD_WR_DATA(0x94);
  LCD_WR_REG(0xD62C); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD62D); LCD_WR_DATA(0x9F);
  LCD_WR_REG(0xD62E); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD62F); LCD_WR_DATA(0xB3);
  LCD_WR_REG(0xD630); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD631); LCD_WR_DATA(0xB9);
  LCD_WR_REG(0xD632); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xD633); LCD_WR_DATA(0xC1);

  //#Enable Page0
  LCD_WR_REG(0xF000); LCD_WR_DATA(0x55);
  LCD_WR_REG(0xF001); LCD_WR_DATA(0xAA);
  LCD_WR_REG(0xF002); LCD_WR_DATA(0x52);
  LCD_WR_REG(0xF003); LCD_WR_DATA(0x08);
  LCD_WR_REG(0xF004); LCD_WR_DATA(0x00);
  //# RGB I/F Setting
  LCD_WR_REG(0xB000); LCD_WR_DATA(0x08);
  LCD_WR_REG(0xB001); LCD_WR_DATA(0x05);
  LCD_WR_REG(0xB002); LCD_WR_DATA(0x02);
  LCD_WR_REG(0xB003); LCD_WR_DATA(0x05);
  LCD_WR_REG(0xB004); LCD_WR_DATA(0x02);
  //## SDT:
  LCD_WR_REG(0xB600); LCD_WR_DATA(0x08);
  LCD_WR_REG(0xB500); LCD_WR_DATA(0x50);

  //## Gate EQ:
  LCD_WR_REG(0xB700); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xB701); LCD_WR_DATA(0x00);

  //## Source EQ:
  LCD_WR_REG(0xB800); LCD_WR_DATA(0x01);
  LCD_WR_REG(0xB801); LCD_WR_DATA(0x05);
  LCD_WR_REG(0xB802); LCD_WR_DATA(0x05);
  LCD_WR_REG(0xB803); LCD_WR_DATA(0x05);

  //# Inversion: Column inversion (NVT)
  LCD_WR_REG(0xBC00); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xBC01); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xBC02); LCD_WR_DATA(0x00);

  //# BOE's Setting(default)
  LCD_WR_REG(0xCC00); LCD_WR_DATA(0x03);
  LCD_WR_REG(0xCC01); LCD_WR_DATA(0x00);
  LCD_WR_REG(0xCC02); LCD_WR_DATA(0x00);

  //# Display Timing:
  LCD_WR_REG(0xBD00); LCD_WR_DATA(0x01);
  LCD_WR_REG(0xBD01); LCD_WR_DATA(0x84);
  LCD_WR_REG(0xBD02); LCD_WR_DATA(0x07);
  LCD_WR_REG(0xBD03); LCD_WR_DATA(0x31);
  LCD_WR_REG(0xBD04); LCD_WR_DATA(0x00);

  LCD_WR_REG(0xBA00); LCD_WR_DATA(0x01);

  LCD_WR_REG(0xFF00); LCD_WR_DATA(0xAA);
  LCD_WR_REG(0xFF01); LCD_WR_DATA(0x55);
  LCD_WR_REG(0xFF02); LCD_WR_DATA(0x25);
  LCD_WR_REG(0xFF03); LCD_WR_DATA(0x01);

  LCD_WR_REG(0x3500); LCD_WR_DATA(0x00);
  LCD_WR_REG(0x3600); LCD_WR_DATA(0x00);
  LCD_WR_REG(0x3a00); LCD_WR_DATA(0x55);
  LCD_WR_REG(0x1100);
  delay_ms(120);
  LCD_WR_REG(0x2900 );
  LCD_WR_REG(0x2c00);

  LCD_FSMC_Write_Time_Set(FSMC_WriteTim, 5, 5);
  LCD_direction(1);
  //  LCD_LED_SET();
  LCD_Clear(WHITE);

}

/*****************************************************************************
   @name       :void LCD_SetWindows(u16 xStar, u16 yStar,u16 xEnd,u16 yEnd)
   @function   :Setting LCD display window
   @parameters :xStar:the bebinning x coordinate of the LCD display window
                yStar:the bebinning y coordinate of the LCD display window
                xEnd:the endning x coordinate of the LCD display window
                yEnd:the endning y coordinate of the LCD display window
   @retvalue   :None
******************************************************************************/
void LCD_SetWindows(u16 xStar, u16 yStar, u16 xEnd, u16 yEnd)
{
  LCD_WR_REG(lcddev.setxcmd); LCD_WR_DATA(xStar >> 8);
  LCD_WR_REG(lcddev.setxcmd + 1); LCD_WR_DATA(xStar & 0XFF);
  LCD_WR_REG(lcddev.setxcmd + 2); LCD_WR_DATA(xEnd >> 8);
  LCD_WR_REG(lcddev.setxcmd + 3); LCD_WR_DATA(xEnd & 0XFF);
  LCD_WR_REG(lcddev.setycmd); LCD_WR_DATA(yStar >> 8);
  LCD_WR_REG(lcddev.setycmd + 1); LCD_WR_DATA(yStar & 0XFF);
  LCD_WR_REG(lcddev.setycmd + 2); LCD_WR_DATA(yEnd >> 8);
  LCD_WR_REG(lcddev.setycmd + 3); LCD_WR_DATA(yEnd & 0XFF);

  LCD_WriteRAM_Prepare();
}

/*****************************************************************************
   @name       :void LCD_SetCursor(u16 Xpos, u16 Ypos)
   @function   :Set coordinate value
   @parameters :Xpos:the  x coordinate of the pixel
                Ypos:the  y coordinate of the pixel
   @retvalue   :None
******************************************************************************/
void LCD_SetCursor(u16 Xpos, u16 Ypos)
{
  LCD_SetWindows(Xpos, Ypos, Xpos, Ypos);
}

/*****************************************************************************
   @name       :void LCD_direction(u8 direction)
   @function   :Setting the display direction of LCD screen
   @parameters :direction:0-0 degree
                          1-90 degree
                          2-180 degree
                          3-270 degree
   @retvalue   :None
******************************************************************************/
void LCD_direction(u8 direction)
{
  lcddev.setxcmd = 0x2A00;
  lcddev.setycmd = 0x2B00;
  lcddev.wramcmd = 0x2C00;
  lcddev.rramcmd = 0x2E00;
  switch (direction) {
    case 0:
      lcddev.width = LCD_H;
      lcddev.height = LCD_W;
      LCD_WriteReg(0x3600, 0x00);
      break;
    case 1:
      lcddev.width = LCD_W;
      lcddev.height = LCD_H;
      LCD_WriteReg(0x3600, (1 << 5) | (1 << 6));
      break;
    case 2:
      lcddev.width = LCD_H;
      lcddev.height = LCD_W;
      LCD_WriteReg(0x3600, (1 << 7) | (1 << 6));
      break;
    case 3:
      lcddev.width = LCD_W;
      lcddev.height = LCD_H;
      LCD_WriteReg(0x3600, (1 << 7) | (1 << 5));
      break;
    default: break;
  }
}

/*****************************************************************************
   @name       :u16 LCD_Read_ID(void)
   @function   :Read ID
   @parameters :None
   @retvalue   :ID value
******************************************************************************/
u16 LCD_Read_ID(void)
{
  u16 val;
  LCD_WR_REG(0xF000);
  LCD_WR_DATA(0x55);
  LCD_WR_REG(0xF001);
  LCD_WR_DATA(0xAA);
  LCD_WR_REG(0xF002);
  LCD_WR_DATA(0x52);
  LCD_WR_REG(0xF003);
  LCD_WR_DATA(0x08);
  LCD_WR_REG(0xF004);
  LCD_WR_DATA(0x01);

  LCD_ReadReg(0xC500, &lcddev.id, 1);
  lcddev.id <<= 8;
  LCD_ReadReg(0xC501, &val, 1);
  lcddev.id |= val;
  return lcddev.id;
}


/*******************************************************************
   @name       :void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2)
   @function   :Draw a line between two points
   @parameters :x1:the beginning x coordinate of the line
                y1:the beginning y coordinate of the line
                x2:the ending x coordinate of the line
                y2:the ending y coordinate of the line
   @retvalue   :None
********************************************************************/
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2)
{
  u16 t;
  int xerr = 0, yerr = 0, delta_x, delta_y, distance;
  int incx, incy, uRow, uCol;

  delta_x = x2 - x1;
  delta_y = y2 - y1;
  uRow = x1;
  uCol = y1;
  if (delta_x > 0)incx = 1;
  else if (delta_x == 0)incx = 0;
  else {
    incx = -1;
    delta_x = -delta_x;
  }
  if (delta_y > 0)incy = 1;
  else if (delta_y == 0)incy = 0;
  else {
    incy = -1;
    delta_y = -delta_y;
  }
  if ( delta_x > delta_y)distance = delta_x;
  else distance = delta_y;
  for (t = 0; t <= distance + 1; t++ )
  {
    LCD_DrawPoint(uRow, uCol);
    xerr += delta_x ;
    yerr += delta_y ;
    if (xerr > distance)
    {
      xerr -= distance;
      uRow += incx;
    }
    if (yerr > distance)
    {
      yerr -= distance;
      uCol += incy;
    }
  }
}

/*******************************************************************
   @name       :void LCD_Fill(u16 sx,u16 sy,u16 ex,u16 ey,u16 color)
   @function   :fill the specified area
   @parameters :sx:the bebinning x coordinate of the specified area
                sy:the bebinning y coordinate of the specified area
                ex:the ending x coordinate of the specified area
                ey:the ending y coordinate of the specified area
                color:the filled color value
   @retvalue   :None
********************************************************************/
void LCD_Fill(u16 sx, u16 sy, u16 ex, u16 ey, u16 color)
{
  u16 i, j;
  u16 width = ex - sx + 1;
  u16 height = ey - sy + 1;
  LCD_SetWindows(sx, sy, ex, ey);
  for (i = 0; i < height; i++)
  {
    for (j = 0; j < width; j++)
      Lcd_WriteData_16Bit(color);
  }
  //  LCD_SetWindows(0, 0, lcddev.width - 1, lcddev.height - 1);
}

/*****************************************************************************
   @name       :void LCD_DrawFillRectangle(u16 x1, u16 y1, u16 x2, u16 y2)
   @function   :Filled a rectangle
   @parameters :x1:the bebinning x coordinate of the filled rectangle
                y1:the bebinning y coordinate of the filled rectangle
                x2:the ending x coordinate of the filled rectangle
                y2:the ending y coordinate of the filled rectangle
   @retvalue   :None
******************************************************************************/
void LCD_DrawFillRectangle(u16 x1, u16 y1, u16 x2, u16 y2)
{
  LCD_Fill(x1, y1, x2, y2, POINT_COLOR);
}



// LCD routines end


// touch routines start

/***************************************************************************************
** Function name:           getTouchRaw
** Description:             read raw touch position.  Always returns true.
***************************************************************************************/
uint8_t getTouchRaw(uint16_t *x, uint16_t *y) {
  uint16_t tmp;

  digitalWrite(TOUCH_CS, LOW);

  // Start YP sample request for x position, read 4 times and keep last sample
  SPI.transfer(0xd0);                    // Start new YP conversion
  SPI.transfer(0);                       // Read first 8 bits
  SPI.transfer(0xd0);                    // Read last 8 bits and start new YP conversion
  SPI.transfer(0);                       // Read first 8 bits
  SPI.transfer(0xd0);                    // Read last 8 bits and start new YP conversion
  SPI.transfer(0);                       // Read first 8 bits
  SPI.transfer(0xd0);                    // Read last 8 bits and start new YP conversion

  tmp = SPI.transfer(0);                   // Read first 8 bits
  tmp = tmp << 5;
  tmp |= 0x1f & (SPI.transfer(0x90) >> 3); // Read last 8 bits and start new XP conversion

  *x = tmp;

  // Start XP sample request for y position, read 4 times and keep last sample
  SPI.transfer(0);                       // Read first 8 bits
  SPI.transfer(0x90);                    // Read last 8 bits and start new XP conversion
  SPI.transfer(0);                       // Read first 8 bits
  SPI.transfer(0x90);                    // Read last 8 bits and start new XP conversion
  SPI.transfer(0);                       // Read first 8 bits
  SPI.transfer(0x90);                    // Read last 8 bits and start new XP conversion
  tmp = SPI.transfer(0);                 // Read first 8 bits
  tmp = tmp << 5;
  tmp |= 0x1f & (SPI.transfer(0) >> 3);  // Read last 8 bits
  *y = tmp;
  digitalWrite(TOUCH_CS, HIGH);

  return true;
}

/***************************************************************************************
** Function name:           getTouchRawZ
** Description:             read raw pressure on touchpad and return Z value.
***************************************************************************************/
uint16_t getTouchRawZ(void) {

  digitalWrite(TOUCH_CS, LOW);
  // Z sample request
  int16_t tz = 0xFFF;
  SPI.transfer(0xb0);               // Start new Z1 conversion
  tz += SPI.transfer16(0xc0) >> 3;  // Read Z1 and start Z2 conversion
  tz -= SPI.transfer16(0x00) >> 3;  // Read Z2
  digitalWrite(TOUCH_CS, HIGH);
  return (uint16_t)tz;
}

/***************************************************************************************
** Function name:           validTouch
** Description:             read validated position. Return false if not pressed.
***************************************************************************************/
#define _RAWERR 20 // Deadband error allowed in successive position samples
uint8_t validTouch(uint16_t *x, uint16_t *y, uint16_t threshold) {
  uint16_t x_tmp, y_tmp, x_tmp2, y_tmp2;

  // Wait until pressure stops increasing to debounce pressure
  uint16_t z1 = 1;
  uint16_t z2 = 0;
  while (z1 > z2)
  {
    z2 = z1;
    z1 = getTouchRawZ();
    if (z1 > z2) {
      //     delay(1);
    }
    //delay(1);
  }

  if (z1 <= threshold) return false;

  getTouchRaw(&x_tmp, &y_tmp);

  delay(1); // Small delay to the next sample
  if (getTouchRawZ() <= threshold) return false;

  delay(2); // Small delay to the next sample
  getTouchRaw(&x_tmp2, &y_tmp2);

  if (abs(x_tmp - x_tmp2) > _RAWERR) return false;
  if (abs(y_tmp - y_tmp2) > _RAWERR) return false;

  *x = x_tmp;
  *y = y_tmp;

  return true;
}

/***************************************************************************************
** Function name:           getTouch
** Description:             read callibrated position. Return false if not pressed.
***************************************************************************************/
#define Z_THRESHOLD 350 // Touch pressure threshold for validating touches
uint8_t getTouch(uint16_t *x, uint16_t *y, uint16_t threshold) {
  uint16_t x_tmp, y_tmp;

  if (threshold < 20) threshold = 20;
  if (_pressTime > millis()) threshold = 20;

  uint8_t n = 5;
  uint8_t valid = 0;
  while (n--)
  {
    if (validTouch(&x_tmp, &y_tmp, threshold)) valid++;;
  }

  if (valid < 1) {
    _pressTime = 0;
    return false;
  }

  _pressTime = millis() + 50;

  convertRawXY(&x_tmp, &y_tmp);

  if (x_tmp >= LCD_W || y_tmp >= LCD_H) return false;

  _pressX = x_tmp;
  _pressY = y_tmp;
  *x = _pressX;
  *y = _pressY;
  return valid;
}

/***************************************************************************************
** Function name:           convertRawXY
** Description:             convert raw touch x,y values to screen coordinates
***************************************************************************************/
void convertRawXY(uint16_t *x, uint16_t *y)
{
  uint16_t x_tmp = *x, y_tmp = *y, xx, yy;

  if (!touchCalibration_rotate) {
    xx = (x_tmp - touchCalibration_x0) * LCD_W / touchCalibration_x1;
    yy = (y_tmp - touchCalibration_y0) * LCD_H / touchCalibration_y1;
    if (touchCalibration_invert_x)
      xx = LCD_W - xx;
    if (touchCalibration_invert_y)
      yy = LCD_H - yy;
  } else {
    xx = (y_tmp - touchCalibration_x0) * LCD_W / touchCalibration_x1;
    yy = (x_tmp - touchCalibration_y0) * LCD_H / touchCalibration_y1;
    if (touchCalibration_invert_x)
      xx = LCD_W - xx;
    if (touchCalibration_invert_y)
      yy = LCD_H - yy;
  }
  *x = xx;
  *y = yy;
}

/***************************************************************************************
** Function name:           calibrateTouch
** Description:             generates calibration parameters for touchscreen.
***************************************************************************************/
void calibrateTouch(uint16_t *parameters, uint8_t size) {
  int16_t values[] = {0, 0, 0, 0, 0, 0, 0, 0};
  uint16_t x_tmp, y_tmp;
  LCD_Clear(WHITE);
  for (uint8_t i = 0; i < 4; i++) {
    POINT_COLOR = RED;
    LCD_DrawFillRectangle(0, 0, size + 1, size + 1);
    LCD_DrawFillRectangle(0, LCD_H - size - 1, size + 1, LCD_H - 1);
    LCD_DrawFillRectangle(LCD_W - size - 1, LCD_H - size - 1, LCD_W - 1, LCD_H - 1);
    LCD_DrawFillRectangle(LCD_W - size - 1, 0, LCD_W - 1, size + 1);

    POINT_COLOR = WHITE;
    if (i == 5) break; // used to clear the arrows

    switch (i) {
      case 0: // up left
        LCD_DrawLine(0, 0, 0, size);
        LCD_DrawLine(0, 0, size, 0);
        LCD_DrawLine(0, 0, size , size);
        break;
      case 1: // bot left
        LCD_DrawLine(0, LCD_H - size - 1, 0, LCD_H - 1);
        LCD_DrawLine(0, LCD_H - 1, size, LCD_H - 1);
        LCD_DrawLine(size, LCD_H - size - 1, 0, LCD_H - 1);
        break;
      case 2: // up right
        LCD_DrawLine(LCD_W - size - 1, 0, LCD_W - 1, 0);
        LCD_DrawLine(LCD_W - size - 1, size, LCD_W - 1, 0);
        LCD_DrawLine(LCD_W - 1, size, LCD_W - 1, 0);
        break;
      case 3: // bot right
        LCD_DrawLine(LCD_W - size - 1, LCD_H - size - 1, LCD_W - 1, LCD_H - 1);
        LCD_DrawLine(LCD_W - 1, LCD_H - 1 - size, LCD_W - 1, LCD_H - 1);
        LCD_DrawLine(LCD_W - 1 - size, LCD_H - 1, LCD_W - 1, LCD_H - 1);
        break;
    }

    // user has to get the chance to release
    if (i > 0) delay(1000);

    for (uint8_t j = 0; j < 8; j++) {
      // Use a lower detect threshold as corners tend to be less sensitive
      while (!validTouch(&x_tmp, &y_tmp, Z_THRESHOLD / 2));
      values[i * 2  ] += x_tmp;
      values[i * 2 + 1] += y_tmp;
    }
    values[i * 2  ] /= 8;
    values[i * 2 + 1] /= 8;
  }

  // from case 0 to case 1, the y value changed.
  // If the measured delta of the touch x axis is bigger than the delta of the y axis, the touch and TFT axes are switched.
  touchCalibration_rotate = false;
  if (abs(values[0] - values[2]) > abs(values[1] - values[3])) {
    touchCalibration_rotate = true;
    touchCalibration_x0 = (values[1] + values[3]) / 2; // calc min x
    touchCalibration_x1 = (values[5] + values[7]) / 2; // calc max x
    touchCalibration_y0 = (values[0] + values[4]) / 2; // calc min y
    touchCalibration_y1 = (values[2] + values[6]) / 2; // calc max y
  } else {
    touchCalibration_x0 = (values[0] + values[2]) / 2; // calc min x
    touchCalibration_x1 = (values[4] + values[6]) / 2; // calc max x
    touchCalibration_y0 = (values[1] + values[5]) / 2; // calc min y
    touchCalibration_y1 = (values[3] + values[7]) / 2; // calc max y
  }

  // in addition, the touch screen axis could be in the opposite direction of the TFT axis
  touchCalibration_invert_x = false;
  if (touchCalibration_x0 > touchCalibration_x1) {
    values[0] = touchCalibration_x0;
    touchCalibration_x0 = touchCalibration_x1;
    touchCalibration_x1 = values[0];
    touchCalibration_invert_x = true;
  }
  touchCalibration_invert_y = false;
  if (touchCalibration_y0 > touchCalibration_y1) {
    values[0] = touchCalibration_y0;
    touchCalibration_y0 = touchCalibration_y1;
    touchCalibration_y1 = values[0];
    touchCalibration_invert_y = true;
  }

  // pre calculate
  touchCalibration_x1 -= touchCalibration_x0;
  touchCalibration_y1 -= touchCalibration_y0;

  if (touchCalibration_x0 == 0) touchCalibration_x0 = 1;
  if (touchCalibration_x1 == 0) touchCalibration_x1 = 1;
  if (touchCalibration_y0 == 0) touchCalibration_y0 = 1;
  if (touchCalibration_y1 == 0) touchCalibration_y1 = 1;

  // export parameters, if pointer valid
  if (parameters != NULL) {
    parameters[0] = touchCalibration_x0;
    parameters[1] = touchCalibration_x1;
    parameters[2] = touchCalibration_y0;
    parameters[3] = touchCalibration_y1;
    parameters[4] = touchCalibration_rotate | (touchCalibration_invert_x << 1) | (touchCalibration_invert_y << 2);

    savetouch(parameters);
  }
}


/***************************************************************************************
** Function name:           setTouch
** Description:             imports calibration parameters for touchscreen.
***************************************************************************************/
void setTouch(uint16_t *parameters) {
  touchCalibration_x0 = parameters[0];
  touchCalibration_x1 = parameters[1];
  touchCalibration_y0 = parameters[2];
  touchCalibration_y1 = parameters[3];

  if (touchCalibration_x0 == 0) touchCalibration_x0 = 1;
  if (touchCalibration_x1 == 0) touchCalibration_x1 = 1;
  if (touchCalibration_y0 == 0) touchCalibration_y0 = 1;
  if (touchCalibration_y1 == 0) touchCalibration_y1 = 1;

  touchCalibration_rotate = parameters[4] & 0x01;
  touchCalibration_invert_x = parameters[4] & 0x02;
  touchCalibration_invert_y = parameters[4] & 0x04;
}

// save touch parameters
void savetouch(uint16_t *parameters) {
  // Declare byte array
  byte inputBytes[10] = { 0 };
  uint16_t i;
  byte address = 0;
  for (byte count = 0; count < 5; count++)
  {
    i = (parameters[count] & 0xFF00) >> 8;
    inputBytes[address++] = (byte)i;
    i = parameters[count] & 0x00FF;
    inputBytes[address++] = (byte)i;
  }
  // Write input array to EEPROM memory.
  eeprom.writeBytes(0, 10, inputBytes);
}

// read touch parameters
void readtouch(uint16_t *parameters) {
  // Declare byte array
  byte outputBytes[10] = { 0 };
  uint16_t i;
  byte address = 0;
  eeprom.readBytes(0, 10, outputBytes);
  for (byte count = 0; count < 5; count++)
  {
    i = (outputBytes[address++] << 8) & 0xFF00;
    i = i | outputBytes[address++];
    parameters[count] = i;
  }
}


void error1(void) {
  while (1) {
    // do something to alert developer

  }
}



// write float
void EEPROM_Write(float * num, word MemPos)
{
  byte ByteArray[4];
  memcpy(ByteArray, num, 4);
  //  for (word x = 0; x < 4; x++)
  // {
  //       Serial.println(ByteArray[x]);
  eeprom.writeBytes(MemPos, 4, ByteArray);
  //   eeprom.writeByte(MemPos + x, ByteArray[x]);
  //  }
}

// read float
void EEPROM_Read(float * num, word MemPos)
{
  byte ByteArray[4];
  eeprom.readBytes(MemPos, 4, ByteArray);
  //  for (word x = 0; x < 4; x++)
  //  {
  //    ByteArray[x] = eeprom.readByte(MemPos + x);
  //  }
  memcpy(num, ByteArray, 4);
}
