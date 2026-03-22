/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ---- UI / logic constants ----
// Hysteresis for OOR: once you're "out", you must come back below OOR_EXIT_CM to re-enter range.
#define OOR_ENTER_CM    60    // distance >= this -> go OOR
#define OOR_EXIT_CM     56    // distance <= this -> back in range

#define OUT_MAX_CM      OOR_ENTER_CM   // leave this name for existing code paths (LCD/buzzer)
#define SLOW_BAND_CM    25    // stop_cm..stop_cm+25 = "SLOW"; else "SAFE"
#define BAR_LEN         12    // meter cells on 2nd line
#define DEBOUNCE_MS     40
#define LONG_PRESS_MS   700   // hold >= this => enable/disable toggle


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// ---- Ultrasonic shared state (EXTI + TIM2) ----
volatile uint32_t t_rise = 0, t_fall = 0;
volatile uint8_t  echo_done = 0;       // set on falling edge
volatile uint32_t us_pulse = 0;        // pulse width in us
volatile uint16_t cm10     = 0;        // distance in 0.1 cm (integer)

// ---- Pot values ----
volatile uint16_t pot_raw = 0;         // 0..4095
volatile uint16_t stop_cm = 60;        // user "stop" setpoint (cm), from pot

// ---- Buzzer pattern (non-blocking) ----
uint32_t beep_next_ms = 0;
uint16_t beep_on_ms   = 60;
uint16_t beep_off_ms  = 300;

// ---- App state ----
volatile uint8_t  system_enabled = 1;   // long-press toggles this
volatile uint8_t  buzzer_muted   = 0;   // short-press toggles this

// ---- Button (PA8) edge tracking ----
volatile uint8_t  btn_down = 0;
volatile uint32_t btn_t0   = 0;

// ---- MCP23008 LCD port mirror (backlight on) ----
static uint8_t mcp_port = (1u<<7);     // LCD_BL

// Line caches to avoid flicker (16 chars + NUL)
static char line0_prev[17] = "                ";
static char line1_prev[17] = "                ";
static uint32_t next_lcd_ms = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
// microsecond delay via DWT
static void delay_us(uint32_t us);

// ultrasonic
static void usonic_timer_start(void);
static uint8_t usonic_single(void);   // fills us_pulse & cm10, returns 1 if ok

// pot
static uint16_t pot_read_raw(void);
static uint16_t pot_to_stop_cm(uint16_t v); // map 0..4095 -> 20..150 cm

// RGB & buzzer
static void rgb_set(uint8_t r_on, uint8_t g_on, uint8_t b_on); // 1=ON
static void buzzer_on(void);
static void buzzer_off(void);
static void buzzer_update(uint32_t now_ms, uint16_t dist_cm);

// UI helpers (only the drawer is prototyped here;
// lcd_write_line_cached() is defined before use in USER CODE 0)
static void lcd_draw_ui(uint16_t dist_cm, uint16_t stop_cm, uint8_t muted, uint8_t enabled);

// LCD (MCP23008 @ 0x20)
static void lcd_i2c_scan(void);
static HAL_StatusTypeDef mcp_write(uint8_t reg, uint8_t val);
static void mcp_apply(void);
static void lcd_pulseE(void);
static void lcd_write4(uint8_t nibble, uint8_t rs);
static void lcd_write8(uint8_t v, uint8_t rs);
static void lcd_cmd(uint8_t c);
static void lcd_data(uint8_t d);
static void lcd_init_4bit(void);
static void lcd_set_cursor(uint8_t col, uint8_t row);
static void lcd_print(const char *s);
static void lcd_print_u16(uint16_t v);
static void lcd_bar16(uint8_t filled);
static void lcd_clear_line(uint8_t row);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// ---- microsecond delay via DWT cycle counter ----
static void delay_us(uint32_t us){
  static uint8_t ready = 0;
  if(!ready){
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
    ready = 1;
  }
  const uint32_t cycles_per_us = 32000000u/1000000u; // SYSCLK ~32 MHz
  uint32_t start = DWT->CYCCNT, target = start + us*cycles_per_us;
  while((int32_t)(DWT->CYCCNT - target) < 0) { __NOP(); }
}

// ---- Ultrasonic helpers ----
static void usonic_timer_start(void){
  // TIM2 is Base @ 1 MHz (PSC=31, ARR=65535)
  __HAL_TIM_SET_PRESCALER(&htim2, 31);
  __HAL_TIM_SET_AUTORELOAD(&htim2, 65535);
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  HAL_TIM_Base_Start(&htim2);
  // TRIG idle low
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
}

// one shot: returns 1 if echo captured, updates us_pulse and cm10
static uint8_t usonic_single(void){
  echo_done = 0;
  // 10–12 us TRIG
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
  delay_us(12);
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

  // wait up to 40 ms
  uint32_t t0 = HAL_GetTick();
  while(!echo_done){
    if (HAL_GetTick() - t0 > 40) return 0;
  }
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim2) + 1;
  us_pulse = (t_fall >= t_rise) ? (t_fall - t_rise) : (t_fall + arr - t_rise);
  // distance in 0.1 cm: (us / 58) * 10 = us * 10 / 58
  cm10 = (uint16_t)((us_pulse * 10u) / 58u);
  return 1;
}

// ---- Potentiometer ----
static uint16_t pot_read_raw(void){
  HAL_ADC_Start(&hadc);
  HAL_ADC_PollForConversion(&hadc, 5);
  uint16_t v = (uint16_t)HAL_ADC_GetValue(&hadc);
  HAL_ADC_Stop(&hadc);
  return v;
}
// map 0..4095 -> 20..150 cm (linear)
static uint16_t pot_to_stop_cm(uint16_t v){
  if (v > 4095) v = 4095;
  return (uint16_t)(20u + ((150u - 20u) * (uint32_t)v)/4095u);
}

// ---- RGB (common-anode: LOW=ON) ----
static void rgb_set(uint8_t r_on, uint8_t g_on, uint8_t b_on){
  // drive low to turn ON
  HAL_GPIO_WritePin(RGB_LED1_GPIO_Port, RGB_LED1_Pin, r_on ? GPIO_PIN_RESET : GPIO_PIN_SET);
  HAL_GPIO_WritePin(RGB_LED2_GPIO_Port, RGB_LED2_Pin, g_on ? GPIO_PIN_RESET : GPIO_PIN_SET);
  HAL_GPIO_WritePin(RGB_LED3_GPIO_Port, RGB_LED3_Pin, b_on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

// ---- Buzzer @ 2 kHz (TIM3 CH1; CCR=50% when ON) ----
static void buzzer_on(void){
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (__HAL_TIM_GET_AUTORELOAD(&htim3)+1)/2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}
static void buzzer_off(void){
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
}
// --- small median-of-3 helper (robust to one outlier) ---
static uint16_t median3(uint16_t a, uint16_t b, uint16_t c)
{
  if (a > b) { uint16_t t=a; a=b; b=t; }
  if (b > c) { uint16_t t=b; b=c; c=t; }
  if (a > b) { uint16_t t=a; a=b; b=t; }
  return b; // now b is median
}

// ---- UI + LED helpers ------------------------------------------------------

// Color enum and LED setter (common-anode: LOW = ON)
typedef enum { LED_OFF=0, LED_GREEN, LED_YELLOW, LED_RED } LEDColor;

static void set_led_color(LEDColor c)
{
  switch (c) {
    case LED_OFF:    rgb_set(0,0,0); break;
    case LED_GREEN:  rgb_set(0,1,0); break;
    case LED_YELLOW: rgb_set(1,1,0); break; // red+green
    case LED_RED:    rgb_set(1,0,0); break;
  }
}

// Write a full 16-char line only if contents changed (reduces flicker)
static void lcd_write_line_cached(uint8_t row, const char *line16, char prev16[17])
{
  // compare
  if (memcmp(prev16, line16, 16) == 0) return;

  // write
  lcd_set_cursor(0, row);
  for (uint8_t i = 0; i < 16; ++i) {
    lcd_data((uint8_t)line16[i]);
  }

  // cache
  memcpy(prev16, line16, 16);
  prev16[16] = '\0';
}

// Line0: "D:XXcm S:YY " + 'M' at col 15
// Line1: [BAR] + "SAFE/SLOW/STOP" or "OOR "
static void lcd_draw_ui(uint16_t dist_cm, uint16_t stop_cm, uint8_t muted, uint8_t enabled)
{
  extern char line0_prev[17];
  extern char line1_prev[17];
  extern uint32_t next_lcd_ms;

  uint32_t now = HAL_GetTick();
  if ((int32_t)(now - next_lcd_ms) < 0) return;  // rate-limit
  next_lcd_ms = now + 50;                         // ~20 Hz max

  char l0[17]; char l1[17];
  memset(l0, ' ', 16); l0[16] = '\0';
  memset(l1, ' ', 16); l1[16] = '\0';

  if (!enabled) {
    lcd_write_line_cached(0, l0, line0_prev);
    lcd_write_line_cached(1, l1, line1_prev);
    return;
  }

  // ----- Line 0 -----
  const uint8_t invalid = (dist_cm == 999 || dist_cm > OUT_MAX_CM);
  char dstr[4] = "---";
  if (!invalid) snprintf(dstr, sizeof dstr, "%3u", dist_cm);
  char sstr[4]; snprintf(sstr, sizeof sstr, "%-3u", stop_cm);

  l0[0]='D'; l0[1]=':'; memcpy(&l0[2], dstr, 3);
  l0[5]='c'; l0[6]='m'; l0[7]=' ';
  l0[8]='S'; l0[9]=':'; memcpy(&l0[10], sstr, 3);
  l0[15] = muted ? 'M' : ' ';

  // ----- Line 1 -----
  if (invalid) {
    memcpy(&l1[12], "OOR ", 4);
  } else {
    int16_t remain = (int16_t)stop_cm - (int16_t)dist_cm; // >0 when closer than setpoint
    uint8_t fill = (remain <= 0) ? 0 :
                   (remain >= (BAR_LEN*2) ? BAR_LEN : (uint8_t)(remain/2));
    for (uint8_t i=0;i<BAR_LEN && i<16;i++) l1[i] = (i<fill) ? (char)0xFF : ' ';

    const char* status =
      (dist_cm <= stop_cm) ? "STOP" :
      (dist_cm < (uint16_t)(stop_cm + SLOW_BAND_CM)) ? "SLOW" : "SAFE";
    memcpy(&l1[12], status, 4); // fits at columns 12..15
  }

  // push if changed
  lcd_write_line_cached(0, l0, line0_prev);
  lcd_write_line_cached(1, l1, line1_prev);
}


// Final cadence: SAFE = very infrequent, SLOW = ~1 Hz, STOP = continuous tone
static void buzzer_update(uint32_t now_ms, uint16_t dist_cm)
{
  // Global guards
  if (!system_enabled || buzzer_muted || dist_cm == 999 || dist_cm > OUT_MAX_CM) {
    buzzer_off();
    // don't reset state; when re-enabled we’ll re-sync on zone change
    return;
  }

  // Zone selection
  enum { Z_SAFE=0, Z_SLOW, Z_STOP } zone;
  if (dist_cm <= stop_cm) {
    zone = Z_STOP;                 // RED
  } else if (dist_cm < (uint16_t)(stop_cm + SLOW_BAND_CM)) {
    zone = Z_SLOW;                 // YELLOW
  } else {
    zone = Z_SAFE;                 // GREEN
  }

  // Persistent state
  static uint8_t  phase_on = 0;    // 0 = OFF, 1 = ON (for chirps)
  static uint32_t t_next   = 0;    // next toggle timestamp
  static uint8_t  last_zone = 0xFF;

  // STOP => continuous tone
  if (zone == Z_STOP) {
    if (last_zone != Z_STOP) {
      // entering STOP: turn solid ON
      buzzer_on();
      phase_on  = 1;
      // t_next unused while in STOP
    }
    last_zone = Z_STOP;
    return; // keep buzzing continuously
  }

  // If we just left STOP, ensure we start from OFF to make chirps clear
  if (last_zone == Z_STOP && zone != Z_STOP) {
    buzzer_off();
    phase_on = 0;
    t_next   = now_ms; // ready to start new cadence
  }
  last_zone = zone;

  // Per-zone ON/OFF timings (ms)
  // SAFE: ~0.33 Hz => 60 ms ON, 2940 ms OFF (very infrequent)
  // SLOW: ~1.0  Hz => 120 ms ON, 880  ms OFF (clearly more frequent)
  uint16_t on_ms, off_ms;
  if (zone == Z_SLOW) {
    on_ms = 120; off_ms = 880;
  } else { // Z_SAFE
    on_ms = 60;  off_ms = 2940;
  }

  // Initialize first chirp promptly when entering a chirp zone
  static uint8_t initialized = 0;
  if (!initialized) {
    initialized = 1;
    phase_on = 1;
    buzzer_on();
    t_next = now_ms + on_ms;
  }

  // Toggle by schedule
  if ((int32_t)(now_ms - t_next) >= 0) {
    if (phase_on) {
      // go OFF for the rest period
      buzzer_off();
      phase_on = 0;
      t_next = now_ms + off_ms;
    } else {
      // start a chirp
      buzzer_on();
      phase_on = 1;
      t_next = now_ms + on_ms;
    }
  }
}


// ---- LCD (MCP23008 @ 0x20) ----
#define LCD_RS   (1u<<1)
#define LCD_E    (1u<<2)
#define LCD_D4   (1u<<3)
#define LCD_D5   (1u<<4)
#define LCD_D6   (1u<<5)
#define LCD_D7   (1u<<6)
#define LCD_BL   (1u<<7)
#define MCP_ADDR (0x20u<<1)
#define MCP_IODIR 0x00
#define MCP_GPIO  0x09

static void lcd_i2c_scan(void){
  volatile uint8_t seen = 0;
  for(uint8_t a=0x03;a<0x78;a++){
    if(HAL_I2C_IsDeviceReady(&hi2c1, a<<1, 1, 5)==HAL_OK){
      if (a==0x20) seen=1;
    }
  }
  (void)seen; // set a breakpoint to verify seen==1
}
static HAL_StatusTypeDef mcp_write(uint8_t reg, uint8_t val){
  uint8_t buf[2] = {reg, val};
  return HAL_I2C_Master_Transmit(&hi2c1, MCP_ADDR, buf, 2, 50);
}
static void mcp_apply(void){ (void)mcp_write(MCP_GPIO, mcp_port); }
static void lcd_pulseE(void){
  mcp_port |= LCD_E;  mcp_apply();
  for(volatile int i=0;i<160;i++) __NOP();
  mcp_port &= ~LCD_E; mcp_apply();
}
static void lcd_write4(uint8_t nibble, uint8_t rs){
  mcp_port &= ~(LCD_D4|LCD_D5|LCD_D6|LCD_D7|LCD_RS);
  if (rs) mcp_port |= LCD_RS;
  if (nibble & 0x01) mcp_port |= LCD_D4;
  if (nibble & 0x02) mcp_port |= LCD_D5;
  if (nibble & 0x04) mcp_port |= LCD_D6;
  if (nibble & 0x08) mcp_port |= LCD_D7;
  mcp_apply();
  lcd_pulseE();
}
static void lcd_write8(uint8_t v, uint8_t rs){
  lcd_write4((v>>4)&0x0F, rs);
  lcd_write4( v    &0x0F, rs);
}
static void lcd_cmd(uint8_t c){ lcd_write8(c, 0); HAL_Delay(2); }
static void lcd_data(uint8_t d){ lcd_write8(d, 1); }
static void lcd_init_4bit(void){
  (void)mcp_write(MCP_IODIR, 0x00);
  mcp_port = LCD_BL; mcp_apply();
  HAL_Delay(50);
  lcd_write4(0x03,0); HAL_Delay(5);
  lcd_write4(0x03,0); HAL_Delay(5);
  lcd_write4(0x03,0); HAL_Delay(1);
  lcd_write4(0x02,0); HAL_Delay(1);  // 4-bit
  lcd_cmd(0x28); // 4-bit, 2-line, 5x8
  lcd_cmd(0x08); // display off
  lcd_cmd(0x01); HAL_Delay(2);
  lcd_cmd(0x06); // entry mode
  lcd_cmd(0x0C); // display on
}
static void lcd_set_cursor(uint8_t col, uint8_t row){
  static const uint8_t rowaddr[] = {0x00, 0x40, 0x14, 0x54};
  if (row>1) row=1;
  lcd_cmd(0x80 | (rowaddr[row] + col));
}
static void lcd_print(const char *s){ while(*s) lcd_data((uint8_t)*s++); }
static void lcd_print_u16(uint16_t v){
  char buf[6]; // up to 65535
  int i=5; buf[i--]='\0';
  if(v==0){ lcd_data('0'); return; }
  while(v && i>=0){ buf[i--] = '0'+(v%10); v/=10; }
  lcd_print(&buf[i+1]);
}
// simple 0..BAR_LEN bar on row 2
static void lcd_bar16(uint8_t filled){
  if (filled>BAR_LEN) filled=BAR_LEN;
  for(uint8_t i=0;i<BAR_LEN;i++) lcd_data(i<filled ? 0xFF : ' ');
}

static void lcd_clear_line(uint8_t row){
  lcd_set_cursor(0,row);
  lcd_print("                "); // 16 spaces
  lcd_set_cursor(0,row);
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  // RGB off until first valid reading (common-anode => pass zeros)
  rgb_set(0,0,0);

  // I2C presence (optional breakpoint inside)
  lcd_i2c_scan();

  // LCD hello
  lcd_init_4bit();
  lcd_set_cursor(0,0); lcd_print("Parking Assist ");
  lcd_set_cursor(0,1); lcd_print("Init...        ");

  // Ultrasonic timing base + TRIG low
  usonic_timer_start();

  // Pot seed
  pot_raw = pot_read_raw();
  stop_cm = pot_to_stop_cm(pot_raw);

  // Buzzer idle, quick self-test 300 ms
  buzzer_off();
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (__HAL_TIM_GET_AUTORELOAD(&htim3)+1)/2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_Delay(300);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);

  // Initial UI paint
  lcd_draw_ui(999, stop_cm, buzzer_muted, system_enabled);
  set_led_color(LED_OFF);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // read pot -> stop_cm
    pot_raw = pot_read_raw();
    stop_cm = pot_to_stop_cm(pot_raw);

    // --- ultrasonic read ---
    uint16_t dist_sample = 999;
    if (usonic_single()) {
      dist_sample = (uint16_t)(cm10/10u);
    }

    // --- 3-sample median filter + OOR hysteresis ---
    static uint16_t d0 = 999, d1 = 999, d2 = 999;   // history
    static uint8_t  oor = 1;                        // start OOR until we see close
    // shift in latest sample (if invalid keep previous values as-is)
    d2 = d1; d1 = d0; d0 = dist_sample;
    uint16_t dist_med = median3(d0, d1, d2);

    // Hysteresis: only flip states when we truly cross the spaced thresholds
    if (oor) {
      if (dist_med <= OOR_EXIT_CM) oor = 0;            // back in range
    } else {
      if (dist_med >= OOR_ENTER_CM) oor = 1;           // go OOR
    }

    // Use 999 to signal "invalid/OOR" to the rest of the UI/buzzer logic
    uint16_t dist_logic = oor ? 999 : dist_med;

    // ----- LED selection -----
    if (!system_enabled || dist_logic == 999) {
      set_led_color(LED_OFF);
    } else if (dist_logic <= stop_cm) {
      set_led_color(LED_RED);
    } else if (dist_logic < (uint16_t)(stop_cm + SLOW_BAND_CM)) {
      set_led_color(LED_YELLOW);
    } else {
      set_led_color(LED_GREEN);
    }

    // ----- Buzzer cadence (handles mute/system/OOR internally) -----
    buzzer_update(HAL_GetTick(), dist_logic);

    // ----- LCD: top line & bar + status (no clears; cached) -----
    lcd_draw_ui(dist_logic, stop_cm, buzzer_muted, system_enabled);

    HAL_Delay(5); // give time to SysTick/EXTI/I2C
  }

  /* USER CODE END 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_96CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 31;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 31;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 499;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 200;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RGB_LED1_Pin|RGB_LED2_Pin|RGB_LED3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ECHO_DIV_Pin */
  GPIO_InitStruct.Pin = ECHO_DIV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_DIV_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_IN_Pin */
  GPIO_InitStruct.Pin = BTN_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIG_Pin */
  GPIO_InitStruct.Pin = TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(TRIG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RGB_LED1_Pin RGB_LED2_Pin RGB_LED3_Pin */
  GPIO_InitStruct.Pin = RGB_LED1_Pin|RGB_LED2_Pin|RGB_LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
  // Button on PA8
  if (pin == BTN_IN_Pin) {
    uint32_t now = HAL_GetTick();

    // Read current level to know edge (pull-up idle => HIGH; press => LOW)
    GPIO_PinState level = HAL_GPIO_ReadPin(BTN_IN_GPIO_Port, BTN_IN_Pin);

    if (level == GPIO_PIN_RESET) { // FALLING = press
      if (!btn_down && (now - btn_t0) > DEBOUNCE_MS) {
        btn_down = 1;
        btn_t0   = now;
      }
    } else { // RISING = release
      if (btn_down) {
        btn_down = 0;
        uint32_t dt = now - btn_t0;

        if (dt >= LONG_PRESS_MS) {
          // ---- LONG PRESS: toggle system enabled ----
          system_enabled ^= 1;
          if (!system_enabled) {
            // turn everything off/blank when disabling
            set_led_color(LED_OFF);
            buzzer_off();
            lcd_set_cursor(0,0); lcd_print("                ");
            lcd_set_cursor(0,1); lcd_print("                ");
          } else {
            // redraw when enabling
            lcd_draw_ui(999, stop_cm, buzzer_muted, system_enabled);
          }
        } else if (dt >= DEBOUNCE_MS) {
          // ---- SHORT PRESS: toggle mute ----
          buzzer_muted ^= 1;
          // update 'M' flag immediately
          lcd_set_cursor(15,0); lcd_print(buzzer_muted ? "M" : " ");
        }
      }
    }
    return; // done with button
  }

  // Ultrasonic echo (PA0 via EXTI0)
  if (pin == ECHO_DIV_Pin){
    if (HAL_GPIO_ReadPin(ECHO_DIV_GPIO_Port, ECHO_DIV_Pin) == GPIO_PIN_SET){
      t_rise = __HAL_TIM_GET_COUNTER(&htim2);
      echo_done = 0;
    } else {
      t_fall = __HAL_TIM_GET_COUNTER(&htim2);
      echo_done = 1;
    }
  }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
