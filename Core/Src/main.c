/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stm32f407xx.h>
#include <string.h>
#include "nlms_square_protect.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//^ ADC 采样频率 50KHz 由 TIM3 触发，ADC 缓存区 128*2(uint32_t)
//^ ADC 使用 Dual Regular Simultaneous Mode Only 模式
//^ 实现 ADC1 和 ADC2 同步触发采样
//^ DAC 输出频率 200KHz 由 TIM2 触发，DAC 缓存区 1024(uint16_t)
//^ 为了 ADC 与 DAC 同步，需要对写入的DAC数据进行插值
//^ TIM2 与 TIM3 的主从触发关系，保证 ADC 与 DAC 时钟同步

#define ADC_BUFFER_SIZE 256
#define ADC_PAIR_COUNT 128
#define DAC_INTERP_FACTOR 4
#define DAC_BUFFER_SIZE (ADC_PAIR_COUNT * DAC_INTERP_FACTOR * 2)

#define NLMS_TAPS 96      //^ 滤波器 抽头数
#define NLMS_MU (0.0003f) //^ 滤波器 步长
#define NLMS_GAIN 1.98f
#define NLMS_EPS (1e-6f) //^ 滤波器 最小

#define NLMS_SQUARE_PROTECT_ON 1
#define NLMS_SQUARE_PROTECT_MU (0.01f)
#define NLMS_SQUARE_PROTECT_FREQ_UPDATE_TH_HZ (1.0f)

#define HIGH_PASS_FILTER_ON 0 // 1: DAC 输出前经过高通滤波
#define NLMS_CUSTOM_INIT_ON 1 // 1: 使用 nlms_initial_w 初始化权重；0: 权重全 0
#define NLMS_INITIAL_W_SET 2 // 1: 使用 nlms_initial_w；2: 使用 nlms_initial_w_2
#define NLMS_FREEZE_W_ON 0   // 1: 固定 NLMS 权重，不执行自适应更新
#define DIRECT_MIX_MINUS_NOISE_ON 0 // 1: 跳过 NLMS，直接输出 混合波-噪声
#define NLMS_DESPIKE_ON 0           // 1: 启用插值前单点尖峰修复
#define NLMS_DESPIKE_TH 80.0f       // 尖峰阈值，单位为中心化 ADC 码值

#define NLMS_FREQ_SAMPLE_RATE 50000.0f // NLMS e 的采样率，单位 Hz
#define NLMS_FREQ_WINDOW_SIZE 2048     // 频率计统计窗口长度
#define NLMS_FREQ_HYST_TH 50.0f        // 参考方波测频上升沿零交叉迟滞阈值
#define NLMS_FREQ_MIN_RMS 80.0f        // RMS 低于该值时不更新频率
#define NLMS_FREQ_MIN_HZ 95.0f         // 有效频率下限
#define NLMS_FREQ_MAX_HZ 5010.0f       // 有效频率上限
#define NLMS_FREQ_LP_B0 0.0697341272102174f
#define NLMS_FREQ_LP_B1 0.139468254420435f
#define NLMS_FREQ_LP_B2 0.0697341272102174f
#define NLMS_FREQ_LP_A1 -1.12675955771016f
#define NLMS_FREQ_LP_A2 0.405696066551028f
#define NLMS_FREQ_AVG_SHIFT_HIGH 4U
#define NLMS_FREQ_AVG_SHIFT_LOW 9U
#define NLMS_FREQ_PREDICT_SIZE 1024U
#define NLMS_FREQ_PREDICT_STABLE_COUNT 2U
#define NLMS_FREQ_FALLBACK_STABLE_COUNT 14U
#define NLMS_FREQ_PREDICT_TH_HZ 800.0f
#define NLMS_FREQ_ASYNC_CHUNK_SIZE 64U
#define DEBUG_NLMS_FREQ_HISTORY_SIZE 256U // 记录每次测得的 NLMS 频率

#define NLMS_REF_NOTCH_ON 1                  // 1: 对 NLMS 参考输入启用动态陷波
#define NLMS_REF_NOTCH_SAMPLE_RATE 50000.0f  // 参考输入陷波器采样率，单位 Hz
#define NLMS_REF_NOTCH_MIN_FREQ 100.0f       // notch 有效频率下限
#define NLMS_REF_NOTCH_MAX_FREQ 5000.0f      // notch 有效频率上限
#define NLMS_REF_NOTCH_R 0.990f              // notch 极点半径，越接近 1 越窄
#define NLMS_REF_NOTCH_UPDATE_ALPHA 0.02f    // 锁定频率平滑更新系数
#define NLMS_REF_NOTCH_FREQ_CHANGE_TH 20.0f  // 频率变化超过该值才更新
#define NLMS_REF_NOTCH_MIN_CONFIDENCE 0.5f   // 频率计置信度下限
#define NLMS_REF_NOTCH_BYPASS_LOW_FREQ_ON 1  // 1: 低频时旁路参考输入陷波
#define NLMS_REF_NOTCH_BYPASS_FREQ_HZ 250.0f // 低于该频率不进行陷波
#define NLMS_FREQ_JUMP_RESET_ON 0     // 1: 频率突变时重置 NLMS/notch 状态
#define NLMS_FREQ_JUMP_RESET_TH 60.0f // 频率突变重置阈值，单位 Hz

#define REF_SQUARE_DETECT_WINDOW_SIZE 256  // 参考噪声方波检测窗口长度
#define REF_SQUARE_DETECT_MIN_PP 160.0f    // 峰峰值低于该值时判定为非方波
#define REF_SQUARE_EDGE_TH_RATIO 0.35f     // 超过该比例峰峰值的变化视为跳变边沿
#define REF_SQUARE_EDGE_COUNT_MIN 1U       // 每窗口至少需要的跳变边沿数
#define REF_SQUARE_PLATEAU_RATIO_MIN 0.65f // 样本多数停在高/低平台才像方波

#define ADC_MIX_MID (2048.0f)   //^ 混合波 目标抬升电压，看示波器的平均值
#define ADC_NOISE_MID (2048.0f) //^ 噪声 目标抬升电压，看示波器的平均值
#define DAC_MAX (4095U)

#define NLMS_RESET_LED_HOLD_MS 500U
#define NLMS_RESET_LED_PORT GPIOC
#define NLMS_RESET_LED_PIN GPIO_PIN_13
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
typedef struct {
  float prev_e;
  uint8_t prev_valid;

  uint8_t armed_positive;
  uint8_t armed_negative;

  uint32_t sample_count;
  uint32_t zero_cross_count;

  float sum2;
  float min_e;
  float max_e;
  float last_freq_hz;
  float confidence;
} nlms_freq_meter_t;

typedef enum {
  NLMS_FREQ_ASYNC_IDLE = 0,
  NLMS_FREQ_ASYNC_FORWARD,
  NLMS_FREQ_ASYNC_REVERSE_1,
  NLMS_FREQ_ASYNC_BACKWARD,
  NLMS_FREQ_ASYNC_REVERSE_2,
  NLMS_FREQ_ASYNC_BINARY_HIGH,
  NLMS_FREQ_ASYNC_ESTIMATE_PREDICT,
  NLMS_FREQ_ASYNC_BINARY_FINAL,
  NLMS_FREQ_ASYNC_ESTIMATE_FINAL,
} nlms_freq_async_phase_t;

typedef struct {
  nlms_freq_async_phase_t phase;
  const float *input;
  uint32_t sample_count;
  uint32_t index;

  float x1;
  float x2;
  float y1;
  float y2;

  float avg1;
  float avg2;
  float avg3;
  float avg_divisor;

  uint8_t current_state_high;
  uint8_t candidate_state_high;
  uint32_t candidate_count;
  uint32_t stable_sample_count;
  uint32_t rising_count;
  uint32_t first_edge;
  uint32_t last_edge;

  float rms;
  float min_e;
  float max_e;
  float predict_freq_hz;
} nlms_freq_async_t;

typedef struct {
  float b0;
  float b1;
  float b2;
  float a1;
  float a2;

  float x1;
  float x2;
  float y1;
  float y2;

  float locked_freq_hz;
  uint8_t freq_valid;
} dynamic_notch_t;

typedef struct {
  float prev_x;
  uint8_t prev_valid;

  uint32_t sample_count;
  uint32_t edge_count;
  uint32_t plateau_count;
  uint32_t high_plateau_count;
  uint32_t low_plateau_count;
  float high_plateau_sum;
  float low_plateau_sum;
  float min_x;
  float max_x;
  float prev_min_x;
  float prev_max_x;
} ref_square_detector_t;

typedef struct {
  float w[NLMS_TAPS];
  float x_hist[NLMS_TAPS * 2];
  uint8_t idx;

  float mu;
  float gain;

  float prev_out_f;
  uint8_t prev_out_valid;

  float despike_hist[5];
  uint8_t despike_valid_count;

  nlms_freq_meter_t freq_meter;
  nlms_freq_meter_t ref_square_freq_meter;
  uint8_t ref_square_freq_meter_active;
  dynamic_notch_t ref_notch;
  ref_square_detector_t ref_square_detector;

  nlms_update_notch_t square_update_notch;
  nlms_square_protect_config_t square_protect_cfg;
  uint8_t square_protect_enabled;
  float square_protect_mu_eff;
  float square_protect_e_update;
} nlms32_t;

typedef struct {
  float b0;
  float b1;
  float b2;
  float a1;
  float a2;

  float x1;
  float x2;
  float y1;
  float y2;
} biquad_hp_t;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t adcRAwValues[ADC_BUFFER_SIZE * 2];
uint16_t dacOutputValues[DAC_BUFFER_SIZE];
volatile uint16_t DMA_FULL_SIGN = 0;
volatile uint16_t DMA_HALF_SIGN = 0;

volatile uint32_t tim5_last_half = 0;
volatile uint32_t tim5_last_full = 0;

volatile uint32_t tim5_delta_half = 0;
volatile uint32_t tim5_delta_full = 0;

volatile float adc_fs_half = 0.0f;
volatile float adc_fs_full = 0.0f;

volatile uint32_t nlms_time_us_half = 0;
volatile uint32_t nlms_time_us_full = 0;

volatile float debug_nlms_freq_hz = 0.0f;
volatile float debug_nlms_freq_confidence = 0.0f;
volatile uint8_t debug_nlms_freq_updated = 0;
volatile uint32_t debug_nlms_zero_cross_count = 0;
volatile float debug_nlms_e_rms = 0.0f;
volatile float debug_nlms_e_pp = 0.0f;
volatile uint32_t debug_nlms_freq_async_drop_count = 0;
volatile uint32_t debug_nlms_freq_async_complete_count = 0;
volatile uint8_t debug_nlms_freq_async_phase = 0;
volatile float debug_nlms_freq_history[DEBUG_NLMS_FREQ_HISTORY_SIZE];
volatile uint32_t debug_nlms_freq_history_index = 0;
volatile uint32_t debug_nlms_freq_history_count = 0;
volatile uint8_t debug_nlms_freq_history_full = 0;

volatile float debug_ref_notch_locked_freq_hz = 0.0f;
volatile uint8_t debug_ref_notch_valid = 0;
volatile float debug_ref_notch_last_in = 0.0f;
volatile float debug_ref_notch_last_out = 0.0f;
volatile uint8_t ref_noise_is_square = 0;
volatile float debug_ref_square_pp = 0.0f;
volatile float debug_ref_square_amplitude = 0.0f;
volatile uint32_t debug_ref_square_edge_count = 0;
volatile float debug_ref_square_plateau_ratio = 0.0f;
volatile float debug_ref_square_freq_hz = 0.0f;
volatile float square_nlms_freq_ratio = 0.0f;
volatile float debug_ref_square_freq_confidence = 0.0f;
volatile uint32_t debug_ref_square_zero_cross_count = 0;
volatile float debug_ref_square_rms = 0.0f;

volatile uint8_t debug_square_protect_enabled = 0;
volatile float debug_square_protect_mu_eff = 0.0f;
volatile float debug_square_protect_notch_freq_hz = 0.0f;
volatile float debug_square_protect_e_update = 0.0f;

volatile uint32_t nlms_reset_led_until_tick = 0;

biquad_hp_t hp_filter;

static float nlms_freq_window_buffers[2][NLMS_FREQ_WINDOW_SIZE];
static uint8_t nlms_freq_capture_buffer_index = 0;
static float nlms_freq_forward_buffer[NLMS_FREQ_WINDOW_SIZE];
static float nlms_freq_reverse_buffer[NLMS_FREQ_WINDOW_SIZE];
static uint8_t nlms_freq_binary_buffer[NLMS_FREQ_WINDOW_SIZE];
static nlms_freq_async_t nlms_freq_async;

static const float nlms_initial_w[NLMS_TAPS] = {
    0.55393779f,  0.05828273f,  0.00947517f,  0.02123324f,  0.02595950f,
    0.02512634f,  0.01482946f,  0.01859419f,  0.02164500f,  0.01328076f,
    0.00589223f,  0.00454398f,  -0.00055090f, -0.00355160f, -0.00596500f,
    -0.01071970f, -0.00854940f, -0.01172730f, -0.01025810f, -0.00368720f,
    -0.00548850f, -0.01357660f, -0.01590370f, -0.01937480f, -0.01537280f,
    -0.01438860f, -0.01659710f, -0.01232130f, -0.01207550f, -0.01303160f,
    -0.00795730f, -0.00869580f, -0.00083830f, -0.00872900f, -0.00831830f,
    -0.00254990f, -0.00006860f, -0.00354860f, -0.00049800f, -0.00392100f,
    -0.00126310f, 0.00219566f,  0.00776127f,  0.01247297f,  0.01972256f,
    0.01852969f,  0.02206242f,  0.03216345f,  0.02739611f,  0.02190173f,
    0.01558967f,  0.00816376f,  0.01403034f,  0.01517150f,  -0.00511280f,
    -0.00408490f, 0.00387149f,  -0.00158420f, -0.00311990f, 0.00480921f,
    -0.00336530f, -0.00593300f, -0.00551580f, -0.00852790f, -0.00393320f,
    -0.00571260f, -0.00478450f, -0.01144030f, -0.01700700f, -0.01155420f,
    -0.01256050f, -0.02012080f, -0.01462420f, -0.01228270f, -0.00848400f,
    -0.00756340f, -0.00423900f, -0.01135800f, -0.01206830f, -0.01063920f,
    -0.00933140f, -0.01045170f, -0.00353620f, -0.00649520f, -0.00359620f,
    -0.00189810f, 0.00868736f,  0.00626283f,  0.00880507f,  0.01333197f,
    0.01328707f,  0.01103873f,  0.02362979f,  0.02223933f,  0.02087310f,
    0.03760396f,
};

static const float nlms_initial_w_2[NLMS_TAPS] = {
    0.50588584f,  -0.00007370f, -0.00231630f, -0.00237890f, -0.00274020f,
    -0.00294380f, -0.00316070f, -0.00365920f, -0.00319390f, -0.00196140f,
    -0.00143670f, -0.00132760f, -0.00132880f, -0.00091140f, -0.00071150f,
    -0.00126180f, -0.00100440f, -0.00031120f, 0.00004780f,  0.00044158f,
    -0.00000691f, 0.00067347f,  0.00192990f,  0.00179262f,  0.00232234f,
    0.00244144f,  0.00316485f,  0.00333345f,  0.00338150f,  0.00323981f,
    0.00354344f,  0.00335923f,  0.00340161f,  0.00246785f,  0.00201835f,
    0.00149749f,  0.00158954f,  0.00114187f,  0.00093396f,  0.00086173f,
    0.00062118f,  0.00066478f,  -0.00031550f, -0.00032140f, -0.00061320f,
    -0.00091700f, 0.00076454f,  0.00089096f,  0.00082928f,  0.00168426f,
    0.00137873f,  0.00173632f,  0.00172992f,  0.00223290f,  0.00241716f,
    0.00230509f,  0.00293143f,  0.00238955f,  0.00300679f,  0.00359567f,
    0.00393515f,  0.00317107f,  0.00295882f,  0.00207902f,  0.00094402f,
    -0.00016500f, -0.00049300f, -0.00146580f, -0.00162620f, -0.00222820f,
    -0.00245050f, -0.00243730f, -0.00303720f, -0.00353640f, -0.00340710f,
    -0.00358830f, -0.00293820f, -0.00311060f, -0.00281000f, -0.00210160f,
    -0.00179770f, -0.00110250f, -0.00015170f, 0.00067876f,  0.00142154f,
    0.00075833f,  0.00060854f,  0.00132362f,  0.00197883f,  0.00188946f,
    0.00206669f,  0.00276133f,  0.00387775f,  0.00470601f,  0.00544337f,
    0.00981311f,
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void ClearDACOutputValues();
uint16_t clamp_to_dac_u16(float v);
void nlms32_init(nlms32_t *s);
void nlms32_process_block_u16(nlms32_t *s,
                              const uint16_t *restrict adc_interleaved,
                              uint16_t *restrict out, uint32_t pair_count);
void nlms32_reset(nlms32_t *s);

void biquad_hp_init(biquad_hp_t *f);
float biquad_hp_process(biquad_hp_t *f, float x0);
void nlms_freq_meter_init(nlms_freq_meter_t *m);
void nlms_freq_meter_process(nlms_freq_meter_t *m, float e_raw, nlms32_t *s);
static void nlms_freq_async_init(void);
static uint8_t nlms_freq_async_start(const float *input, uint32_t sample_count,
                                     float rms, float min_e, float max_e);
static void nlms_freq_async_process_slice(void);
void ref_square_freq_meter_init(nlms_freq_meter_t *m);
void ref_square_freq_meter_process(nlms_freq_meter_t *m, float x_raw);
void dynamic_notch_init(dynamic_notch_t *f);
void dynamic_notch_set_freq(dynamic_notch_t *f, float freq_hz);
void dynamic_notch_update_freq(dynamic_notch_t *f, float measured_freq_hz,
                               float confidence);
float dynamic_notch_process(dynamic_notch_t *f, float x);
void ref_square_detector_init(ref_square_detector_t *d);
void ref_square_detector_process(ref_square_detector_t *d, float x_raw);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_ADC2_Init();
  MX_DAC_Init();
  MX_TIM5_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  ClearDACOutputValues();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  nlms32_t nlms;
  nlms32_init(&nlms);

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);

  biquad_hp_init(&hp_filter);

  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t *)dacOutputValues,
                    DAC_BUFFER_SIZE, DAC_ALIGN_12B_R);

  HAL_ADC_Start(&hadc2);
  HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)adcRAwValues,
                               ADC_BUFFER_SIZE);
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim5);

  // 调试用，解除以下注释并停止调用nlms32_process_block_u16函数，可以让DAC输出方波，
  // 可以用示波器测 高/低电平持续时间 得到 DAC 输出频率
  // for (int i = 0; i < DAC_BUFFER_SIZE; i++) {
  //   dacOutputValues[i] = (i % 2 == 0) ? 0 : 4095;
  // }

  while (1) {
    static uint32_t last_print = 0;
    if (HAL_GetTick() - last_print > 500) {
      last_print = HAL_GetTick();
      volatile uint8_t breakpoint_marker = 0; //^ 断点行
      (void)breakpoint_marker;
      // 在这一行打断点可以观察：
      // adc_fs_full ADC采样频率
      // tim5_delta_half ADC采样半个缓冲区所需时间
      // nlms_time_us_half 主循环处理半个缓冲区数据所需时间
      //! tim5_delta_half 必须大于 nlms_time_us_half
    }

    if ((int32_t)(HAL_GetTick() - nlms_reset_led_until_tick) < 0) {
      HAL_GPIO_WritePin(NLMS_RESET_LED_PORT, NLMS_RESET_LED_PIN,
                        GPIO_PIN_RESET);
    } else {
      HAL_GPIO_WritePin(NLMS_RESET_LED_PORT, NLMS_RESET_LED_PIN, GPIO_PIN_SET);
    }

    if (DMA_HALF_SIGN == 1) {
      DMA_HALF_SIGN = 0;
      uint32_t t_start = TIM5->CNT;

      nlms32_process_block_u16(&nlms, adcRAwValues, dacOutputValues,
                               ADC_PAIR_COUNT);
      nlms_time_us_half = TIM5->CNT - t_start;
    }

    if (DMA_FULL_SIGN == 1) {
      DMA_FULL_SIGN = 0;
      uint32_t t_start = TIM5->CNT;

      nlms32_process_block_u16(&nlms, adcRAwValues + ADC_BUFFER_SIZE,
                               dacOutputValues + (DAC_BUFFER_SIZE / 2),
                               ADC_PAIR_COUNT);
      nlms_time_us_full = TIM5->CNT - t_start;
    }

    if ((DMA_HALF_SIGN == 0) && (DMA_FULL_SIGN == 0)) {
      nlms_freq_async_process_slice();
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void nlms32_init(nlms32_t *s) {
  memset(s, 0, sizeof(*s));
  if (NLMS_CUSTOM_INIT_ON == 1) {
    memcpy(s->w, (NLMS_INITIAL_W_SET == 2) ? nlms_initial_w_2 : nlms_initial_w,
           sizeof(s->w));
  }
  s->mu = NLMS_MU;
  s->gain = 1.0f;
  s->idx = 0;
  s->prev_out_f = ADC_MIX_MID;
  s->prev_out_valid = 0;
  memset(s->despike_hist, 0, sizeof(s->despike_hist));
  s->despike_valid_count = 0;
  nlms_freq_meter_init(&s->freq_meter);
  ref_square_freq_meter_init(&s->ref_square_freq_meter);
  s->ref_square_freq_meter_active = 0;
  dynamic_notch_init(&s->ref_notch);
  ref_square_detector_init(&s->ref_square_detector);
  nlms_square_protect_default_config(&s->square_protect_cfg, NLMS_MU,
                                     NLMS_SQUARE_PROTECT_MU);
  nlms_update_notch_init(&s->square_update_notch, NLMS_FREQ_SAMPLE_RATE,
                         s->square_protect_cfg.notch_r);
  s->square_protect_enabled = 0;
  s->square_protect_mu_eff = NLMS_MU;
  s->square_protect_e_update = 0.0f;
  debug_square_protect_enabled = 0;
  debug_square_protect_mu_eff = NLMS_MU;
  debug_square_protect_notch_freq_hz = 0.0f;
  debug_square_protect_e_update = 0.0f;
}

void nlms32_reset(nlms32_t *s) {
  if (NLMS_CUSTOM_INIT_ON == 1) {
    memcpy(s->w, (NLMS_INITIAL_W_SET == 2) ? nlms_initial_w_2 : nlms_initial_w,
           sizeof(s->w));
  } else {
    memset(s->w, 0, sizeof(s->w));
  }
  memset(s->x_hist, 0, sizeof(s->x_hist));
  s->mu = NLMS_MU;
  s->gain = 1.0f;
  s->idx = 0;
  s->prev_out_f = ADC_MIX_MID;
  s->prev_out_valid = 0;
  memset(s->despike_hist, 0, sizeof(s->despike_hist));
  s->despike_valid_count = 0;
  nlms_freq_meter_init(&s->freq_meter);
  ref_square_freq_meter_init(&s->ref_square_freq_meter);
  s->ref_square_freq_meter_active = 0;
  dynamic_notch_init(&s->ref_notch);
  ref_square_detector_init(&s->ref_square_detector);
  nlms_square_protect_default_config(&s->square_protect_cfg, NLMS_MU,
                                     NLMS_SQUARE_PROTECT_MU);
  nlms_update_notch_init(&s->square_update_notch, NLMS_FREQ_SAMPLE_RATE,
                         s->square_protect_cfg.notch_r);
  s->square_protect_enabled = 0;
  s->square_protect_mu_eff = NLMS_MU;
  s->square_protect_e_update = 0.0f;
  debug_square_protect_enabled = 0;
  debug_square_protect_mu_eff = NLMS_MU;
  debug_square_protect_notch_freq_hz = 0.0f;
  debug_square_protect_e_update = 0.0f;
}

void ref_square_detector_init(ref_square_detector_t *d) {
  memset(d, 0, sizeof(*d));
  d->min_x = 1.0e30f;
  d->max_x = -1.0e30f;
  d->prev_min_x = 0.0f;
  d->prev_max_x = 0.0f;
  ref_noise_is_square = 0;
  debug_ref_square_pp = 0.0f;
  debug_ref_square_amplitude = 0.0f;
  debug_ref_square_edge_count = 0;
  debug_ref_square_plateau_ratio = 0.0f;
}

void ref_square_detector_process(ref_square_detector_t *d, float x_raw) {
  if (x_raw < d->min_x) {
    d->min_x = x_raw;
  }
  if (x_raw > d->max_x) {
    d->max_x = x_raw;
  }

  const float last_pp = d->prev_max_x - d->prev_min_x;
  if (last_pp >= REF_SQUARE_DETECT_MIN_PP) {
    const float low_plateau_th = d->prev_min_x + 0.25f * last_pp;
    const float high_plateau_th = d->prev_max_x - 0.25f * last_pp;

    if (x_raw <= low_plateau_th) {
      d->plateau_count++;
      d->low_plateau_count++;
      d->low_plateau_sum += x_raw;
    } else if (x_raw >= high_plateau_th) {
      d->plateau_count++;
      d->high_plateau_count++;
      d->high_plateau_sum += x_raw;
    }
  }

  if (d->prev_valid) {
    const float dx = x_raw - d->prev_x;
    if ((last_pp >= REF_SQUARE_DETECT_MIN_PP) &&
        (fabsf(dx) >= (REF_SQUARE_EDGE_TH_RATIO * last_pp))) {
      d->edge_count++;
    }
  } else {
    d->prev_valid = 1;
  }

  d->prev_x = x_raw;
  d->sample_count++;

  if (d->sample_count >= REF_SQUARE_DETECT_WINDOW_SIZE) {
    const float pp = d->max_x - d->min_x;
    const float plateau_ratio =
        (float)d->plateau_count / (float)d->sample_count;
    const uint8_t has_robust_levels =
        (d->high_plateau_count > 0U) && (d->low_plateau_count > 0U);
    const float high_level =
        has_robust_levels ? (d->high_plateau_sum / (float)d->high_plateau_count)
                          : d->max_x;
    const float low_level =
        has_robust_levels ? (d->low_plateau_sum / (float)d->low_plateau_count)
                          : d->min_x;

    ref_noise_is_square = (pp >= REF_SQUARE_DETECT_MIN_PP) &&
                          (d->edge_count >= REF_SQUARE_EDGE_COUNT_MIN) &&
                          (plateau_ratio >= REF_SQUARE_PLATEAU_RATIO_MIN);

    debug_ref_square_pp = pp;
    debug_ref_square_amplitude =
        ((ref_noise_is_square == 1) ? (high_level - low_level) : 0.0f) /
        4095.0 * 3.3 / 2;
    debug_ref_square_edge_count = d->edge_count;
    debug_ref_square_plateau_ratio = plateau_ratio;

    d->sample_count = 0;
    d->edge_count = 0;
    d->plateau_count = 0;
    d->high_plateau_count = 0;
    d->low_plateau_count = 0;
    d->high_plateau_sum = 0.0f;
    d->low_plateau_sum = 0.0f;
    d->prev_min_x = d->min_x;
    d->prev_max_x = d->max_x;
    d->min_x = 1.0e30f;
    d->max_x = -1.0e30f;
  }
}

void dynamic_notch_init(dynamic_notch_t *f) {
  memset(f, 0, sizeof(*f));
  f->b0 = 1.0f;
  f->b1 = 0.0f;
  f->b2 = 0.0f;
  f->a1 = 0.0f;
  f->a2 = 0.0f;
  f->locked_freq_hz = 0.0f;
  f->freq_valid = 0;

  debug_ref_notch_locked_freq_hz = 0.0f;
  debug_ref_notch_valid = 0;
  debug_ref_notch_last_in = 0.0f;
  debug_ref_notch_last_out = 0.0f;
}

void dynamic_notch_set_freq(dynamic_notch_t *f, float freq_hz) {
  if ((freq_hz < NLMS_REF_NOTCH_MIN_FREQ) ||
      (freq_hz > NLMS_REF_NOTCH_MAX_FREQ)) {
    return;
  }

  const float w0 = 2.0f * (float)M_PI * freq_hz / NLMS_REF_NOTCH_SAMPLE_RATE;
  const float c = cosf(w0);
  const float r = NLMS_REF_NOTCH_R;

  f->b0 = 1.0f;
  f->b1 = -2.0f * c;
  f->b2 = 1.0f;
  f->a1 = -2.0f * r * c;
  f->a2 = r * r;
  f->locked_freq_hz = freq_hz;
  f->freq_valid = 1;

  debug_ref_notch_locked_freq_hz = f->locked_freq_hz;
  debug_ref_notch_valid = f->freq_valid;
}

void dynamic_notch_update_freq(dynamic_notch_t *f, float measured_freq_hz,
                               float confidence) {
  if ((confidence < NLMS_REF_NOTCH_MIN_CONFIDENCE) ||
      (measured_freq_hz < NLMS_REF_NOTCH_MIN_FREQ) ||
      (measured_freq_hz > NLMS_REF_NOTCH_MAX_FREQ)) {
    return;
  }

  if (f->freq_valid == 0) {
    dynamic_notch_set_freq(f, measured_freq_hz);
    return;
  }

  if (fabsf(measured_freq_hz - f->locked_freq_hz) >
      NLMS_REF_NOTCH_FREQ_CHANGE_TH) {
    const float new_freq =
        f->locked_freq_hz +
        NLMS_REF_NOTCH_UPDATE_ALPHA * (measured_freq_hz - f->locked_freq_hz);
    dynamic_notch_set_freq(f, new_freq);
  }
}

float dynamic_notch_process(dynamic_notch_t *f, float x) {
  if ((NLMS_REF_NOTCH_ON == 0) || (f->freq_valid == 0)) {
    return x;
  }
  if ((NLMS_REF_NOTCH_BYPASS_LOW_FREQ_ON == 1) &&
      (debug_nlms_freq_hz < NLMS_REF_NOTCH_BYPASS_FREQ_HZ)) {
    return x;
  }

  const float y =
      f->b0 * x + f->b1 * f->x1 + f->b2 * f->x2 - f->a1 * f->y1 - f->a2 * f->y2;

  f->x2 = f->x1;
  f->x1 = x;
  f->y2 = f->y1;
  f->y1 = y;

  return y;
}

void nlms_freq_meter_init(nlms_freq_meter_t *m) {
  memset(m, 0, sizeof(*m));
  m->min_e = 1.0e30f;
  m->max_e = -1.0e30f;
  debug_nlms_freq_hz = 0.0f;
  debug_nlms_freq_confidence = 0.0f;
  debug_nlms_freq_updated = 0;
  debug_nlms_zero_cross_count = 0;
  debug_nlms_e_rms = 0.0f;
  debug_nlms_e_pp = 0.0f;
  nlms_freq_capture_buffer_index = 0;
  nlms_freq_async_init();
}

static void nlms_freq_async_init(void) {
  memset(&nlms_freq_async, 0, sizeof(nlms_freq_async));
  nlms_freq_async.phase = NLMS_FREQ_ASYNC_IDLE;
  debug_nlms_freq_async_phase = (uint8_t)nlms_freq_async.phase;
}

static void nlms_freq_async_begin_biquad(nlms_freq_async_phase_t phase,
                                         const float *input) {
  nlms_freq_async.phase = phase;
  nlms_freq_async.input = input;
  nlms_freq_async.index = 0;
  nlms_freq_async.x1 = input[0];
  nlms_freq_async.x2 = input[0];
  nlms_freq_async.y1 = input[0];
  nlms_freq_async.y2 = input[0];
  debug_nlms_freq_async_phase = (uint8_t)nlms_freq_async.phase;
}

static void nlms_freq_async_begin_reverse(nlms_freq_async_phase_t phase) {
  nlms_freq_async.phase = phase;
  nlms_freq_async.index = 0;
  debug_nlms_freq_async_phase = (uint8_t)nlms_freq_async.phase;
}

static void nlms_freq_async_begin_binary(nlms_freq_async_phase_t phase,
                                         uint32_t avg_filter_shift) {
  nlms_freq_async.phase = phase;
  nlms_freq_async.index = 1;
  nlms_freq_async.avg1 = nlms_freq_forward_buffer[0];
  nlms_freq_async.avg2 = nlms_freq_forward_buffer[0];
  nlms_freq_async.avg3 = nlms_freq_forward_buffer[0];
  nlms_freq_async.avg_divisor = (float)(1UL << avg_filter_shift);
  nlms_freq_binary_buffer[0] = 0U;
  debug_nlms_freq_async_phase = (uint8_t)nlms_freq_async.phase;
}

static void nlms_freq_async_begin_estimate(nlms_freq_async_phase_t phase,
                                           uint32_t stable_sample_count) {
  nlms_freq_async.phase = phase;
  nlms_freq_async.index = 1;
  nlms_freq_async.stable_sample_count = stable_sample_count;
  nlms_freq_async.rising_count = 0;
  nlms_freq_async.first_edge = 0;
  nlms_freq_async.last_edge = 0;
  nlms_freq_async.current_state_high =
      (nlms_freq_binary_buffer[0] != 0U) ? 1U : 0U;
  nlms_freq_async.candidate_state_high = nlms_freq_async.current_state_high;
  nlms_freq_async.candidate_count = 1U;
  debug_nlms_freq_async_phase = (uint8_t)nlms_freq_async.phase;
}

static uint8_t nlms_freq_async_start(const float *input, uint32_t sample_count,
                                     float rms, float min_e, float max_e) {
  if (nlms_freq_async.phase != NLMS_FREQ_ASYNC_IDLE) {
    debug_nlms_freq_async_drop_count++;
    return 0U;
  }

  nlms_freq_async.sample_count = sample_count;
  nlms_freq_async.rms = rms;
  nlms_freq_async.min_e = min_e;
  nlms_freq_async.max_e = max_e;
  nlms_freq_async.predict_freq_hz = 0.0f;
  nlms_freq_async_begin_biquad(NLMS_FREQ_ASYNC_FORWARD, input);
  return 1U;
}

static void nlms_freq_async_finish(void) {
  float freq_hz = 0.0f;
  float confidence = 0.0f;

  if ((nlms_freq_async.rising_count >= 2U) &&
      (nlms_freq_async.last_edge > nlms_freq_async.first_edge)) {
    freq_hz =
        ((float)(nlms_freq_async.rising_count - 1U) * NLMS_FREQ_SAMPLE_RATE) /
        (float)(nlms_freq_async.last_edge - nlms_freq_async.first_edge);
  }

  if ((nlms_freq_async.rms > NLMS_FREQ_MIN_RMS) &&
      (nlms_freq_async.rising_count >= 2U) && (freq_hz >= NLMS_FREQ_MIN_HZ) &&
      (freq_hz <= NLMS_FREQ_MAX_HZ)) {
    debug_nlms_freq_hz = freq_hz;
    confidence = nlms_freq_async.rms / (nlms_freq_async.rms + 100.0f);
  }

  debug_nlms_freq_confidence = confidence;
  debug_nlms_zero_cross_count = nlms_freq_async.rising_count;
  debug_nlms_e_rms = nlms_freq_async.rms;
  debug_nlms_e_pp =
      (nlms_freq_async.max_e - nlms_freq_async.min_e) / 4095.0f * 3.3f;
  debug_nlms_freq_history[debug_nlms_freq_history_index] = debug_nlms_freq_hz;
  debug_nlms_freq_history_index++;
  if (debug_nlms_freq_history_index >= DEBUG_NLMS_FREQ_HISTORY_SIZE) {
    debug_nlms_freq_history_index = 0;
    debug_nlms_freq_history_full = 1U;
  }
  if (debug_nlms_freq_history_count < DEBUG_NLMS_FREQ_HISTORY_SIZE) {
    debug_nlms_freq_history_count++;
  }
  debug_nlms_freq_async_complete_count++;
  debug_nlms_freq_updated = 1U;
  nlms_freq_async.phase = NLMS_FREQ_ASYNC_IDLE;
  debug_nlms_freq_async_phase = (uint8_t)nlms_freq_async.phase;
}

static void nlms_freq_async_process_biquad(float *output) {
  uint32_t end = nlms_freq_async.index + NLMS_FREQ_ASYNC_CHUNK_SIZE;
  if (end > nlms_freq_async.sample_count) {
    end = nlms_freq_async.sample_count;
  }

  for (uint32_t i = nlms_freq_async.index; i < end; i++) {
    const float x0 = nlms_freq_async.input[i];
    const float y0 = NLMS_FREQ_LP_B0 * x0 +
                     NLMS_FREQ_LP_B1 * nlms_freq_async.x1 +
                     NLMS_FREQ_LP_B2 * nlms_freq_async.x2 -
                     NLMS_FREQ_LP_A1 * nlms_freq_async.y1 -
                     NLMS_FREQ_LP_A2 * nlms_freq_async.y2;

    output[i] = y0;
    nlms_freq_async.x2 = nlms_freq_async.x1;
    nlms_freq_async.x1 = x0;
    nlms_freq_async.y2 = nlms_freq_async.y1;
    nlms_freq_async.y1 = y0;
  }

  nlms_freq_async.index = end;
}

static void nlms_freq_async_process_reverse(const float *input, float *output) {
  uint32_t end = nlms_freq_async.index + NLMS_FREQ_ASYNC_CHUNK_SIZE;
  if (end > nlms_freq_async.sample_count) {
    end = nlms_freq_async.sample_count;
  }

  for (uint32_t i = nlms_freq_async.index; i < end; i++) {
    output[i] = input[nlms_freq_async.sample_count - 1U - i];
  }

  nlms_freq_async.index = end;
}

static void nlms_freq_async_process_binary(void) {
  uint32_t end = nlms_freq_async.index + NLMS_FREQ_ASYNC_CHUNK_SIZE;
  if (end > nlms_freq_async.sample_count) {
    end = nlms_freq_async.sample_count;
  }

  for (uint32_t i = nlms_freq_async.index; i < end; i++) {
    nlms_freq_async.avg1 +=
        (nlms_freq_forward_buffer[i] - nlms_freq_async.avg1) /
        nlms_freq_async.avg_divisor;
    nlms_freq_async.avg2 += (nlms_freq_async.avg1 - nlms_freq_async.avg2) /
                            nlms_freq_async.avg_divisor;
    nlms_freq_async.avg3 += (nlms_freq_async.avg2 - nlms_freq_async.avg3) /
                            nlms_freq_async.avg_divisor;

    nlms_freq_binary_buffer[i] =
        (nlms_freq_forward_buffer[i] > nlms_freq_async.avg3) ? 1U : 0U;
  }

  nlms_freq_async.index = end;
}

static void nlms_freq_async_process_estimate(uint32_t sample_count) {
  uint32_t end = nlms_freq_async.index + NLMS_FREQ_ASYNC_CHUNK_SIZE;
  if (end > sample_count) {
    end = sample_count;
  }

  for (uint32_t i = nlms_freq_async.index; i < end; i++) {
    const uint8_t is_high = (nlms_freq_binary_buffer[i] != 0U) ? 1U : 0U;

    if (is_high == nlms_freq_async.candidate_state_high) {
      nlms_freq_async.candidate_count++;
    } else {
      nlms_freq_async.candidate_state_high = is_high;
      nlms_freq_async.candidate_count = 1U;
    }

    if ((nlms_freq_async.candidate_state_high !=
         nlms_freq_async.current_state_high) &&
        (nlms_freq_async.candidate_count >=
         nlms_freq_async.stable_sample_count)) {
      if ((nlms_freq_async.current_state_high == 0U) &&
          (nlms_freq_async.candidate_state_high != 0U)) {
        const uint32_t edge_index =
            i - nlms_freq_async.stable_sample_count + 1U;

        nlms_freq_async.rising_count++;
        if (nlms_freq_async.rising_count == 1U) {
          nlms_freq_async.first_edge = edge_index;
        }
        nlms_freq_async.last_edge = edge_index;
      }

      nlms_freq_async.current_state_high = nlms_freq_async.candidate_state_high;
    }
  }

  nlms_freq_async.index = end;
}

static void nlms_freq_async_process_slice(void) {
  switch (nlms_freq_async.phase) {
  case NLMS_FREQ_ASYNC_IDLE:
    break;

  case NLMS_FREQ_ASYNC_FORWARD:
    nlms_freq_async_process_biquad(nlms_freq_forward_buffer);
    if (nlms_freq_async.index >= nlms_freq_async.sample_count) {
      nlms_freq_async_begin_reverse(NLMS_FREQ_ASYNC_REVERSE_1);
    }
    break;

  case NLMS_FREQ_ASYNC_REVERSE_1:
    nlms_freq_async_process_reverse(nlms_freq_forward_buffer,
                                    nlms_freq_reverse_buffer);
    if (nlms_freq_async.index >= nlms_freq_async.sample_count) {
      nlms_freq_async_begin_biquad(NLMS_FREQ_ASYNC_BACKWARD,
                                   nlms_freq_reverse_buffer);
    }
    break;

  case NLMS_FREQ_ASYNC_BACKWARD:
    nlms_freq_async_process_biquad(nlms_freq_reverse_buffer);
    if (nlms_freq_async.index >= nlms_freq_async.sample_count) {
      nlms_freq_async_begin_reverse(NLMS_FREQ_ASYNC_REVERSE_2);
    }
    break;

  case NLMS_FREQ_ASYNC_REVERSE_2:
    nlms_freq_async_process_reverse(nlms_freq_reverse_buffer,
                                    nlms_freq_forward_buffer);
    if (nlms_freq_async.index >= nlms_freq_async.sample_count) {
      nlms_freq_async_begin_binary(NLMS_FREQ_ASYNC_BINARY_HIGH,
                                   NLMS_FREQ_AVG_SHIFT_HIGH);
    }
    break;

  case NLMS_FREQ_ASYNC_BINARY_HIGH:
    nlms_freq_async_process_binary();
    if (nlms_freq_async.index >= nlms_freq_async.sample_count) {
      nlms_freq_async_begin_estimate(NLMS_FREQ_ASYNC_ESTIMATE_PREDICT,
                                     NLMS_FREQ_PREDICT_STABLE_COUNT);
    }
    break;

  case NLMS_FREQ_ASYNC_ESTIMATE_PREDICT:
    nlms_freq_async_process_estimate(NLMS_FREQ_PREDICT_SIZE);
    if (nlms_freq_async.index >= NLMS_FREQ_PREDICT_SIZE) {
      if ((nlms_freq_async.rising_count >= 2U) &&
          (nlms_freq_async.last_edge > nlms_freq_async.first_edge)) {
        nlms_freq_async.predict_freq_hz =
            ((float)(nlms_freq_async.rising_count - 1U) *
             NLMS_FREQ_SAMPLE_RATE) /
            (float)(nlms_freq_async.last_edge - nlms_freq_async.first_edge);
      } else {
        nlms_freq_async.predict_freq_hz = 0.0f;
      }

      if (nlms_freq_async.predict_freq_hz > NLMS_FREQ_PREDICT_TH_HZ) {
        nlms_freq_async_begin_estimate(NLMS_FREQ_ASYNC_ESTIMATE_FINAL,
                                       NLMS_FREQ_PREDICT_STABLE_COUNT);
      } else {
        nlms_freq_async_begin_binary(NLMS_FREQ_ASYNC_BINARY_FINAL,
                                     NLMS_FREQ_AVG_SHIFT_LOW);
      }
    }
    break;

  case NLMS_FREQ_ASYNC_BINARY_FINAL:
    nlms_freq_async_process_binary();
    if (nlms_freq_async.index >= nlms_freq_async.sample_count) {
      nlms_freq_async_begin_estimate(NLMS_FREQ_ASYNC_ESTIMATE_FINAL,
                                     NLMS_FREQ_FALLBACK_STABLE_COUNT);
    }
    break;

  case NLMS_FREQ_ASYNC_ESTIMATE_FINAL:
    nlms_freq_async_process_estimate(nlms_freq_async.sample_count);
    if (nlms_freq_async.index >= nlms_freq_async.sample_count) {
      nlms_freq_async_finish();
    }
    break;

  default:
    nlms_freq_async_init();
    break;
  }
}

void nlms_freq_meter_process(nlms_freq_meter_t *m, float e_raw, nlms32_t *s) {
  (void)s;

  m->sum2 += e_raw * e_raw;
  if (e_raw < m->min_e) {
    m->min_e = e_raw;
  }
  if (e_raw > m->max_e) {
    m->max_e = e_raw;
  }
  nlms_freq_window_buffers[nlms_freq_capture_buffer_index][m->sample_count] =
      e_raw;
  m->sample_count++;

  m->prev_e = e_raw;
  m->prev_valid = 1;

  if (m->sample_count >= NLMS_FREQ_WINDOW_SIZE) {
    const float rms = sqrtf(m->sum2 / (float)m->sample_count);
    const uint8_t filled_buffer_index = nlms_freq_capture_buffer_index;

    if (nlms_freq_async_start(nlms_freq_window_buffers[filled_buffer_index],
                              NLMS_FREQ_WINDOW_SIZE, rms, m->min_e,
                              m->max_e) == 1U) {
      nlms_freq_capture_buffer_index ^= 1U;
    }

    m->sample_count = 0;
    m->zero_cross_count = 0;
    m->sum2 = 0.0f;
    m->min_e = 1.0e30f;
    m->max_e = -1.0e30f;
  }
}

void ref_square_freq_meter_init(nlms_freq_meter_t *m) {
  memset(m, 0, sizeof(*m));
  m->min_e = 1.0e30f;
  m->max_e = -1.0e30f;
  debug_ref_square_freq_hz = 0.0f;
  debug_ref_square_freq_confidence = 0.0f;
  debug_ref_square_zero_cross_count = 0;
  debug_ref_square_rms = 0.0f;

  square_nlms_freq_ratio = 0;
}

void ref_square_freq_meter_process(nlms_freq_meter_t *m, float x_raw) {
  m->sum2 += x_raw * x_raw;
  m->sample_count++;

  if (x_raw < -NLMS_FREQ_HYST_TH) {
    m->armed_positive = 1;
  }

  if ((m->armed_positive == 1) && (x_raw > NLMS_FREQ_HYST_TH)) {
    m->zero_cross_count++;
    m->armed_positive = 0;
  }

  m->prev_e = x_raw;
  m->prev_valid = 1;

  if (m->sample_count >= NLMS_FREQ_WINDOW_SIZE) {
    const float rms = sqrtf(m->sum2 / (float)m->sample_count);
    float confidence = 0.0f;

    if ((rms > NLMS_FREQ_MIN_RMS) && (m->zero_cross_count > 0U)) {
      const float freq_hz = (float)m->zero_cross_count * NLMS_FREQ_SAMPLE_RATE /
                            (float)m->sample_count;

      if ((freq_hz >= NLMS_FREQ_MIN_HZ) && (freq_hz <= NLMS_FREQ_MAX_HZ)) {
        m->last_freq_hz = freq_hz;
        confidence = rms / (rms + 100.0f);
      }
    }

    m->confidence = confidence;
    debug_ref_square_freq_hz = m->last_freq_hz;
    if (debug_ref_square_freq_hz != 0) {
      square_nlms_freq_ratio = debug_nlms_freq_hz / debug_ref_square_freq_hz;
    } else {
      square_nlms_freq_ratio = 0;
    }
    debug_ref_square_freq_confidence = m->confidence;
    debug_ref_square_zero_cross_count = m->zero_cross_count;
    debug_ref_square_rms = rms;

    m->sample_count = 0;
    m->zero_cross_count = 0;
    m->sum2 = 0.0f;
  }
}

void biquad_hp_init(biquad_hp_t *f) {
  f->b0 = 0.9920347;
  f->b1 = -1.98406941;
  f->b2 = 0.9920347;
  f->a1 = -1.98400596;
  f->a2 = 0.98413285;

  f->x1 = 0.0f;
  f->x2 = 0.0f;
  f->y1 = 0.0f;
  f->y2 = 0.0f;
}

float biquad_hp_process(biquad_hp_t *f, float x0) {
  float y0 = f->b0 * x0 + f->b1 * f->x1 + f->b2 * f->x2 - f->a1 * f->y1 -
             f->a2 * f->y2;

  f->x2 = f->x1;
  f->x1 = x0;
  f->y2 = f->y1;
  f->y1 = y0;

  return y0;
}

uint16_t clamp_to_dac_u16(float v) {
  if (v < 0.0f)
    return 0;
  if (v > (float)DAC_MAX)
    return DAC_MAX;
  return (uint16_t)(v + 0.5f);
}

double poly33_3k(double x, double y) {
  const double p00 = -1.292;
  const double p10 = -2.809;
  const double p01 = 9.671;
  const double p20 = -2.627;
  const double p11 = 4.891;
  const double p02 = -9.536;
  const double p30 = 0.216;
  const double p21 = 3.043;
  const double p12 = -4.130;
  const double p03 = 3.638;

  double x2 = x * x;
  double y2 = y * y;
  double x3 = x2 * x;
  double y3 = y2 * y;

  return p00 + p10 * x + p01 * y + p20 * x2 + p11 * x * y + p02 * y2 +
         p30 * x3 + p21 * x2 * y + p12 * x * y2 + p03 * y3;
}

double poly33_5k(double x, double y) {
  const double p00 = 2.314;
  const double p10 = 3.545;
  const double p01 = -5.606;
  const double p20 = -1.721;
  const double p11 = -6.22;
  const double p02 = 9.128;
  const double p30 = 1.137;
  const double p21 = -1.388;
  const double p12 = 4.707;
  const double p03 = -4.512;

  double x2 = x * x;
  double y2 = y * y;
  double x3 = x2 * x;
  double y3 = y2 * y;

  return p00 + p10 * x + p01 * y + p20 * x2 + p11 * x * y + p02 * y2 +
         p30 * x3 + p21 * x2 * y + p12 * x * y2 + p03 * y3;
}

void nlms_adjust_square_wave_mode(nlms32_t *s) {
  enum {
    NLMS_SQUARE_MODE_NONE = 0,
    NLMS_SQUARE_MODE_NON_ODD_HARMONIC = 1,
    NLMS_SQUARE_MODE_THIRD_HARMONIC = 2,
    NLMS_SQUARE_MODE_FIFTH_HARMONIC = 3,
  };

  static uint8_t stable_count = 0;
  static uint8_t square_mode = NLMS_SQUARE_MODE_NONE;
  static uint8_t gain_adjusted = 0;

  static uint8_t harmonic_stable_count = 0;

  if (ref_noise_is_square == 1) {
    if ((square_nlms_freq_ratio > 2.95) && (square_nlms_freq_ratio < 3.03)) {
      if (square_mode == NLMS_SQUARE_MODE_THIRD_HARMONIC) {
        stable_count++;
      } else {
        square_mode = NLMS_SQUARE_MODE_THIRD_HARMONIC;
        stable_count = 0;
      }
    } else if ((square_nlms_freq_ratio > 4.92) &&
               (square_nlms_freq_ratio < 5.08)) {
      if (square_mode == NLMS_SQUARE_MODE_FIFTH_HARMONIC) {
        stable_count++;
      } else {
        square_mode = NLMS_SQUARE_MODE_FIFTH_HARMONIC;
        stable_count = 0;
      }
    } else {
      if (square_mode == NLMS_SQUARE_MODE_NON_ODD_HARMONIC) {
        stable_count++;
      } else {
        square_mode = NLMS_SQUARE_MODE_NON_ODD_HARMONIC;
        stable_count = 0;
      }
    }
  } else {
    if (square_mode == NLMS_SQUARE_MODE_NONE) {
      stable_count++;
    } else {
      square_mode = NLMS_SQUARE_MODE_NONE;
      stable_count = 0;
    }
  }

  if (stable_count <= 50) {
    return;
  }

  switch (square_mode) {
  case NLMS_SQUARE_MODE_NONE:
    s->mu = NLMS_MU;
    s->gain = NLMS_GAIN;
    break;

  case NLMS_SQUARE_MODE_NON_ODD_HARMONIC:
    s->mu = NLMS_MU * 30;
    s->gain = NLMS_GAIN;
    break;
  case NLMS_SQUARE_MODE_THIRD_HARMONIC:
    s->mu = 0.00000001f;

    if ((gain_adjusted == 0) && (harmonic_stable_count == 4)) {
      gain_adjusted = 1;
      s->gain = poly33_3k(debug_ref_square_amplitude, debug_nlms_e_pp);
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
    } else {
      harmonic_stable_count++;
    }
    break;
  case NLMS_SQUARE_MODE_FIFTH_HARMONIC:
    s->mu = 0.00000001f;

    if ((gain_adjusted == 0) && (harmonic_stable_count == 4)) {
      gain_adjusted = 1;
      s->gain = poly33_5k(debug_ref_square_amplitude, debug_nlms_e_pp);
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
    } else {
      harmonic_stable_count++;
    }
    break;

  default:
    break;
  }
  stable_count = 0;
}

/*
 * 一次处理一个 block（双 ADC 交错输入版）
 *
 * adc_interleaved : 双 ADC 规则同时采样后的交错输入数组
 *                   ][d0, x0, d1, x1, d2, x2, ...
 *                   其中 d 为主输入块（混合信号/期望输入）
 *                   x 为参考输入块（噪声参考）
 * out             : 输出块（误差信号），长度为 pair_count * DAC_INTERP_FACTOR
 * pair_count      : 采样对数；例如传 128 表示处理 128 对双 ADC 数据
 */
void nlms32_process_block_u16(nlms32_t *s,
                              const uint16_t *restrict adc_interleaved,
                              uint16_t *restrict out, uint32_t pair_count) {
  float *restrict w = s->w;
  float *restrict hist = s->x_hist;
  uint8_t idx = s->idx;

  float prev_out_f = s->prev_out_f;
  uint8_t prev_out_valid = s->prev_out_valid;
  uint8_t despike_valid_count = s->despike_valid_count;

  // 小巧思
  nlms_adjust_square_wave_mode(s);

  for (uint32_t n = 0; n < pair_count; n++) {
    const uint32_t base = (uint32_t)(n << 1);

    const float d = (float)adc_interleaved[base + 0] - ADC_MIX_MID;
    const float x_new_raw = (float)adc_interleaved[base + 1] - ADC_NOISE_MID;
    ref_square_detector_process(&s->ref_square_detector, x_new_raw);
    if (ref_noise_is_square == 1) {
      if (s->ref_square_freq_meter_active == 0) {
        ref_square_freq_meter_init(&s->ref_square_freq_meter);
        s->ref_square_freq_meter_active = 1;
      }
      ref_square_freq_meter_process(&s->ref_square_freq_meter, x_new_raw);
    } else if (s->ref_square_freq_meter_active == 1) {
      ref_square_freq_meter_init(&s->ref_square_freq_meter);
      s->ref_square_freq_meter_active = 0;
    }

    const float x_ref = (NLMS_REF_NOTCH_ON == 1)
                            ? dynamic_notch_process(&s->ref_notch, x_new_raw)
                            : x_new_raw;

    debug_ref_notch_last_in = x_new_raw;
    debug_ref_notch_last_out = x_ref;
    debug_ref_notch_locked_freq_hz = s->ref_notch.locked_freq_hz;
    debug_ref_notch_valid = s->ref_notch.freq_valid;

    float e;

    if (DIRECT_MIX_MINUS_NOISE_ON == 1) {
      e = d - x_new_raw;
    } else {
      idx = (idx == 0) ? (NLMS_TAPS - 1) : (idx - 1);
      hist[idx] = x_ref;
      hist[idx + NLMS_TAPS] = x_ref;

      const float *restrict x = &hist[idx];

      float y = 0.0f;
      float norm = 0.0f;

      // 四阶展开
      for (int i = 0; i < NLMS_TAPS; i += 4) {
        const float x0 = x[i + 0];
        const float x1 = x[i + 1];
        const float x2 = x[i + 2];
        const float x3 = x[i + 3];

        y += w[i + 0] * x0 + w[i + 1] * x1 + w[i + 2] * x2 + w[i + 3] * x3;
        norm += x0 * x0 + x1 * x1 + x2 * x2 + x3 * x3;
      }

      e = d - y;

      float e_update = e;
      float mu_eff = s->mu;
      uint8_t square_protect_enabled = 0U;

#if NLMS_SQUARE_PROTECT_ON == 1
      const uint8_t square_ratio_is_odd_harmonic =
          (((square_nlms_freq_ratio > 2.90f) &&
            (square_nlms_freq_ratio < 3.10f)) ||
           ((square_nlms_freq_ratio > 4.80f) &&
            (square_nlms_freq_ratio < 5.20f)))
              ? 1U
              : 0U;

      const uint8_t square_protect_allowed =
          (square_ratio_is_odd_harmonic == 0U) ? 1U : 0U;

      if ((square_protect_allowed == 1U) &&
          (nlms_square_protect_should_enable(
               &s->square_protect_cfg, ref_noise_is_square,
               debug_ref_square_freq_hz, debug_nlms_freq_hz,
               debug_nlms_freq_confidence) == 1U)) {
        if ((s->square_update_notch.valid == 0U) ||
            (fabsf(debug_nlms_freq_hz -
                   s->square_update_notch.notch_freq_hz) >
             NLMS_SQUARE_PROTECT_FREQ_UPDATE_TH_HZ)) {
          if (nlms_update_notch_set_freq(&s->square_update_notch,
                                         debug_nlms_freq_hz) == 1U) {
            nlms_update_notch_reset(&s->square_update_notch);
          }
        }

        if (s->square_update_notch.valid == 1U) {
          e_update = nlms_update_notch_process(&s->square_update_notch, e);
          mu_eff = nlms_square_protect_mu(&s->square_protect_cfg, 1U);
          square_protect_enabled = 1U;
        }
      } else {
        s->square_update_notch.valid = 0U;
        nlms_update_notch_reset(&s->square_update_notch);
      }
#endif

      s->square_protect_enabled = square_protect_enabled;
      s->square_protect_mu_eff = mu_eff;
      s->square_protect_e_update = e_update;
      debug_square_protect_enabled = square_protect_enabled;
      debug_square_protect_mu_eff = mu_eff;
      debug_square_protect_notch_freq_hz =
          s->square_update_notch.valid ? s->square_update_notch.notch_freq_hz
                                       : 0.0f;
      debug_square_protect_e_update = e_update;

      if (NLMS_FREEZE_W_ON == 0) {
        const float g = (mu_eff * e_update) / (NLMS_EPS + norm);

        for (int i = 0; i < NLMS_TAPS; i += 4) {
          w[i + 0] += g * x[i + 0];
          w[i + 1] += g * x[i + 1];
          w[i + 2] += g * x[i + 2];
          w[i + 3] += g * x[i + 3];
        }
      }
    }

    nlms_freq_meter_process(&s->freq_meter, e, s);
    if (debug_nlms_freq_updated == 1U) {
      debug_nlms_freq_updated = 0U;
      const float measured_freq_hz = debug_nlms_freq_hz;
      const float confidence = debug_nlms_freq_confidence;
      uint8_t did_jump_reset = 0;

      if ((NLMS_FREQ_JUMP_RESET_ON == 1) &&
          (confidence >= NLMS_REF_NOTCH_MIN_CONFIDENCE) &&
          (measured_freq_hz >= NLMS_REF_NOTCH_MIN_FREQ) &&
          (measured_freq_hz <= NLMS_REF_NOTCH_MAX_FREQ) &&
          (s->ref_notch.freq_valid == 1) &&
          (fabsf(measured_freq_hz - s->ref_notch.locked_freq_hz) >
           NLMS_FREQ_JUMP_RESET_TH)) {
        nlms32_reset(s);
        dynamic_notch_set_freq(&s->ref_notch, measured_freq_hz);
        nlms_reset_led_until_tick = HAL_GetTick() + NLMS_RESET_LED_HOLD_MS;

        w = s->w;
        hist = s->x_hist;
        idx = s->idx;
        prev_out_f = s->prev_out_f;
        prev_out_valid = s->prev_out_valid;
        despike_valid_count = s->despike_valid_count;
        did_jump_reset = 1;
      }

      if (did_jump_reset == 0) {
        dynamic_notch_update_freq(&s->ref_notch, measured_freq_hz, confidence);
      }
    }

    float e_for_dac = e;
    if (NLMS_DESPIKE_ON == 1) {
      if (despike_valid_count < 4) {
        s->despike_hist[despike_valid_count] = e;
        despike_valid_count++;
      } else {
        if (despike_valid_count == 4) {
          s->despike_hist[4] = e;
          despike_valid_count = 5;
        } else {
          s->despike_hist[0] = s->despike_hist[1];
          s->despike_hist[1] = s->despike_hist[2];
          s->despike_hist[2] = s->despike_hist[3];
          s->despike_hist[3] = s->despike_hist[4];
          s->despike_hist[4] = e;
        }

        const float p0 = s->despike_hist[0];
        const float p1 = s->despike_hist[1];
        const float p2 = s->despike_hist[2];
        const float p3 = s->despike_hist[3];
        const float p4 = s->despike_hist[4];
        const float predicted = (-p0 + 9.0f * p1 + 9.0f * p3 - p4) / 16.0f;
        const float residual = p2 - predicted;

        const uint8_t positive_spike =
            ((p2 - p1) > NLMS_DESPIKE_TH) && ((p2 - p3) > NLMS_DESPIKE_TH);
        const uint8_t negative_spike =
            ((p1 - p2) > NLMS_DESPIKE_TH) && ((p3 - p2) > NLMS_DESPIKE_TH);
        const uint8_t spike = (positive_spike || negative_spike) &&
                              (fabsf(residual) > NLMS_DESPIKE_TH);

        e_for_dac = spike ? predicted : p2;
      }
    }

    const float out_signal = (HIGH_PASS_FILTER_ON == 1)
                                 ? biquad_hp_process(&hp_filter, e_for_dac)
                                 : e_for_dac;
    const float curr_out_f = out_signal * s->gain + ADC_MIX_MID;

    const uint32_t out_base = (uint32_t)(n * DAC_INTERP_FACTOR);

    if (!prev_out_valid) {
      for (uint32_t k = 0; k < DAC_INTERP_FACTOR; k++) {
        out[out_base + k] = clamp_to_dac_u16(curr_out_f);
      }
      prev_out_valid = 1;
    } else {
      const float step = (curr_out_f - prev_out_f) / (float)DAC_INTERP_FACTOR;
      for (uint32_t k = 0; k < DAC_INTERP_FACTOR; k++) {
        out[out_base + k] =
            clamp_to_dac_u16(prev_out_f + step * (float)(k + 1));
      }
    }

    prev_out_f = curr_out_f;
  }

  s->idx = idx;
  s->prev_out_f = prev_out_f;
  s->prev_out_valid = prev_out_valid;
  s->despike_valid_count = despike_valid_count;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc == &hadc1) {
    uint32_t now = TIM5->CNT;

    tim5_delta_half = now - tim5_last_full; // full -> half，半个buffer时间
    tim5_last_half = now;

    // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
    DMA_HALF_SIGN = 1;
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc == &hadc1) {
    uint32_t now = TIM5->CNT;

    tim5_delta_full = now - tim5_last_half; // half -> full，半个buffer时间
    tim5_last_full = now;

    if (tim5_delta_full != 0) {
      adc_fs_full = ADC_PAIR_COUNT * 1000000.0f / (float)tim5_delta_full;
    }

    // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
    DMA_FULL_SIGN = 1;
  }
}

void ClearDACOutputValues() {
  for (int i = 0; i < DAC_BUFFER_SIZE; i++) {
    dacOutputValues[i] = 0;
  }
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state
   */
  __disable_irq();
  while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
     file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
