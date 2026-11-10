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
//#include <stdio.h> //ここ追加
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//ロボマスモータの回転指定
//typedef struct
//{
//	uint16_t CANID;
//	uint8_t motorID;
//	int16_t actVel;
//	double actangle;
//	int16_t actCurrent;
//	int8_t dir_sign; // +1: 正転, -1: 反転
//} motor;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;

//FDCANの指定
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

//FDCAN_HandleTypeDef hfdcan1;
//FDCAN_HandleTypeDef hfdcan3;
//
//TIM_HandleTypeDef htim2;
//
//UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//ライントレースの設定
#define LINE_THRESHOLD 2000U

uint32_t adcValue[7] = {0};
uint8_t lineState[7] = {0};

//CANとロボマスモータの設定
//#define RM_ENC_COUNTS_PER_REV  8192        // C610 が返すロータ1回転のカウント数（0〜8191）
//#define RM_REDUCTION_RATIO     36          // M2006 P36 の減速比 36:1
//#define RM_CNT_PER_OUTPUT_REV  (RM_ENC_COUNTS_PER_REV * RM_REDUCTION_RATIO)
//
//// どのモータを何回転させるか
//#define TARGET_MOTOR_ID_L        3           // 動かしたい C610 の ID 3
//#define TARGET_MOTOR_ID_R        2           // 動かしたい C610 の ID (1〜8)
//
//typedef struct
//{
//	uint8_t motor_id; // C610 のモータID (1〜8)
//
//	// 状態（ロータ側の多回転位置）
//	int16_t last_raw_angle; // 最後に受信した生角度 (0〜8191)
//	int32_t multi_pos_counts; // ロータ側多回転カウント（unwrap 済み）
//	uint8_t pos_initialized; // unwrap 初期化済みか
//
//	// 目標値・制御パラメータ（ロータ側カウント）
//	int32_t target_pos_counts;
//	int32_t tol_counts; // 許容誤差 [カウント]
//	float kp; // 位置Pゲイン
//	float kd;
//
//	int16_t current_vel;
//	int16_t current_limit; // 指令電流上限（C610 の単位：±10000）
//
//	// 動作フラグ
//	uint8_t move_started; // 目標に向けて動作中
//	uint8_t move_finished; // 目標到達済み
//	uint8_t enabled; // このモータの制御を有効にするか
//} RM_MotorCtrlCtx;
//
//static RM_MotorCtrlCtx g_motor_ctx[8]; // max 8
//
//// 1フレーム集約送信用 現在指令値（1-4:0x200, 5-8:0x1FF）
//int16_t g_cmd_current_200[4] = {0, 0, 0, 0};
//int16_t g_cmd_current_1FF[4] = {0, 0, 0, 0};
//
//
//FDCAN_TxHeaderTypeDef TxHeader;
//FDCAN_RxHeaderTypeDef RxHeader;
//FDCAN_TxHeaderTypeDef TxHeader_motor;
//FDCAN_RxHeaderTypeDef RxHeader_motor;
//FDCAN_FilterTypeDef sFilterConfig;
//
//uint8_t TxData[8] = {};
//uint8_t RxData[8] = {};
//uint8_t TxData_motor[8] = {};
//uint8_t RxData_motor[8] = {};
//
//motor robomas[8] = {
//	{0x201, 1, 0, 0, 0, +1},
//	{0x202, 2, 0, 0, 0, -1},
//	{0x203, 3, 0, 0, 0, +1},
//	{0x204, 4, 0, 0, 0, +1},
//	{0x205, 5, 0, 0, 0, +1},
//	{0x206, 6, 0, 0, 0, +1},
//	{0x207, 7, 0, 0, 0, +1},
//	{0x208, 8, 0, 0, 0, +1},
//};
// 各モータの反転方向は robomas[].dir_sign に内包（+1: 正転、-1: 反転）


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN2_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */

// 指定したモータID配列の全てが一度でもデータ受信（pos_initialized=1）になるまで待機
//static void WaitUntilMotorsInitialized(const uint8_t *ids, uint32_t count)
//{
//	uint32_t last_print = HAL_GetTick();
//	for (;;)
//	{
//		uint8_t all_ready = 1;
//		for (uint32_t i = 0; i < count; i++)
//		{
//			uint8_t id = ids[i];
//			if (id < 1 || id > 8)
//			{
//				// 範囲外IDは無視（必要ならエラーメッセージに変更可）
//				continue;
//			}
//			if (!g_motor_ctx[id - 1].pos_initialized)
//			{
//				all_ready = 0;
//				break;
//			}
//		}
//
//		if (all_ready)
//		{
//			printf("All motors ready!\r\n");
//			return;
//		}
//
//		if (HAL_GetTick() - last_print >= 500)
//		{
//			printf("Waiting...");
//			for (uint32_t i = 0; i < count; i++)
//			{
//				uint8_t id = ids[i];
//				if (id < 1 || id > 8)
//				{
//					continue;
//				}
//				printf(" ID%d:%d", id, g_motor_ctx[id - 1].pos_initialized);
//			}
//			printf("\r\n");
//			last_print = HAL_GetTick();
//		}
//
//		HAL_Delay(10);
//	}
//}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// C610 へ電流指令を送信する
//static int32_t i32_abs(int32_t x)
//{
//	return (x >= 0) ? x : -x;
//}
//
//static void RM_SendCurrent(uint8_t motor_id, int16_t current)
//{
//	// 1フレーム集約送信方式へ変更：
//	// この関数では送信せず、モータIDに対応するスロットの最新電流値を更新のみ行う。
//
//	// 方向係数(+1/-1)を適用（robomas 構造体に内包）
//	int16_t signed_current = current;
//	if (motor_id >= 1 && motor_id <= 8)
//	{
//		int8_t sign = robomas[motor_id - 1].dir_sign;
//		if (sign != 1 && sign != -1)
//		{
//			sign = 1; // フォールバック
//		}
//		signed_current = (int16_t) (current * sign);
//	}
//
//	if (motor_id >= 1 && motor_id <= 4)
//	{
//		uint8_t index = (uint8_t) (motor_id - 1);
//		g_cmd_current_200[index] = signed_current;
//	} else if (motor_id >= 5 && motor_id <= 8)
//	{
//		uint8_t index = (uint8_t) (motor_id - 5);
//		g_cmd_current_1FF[index] = signed_current;
//	} else
//	{
//		return; // 不正ID
//	}
//}
//
//// 集約送信: 0x200(1-4), 0x1FF(5-8) をそれぞれ1フレームで送る
//static void RM_FlushMotorCurrents(void)
//{
//	uint8_t data[8];
//
//	// 1〜4 (0x200)
//	TxHeader_motor.Identifier = 0x200;
//	for (int i = 0; i < 4; ++i)
//	{
//		data[2 * i] = (uint8_t) ((g_cmd_current_200[i] >> 8));
//		data[2 * i + 1] = (uint8_t) (g_cmd_current_200[i] & 0xFF);
//	}
//	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader_motor, data) != HAL_OK)
//	{
//		printf("motor tx (0x200) error\r\n");
//	}
//
//	// // 5〜8 (0x1FF)
//	// extern int16_t g_cmd_current_1FF[4];
//	// TxHeader_motor.Identifier = 0x1FF;
//	// for (int i = 0; i < 4; ++i) {
//	//     data[2*i]   = (uint8_t)((g_cmd_current_1FF[i] >> 8) & 0xFF);
//	//     data[2*i+1] = (uint8_t)( g_cmd_current_1FF[i]       & 0xFF);
//	// }
//	// if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader_motor, data) != HAL_OK) {
//	//     printf("motor tx (0x1FF) error\r\n");
//	// }
//}
//
//static void RM_MotorCtrl_InitContext(uint8_t motor_id)
//{
//	RM_MotorCtrlCtx *ctx = &g_motor_ctx[motor_id - 1];
//	ctx->motor_id = motor_id;
//	ctx->kp = 1.5f; // 仮のPゲイン（要実機調整）
//	ctx->kd = 0.1;
//	ctx->current_limit = 1000; // とりあえず1A
//	ctx->tol_counts = RM_CNT_PER_OUTPUT_REV / 10; // 出力軸で約36度相当
//	ctx->enabled = 1;
//}
//
///* フィードバックから多回転位置を更新する（ロータ側） */
//static void RM_MotorCtrl_UpdateFromFeedback(RM_MotorCtrlCtx *ctx, int16_t raw_angle, int16_t vel) {
//	if (!ctx->enabled) return;
//	ctx->current_vel = vel;
//	if (!ctx->pos_initialized)
//	{
//		ctx->last_raw_angle = raw_angle;
//		ctx->multi_pos_counts = 0; // 今の位置を原点とみなす
//		ctx->pos_initialized = 1;
//		return;
//	}
//
//	int16_t diff = raw_angle - ctx->last_raw_angle;
//
//	// 0〜8191 の範囲を跨いだときの unwrap
//	if (diff > RM_ENC_COUNTS_PER_REV / 2) diff -= RM_ENC_COUNTS_PER_REV;
//	if (diff < -RM_ENC_COUNTS_PER_REV / 2) diff += RM_ENC_COUNTS_PER_REV;
//
//	// フィードバックの向きもモータの反転設定に合わせる
//	// robomas[].dir_sign が -1 の場合、カウント増減を反転して蓄積する
//	{
//		int8_t sign = 1;
//		uint8_t mid = ctx->motor_id;
//		if (mid >= 1 && mid <= 8)
//		{
//			int8_t s = robomas[mid - 1].dir_sign;
//			sign = (s == 1 || s == -1) ? s : 1; // フォールバック
//		}
//		diff = (int16_t) (diff * sign);
//	}
//
//	ctx->multi_pos_counts += diff;
//	ctx->last_raw_angle = raw_angle;
//}
//
//
///* 出力軸の「相対回転数」で目標をセットする（今の位置から N 回転分） */
//static void RM_MotorCtrl_RequestRelativeOutputTurns(RM_MotorCtrlCtx *ctx, double output_turns)
//{
//	if (!ctx->enabled || !ctx->pos_initialized)
//	{
//		// まだ初期位置が分からない場合は何もしない（必要なら「保留フラグ」を自作しても良い）
//		return;
//	}
//
//	// 出力軸 N 回転分をロータ側のカウントに換算
//	int32_t delta_counts = output_turns * RM_CNT_PER_OUTPUT_REV;
//
//	ctx->target_pos_counts = ctx->multi_pos_counts + delta_counts;
//	ctx->move_started = 1;
//	ctx->move_finished = 0;
//
//	printf("Motor %u: request %ld output turns\r\n",
//	       ctx->motor_id, (long) output_turns);
//}
//
//
//
///* 1ステップ分の制御（位置P制御 → 電流指令） */
//static void RM_MotorCtrl_Step(RM_MotorCtrlCtx *ctx)
//{
//	// ★修正: enabledでない場合は、安全のため電流0をセットしてリターンする
//	if (!ctx->enabled)
//	{
//		RM_SendCurrent(ctx->motor_id, 0);
//		return;
//	}
//
//	// pos_initialized は enabled のチェックの後にする
//	if (!ctx->pos_initialized) return;
//
//	// move_startedフラグだけで判定し、finishedでも制御を続けるように変更
//	if (!ctx->move_started)
//	{
//		// 完全に脱力したい場合だけ0を送る
//		RM_SendCurrent(ctx->motor_id, 0);
//		return;
//	}
//
//	int32_t err = ctx->target_pos_counts - ctx->multi_pos_counts;
//
//	// --- 変更点：許容誤差内でも電流を切らず、位置保持(Hold)モードへ ---
//	if (i32_abs(err) < ctx->tol_counts)
//	{
//		ctx->move_finished = 1;
//		// ここで return しない！制御を継続する＝ブレーキがかかる
//	} else
//	{
//		ctx->move_finished = 0;
//	}
//
//	// --- PD制御の計算 ---
//	// P項: 目標位置へ引っ張る力（バネ）
//	float p_term = ctx->kp * (float)err;
//
//	// D項: 速度を殺す力（ダンパー・ブレーキ）
//	// 目標に向かうときはオーバーシュートを防ぎ、止まっている時は外乱に抵抗します
//	float d_term = ctx->kd * (float)ctx->current_vel;
//
//	// 操作量 = P項 - D項
//	float cur_f = p_term - d_term;
//
//	// リミット処理
//	if (cur_f >  ctx->current_limit) cur_f =  ctx->current_limit;
//	if (cur_f < -ctx->current_limit) cur_f = -ctx->current_limit;
//
//	RM_SendCurrent(ctx->motor_id, (int16_t)cur_f);
//}
//
///* ===== ここからIDベースの公開インターフェース ===== */
//static inline int valid_motor_id(uint8_t motor_id) { return (motor_id >= 1 && motor_id <= 8); }
//
//// ID指定で1ステップ制御を行う
//static void RM_MotorCtrl_StepId(uint8_t motor_id)
//{
//	if (!valid_motor_id(motor_id)) return;
//	RM_MotorCtrlCtx *ctx = &g_motor_ctx[motor_id - 1];
//	RM_MotorCtrl_Step(ctx);
//}
//
//// ID指定で「出力軸 N 回転」の相対移動を要求する
//static void RM_MotorCtrl_RequestRelativeOutputTurnsId(uint8_t motor_id, double output_turns)
//{
//	if (!valid_motor_id(motor_id)) return;
//	RM_MotorCtrlCtx *ctx = &g_motor_ctx[motor_id - 1];
//	RM_MotorCtrl_RequestRelativeOutputTurns(ctx, output_turns);
//}
//
//// 指定IDのコマンドが完了したら真を返す
//static uint8_t RM_MotorCtrl_IsFinished(uint8_t motor_id)
//{
//	if (!valid_motor_id(motor_id)) return 1; // 不正IDは「完了扱い」して抜けを促す
//	RM_MotorCtrlCtx *ctx = &g_motor_ctx[motor_id - 1];
//	return ctx->move_finished ? 1 : 0;
//}
//
//
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	// 設定したタイマかどうか確認 (例: TIM6)
//	if (htim->Instance == TIM2)
//	{
//		// 1. 各モーターの制御計算 (PID計算 -> バッファセット)
//		RM_MotorCtrl_StepId(TARGET_MOTOR_ID_L);
//		RM_MotorCtrl_StepId(TARGET_MOTOR_ID_R);
//
//		// 2. CAN送信 (バッファの内容をまとめて送信)
//		RM_FlushMotorCurrents();
//
//		// ※ここには printf を書かないこと！
//	}
//}
//
//static void RM_MotorCtrl_DisableId(uint8_t motor_id)
//{
//	if (!valid_motor_id(motor_id)) return;
//	RM_MotorCtrlCtx *ctx = &g_motor_ctx[motor_id - 1];
//
//	// 制御フラグを下ろす
//	// → これにより次の割り込みから RM_MotorCtrl_Step 内で 0 がセットされる
//	ctx->enabled = 0;
//
//	// 即座にバッファも0にしておく（念の為）
//	RM_SendCurrent(motor_id, 0);
//}
//
//void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
//{
//	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
//	{
//		/* Retrieve Rx messages from RX FIFO0 */
//		if (HAL_FDCAN_GetRxMessage(&hfdcan3, FDCAN_RX_FIFO0, &RxHeader_motor, RxData_motor) != HAL_OK)
//		{
//			printf("fdcan_getrxmessage_motor is error\r\n");
//			Error_Handler();
//		}
//		/*receive robomas's status*/
//		for (int i = 0; i <= 7; i++)
//		{
//			if (RxHeader_motor.Identifier == (robomas[i].CANID))
//			{
//				robomas[i].actangle = (int16_t) ((RxData_motor[0] << 8) | RxData_motor[1]);
//				robomas[i].actVel = (int16_t) ((RxData_motor[2] << 8) | RxData_motor[3]);
//				robomas[i].actCurrent = (int16_t) ((RxData_motor[4] << 8) | RxData_motor[5]);
//
//				uint8_t mid = robomas[i].motorID;
//				if (mid >= 1 && mid <= 8)
//				{
//					RM_MotorCtrl_UpdateFromFeedback(&g_motor_ctx[mid - 1], robomas[i].actangle, robomas[i].actVel);
//				}
//			}
//		}
//	}
//}
//
//void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
//{
//	int16_t CANID = 0;
//	if (RESET != (RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE))
//	{
//		/* Retrieve Rx messages from RX FIFO0 */
//
//		if (HAL_OK != HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, RxData))
//		{
//			printf("fdcan_getrxmessage is error\r\n");
//			Error_Handler();
//		}
//
//		if (CANID == RxHeader.Identifier)
//		{
//		}
//	}
//}
//
///*Initializing fdcan1*/
//void FDCAN_RxTxSettings(void)
//{
//	FDCAN_FilterTypeDef FDCAN_Filter_settings;
//	FDCAN_Filter_settings.IdType = FDCAN_STANDARD_ID;
//	FDCAN_Filter_settings.FilterIndex = 0;
//	FDCAN_Filter_settings.FilterType = FDCAN_FILTER_RANGE;
//	FDCAN_Filter_settings.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
//	FDCAN_Filter_settings.FilterID1 = 0;
//	FDCAN_Filter_settings.FilterID2 = 0x7FF;
//
//	TxHeader.Identifier = 0x000;
//	TxHeader.IdType = FDCAN_STANDARD_ID;
//	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
//	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
//	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
//	TxHeader.BitRateSwitch = FDCAN_BRS_ON;
//	TxHeader.FDFormat = FDCAN_FD_CAN;
//	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
//	TxHeader.MessageMarker = 0;
//
//
//	if (HAL_OK != HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN_Filter_settings))
//	{
//		printf("fdcan_configfilter is error\r\n");
//		Error_Handler();
//	}
//
//	if (HAL_OK != HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_FILTER_REJECT, FDCAN_FILTER_REMOTE,
//	                                           FDCAN_FILTER_REMOTE))
//	{
//		printf("fdcan_configglobalfilter is error\r\n");
//		Error_Handler();
//	}
//
//	if (HAL_OK != HAL_FDCAN_Start(&hfdcan1))
//	{
//		printf("fdcan_start is error\r\n");
//		Error_Handler();
//	}
//
//	if (HAL_OK != HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0))
//	{
//		printf("fdcan_activatenotification is error\r\n");
//		Error_Handler();
//	}
//}
//
///*Initializing fdcan3*/
//void FDCAN_motor_RxTxSettings(void)
//{
//	FDCAN_FilterTypeDef FDCAN_Filter_settings_motor;
//	FDCAN_Filter_settings_motor.IdType = FDCAN_STANDARD_ID;
//	FDCAN_Filter_settings_motor.FilterIndex = 0;
//	FDCAN_Filter_settings_motor.FilterType = FDCAN_FILTER_RANGE;
//	FDCAN_Filter_settings_motor.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
//	FDCAN_Filter_settings_motor.FilterID1 = 0x200;
//	FDCAN_Filter_settings_motor.FilterID2 = 0x210;
//
//	TxHeader_motor.Identifier = 0x200;
//	TxHeader_motor.IdType = FDCAN_STANDARD_ID;
//	TxHeader_motor.TxFrameType = FDCAN_DATA_FRAME;
//	TxHeader_motor.DataLength = FDCAN_DLC_BYTES_8;
//	TxHeader_motor.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
//	TxHeader_motor.BitRateSwitch = FDCAN_BRS_OFF;
//	TxHeader_motor.FDFormat = FDCAN_CLASSIC_CAN;
//	TxHeader_motor.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
//	TxHeader_motor.MessageMarker = 0;
//
//	if (HAL_OK != HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN_Filter_settings_motor))
//	{
//		printf("fdcan_configfilter is error\r\n");
//		Error_Handler();
//	}
//
//	if (HAL_OK != HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_FILTER_REJECT, FDCAN_FILTER_REMOTE,
//	                                           FDCAN_FILTER_REMOTE))
//	{
//		printf("fdcan_configglobalfilter is error\r\n");
//		Error_Handler();
//	}
//
//	if (HAL_OK != HAL_FDCAN_Start(&hfdcan3))
//	{
//		printf("fdcan_start is error\r\n");
//		Error_Handler();
//	}
//
//	if (HAL_OK != HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0))
//	{
//		printf("fdcan_activatenotification is error\r\n");
//		Error_Handler();
//	}
//}
//
//
//int _write(int file, char *ptr, int len)
//{
//	HAL_UART_Transmit(&huart2, (uint8_t *) ptr, len, 10);
//	return len;
//}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

	//なんか書いてる
	//setbuf(stdout, NULL);

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
  MX_CAN1_Init();
  //MX_FDCAN1_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_CAN2_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_UART4_Init();

  /* USER CODE BEGIN 2 */

  //CANの設定
//  printf("start\r\n");
//  FDCAN_motor_RxTxSettings(); //Initialize fdcan3
//  printf("can_motor_start\r\n");
//  FDCAN_RxTxSettings(); //Initialize fdcan1
//  printf("can start\r\n");
//  RM_SendCurrent(TARGET_MOTOR_ID_L, 0);
//  RM_SendCurrent(TARGET_MOTOR_ID_R, 0);
//  RM_FlushMotorCurrents();
//
//
//  RM_MotorCtrl_InitContext(TARGET_MOTOR_ID_L);
//  RM_MotorCtrl_InitContext(TARGET_MOTOR_ID_R);
//
//  uint32_t last_print = HAL_GetTick();
//
//  // 待機するモータID配列を指定して、初期化完了まで待つ
//  uint8_t ids_to_wait[] = {TARGET_MOTOR_ID_L, TARGET_MOTOR_ID_R};
//  WaitUntilMotorsInitialized(ids_to_wait, sizeof(ids_to_wait) / sizeof(ids_to_wait[0]));
//  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	//デバッグとか
//	RM_MotorCtrl_RequestRelativeOutputTurnsId(TARGET_MOTOR_ID_L, 5.5);
//	RM_MotorCtrl_RequestRelativeOutputTurnsId(TARGET_MOTOR_ID_R, 5.5);
//	while (1)
//	{
//		if (RM_MotorCtrl_IsFinished(TARGET_MOTOR_ID_L) && RM_MotorCtrl_IsFinished(TARGET_MOTOR_ID_R))
//		{
//			break;
//		}
//
//	  	/* デバッグ出力（1秒ごと） */
//	  	// if (HAL_GetTick() - last_print >= 1000) {
//	  	// 	for (int i = 0; i < 8; i++) {
//	  	// 		printf("Motor%d: ID=0x%x, motorID=%d, vel=%d, angle_raw=%d, current=%d\r\n",
//	  	// 		 i + 1, robomas[i].CANID, robomas[i].motorID,
//	  	// 		 robomas[i].actVel, (int)robomas[i].actangle, robomas[i].actCurrent);
//	  	// 	}
//	  	// 	last_print = HAL_GetTick();
//	  	// }
//
//	  	// HAL_Delay(1);
//	}
//
//	RM_MotorCtrl_RequestRelativeOutputTurnsId(TARGET_MOTOR_ID_L, 2);
//	RM_MotorCtrl_RequestRelativeOutputTurnsId(TARGET_MOTOR_ID_R, -2);
//	while (1)
//	{
//		if (RM_MotorCtrl_IsFinished(TARGET_MOTOR_ID_L) && RM_MotorCtrl_IsFinished(TARGET_MOTOR_ID_R))
//		{
//			break;
//		}
//	}
//	RM_MotorCtrl_RequestRelativeOutputTurnsId(TARGET_MOTOR_ID_L, 10);
//	RM_MotorCtrl_RequestRelativeOutputTurnsId(TARGET_MOTOR_ID_R, 10);
//	while (1)
//	{
//		if (RM_MotorCtrl_IsFinished(TARGET_MOTOR_ID_L) && RM_MotorCtrl_IsFinished(TARGET_MOTOR_ID_R))
//		{
//			break;
//		}
//	}
//	RM_MotorCtrl_DisableId(TARGET_MOTOR_ID_L);
//	RM_MotorCtrl_DisableId(TARGET_MOTOR_ID_R);
//	while (1)
//	{
//
//	}

    /* USER CODE BEGIN 3 */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); //LEDを点灯
	HAL_Delay(500); //500ms待つ
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); //LEDを消灯
	HAL_Delay(500); //500ms待つ
	//基板上のLEDを点灯
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET); //LED_redを点灯
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); //LED_orangeを点灯
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET); //LED_blueを点灯
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET); //LED_greenを点灯
	HAL_GPIO_WritePin(GPIB, GPIO_PIN_1, GPIO_PIN_SET); //LED_yellowを点灯

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  //
//  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
//  RCC_OscInitStruct.PLL.PLLN = 10;

  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;

  //7チャンネルを順番に変換 kokokara
  hadc1.Init.ScanConvMode = ENABLE;

  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;

  //ラインセンサ７個
  hadc1.Init.NbrOfConversion = 7;

  hadc1.Init.DMAContinuousRequests = DISABLE;

  /*
   * 各ADC変換が終了するごとにEOCを発生
   * → 7チャンネルを1個ずつPollして取得する
   */
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  //ライントレース
  /* PA4 : Sensor 1 */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
	  Error_Handler();
  }

  /* PC0 : Sensor 2 */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
	  Error_Handler();
  }

  /* PA6 : Sensor 3 */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 3;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
	  Error_Handler();
  }

  /* PC1 : Sensor 4 */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 4;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
	  Error_Handler();
  }

  /* PA7 : Sensor 5 */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 5;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
	  Error_Handler();
  }

  /* PC2 : Sensor 6 */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 6;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
	  Error_Handler();
  }

  /* PC3 : Sensor 7 */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 7;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
	  Error_Handler();
  }

  //


  //判定
  if (adcValue[i] > LINE_THRESHOLD)
  {
      lineState[i] = 1;
  }
  else
  {
      lineState[i] = 0;
  }

  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 16;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA10 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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
