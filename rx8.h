#ifndef GLOBALS_H
#define GLOBALS_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <i2c_t3.h>								// to get the I2C working.
#include <FlexCAN.h>


#define D_P_VR		12
#define D_P_LED		13

#define D_P_SPK1	2							// ignition spark #1
#define D_P_SPK4	3							// ignition spark #4
#define D_P_SPK2	4							// ignition spark #2
#define D_P_SPK3	5							// ignition spark #3

/* ----------------------------------------------------------------------------------------------------
 * Constant values
 * ---------------------------------------------------------------------------------------------------- */
/*
 * 經過實際測試 2Kohm pull-up resistor + 5V 可讓 I2C 的 cable 超過 1M 還可以正常在 800KHz 下運作!
 * ->不過... 使用這個阻值的 Teensy - died!, Arduino Nano - A5 & A6 似乎掛了...
 * ->目前測試可以穩定使用的阻值為 3K
 *
 * 400KHz  - D_I2C_TIMEOUT = 5000
 * 800KHz  - D_I2C_TIMEOUT = 4000
 * 1200KHz - D_I2C_TIMEOUT = 2000
 */
#define D_I2C_TIMEOUT	6000							// I2C write timeout, in microseconds; 注意! 如果 I2C bus 的 freq 低於 2,400KHz, 則需將此值放慢!


/* ----------------------------------------------------------------------------------------------------
 * Options / Flags
 * ---------------------------------------------------------------------------------------------------- */
#define DEBUG		1							// DEBUG enables human readable logging and info messages


/* ----------------------------------------------------------------------------------------------------
 * Structures
 * ---------------------------------------------------------------------------------------------------- */
struct statuses
{
	volatile float volt;							// charging voltage
	volatile bool volt_over;
	volatile bool volt_under;

	volatile bool brake;							// 判斷 user 是按/踩煞車的變數
	volatile uint32_t brake_ts0;
	volatile uint32_t brake_ts1;

	volatile uint32_t speed;
	volatile float speed_kps;						// KM per second
	volatile uint32_t speed_alive;						// 判斷 speed single 是否正常的時間變數
	volatile uint32_t speed_ts0;
	volatile uint32_t speed_ts1;
	volatile uint32_t speed_max;						// 記錄最高速度
	volatile uint32_t speed_avg;						// 平均速度(average speed = distance traveled / time taken)
	volatile uint32_t odo;							// 累計走行距離; !注意! 根據實際上的測試 - float 的最大值僅到 16384 就不能再增加小數!

	volatile uint32_t conf_ts;						// 達到可以儲存設定至 EEPROM 的轉速時間
	volatile bool conf_update;						// 可寫入 EEPROM 的識別旗標
	volatile uint32_t conf_update_ts;					// 最後一次寫入 EEPROM 的時間(避免過於頻繁更新)

	float tps;								// The current TPS reading (0% - 100%)
	uint8_t tps_last;							// The previous TPS reading
	uint32_t tps_ts;							// 用於避免因過度頻繁讀取 probe sensor 造成的效能問題
	float tps_volt_min;							// idle voltage - When engine is idling: 0.9 ∼ 1.1 V
	float tps_volt_max;							// WOT voltage - When grip is fully opened: 4.06 ∼ 4.26 V

	float fuel_level;							// fuel level: 0 ~ 100%
	uint32_t fuel_ts;							// 用於避免因過度頻繁讀取 probe sensor 造成的效能問題
	bool fuel_low;								// 剩餘油量過低旗標
	volatile float fuel_cs;
	volatile float fuel_cs_hr;
	volatile float fuel_cs_kpl;
	volatile uint32_t fuel_cs_ts0;
	volatile uint32_t fuel_cs_ts1;

	float clt;								// coolant temperature
	uint32_t clt_ts;							// 用於避免因過度頻繁讀取 probe sensor 造成的效能問題
	bool clt_high;								// 水溫過高旗標

	float egt = 0;
	uint32_t egt_ts;							// 用於避免因過度頻繁讀取 probe sensor 造成的效能問題
	bool egt_exist;								// thermocouple attached flag
	bool egt_high;

	float iat;								// intake air temperature
	uint32_t iat_ts;							// 用於避免因過度頻繁讀取 probe sensor 造成的效能問題
	bool iat_exist;								// DS18Bx family device flag

	uint32_t time;								// the current time stamp
	uint32_t watchdog;							// 用於計算 watchdog 的更新時間

	float lambda;

	uint32_t com_ts;							// 用於避免因過度頻繁讀取 serial port 造成的效能問題
	uint8_t com_cmd;

	volatile uint8_t sig_status;						// 用於比對燈號的旗標變數
	volatile uint8_t sig_reg;						// 寫入 PCA9551 register 的值

	uint8_t loop_count;							// 計算迴圈執行次數

	bool scr_init;								// "init" screen flag
	bool scr_error;								// error flag
	uint8_t scr_index;							// current main screen index number
	uint8_t scr_sub;							// current sub screen index number
	uint8_t scr_sub_max;							// sub-screen 數
	uint8_t scr_contrast;							// default contrast
	uint32_t scr_ts0;							// screen refresh interval
	uint32_t scr_ts1;							// screen refresh interval
	uint32_t scr_status;							// status bar 的 icon 顯示狀態旗標
	uint32_t scr_status_ts;							// status bar 的 icon 閃爍用 "timer"
	bool scr_status_sw;							// "閃爍" 旗標

#ifdef DEBUG
	uint32_t dbg_loop_ts;							// 判斷已執行 >= 1 秒鐘的識別變數
	uint32_t dbg_loop_count;						// 計算迴圈執行次數
	uint32_t dbg_loop_count2;						// 累計迴圈執行次數用的變數
	uint8_t dbg_loop_loading[8];
	uint8_t dbg_loop_ptr;
#endif
};

struct wheel
{
	uint8_t tooth;								// tooths
	uint8_t count;								//
	uint32_t filter;							// the shortest possible time (in uS) that there can be between crank teeth
	uint32_t angle_th;							// 每齒間隔角度
	uint32_t angle_sw;							// 各缸的間隔角度
	volatile uint32_t angle_ex;						// 需要額外增加的角度
	uint32_t angle_max;							// 總角度
	int32_t angle_b;							// coil 充電角度
	int32_t angle;								// crankshaft angle
	uint32_t tpd;								// time per degree
	volatile uint32_t ts0;							//
	volatile uint32_t ts1;							//
	volatile uint32_t ts2;							//
	volatile uint32_t ts_to0;						// tooth#1 - current
	volatile uint32_t ts_to1;						// tooth#1 - last
	volatile uint32_t gap;							// 此次與上次 2 齒間的時間差
	volatile uint8_t history_idx;
	volatile uint32_t history[256];						//
};

struct engine
{
	volatile bool sync;							// 引擎 "同步" 旗標
	volatile bool sync_whl;							// wheel "同步" 旗標
	volatile bool sync_cyl;							// cylinder "同步" 旗標

	volatile uint32_t dwell;						// coil 充電時間長度 - us
	uint32_t advance;							// 提前點火角度
	volatile bool cyl1;							// 第 1 缸識別旗標
	volatile uint8_t cyls[5];
	int32_t cyls_tdc[4];							// 各 cylinder TDC 的相對角度

	volatile uint32_t rpm;
	volatile uint32_t rpm_alive;						// 判斷 RPM single 是否正常的時間變數
	uint32_t rpm_ts;

	/*
	 * status code:
	 * -1 - pending
	 *  0 - preparing
	 *  1 - charging
	 *  2 - discharged
	 */
	volatile uint8_t coil_s[4];						// 各 coil 的 status code
	volatile uint32_t coil_c[4];						// 各 coil 的 charge time point
	volatile uint32_t coil_d[4];						// 各 coil 的 discharge time point
	volatile uint32_t coil_r[4];						// 各 coil 的 interrupt 發生時間

	volatile bool dwell1;							// #1 / #4 充電旗標
	volatile bool dwell2;							// #2 / #3 充電旗標
};


/* ----------------------------------------------------------------------------------------------------
 * Global variables
 * ---------------------------------------------------------------------------------------------------- */
uint16_t G_rpm;
uint16_t G_rpm_raw;
char G_ary_buf[12];								// "-2147483648\0"

extern struct statuses G_status;
extern struct wheel G_wheel;
extern struct engine G_eng;

#ifdef DEBUG
	#define DP(...) Serial.print(__VA_ARGS__)
	#define DPL(...) Serial.println(__VA_ARGS__)
#else
	#define DP(...)
	#define DPL(...)
#endif

#endif
