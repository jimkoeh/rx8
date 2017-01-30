/*
 * -------------------------------------------------------------------
 *
 * https://www.xsimulator.net/community/threads/driving-real-gauges.3278/
 *
 * Speedo / tacho:
 * CAN.setMessageID (message_id_201);
 * my_data[0] = (RPM * 4) / 256;		// rpm
 * my_data[1] = (RPM * 4) % 256;		// rpm
 * my_data[2] = 0xFF;				// Unknown, 0xFF from 'live'.
 * my_data[3] = 0xFF;				// Unknown, 0xFF from 'live'.
 * my_data[4] = (kph * 100 + 10000) / 256;	// speed
 * my_data[5] = (kph * 100 + 10000) % 256;	// speed
 * my_data[6] = 0x00;				// Unknown possible accelerator pedel if Madox is correc
 * my_data[7] = 0x00;				// Unknown
 *
 * Warning Lights:
 * CAN.setMessageID (message_id_212);
 * my_data[0] = 0xFE;				// Unknown
 * my_data[1] = 0xFE;				// Unknown
 * my_data[2] = 0xFE;				// Unknown
 * my_data[3] = 0x34;				// DSC OFF in combo with byte 5 Live data only seen 0x34
 * my_data[4] = 0x00;				// B01000000: Brake warning, B00001000: ABS warning
 * my_data[5] = 0x40;				// TCS in combo with byte 3
 * my_data[6] = 0x00;				// ETC
 * my_data[7] = 0x00;				// Unused
 *
 * Other gauges / warning lights:
 * CAN.setMessageID (message_id_420);
 * my_data[0] = 0x98;				// temp gauge //~168 is red, ~163 last bar, 151/152 centre, 90 first bar, 92 second bar
 * my_data[1] = 0x00;				// Distance (0.2m resolution – resets every 51m); something to do with trip meter 0x10, 0x11, 0x17 increments by 0.1 miles
 * my_data[2] = 0x00;				// unknown
 * my_data[3] = 0x00;				// unknown
 * my_data[4] = 0x01;				// Oil Pressure (not really a gauge)
 * my_data[5] = 0x00;				// check engine light
 * my_data[6] = 0x00;				// Coolant, oil and battery
 * my_data[7] = 0x00;				// unused
 * 
 * Cruise Control Light:
 * CAN.setMessageID (message_id_650);
 * my_data[0] = 0xFF;				// cruise control light 0x80 is yellow, 0xFF is green
 *
 * -------------------------------------------------------------------
 *
 * https://docs.google.com/spreadsheets/d/1SKfXAyo6fbAfMUENw1KR3w4Fvx_Ihj6sTPSVXBdOXKk/edit#gid=0
 *
 * Mazda3 2nd gen - HS CAN Bus
 *
 * -------------------------------------------------------------------
 *
 * http://www.diyelectriccar.com/forums/showpost.php?s=46f8f5a424ac112421c829665f41028b&p=577066&postcount=178
 *
 * RPM Meter: CAN message ID 513 (hex 0x201), length 8 bit. Controlled by B0 and B1.
 * 
 * 0%: B0 = 0; B1 = 0
 *
 * first pointer movement: B0 = 2; B1 = 200
 * 3000 RPM: B0 = 45; B1 = 120
 * 6000 RPM: B0 = 89; B1 = 0
 *
 * The two bits resolution is unnecessarily big I think, because the analog gauge can not show so detailed values.
 * I think controlling RPM gauge for Kostov K11 Alpha is very much possible only by linear signal where B0 = <0 - 89>,
 * but Power Steering Controller will definitely need the full range for correct and error free operation.
 *
 * If I will translate high resolution pulse output of Kostov K11 Alpha sensor into CAN messages for 
 * Instrument cluster AND PSC, I think it should be relatively easy and very elegant to integrate signal 
 * divider and make my own Rebbl signal converter board.
 *
 * -------------------------------------------------------------------
 *
 * Speed: CAN message ID513 (hex 0x201), length 8 bit, Controlled by B4 and B5
 * 
 * 0 km/h: B4 = 39; B5 = 16
 * 1 km/h: B4 = 39; B5 = 61
 * 2 km/h: B4 = 39; B5 = 157
 * 3 km/h: B4 = 39; B5 = 255
 * 4 km/h: B4 = 40; B5 = 100
 * 298 km/h: B4 = 154; B5 = 200
 * 299 km/h: B4 = 155+ B5 = 0
 * 
 * Again the resolution is very big, but digital gauge can use more of it. I think speed CAN messages are sent by ABS computer, 
 * because there is no connection for ABS wires in Power Steering Controller. Since power steering assist amount is speed dependent, 
 * it means that PSC will probably need to be fed by speed signal in full resolution of two bits to prevent errors.
 * 
 * At least it seems to be linear. So I will have to catch the signal from vehicle CAN line and send it into PSC over separated 
 * CAN line to protect PSC from nonsenses sent by ECU.
 *
 * -------------------------------------------------------------------
 *
 * Oil Pressure: CAN message ID 1056 (hex 0x420), length 7 bits, controlled by B4
 * 
 * 0% oil pressure: B4 = 0
 * 75% oil pressure: B4 = 1
 * 
 * This is all, the gauge is kind of fake, it might be simple MIL and the user would still be same informed.
 * You can not make it show for example 25%.
 * I think this might be suitable for DC/DC active / inactive status.
 * The oil pressure signal (both B4 = 0 and 1) shuts off the red MIL which looks like merry-go-around,
 * I suppose it is equivalent of "Check engine soon" MIL.
 *
 * -------------------------------------------------------------------
 *
 * Engine temperature: CAN message ID 1056 (hex 0x420), length 7 bits, controlled by B0
 * 
 * no temp shown: B0 % 69
 * 
 * Beginning of white field (0% temp): B0 = 90
 * 45% temp: B0 = 110
 * 50% temp: B0 = 151
 * 60% temp: B0 = 155
 * end of white field (100% temp): B0 = 165
 * redline (overheating): B0 = 170
 * 
 * Temp. readings does not appear to be linear. The pointer is significantly "slower" between B0 = 110 and B0 = 150.
 * I would like to use temp. meter to show me temperature of the kostov motor.
 * For this I will need my hardware to translate resistance values from Kostov motor thermistor from let's say 20 C degrees 
 * to 120 C degrees into can message 0x420 where B0 = from 90 to 170 with respect to non-linearity of the CAN message.
 *
 * -------------------------------------------------------------------
 *
 * http://www.diyelectriccar.com/forums/showpost.php?p=597082&postcount=190
 * 
 * PSC lamp: ID 768 (hex 0x300), length 1, bit 0 = 128 LAMP ON, bit0 = 1 LAMP OFF
 * 
 * DSC lamp, slide lamp: ID 530 (hex 0x212), lenght 7, bit 5:
 * DSC OFF, slide OFF: 64
 * DSC OFF, slide ON: 80
 * DSC ON, slide OFF: 0 / 8
 * DSC ON, slide ON: 16
 * DSC ON, slide blinking: 34 and around
 * DSC OFF, slide blinking: 98 and around
 * 
 * ABS lamp, brake pad lamp: ID530, length 7, bit4
 * ABS ON, brake pad OFF: 8
 * ABS OFF, brake pad OFF: 0
 * ABS OFF, brake pad ON: 64
 * ABS ON, brake pad on: 88
 * 
 * Check engine soon lamp: message 1056, lenght 7, bit 5:
 * 0 - OFF
 * 64 - ON
 * 128 - blinking
 * 
 * Merry go round (radiator level), charging, oil pressure: message 1056, length 7, bit 6:
 * radiator OFF, charging OFF, oil press OFF: 0
 * radiator ON, charging OFF, oil press OFF: 2
 * radiator OFF, charging ON, oil press OFF: 64
 * radiator ON, charging ON, oil press OFF: 66
 * radiator OFF, charging OFF, oil press ON: 128
 * radiator ON, charging OFF, oil press ON: 130
 * radiator OFF, charging ON, oil press ON: 192
 * radiator ON, charging ON, oil press ON: 194
 *
 * -------------------------------------------------------------------
 *
 * http://www.diyelectriccar.com/forums/showpost.php?p=709234&postcount=261
 * 
 * The ECU must be removed and following CAN messages must be simulated to have no errors on display,
 * control cluster, etc. The stability control, ABS works with this setup! Tested on Mazda RX 2004
 * 
 * 201 38 A8 FF FF 3A 2B C8 81		// 0,1 bytes RPM; 4 - Speed (26=0;3F=65; 4F=106; 5F=147; 6F=189)
 * 420 6B 23 C7 00 00 00 61 81		// 0 byte Temp (non linear) (5B-0%; 60-10%; 68-25%; 98=50%; 9E-75%; A4=100%)
 * 						Fault codes: 01-ok; 00 error: 4 byte: Oil pressure, 5 Check engine, 6 battery charge
 * 215 02 2D 02 2D 02 2A 06 81		// Some ECU status
 * 231 0F 00 FF FF 02 2D 06 81		// Some ECU status
 * 240 04 00 28 00 02 37 06 81		// Some ECU status
 * 250 00 00 CF 87 7F 83 00 00		// Some ECU status
 * 200 00 00 FF FF 00 32 06 81		// EPS doesn't work without this
 * 202 89 89 89 19 34 1F C8 FF		// EPS doesn't work without this
 *
 * -------------------------------------------------------------------
 *
 * http://blog.hiroaki.jp/2010/04/000470.html#extended
 * 
 * RPM: ID 0x201, (data[0] * 256 + data[1]) / 4
 * Vehicle Speed(km/h): ID 0x201, (data[4] * 256 + data[5] - 10000) / 100
 * Accel Throttle Position(0 - 255): ID 0x201, data[6]
 * Hand Brake(on/off): ID 0x212, (data[4] & 0x40)
 * Foot Brake(on/off): ID 0x212, (data[5] & 0x08)
 * Declutching(on/off): ID 0x231, (data[0] & 0xf0)
 * Engine Coolant Temp(degree celsius): ID 0x240, data[3] - 40
 * Intake Air Temp(degree celsius): ID 0x250, data[3] - 40
 */

#include "rx8.h"

#define CANbaund	500000

FlexCAN CANbus(CANbaund);

/* ----------------------------------------------------------------------------------------------------
 * Global variables
 * ---------------------------------------------------------------------------------------------------- */
struct statuses G_status;
struct wheel G_wheel;
struct engine G_eng;

static CAN_message_t msg_tx;

int16_t ary_count[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t ary_RPM[8] = { 0, 0, 0xFF, 0xFF, 0, 0, 0, 0 };
uint8_t ary_PCM[8] = { 0x04, 0x00, 0x28, 0x00, 0x02, 0x37, 0x06, 0x81 };
uint8_t ary_MIL[8] = { 0x98, 0, 0, 0, 1, 0, 0, 0 };
uint8_t ary_DSC[8] = { 0xFE, 0xFE, 0xFE, 0x34, 0, 0x40, 0, 0 };
uint8_t ary_RAN[8] = { 0x02, 0x2D, 0x02, 0x2D, 0x02, 0x2A, 0x06, 0x81 };
uint8_t ary_ECU[8] = { 0x0F, 0x00, 0xFF, 0xFF, 0x02, 0x2D, 0x06, 0x81 };
uint8_t ary_TCP[8] = { 0x00, 0x00, 0xCF, 0x87, 0x7F, 0x83, 0x00, 0x00 };
uint8_t ary_EPS[8] = { 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x32, 0x06, 0x81 };
uint8_t ary_CY[8] = { 0x89, 0x89, 0x89, 0x19, 0x34, 0x1F, 0xC8, 0xFF };
uint8_t ary_FL[8] = { 0x47, 0x92, 0x91, 0, 0, 0xcc, 0, 0 };
uint8_t ary_STR[8] = { 0x33, 0x96, 0x99, 0, 0, 0, 0, 0 };

/*
 * value	RPM
 * 980		1,000
 * 1,450	1,500
 * 1,920	2,000
 * 2,420	2,500
 * 2,890	3,000
 * 3,370	3,500
 * 3,840	4,000
 * 4,330	4,500
 * 4,800	5,000
 * 5,280	5,500
 * 5,770	6,000
 * 6,240	6,500
 * 6,720	7,000
 * 7,200	7,500
 * 7,670	8,000
 * 8,125	8,500
 * 8,580	9,000
 * 9,070	9,500
 * 9,520	10,000
 */
const uint16_t ary_map_RPM[38] = {
	1000,  980, 1500, 1450, 2000, 1920, 2500, 2420, 3000, 2890,
	3500, 3370, 4000, 3840, 4500, 4330, 5000, 4800, 5500, 5280,
	6000, 5770, 6500, 6240, 7000, 6720, 7500, 7200, 8000, 7670,
	8500, 8125, 9000, 8580, 9500, 9070, 10000, 9520
};

#ifdef DEBUG
static CAN_message_t msg_rx;
static uint8_t hex[17] = "0123456789abcdef";
#endif

/** **************************************************************************************************************
 * setup()
 */
void setup()
{
	Serial.begin(115200);

	delay(1000);

	DPL("Init...");

	Wire.begin(I2C_MASTER, 0, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);

	pinMode(D_P_VR, INPUT);
	pinMode(D_P_LED, OUTPUT);

	DPL("setup(): init. LCD module...");

	lcd_init();
	lcd_clear(true);

	noInterrupts();								// disable global interrupts

	attachInterrupt(D_P_VR, ISR_vr_puc, RISING);				// pick-up coil signal

	interrupts();								// enable global interrupts

	CANbus.begin();
	msg_tx.len = 8;

	DPL("OK!");
}

/** **************************************************************************************************************
 * loop()
 */
void loop()
{
	G_status.time = millis();						// Current Time

	/*
	 * ary_count[0] -> 較慢迴圈的計數器
	 * ary_count[1] -> 預計增加的 trip 數(單位 KM)
	 * ary_count[2] -> 目標 RPM
	 * ary_count[3] -> 目標 RPM 的比對值
	 * ary_count[4] -> 目標 Speed(KPH)
	 * ary_count[5] -> 目標 Speed 的比對值
	 * ary_count[6] -> 配合 RPM & Speed 使用的 "延遲" counter
	 */

	if (ary_count[0] == 0)							// *** 需要送出 "可較慢/少" 的 CAN messages?
	{
		if (Serial.available())
		{
			char ary_str[8];
			uint8_t idx = 0;

			ary_str[idx] = Serial.read();

			if (ary_str[idx] == 'T')				// *** temperture?
			{
				while (Serial.available())
				{
					ary_str[idx++] = Serial.read();
					ary_str[idx] = '\0';
				}

				ary_MIL[0] = atoi(ary_str);

				DP("temperture: ");
				DPL(ary_MIL[0]);
			}
			else if (toupper(ary_str[idx]) == 'I')			// *** Trip distance?
			{
				while (Serial.available())
				{
					ary_str[idx++] = Serial.read();
					ary_str[idx] = '\0';
				}

				ary_count[1] = atoi(ary_str);

				DP("Trip distance: ");
				DP(ary_count[1]);

				ary_count[1] = ary_count[1] * 2570;

				DP("KM, count: ");
				DPL(ary_count[1]);
			}
			else if (toupper(ary_str[idx]) == 'N')			// *** Check engine warning?
			{
				ary_str[0] = Serial.read();

				switch (ary_str[0])
				{
					default :
					case '0' :
						ary_MIL[5] = 0;
						break;

					case '1' :
						ary_MIL[5] = 0b01000000;
						break;

					case '2' :
						ary_MIL[5] = 0b10000000;
						break;
				}

				DP("Check engine warning: ");
				DPL(ary_MIL[5], BIN);
			}
			else if (toupper(ary_str[idx]) == 'O')			// *** Oil Pressure?
			{
				if (ary_str[idx] == 'o')
				{
					ary_MIL[4] = 1;
					ary_MIL[6] &= 0b01111111;
				}
				else
				{
					ary_MIL[4] = 0;
					ary_MIL[6] |= 0b10000000;
				}

				DP("Oil Pressure: ");
				DP(ary_MIL[4], HEX);
				DP(", ");
				DPL(ary_MIL[6], HEX);
			}
			else if (toupper(ary_str[idx]) == 'R')			// *** Bat charge warning?
			{
				if (ary_str[idx] == 'r')
				{
					ary_MIL[6] &= 0b10111111;
				}
				else
				{
					ary_MIL[6] |= 0b01000000;
				}

				DP("Bat charge warning: ");
				DPL(ary_MIL[6], BIN);
			}
			else if (toupper(ary_str[idx]) == 'W')			// *** Low water warning?
			{
				if (ary_str[idx] == 'w')
				{
					ary_MIL[6] &= 0b11111101;
				}
				else
				{
					ary_MIL[6] |= 0b00000010;
				}

				DP("Low water warning: ");
				DPL(ary_MIL[6], BIN);
			}
			else if (toupper(ary_str[idx]) == 'E')			// *** ETC status?
			{
				ary_DSC[6] = ary_str[idx] == 'e' ? 0b00000100 : 0b00001000;

				DP("ETC status: ");
				DPL(ary_DSC[6], BIN);
			}
			else if (toupper(ary_str[idx]) == 'B')			// *** Brake warning?
			{
				if (ary_str[idx] == 'b')
				{
					ary_DSC[4] &= 0b10111111;
				}
				else
				{
					ary_DSC[4] |= 0b01000000;
				}

				DP("Brake warning: ");
				DPL(ary_DSC[4], BIN);
			}
			else if (toupper(ary_str[idx]) == 'A')			// *** ABS warning?
			{
				if (ary_str[idx] == 'a')
				{
					ary_DSC[4] &= 0b11110111;
				}
				else
				{
					ary_DSC[4] |= 0b00001000;
				}

				DP("ABS warning: ");
				DPL(ary_DSC[4], BIN);
			}
			else if (toupper(ary_str[idx]) == 'D')			// *** DSC & TCS warning?
			{
				if (ary_str[idx] == 'D')
				{
					ary_DSC[3] = 0;
					ary_DSC[5] = 0b00000000;
				}
				else
				{
					ary_DSC[3] = 0x34;
					ary_DSC[5] = 0x40;
				}

				DP("DSC & TCS warning: ");
				DP(ary_DSC[3], HEX);
				DP(", ");
				DPL(ary_DSC[5], HEX);
			}
			else if (toupper(ary_str[idx]) == 'P')			// *** RPM?
			{
				while (Serial.available())
				{
					ary_str[idx++] = Serial.read();
					ary_str[idx] = '\0';
				}

				ary_count[2] = atoi(ary_str);
				ary_count[6] = 300;

				DP("Target RPM: ");
				DPL(ary_count[2]);
			}
			else if (toupper(ary_str[idx]) == 'S')			// *** Speed?
			{
				while (Serial.available())
				{
					ary_str[idx++] = Serial.read();
					ary_str[idx] = '\0';
				}

				ary_count[4] = atoi(ary_str);
				ary_count[6] = 80;

				DP("Target Speed: ");
				DPL(ary_count[4]);
			}
		}

//		msg_tx.id = 0x300;
//		memcpy(msg_tx.buf, &ary_STR, 8);
//		CANbus.write(msg_tx);
//
//		if (ary_STR[0] == 0x33)
//		{
//			ary_STR[0] = 0;
//			ary_STR[1] = 0;
//			ary_STR[2] = 0xff;
//			ary_STR[3] = 0xff;
//			ary_STR[4] = 0x27;
//			ary_STR[5] = 0x10;
//			ary_STR[6] = 0xc0;
//			ary_STR[7] = 0x01;
//		}

		msg_tx.id = 0x200;
		memcpy(msg_tx.buf, &ary_EPS, 8);
		CANbus.write(msg_tx);

		msg_tx.id = 0x202;
		memcpy(msg_tx.buf, &ary_CY, 8);
		CANbus.write(msg_tx);

		msg_tx.id = 0x212;
		memcpy(msg_tx.buf, &ary_DSC, 8);
		CANbus.write(msg_tx);

		msg_tx.id = 0x215;
		memcpy(msg_tx.buf, &ary_RAN, 8);
		CANbus.write(msg_tx);

		msg_tx.id = 0x231;
		memcpy(msg_tx.buf, &ary_ECU, 8);
		CANbus.write(msg_tx);

		msg_tx.id = 0x240;
		memcpy(msg_tx.buf, &ary_PCM, 8);
		CANbus.write(msg_tx);

		msg_tx.id = 0x250;
		memcpy(msg_tx.buf, &ary_TCP, 8);
		CANbus.write(msg_tx);

		msg_tx.id = 0x420;
		memcpy(msg_tx.buf, &ary_MIL, 8);
		CANbus.write(msg_tx);

		msg_tx.id = 0x430;
		memcpy(msg_tx.buf, &ary_FL, 8);
		CANbus.write(msg_tx);

		if (ary_count[1] > 0)						// *** 有設定預計增加的 trip 數(單位 KM)?
		{
			/*
			 * 根據實驗:
			 * -每送出 257 次 1 即增加 100M 的距離
			 * -每送出 43 次 10 即增加 100M 的距離
			 * -每送出 22 次 20 即增加 100M 的距離
			 * ->每單位 0.23M
			 *
			 * 似乎 ary_MIL[1] 並非 1 單位是 0.23M 的樣子...
			 * 依照每送出 257 次 1 即增加 100M 的距離這個方式計算, 每單位應為 0.389105058M
			 */
			ary_MIL[1] = ary_MIL[1] + 1;

			DP("Trip distance count1: ");
			DP(ary_count[1]);
			DP(", count2: ");
			DPL(ary_MIL[1]);

			ary_count[1] =  ary_count[1] - 1;
		}

		digitalWrite(D_P_LED, !digitalRead(D_P_LED));
	}

	if ((ary_count[0] = ary_count[0] + 1) > 30)
	{
		ary_count[0] = 0;		
	}

	if (ary_count[2] > 0							// *** 指定 RPM?
		|| (G_eng.rpm_alive && G_eng.sync_whl))
	{
		if (G_eng.rpm_alive)
		{
			uint32_t tmp_wheel_ts0 = G_wheel.ts_to0;
			uint32_t tmp_wheel_ts1 = G_wheel.ts_to1;

			G_rpm_raw = 60000000 / (tmp_wheel_ts0 - tmp_wheel_ts1);

			if (G_rpm_raw > 10000)					// 測到的轉速 > 10,000 RPM?
			{
				G_rpm_raw = 10000;
			}

			G_rpm = map_rpm(G_rpm_raw);				// 計算/對應目前的引擎轉速

			if (ary_count[0] % 6 == 0)
			{
				lcd_strxy(dtostrf(G_rpm_raw, 6, 0, G_ary_buf), 0, 0);
				lcd_strxy(dtostrf(G_rpm, 6, 0, G_ary_buf), 0, 1);
			}
		}
		else
		{
			G_rpm = map_rpm(ary_count[3]);

			ary_count[3] = ary_count[3] + 20;

			if (ary_count[3] > ary_count[2])			// *** 已經超過指定轉速值?
			{
				ary_count[2] = 0;
				ary_count[3] = 1;
			}
		}

		ary_RPM[0] = (G_rpm * 4) / 256;
		ary_RPM[1] = (G_rpm * 4) % 256;
	}
	else if (ary_count[3] == 1 && ary_count[6] > 0)
	{
		ary_count[6] = ary_count[6] - 1;

		if (ary_count[6] <= 0)
		{
			ary_count[3] = 0;
			ary_count[6] = 0;
			ary_RPM[0] = 0;
			ary_RPM[1] = 0;
		}
	}

	if (ary_count[4] > 0)							// *** 指定 Speed?
	{
		/*
		 * 根據實際的測試, RX8 的時速會比實際的多出 1 ~ 2 KMPH
		 */

		ary_RPM[4] = (ary_count[5] * 100 + 10000) / 256;
		ary_RPM[5] = (ary_count[5] * 100 + 10000) % 256;

		ary_count[5] = ary_count[5] + 1;

		if (ary_count[5] > ary_count[4])				// *** 已經超過指定 Speed?
		{
			ary_count[4] = 0;
			ary_count[5] = 1;
		}
	}
	else if (ary_count[5] == 1 && ary_count[6] > 0)
	{
		ary_count[6] = ary_count[6] - 1;

		if (ary_count[6] <= 0)
		{
			ary_count[5] = 0;
			ary_count[6] = 0;
			ary_RPM[4] = 0;
			ary_RPM[5] = 0;
		}
	}

	msg_tx.id = 0x201;
	memcpy(msg_tx.buf, &ary_RPM, 8);
	CANbus.write(msg_tx);

#ifdef DEBUG
//	if (CANbus.read(msg_rx))
//	{
//		hexDump(sizeof(msg_rx), (uint8_t *)&msg_rx);
//	}
#endif

	delay(10);
}

/** **************************************************************************************************************
 * 回傳查表後的 RPM 值
 */
uint16_t map_rpm(uint16_t rpm)
{
	int idx = 38 - 2;

	if (rpm < ary_map_RPM[0] || rpm > ary_map_RPM[36])			// *** RPM too low/high?
		return rpm;

	for (int x = 0; x < idx; x += 2)
	{
		if (rpm >= ary_map_RPM[x]					// *** Interpolate the lookup
			&& rpm <= ary_map_RPM[x + 2])
		{
			return (rpm - ary_map_RPM[x]) * (ary_map_RPM[x + 3] - ary_map_RPM[x + 1]) / (ary_map_RPM[x + 2] - ary_map_RPM[x]) + ary_map_RPM[x + 1];
		}
	}
	
	return 0;
}

/** **************************************************************************************************************
 * 計算 crankshaft angle & RPM
 */
void ISR_vr_puc(void)
{
	G_wheel.ts0 = micros();

	if (G_wheel.ts1 == 0)							// *** 第 1 次偵測到訊號?
	{
		G_wheel.ts1 = G_wheel.ts0;

		return;
	}

	G_wheel.gap = G_wheel.ts0 - G_wheel.ts1;

	if (G_wheel.gap < G_wheel.filter)					// *** 異常的訊號?
	{
		DP("ISR_vr_puc(): abnormal signal! ");
		DP(G_wheel.gap);
		DP(" < ");
		DPL(G_wheel.filter);

		return;
	}

	uint8_t tmp_idx = G_wheel.history_idx ? G_wheel.history_idx - 1 : 0;

	/*
	 * ZZR/ZRX 系列的 trigger wheel 與 missing tooth 不同
	 */
	if (G_wheel.gap * 2 < G_wheel.history[tmp_idx])				// *** 即使目前的間隔時間 *2 還是小於最後一次的間隔時間?
	{
		if (!G_eng.sync_whl && G_wheel.ts_to0 > 0)			// *** 尚未設立 wheel 同步旗標?
		{
			G_eng.sync_whl = true;
		}

		if (G_eng.sync_cyl)
		{
			G_wheel.angle_ex = G_wheel.angle_ex ? 0 : 36000;	// 因為 Teensy 3.1 沒有 FPU 的關係, 所以採用這種方式保留小數 2 位
		}

		G_wheel.count = 0;
		G_wheel.ts_to1 = G_wheel.ts_to0;				// 保留 tooth#1 出現的時間 - 計算 RPM 用
		G_wheel.ts_to0 = G_wheel.ts0;
		G_eng.rpm_alive = G_status.time;
	}
	else
	{
		G_wheel.count++;
	}

	G_wheel.ts1 = G_wheel.ts0;
	G_wheel.history[G_wheel.history_idx] = G_wheel.gap;			// 如果 G_wheel.history_idx 溢位就會回到 0 了 XD
	G_wheel.history_idx += 1;
}

#ifdef DEBUG
/** **************************************************************************************************************
 * https://forum.pjrc.com/threads/24720-Teensy-3-1-and-CAN-Bus/page11
 */
static void hexDump(uint8_t dumpLen, uint8_t *bytePtr)
{
	uint8_t working;
	
	while (dumpLen--)
	{
		working = *bytePtr++;
		Serial.write(hex[working >> 4]);
		Serial.write(hex[working & 15]);
	}
	
	Serial.write('\r');
	Serial.write('\n');
}
#endif

/* ************************************************************************************************************** */
