/*
 * RFM12B_reg.h
 *
 *  Created on: 10-06-2013
 *      Author: Piotr Rzeszut
 *
 * Description: Maski rejestrów uk³adu RFM12B i RFM12
 *              (funkcje specyficzne oznaczone w komentarzach)
 */
#ifndef RFM12B_reg_H
#define RFM12B_reg_H

//#define  (uint16_t)0b0000000000000000

#define CONFIGURATION   (uint16_t)0b1000000000000000
	#define EN_DATA_REG (uint16_t)0b0000000010000000
	#define EN_FIFO     (uint16_t)0b0000000001000000
	//tylko dla RFM12:
	#define BAND_315_r  (uint16_t)0b0000000000000000
	#define BAND_433    (uint16_t)0b0000000000010000
	#define BAND_868    (uint16_t)0b0000000000100000
	#define BAND_915    (uint16_t)0b0000000000110000
	#define CAP_8_5     (uint16_t)0b0000000000000000
	#define CAP_9_0     (uint16_t)0b0000000000000001
	#define CAP_9_5     (uint16_t)0b0000000000000010
	#define CAP_10_0    (uint16_t)0b0000000000000011
	#define CAP_10_5    (uint16_t)0b0000000000000100
	#define CAP_11_0    (uint16_t)0b0000000000000101
	#define CAP_11_5    (uint16_t)0b0000000000000110
	#define CAP_12_0    (uint16_t)0b0000000000000111
	#define CAP_12_5    (uint16_t)0b0000000000001000
	#define CAP_13_0    (uint16_t)0b0000000000001001
	#define CAP_13_5    (uint16_t)0b0000000000001010
	#define CAP_14_0    (uint16_t)0b0000000000001011
	#define CAP_14_5    (uint16_t)0b0000000000001100
	#define CAP_15_0    (uint16_t)0b0000000000001101
	#define CAP_15_5    (uint16_t)0b0000000000001110
	#define CAP_16_0    (uint16_t)0b0000000000001111

#define POWER               (uint16_t)0b1000001000000000
	#define EN_RX           (uint16_t)0b0000000010000000
	#define EN_BASEBAND     (uint16_t)0b0000000001000000
	#define EN_TRANSMISSION (uint16_t)0b0000000000100000
	#define EN_SYNTH        (uint16_t)0b0000000000010000
	#define EN_OSC          (uint16_t)0b0000000000001000
	#define EN_BOD          (uint16_t)0b0000000000000100
	#define EN_WAKEUP       (uint16_t)0b0000000000000010
	#define DIS_CLKO        (uint16_t)0b0000000000000001

#define FREQUENCY (uint16_t)0b1010000000000000
	//Uwaga! wyliczona wartoœæ musi byæ z zakresu 96-3903
	//tylko dla RFM12:
	#define RF12_FREQUENCY_CALC_315_r(f) (((f)-315000000UL)/2500UL)
	// Calculate the RFM12 register value for a given Frequency at 315MHz in 2.5khz increments
	#define RF12_FREQUENCY_CALC_433(f) (((f)-430000000UL)/2500UL)
	// Calculate the RFM12 register value for a given Frequency at 433MHz in 2.5khz increments
	#define RF12_FREQUENCY_CALC_868(f) (((f)-860000000UL)/5000UL)
	// Calculate the RFM12 register value for a given Frequency at 868MHz in 5.0Khz increments
	#define RF12_FREQUENCY_CALC_915(f) (((f)-900000000UL)/7500UL)
	// Calculate the RFM12 register value for a given Frequency at 915MHz in 7.5Khz increments

#define BAUD            (uint16_t)0b1100011000000000
	#define BAUD_115200 (uint16_t)0b0000000000000010
	#define BAUD_57600  (uint16_t)0b0000000000000101
	#define BAUD_38400  (uint16_t)0b0000000000001000
	#define BAUD_28800  (uint16_t)0b0000000000001011
	#define BAUD_19200  (uint16_t)0b0000000000010001
	#define BAUD_9600   (uint16_t)0b0000000000100011
	#define BAUD_4800   (uint16_t)0b0000000001000111
	#define BAUD_2400   (uint16_t)0b0000000010010001
	#define BAUD_1200   (uint16_t)0b0000000010011110
	//calculate setting for datarates >= 2700 Baud
	#define RF12_BAUD_CALC_HIGH(baud) ((uint8_t)(344828UL/baud)-1)
	//calculate setting for datarates < 2700 Baud
	#define RF12_BAUD_CALC_LOW(baud) ((uint8_t)(43104/baud)-1)

#define RECEIVER       (uint16_t)0b1001000000000000
	#define P20_VDI    (uint16_t)0b0000010000000000
	#define P20_INT    (uint16_t)0b0000000000000000
	#define VDI_FAST   (uint16_t)0b0000000000000000
	#define VDI_MEDIUM (uint16_t)0b0000000100000000
	#define VDI_SLOW   (uint16_t)0b0000001000000000
	#define VDI_ON     (uint16_t)0b0000001100000000
	#define BW400      (uint16_t)0b0000000000100000
	#define BW340      (uint16_t)0b0000000001000000
	#define BW270      (uint16_t)0b0000000001100000
	#define BW200      (uint16_t)0b0000000010000000
	#define BW134      (uint16_t)0b0000000010100000
	#define BW67       (uint16_t)0b0000000011000000
	#define LNA_0      (uint16_t)0b0000000000000000
	#define LNA_6      (uint16_t)0b0000000000001000
	#define LNA_14     (uint16_t)0b0000000000010000
	#define LNA_20     (uint16_t)0b0000000000011000
	#define RSSI_103   (uint16_t)0b0000000000000000
	#define RSSI_97    (uint16_t)0b0000000000000001
	#define RSSI_91    (uint16_t)0b0000000000000010
	#define RSSI_85    (uint16_t)0b0000000000000011
	#define RSSI_79    (uint16_t)0b0000000000000100
	#define RSSI_73    (uint16_t)0b0000000000000101
	#define RSSI_67    (uint16_t)0b0000000000000110
	#define RSSI_61    (uint16_t)0b0000000000000111

#define DATA_FILTER   (uint16_t)0b1100001000101000
	#define AUTO_CR   (uint16_t)0b0000000010000000
	#define FAST_CR   (uint16_t)0b0000000001000000
	#define DIGITAL_F (uint16_t)0b0000000000000000
	#define ANALOG_F  (uint16_t)0b0000000000010000
	#define DQD_0     (uint16_t)0b0000000000000000
	#define DQD_1     (uint16_t)0b0000000000000001
	#define DQD_2     (uint16_t)0b0000000000000010
	#define DQD_3     (uint16_t)0b0000000000000011
	#define DQD_4     (uint16_t)0b0000000000000100
	#define DQD_5     (uint16_t)0b0000000000000101
	#define DQD_6     (uint16_t)0b0000000000000110
	#define DQD_7     (uint16_t)0b0000000000000111

#define FIFO_RST          (uint16_t)0b1100101000000000
	#define FIFO_IT_0     (uint16_t)0b0000000000000000
	#define FIFO_IT_1     (uint16_t)0b0000000000010000
	#define FIFO_IT_2     (uint16_t)0b0000000000100000
	#define FIFO_IT_3     (uint16_t)0b0000000000110000
	#define FIFO_IT_4     (uint16_t)0b0000000001000000
	#define FIFO_IT_5     (uint16_t)0b0000000001010000
	#define FIFO_IT_6     (uint16_t)0b0000000001100000
	#define FIFO_IT_7     (uint16_t)0b0000000001110000
	#define FIFO_IT_8     (uint16_t)0b0000000010000000
	#define FIFO_IT_9     (uint16_t)0b0000000010010000
	#define FIFO_IT_10    (uint16_t)0b0000000010100000
	#define FIFO_IT_11    (uint16_t)0b0000000010110000
	#define FIFO_IT_12    (uint16_t)0b0000000011000000
	#define FIFO_IT_13    (uint16_t)0b0000000011010000
	#define FIFO_IT_14    (uint16_t)0b0000000011100000
	#define FIFO_IT_15    (uint16_t)0b0000000011110000
	#define ONE_B_PATTERN (uint16_t)0b0000000000001000
	#define FIFO_A_FILL   (uint16_t)0b0000000000000100
	#define FIFO_SYNC     (uint16_t)0b0000000000000000
	#define EN_AFT_SYNC   (uint16_t)0b0000000000000010
	#define HS_RST_DIS    (uint16_t)0b0000000000000001

//tylko RFM12B
#define SYNC_PATTERN (uint16_t)0b1100111000000000

#define FIFO_READ (uint16_t)0b1011000000000000

#define AFC                (uint16_t)0b1100010000000000
		#define AUTO_OFF   (uint16_t)0b0000000000000000
		#define ONCE_PON   (uint16_t)0b0000000001000000
		#define KEEP_RX    (uint16_t)0b0000000010000000
		#define KEEP_ALW   (uint16_t)0b0000000011000000
		#define REST_OFF   (uint16_t)0b0000000000000000
		#define REST_HIGH  (uint16_t)0b0000000000010000
		#define REST_MED   (uint16_t)0b0000000000100000
		#define REST_LOW   (uint16_t)0b0000000000110000
		#define ON_STROBE  (uint16_t)0b0000000000001000
		#define FINE_MODE  (uint16_t)0b0000000000000100
		#define EN_FOFFSET (uint16_t)0b0000000000000010
		#define EN_AFC     (uint16_t)0b0000000000000001

#define TRANSMITER      (uint16_t)0b1001100000000000
	#define FSK_PHASE_1 (uint16_t)0b0000000100000000
	#define FSK_PHASE_0 (uint16_t)0b0000000000000000
	#define FSK_DF_15K  (uint16_t)0b0000000000000000
	#define FSK_DF_30K  (uint16_t)0b0000000000010000
	#define FSK_DF_45K  (uint16_t)0b0000000000100000
	#define FSK_DF_60K  (uint16_t)0b0000000000110000
	#define FSK_DF_75K  (uint16_t)0b0000000001000000
	#define FSK_DF_90K  (uint16_t)0b0000000001010000
	#define FSK_DF_105K (uint16_t)0b0000000001100000
	#define FSK_DF_120K (uint16_t)0b0000000001110000
	#define FSK_DF_135K (uint16_t)0b0000000010000000
	#define FSK_DF_150K (uint16_t)0b0000000010010000
	#define FSK_DF_165K (uint16_t)0b0000000010100000
	#define FSK_DF_180K (uint16_t)0b0000000010110000
	#define FSK_DF_195K (uint16_t)0b0000000011000000
	#define FSK_DF_210K (uint16_t)0b0000000011010000
	#define FSK_DF_225K (uint16_t)0b0000000011100000
	#define FSK_DF_240K (uint16_t)0b0000000011110000
	#define OUT_P_0     (uint16_t)0b0000000000000000
	#define OUT_P_3     (uint16_t)0b0000000000000001
	#define OUT_P_6     (uint16_t)0b0000000000000010
	#define OUT_P_9     (uint16_t)0b0000000000000011
	#define OUT_P_12    (uint16_t)0b0000000000000100
	#define OUT_P_15    (uint16_t)0b0000000000000101
	#define OUT_P_18    (uint16_t)0b0000000000000110
	#define OUT_P_21    (uint16_t)0b0000000000000111

#define PLL               (uint16_t)0b1100110000010010
	#define SEL_CLK_5_10  (uint16_t)0b0000000001100000
	#define SEL_CLK_3_3   (uint16_t)0b0000000001000000
	#define SEL_CLK_2_5   (uint16_t)0b0000000000000000
	#define PLL_DELAY_DIS (uint16_t)0b0000000000001000
	#define PLL_DH_DIS    (uint16_t)0b0000000000000100
	#define MAX_BAUD_256  (uint16_t)0b0000000000000001
	#define MAX_BAUD_86_2 (uint16_t)0b0000000000000000

#define TX_WRITE (uint16_t)0b1011100000000000

#define WAKEUP_TIM     (uint16_t)0b1110000000000000
	#define WUT_X_2_0  (uint16_t)0b0000000000000000
	#define WUT_X_2_1  (uint16_t)0b0000000100000000
	#define WUT_X_2_2  (uint16_t)0b0000001000000000
	#define WUT_X_2_3  (uint16_t)0b0000001100000000
	#define WUT_X_2_4  (uint16_t)0b0000010000000000
	#define WUT_X_2_5  (uint16_t)0b0000010100000000
	#define WUT_X_2_6  (uint16_t)0b0000011000000000
	#define WUT_X_2_7  (uint16_t)0b0000011100000000
	#define WUT_X_2_8  (uint16_t)0b0000100000000000
	#define WUT_X_2_9  (uint16_t)0b0000100100000000
	#define WUT_X_2_10 (uint16_t)0b0000101000000000
	#define WUT_X_2_11 (uint16_t)0b0000101100000000
	#define WUT_X_2_12 (uint16_t)0b0000110000000000
	#define WUT_X_2_13 (uint16_t)0b0000110100000000
	#define WUT_X_2_14 (uint16_t)0b0000111000000000
	#define WUT_X_2_15 (uint16_t)0b0000111100000000
	#define WUT_X_2_16 (uint16_t)0b0001000000000000
	#define WUT_X_2_17 (uint16_t)0b0001000100000000
	#define WUT_X_2_18 (uint16_t)0b0001001000000000
	#define WUT_X_2_19 (uint16_t)0b0001001100000000
	#define WUT_X_2_20 (uint16_t)0b0001010000000000
	#define WUT_X_2_21 (uint16_t)0b0001010100000000
	#define WUT_X_2_22 (uint16_t)0b0001011000000000
	#define WUT_X_2_23 (uint16_t)0b0001011100000000
	#define WUT_X_2_24 (uint16_t)0b0001100000000000
	#define WUT_X_2_25 (uint16_t)0b0001100100000000
	#define WUT_X_2_26 (uint16_t)0b0001101000000000
	#define WUT_X_2_27 (uint16_t)0b0001101100000000
	#define WUT_X_2_28 (uint16_t)0b0001110000000000
	#define WUT_X_2_29 (uint16_t)0b0001110100000000

#define LOW_DC         (uint16_t)0b1100100000000000
	#define LOW_DC_EN  (uint16_t)0b0000000000000001
	#define LOW_DC_DIS (uint16_t)0b0000000000000000
	#define LOW_DC_CALC(dc) ((uint16_t)((dc)*2UL))

#define BOD_CLK      (uint16_t)0b1100000000000000
	#define CLK_1_00 (uint16_t)0b0000000000000000
	#define CLK_1_25 (uint16_t)0b0000000000100000
	#define CLK_1_66 (uint16_t)0b0000000001000000
	#define CLK_2_00 (uint16_t)0b0000000001100000
	#define CLK_2_50 (uint16_t)0b0000000010000000
	#define CLK_3_33 (uint16_t)0b0000000010100000
	#define CLK_5_00 (uint16_t)0b0000000011000000
	#define CLK_10_0 (uint16_t)0b0000000011100000
	#define BOD_2_2  (uint16_t)0b0000000000000000
	#define BOD_2_3  (uint16_t)0b0000000000000001
	#define BOD_2_4  (uint16_t)0b0000000000000010
	#define BOD_2_5  (uint16_t)0b0000000000000011
	#define BOD_2_6  (uint16_t)0b0000000000000100
	#define BOD_2_7  (uint16_t)0b0000000000000101
	#define BOD_2_8  (uint16_t)0b0000000000000110
	#define BOD_2_9  (uint16_t)0b0000000000000111
	#define BOD_3_0  (uint16_t)0b0000000000001000
	#define BOD_3_1  (uint16_t)0b0000000000001001
	#define BOD_3_2  (uint16_t)0b0000000000001010
	#define BOD_3_3  (uint16_t)0b0000000000001011
	#define BOD_3_4  (uint16_t)0b0000000000001100
	#define BOD_3_5  (uint16_t)0b0000000000001101
	#define BOD_3_6  (uint16_t)0b0000000000001110
	#define BOD_3_7  (uint16_t)0b0000000000001111
	#define BOD_3_8  (uint16_t)0b0000000000010000
	#define BOD_3_9  (uint16_t)0b0000000000010001
	#define BOD_4_0  (uint16_t)0b0000000000010010
	#define BOD_4_1  (uint16_t)0b0000000000010011
	#define BOD_4_2  (uint16_t)0b0000000000010100
	#define BOD_4_3  (uint16_t)0b0000000000010101
	#define BOD_4_4  (uint16_t)0b0000000000010110
	#define BOD_4_5  (uint16_t)0b0000000000010111
	#define BOD_4_6  (uint16_t)0b0000000000011000
	#define BOD_4_7  (uint16_t)0b0000000000011001
	#define BOD_4_8  (uint16_t)0b0000000000011010
	#define BOD_4_9  (uint16_t)0b0000000000011011
	#define BOD_5_0  (uint16_t)0b0000000000011100
	#define BOD_5_1  (uint16_t)0b0000000000011101
	#define BOD_5_2  (uint16_t)0b0000000000011110
	#define BOD_5_3  (uint16_t)0b0000000000011111

#define STATUS_READ     (uint16_t)0b0000000000000000
	//maski do dekodowania odebranych danych
	#define M_TX_READY  (uint16_t)0b1000000000000000
	#define M_FIFO_IT   (uint16_t)0b1000000000000000
	#define M_POR       (uint16_t)0b0100000000000000
	#define M_TX_OVF    (uint16_t)0b0010000000000000
	#define M_FIFO_OVF  (uint16_t)0b0010000000000000
	#define M_WKUP_OVF  (uint16_t)0b0001000000000000
	#define M_EXT_INT   (uint16_t)0b0000100000000000
	#define M_BOD       (uint16_t)0b0000010000000000
	#define M_FIFO_EMP  (uint16_t)0b0000001000000000
	#define M_ATS_OK    (uint16_t)0b0000000100000000
	#define M_RSSI_OK   (uint16_t)0b0000000100000000
	#define M_DQD_OK    (uint16_t)0b0000000010000000
	#define M_CLK_RECOV (uint16_t)0b0000000001000000
	#define M_ACF_TGL   (uint16_t)0b0000000000100000
	#define M_F_OFS_SIG (uint16_t)0b0000000000010000
	#define M_F_OFS     (uint16_t)0b0000000000001111

#define SW_RESET (uint16_t)0b1111111000000000

#endif //RFM12B_reg_H













