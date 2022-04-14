#ifndef _CONFIG_F0_N32_H
#define _CONFIG_F0_N32_H

#define PROJECT_ID  (1)


#define DEBUG_EN    (1)

#define FW_VERSION  (2)
#define HW_VERSION  (1)
#define BAT_TYPE  (2)
//#define BAT_TYPE2 (2000)

#define NIU_FW_VER  "F0D10V01"
#define NIU_HW_VER  "V1.0"
#define COMPLIE_TIME "2022-3-1"
//天能内部软件版本号
#define TN_FW_VER  "1.0.0"

#ifndef MCU_LIB_SELECT
#define MCU_LIB_SELECT (2)
#endif

#define USE_485_IF    (0)

#if (USE_485_IF != 1)
#define ONEBUS_TYPE  (2)
#endif

#define USE_SIM_AFEDATA  (0)
#define AFE_CHIP_SELECT (1)

#define UNIT_TEST_EN   (1)
#define CELL_NUMS  (15)

#define BAT_RATED_VOLT  (48)
#define LITHIUM_TYPE  (2)
#define LITHIUM_LOW_POWER_E (1600)
#define LITHIUM_CYCLE_E  (2000)
#define LITHIUM_FULL_AH  (16)
#define SOCSUMCALC_INTVAL (1000)
#define EN_OUTDATA_PROTECT 0

#if(PROJECT_ID == 2)
#define AFE_CUR_SAMPLE_RES   (0.0005)
#else
#define AFE_CUR_SAMPLE_RES   (0.001)
#endif

#define DEFAULT_CHARG_DEMAND_VOLT   (5460)
#define DEFAULT_CHARG_DEMAND_CURR   (300)
#define DEFAULT_BACKCHG_CURR       (400)
#define DEFAULT_DSG_CURR           (300)

#define CHG_FULL_OCV_TH  (3600)

#define MAXCELL_FULL_SLEEP_TH  (3600)
#define SHIP_MODE_CELLV_TH  (2500)
#define CELL_VMAX_OVER_TH  (4000)

#define VDIFF_WARN  (300)


#define BALANCE_START_V  (3800)
#define BALANCE_START_S  (50)
#define BALANCE_START_E  (500)

#define DEFAULT_RATIO_V_K   (1000)
#define DEFAULT_RATIO_V_O   (0)
#define DEFAULT_RATIO_C_K   (1000)
#define DEFAULT_RATIO_C_O   (0)


#if(PROJECT_ID == 1)
//#define OC_VLT_P  (4250)
//#define ROC_VLT_P (4100)
//#define DC_VLT_P  (3000)
//#define RDC_VLT_P (3300)
//#define DC_CUR_P  (40)
//#define ISC_P     (230)
//#define C_CUR_P   (10)
//#define COTP_P    (55)
//#define RCOTP_P   (50)
//#define DCOTP_P   (70)
//#define RDCOTP_P  (60)
//#define DCLTP_P   (-20)
//#define RDCLTP_P  (-15)
//#define CCLTP_P   (0)
//#define RCCLTP_P   (4)
//#define MOSOTP_P  (85)
//#define RMOSOTP_P (75)
//#define MOSOTP_W  (80)

#define OC_VLT_P  (3650) //过压保护
#define ROC_VLT_P (3450) //过压恢复
#define DC_VLT_P  (2300) //欠压保护
#define RDC_VLT_P (2800) //欠压恢复
#define DC_CUR_P  (40)   //放电过流
#define ISC_P     (280)  //短路保护
#define C_CUR_P   (20)   //充电过流保护
#define COTP_P    (55)   //充电温度保护
#define RCOTP_P   (50)   //充电温度恢复
#define DCOTP_P   (70)   //放电高温保护
#define RDCOTP_P  (60)   //放电高温恢复
#define DCLTP_P   (-25)  //放电低温保护
#define RDCLTP_P  (-20)  //放电低温恢复
#define CCLTP_P   (-5)    //充电低温保护
#define RCCLTP_P   (0)   //充电低温恢复
#define MOSOTP_P  (90)   //MOS高温保护
#define RMOSOTP_P (85)   //MOS高温恢复
#define MOSOTP_W  (80)



#define OC_VLT_DELAY_S (2000)
#define OC_VLT_DELAY_H (4000)
#define DC_VLT_DELAY_S (5000)
#define DC_VLT_DELAY_H (6000)
#define ISC_DELAY_H    (256)   //us
#define DC_DELAY_S     (5000)
#define RDC_DELAY_S    (30000)
#define DC_DELAY_H     (600)
#define CC_DELAY_S     (15000)
#define RCC_DELAY_S    (30000)
#define CC_DELAY_H     (8000)
#define COT_DELAY_S    (4000)
#define DCOT_DELAY_S   (4000)
#define DCLT_DELAY_S   (4000)
#define CCLT_DELAY_S   (4000)
#define MOSOT_DELAY_S  (5000)
#elif(PROJECT_ID == 2)
#define OC_VLT_P  (4250)
#define ROC_VLT_P (4050)
#define DC_VLT_P  (3000)
#define RDC_VLT_P (3300)
#define DC_CUR_P  (75)
#define ISC_P     (400)
#define C_CUR_P   (20)
#define COTP_P    (55)
#define RCOTP_P   (50)
#define DCOTP_P   (70)
#define RDCOTP_P  (60)
#define DCLTP_P   (-20)
#define RDCLTP_P  (-15)
#define CCLTP_P   (0)
#define RCCLTP_P   (4)
#define MOSOTP_P  (85)
#define RMOSOTP_P (75)
#define MOSOTP_W  (80)



#define OC_VLT_DELAY_S (2000)
#define OC_VLT_DELAY_H (4000)
#define DC_VLT_DELAY_S (4000)
#define DC_VLT_DELAY_H (6000)
#define ISC_DELAY_H    (256)   //us
#define DC_DELAY_S     (5000)
#define RDC_DELAY_S    (30000)
#define DC_DELAY_H     (600)
#define CC_DELAY_S     (15000)
#define RCC_DELAY_S    (30000)
#define CC_DELAY_H     (8000)
#define COT_DELAY_S    (4000)
#define DCOT_DELAY_S   (4000)
#define DCLT_DELAY_S   (4000)
#define CCLT_DELAY_S   (4000)
#define MOSOT_DELAY_S  (5000)
#endif


#define EE_START_ADDR (FLASH_BASE_ADDR+0x70000)//(FLASH_BASE_ADDR+0x1FF00)
#define BAT_SN_ADDR_START (EE_START_ADDR)
#define SH_EEPROM_ADDR (EE_START_ADDR+0x100)
#define SHG_STATE_ADDR (EE_START_ADDR+0x200)
#define SOC_ENGINEE_ADDR (EE_START_ADDR+0x300)
#define BATCYCLE_COUNT_ADDR (EE_START_ADDR+0x400)
#define SOC_INIT_CAPACITY_ADDR (EE_START_ADDR+0x500)
#define CALI_RCH_TRIM_VAL_ADDR (EE_START_ADDR+0x600)
#define CALI_CURR_PARA_VAL  (EE_START_ADDR+0x700)
#define ALG_PARAM_START      (EE_START_ADDR+0x800)



#if (MCU_LIB_SELECT == 1)
#define FLASH_BASE_ADDR (0)
#elif (MCU_LIB_SELECT == 2)
#define FLASH_BASE_ADDR (0)
#endif


#if(AFE_CHIP_SELECT == 2)

#define AFE_VOL_THRESHOLD_UP      4.200
#define AFE_VOL_THRESHOLD_DOWN    3.000
#define AFE_VOL_THRESHOLD_MIN     2.950
#define AFE_CUR_ZERO_THRESHOLD    0.5


#define AFE_TEMPER_THRESHOLD_UP		(70.0)
#define AFE_TEMPER_THRESHOLD_DOWN 	(-20.0)

//threshold setting
#define AFE_OVT  (4250)  //over voltage
#define AFE_OVRT (4150)  //over voltage resume
#define AFE_UVT  (2800)  //under voltage 
#define AFE_UVRT (4000)  //under voltage resume
#define AFE_OCCT (30)    //charge over current A,fixed
#define AFE_OCDT (50)    //discharge over current A,fixed
#define AFE_SCDT (200)   //short discharge over current A,fixed

#define AFE_OTC  (60)    //charge over temperature
#define AFE_OTCR (55)    //charge over resume Temperature
#define AFE_UTC  (-10)   //charge under Temperature
#define AFE_UTCR (-5)    //charge under resume temperature
#define AFE_OTD  (65)    //discharge over temperature
#define AFE_OTDR (60)    //discharge over resume Temperature
#define AFE_UTD  (-20)   //under temperature for discharge
#define AFE_UTDR (-10)   //under temperature for discharge resume

#define AFE_CFG1_DEF_VAL0   (0x75) //charge over 40A  delay 1280ms                   
#define AFE_CFG1_DEF_VAL1   (0x77) //discharge over 1280ms, 54A
#define AFE_CFG1_DEF_VAL2   (0x2f) //scd delay 400us  200A
#define AFE_CFG1_DEF_VAL3   (0x21) //ovd1d 4s,uv1d 4s
#define AFE_CFG1_DEF_VAL4   ((uint8_t)(AFE_OVT/5.86 -546))//OV1T
#define AFE_CFG1_DEF_VAL5   ((uint8_t)(AFE_UVT/5.86 -546))//UV1T
#define AFE_CFG1_DEF_VAL6   0x00
#define AFE_CFG1_DEF_VAL7   0x00
#define AFE_CFG1_DEF_VAL8   0x00
#define AFE_CFG1_DEF_VAL9   (0x21)
#define AFE_CFG1_DEF_VAL10  ((uint8_t)(AFE_OVT/5.86 -546)) //OV2T
#define AFE_CFG1_DEF_VAL11  ((uint8_t)(AFE_UVT/5.86 -546)) //UV2T
#define AFE_CFG1_DEF_VAL12  0x00
#define AFE_CFG1_DEF_VAL13  0x00
#define AFE_CFG1_DEF_VAL14  0x00
#define AFE_CFG1_DEF_VAL15  (0x21)
#define AFE_CFG1_DEF_VAL16  ((uint8_t)(AFE_OVT/5.86 -546)) //OV3T
#define AFE_CFG1_DEF_VAL17  ((uint8_t)(AFE_UVT/5.86 -546)) //UV3T
#define AFE_CFG1_DEF_VAL18  0x00
#define AFE_CFG1_DEF_VAL19  0x00
#define AFE_CFG1_DEF_VAL20  0x00
#define AFE_CFG1_DEF_VAL21  (0x21)
#define AFE_CFG1_DEF_VAL22  ((uint8_t)(AFE_OVT/5.86 -546)) //OV4T
#define AFE_CFG1_DEF_VAL23  ((uint8_t)(AFE_UVT/5.86 -546)) //UV4T


#define AFE_CFG0_DEF_VAL0   0xfc
#define AFE_CFG0_DEF_VAL1   0x00
#define AFE_CFG0_DEF_VAL2   0x00
#define AFE_CFG0_DEF_VAL3   0xc0
#define AFE_CFG0_DEF_VAL4   0x88
#define AFE_CFG0_DEF_VAL5   0x07
#define AFE_CFG0_DEF_VAL6   0xfc
#define AFE_CFG0_DEF_VAL7   0x00
#define AFE_CFG0_DEF_VAL8   0x00
#define AFE_CFG0_DEF_VAL9   0xc0
#define AFE_CFG0_DEF_VAL10  0x88
#define AFE_CFG0_DEF_VAL11  0x07
#define AFE_CFG0_DEF_VAL12  0xfc
#define AFE_CFG0_DEF_VAL13  0x00
#define AFE_CFG0_DEF_VAL14  0x00
#define AFE_CFG0_DEF_VAL15  0xc0
#define AFE_CFG0_DEF_VAL16  0x88
#define AFE_CFG0_DEF_VAL17  0x07
#define AFE_CFG0_DEF_VAL18  0xfc
#define AFE_CFG0_DEF_VAL19  0x00
#define AFE_CFG0_DEF_VAL20  0x00
#define AFE_CFG0_DEF_VAL21  0xc0
#define AFE_CFG0_DEF_VAL22  0x88
#define AFE_CFG0_DEF_VAL23  0x07

#endif

#endif
