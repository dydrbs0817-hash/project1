/*
 * PMIC.h
 *
 *  Created on: Aug 19, 2025
 *      Author: dydrb
 */

#ifndef PMIC_H_
#define PMIC_H_
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

#define PMIC_I2C_ADDR_7B         (0x60U)		// 데이터시트 정의 7-bit Slave Address
#define PMIC_I2C_ADDR            (PMIC_I2C_ADDR_7B << 1)

#define PMIC_PG_GPIO_Port        GPIOB	   // PG 핀
#define PMIC_PG_Pin              GPIO_PIN_2
#define PMIC_PG_ACTIVE_HIGH      1         // PG=High가 정상. Active-Low면 0으로.


// PMIC.h
typedef enum {
  PMIC_REG_FAULT_VOLT  = 0x07,  // BUCKx_UV/OV status
  PMIC_REG_FAULT_CURR  = 0x08,  // BUCKx_OC/OC_WARNING status
  PMIC_REG_FAULT_TEMP  = 0x09,  // SYSTEM: temp/global faults

  // (아래는 별도 표에서 필요 시 채움)
  PMIC_REG_CLEAR_FAULT = 0x18,  // TODO: DS의 INT/clear 방식에 맞게 실제 주소로
  PMIC_REG_BUCK1_VSEL  = 0x20,  // TODO
  PMIC_REG_BUCK1_CTRL  = 0x21   // TODO
} PMIC_RegAddr_t;


typedef union {
  uint8_t raw[8];  // 0x07/0x08/0x09를 읽어올 때 공용으로 사용

  struct {
    /* 0x07 결과를 넣어 쓰는 뷰 (전압) */
    uint8_t uv_nibble : 4;  // [7:4] BUCKx_UV  (A/B/C/D 1bit씩)
    uint8_t ov_nibble : 4;  // [3:0] BUCKx_OV

    /* 0x08 결과를 넣어 쓰는 뷰 (전류) */
    uint8_t oc_nibble : 4;  // [7:4] BUCKx_OC
    uint8_t ocw_nibbl : 4;  // [3:0] BUCKx_OC_WARNING

    /* 0x09 SYSTEM 비트 */
    uint8_t sys_bits;       // bit6..0 = LDO/VR/OV/Temp 등

    /* 남은 바이트(미사용) */
    uint8_t rsv3;
    uint8_t rsv4;
    uint8_t rsv5;
    uint8_t rsv6;
    uint8_t rsv7;
  } bits;
} PMIC_Fault8_t;

//BUCK1 전압 파라미터 (data 값을 아직 넣지 않음)
#define PMIC_BUCK1_MIN_mV   300    // BUCK1이 만들수 잇는 최소전압
#define PMIC_BUCK1_MAX_mV   2048   // 최대전압
#define PMIC_BUCK1_STEP_mV  2      // 전압설정시 한번에 움직일 수 있는 스텝이 10mV
#define PMIC_BUCK1_EN_MASK  0x01   // BUCK1 CTRL의 EN 비트가 CTRL 레지스터의 bit0에 있음

// 디폴트 — 초기체크에서 사용
#define PMIC_DEF_BUCK1_CTRL   0x01  // Enable=1 가정
#define PMIC_DEF_BUCK1_VSEL   0x190  // 400(dec)

// 단일 8바이트 레지스터(폴링)
HAL_StatusTypeDef PMIC_Read1  (I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *out);
HAL_StatusTypeDef PMIC_Write1 (I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value);
HAL_StatusTypeDef PMIC_Read8  (I2C_HandleTypeDef *hi2c, PMIC_RegAddr_t reg, PMIC_Fault8_t *out);
HAL_StatusTypeDef PMIC_Write8 (I2C_HandleTypeDef *hi2c, PMIC_RegAddr_t reg, const PMIC_Fault8_t *in);

// PG 핀 설정/대기
void  PMIC_ConfigPGasInput(void);
// timeout_ms 동안 1ms 간격 샘플링, stable_count회 연속 OK면 true
bool  PMIC_WaitForPG(GPIO_TypeDef *port, uint16_t pin, uint32_t timeout_ms, uint8_t stable_count);

// 초기체크 : 전압 정상동작 시작 확인(PG + I2C 디폴트)
bool PMIC_InitialProbe(I2C_HandleTypeDef *hi2c, uint32_t timeout_ms);

// 전압 변경(요구사항5)
HAL_StatusTypeDef PMIC_SetBuck1Voltage_mV(I2C_HandleTypeDef *hi2c, uint16_t mv);

#endif /* PMIC_H_ */
