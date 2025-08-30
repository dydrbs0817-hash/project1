/*
 * PMIC.c
 *
 *  Created on: Aug 19, 2025
 *      Author: dydrb
 */

#include "PMIC.h"
#include <string.h>

extern void UART4_Log(const char *fmt, ...);
#define PMIC_LOG(...)  UART4_Log(__VA_ARGS__)

// 저수준 I2C 유틸(폴링)
// 단일 바이트/8바이트 read/write
HAL_StatusTypeDef PMIC_Read1(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *out)
{
  return HAL_I2C_Mem_Read(hi2c, PMIC_I2C_ADDR, reg,
                          I2C_MEMADD_SIZE_8BIT, out, 1, 100);
}

HAL_StatusTypeDef PMIC_Write1(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value)
{
  return HAL_I2C_Mem_Write(hi2c, PMIC_I2C_ADDR, reg,
                           I2C_MEMADD_SIZE_8BIT, &value, 1, 20);
}

HAL_StatusTypeDef PMIC_Read8(I2C_HandleTypeDef *hi2c, PMIC_RegAddr_t reg, PMIC_Fault8_t *out)
{
  return HAL_I2C_Mem_Read(hi2c, PMIC_I2C_ADDR, (uint16_t)reg,
                          I2C_MEMADD_SIZE_8BIT, out->raw, 8, 100);
}

HAL_StatusTypeDef PMIC_Write8(I2C_HandleTypeDef *hi2c, PMIC_RegAddr_t reg, const PMIC_Fault8_t *in)
{
  return HAL_I2C_Mem_Write(hi2c, PMIC_I2C_ADDR, (uint16_t)reg,
                           I2C_MEMADD_SIZE_8BIT, (uint8_t*)in->raw, 8, 20);
}


// PG 핀 설정/대기
// PG를 입력으로 보정하고, Active 레벨로 연속 stable_count회 OK면 true
void PMIC_ConfigPGasInput(void)
{
  // 기존 설정 제거 후, 입력으로 재초기화
  HAL_GPIO_DeInit(PMIC_PG_GPIO_Port, PMIC_PG_Pin);
  GPIO_InitTypeDef gi = {0};
  gi.Pin   = PMIC_PG_Pin;
  gi.Mode  = GPIO_MODE_INPUT;
  gi.Pull  = GPIO_PULLUP;
  gi.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PMIC_PG_GPIO_Port, &gi);
}

// timeout_ms 동안 1ms 간격으로 측정, stable_count회 연속 OK면 true
bool PMIC_WaitForPG(GPIO_TypeDef *port, uint16_t pin, uint32_t timeout_ms, uint8_t stable_count)
{
  uint32_t t0 = HAL_GetTick();
  uint8_t ok_cnt = 0;
  while ((HAL_GetTick() - t0) < timeout_ms) {
    GPIO_PinState s = HAL_GPIO_ReadPin(port, pin);
    bool ok = PMIC_PG_ACTIVE_HIGH ? (s == GPIO_PIN_SET) : (s == GPIO_PIN_RESET);

    if (ok) {
      if (++ok_cnt >= stable_count) return true;  // 연속 OK 횟수 달성
    } else {
      ok_cnt = 0;                                 // 중간에 깨지면 카운트 리셋
    }
    HAL_Delay(1);                                 // 1ms 샘플 간격
  }
  return false;                                   // 타임아웃
}

// 초기체크 (PG + 디폴트 레지스터)
// PG가 안정적이어야 전압이 원하는 레벨로 정상 동작 시작으로 간주함
bool PMIC_InitialProbe(I2C_HandleTypeDef *hi2c, uint32_t timeout_ms){
    PMIC_ConfigPGasInput();
    if (!PMIC_WaitForPG(PMIC_PG_GPIO_Port, PMIC_PG_Pin, timeout_ms, 3)){
        PMIC_LOG("[PMIC] PG unstable\n");
        return false;
    }

    uint8_t ctrl=0, vsel=0;
    if (PMIC_Read1(hi2c, PMIC_REG_BUCK1_CTRL, &ctrl) != HAL_OK) return false;
    if (PMIC_Read1(hi2c, PMIC_REG_BUCK1_VSEL, &vsel) != HAL_OK) return false;

    /* 실제 프로젝트에선 허용오차/범위 비교를 권장 */
    if (ctrl != PMIC_DEF_BUCK1_CTRL){ PMIC_LOG("CTRL exp=0x%02X got=0x%02X\n", PMIC_DEF_BUCK1_CTRL, ctrl); return false; }
    if (vsel != PMIC_DEF_BUCK1_VSEL){ PMIC_LOG("VSEL exp=0x%02X got=0x%02X\n", PMIC_DEF_BUCK1_VSEL, vsel); return false; }

    PMIC_LOG("[PMIC] InitialProbe OK\n");
    return true;
}





