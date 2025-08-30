/*
 * EEPROM.h
 *
 *  Created on: Aug 19, 2025
 *      Author: dydrb
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_


#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define EEPROM_CMD_READ     0x03U      // Read
#define EEPROM_CMD_WRITE    0x02U      // Write
#define EEPROM_CMD_WREN     0x06U      // Write Enable
#define EEPROM_CMD_RDSR     0x05U      // Read Status Register
#define EEPROM_CMD_RDID     0x9FU      // Read JEDEC ID
#define EEPROM_ADDR_BYTES   2U         // 16-bit 주소 사용하는 칩
#define EEPROM_PAGE_SIZE    64U        // page size
#define EEPROM_NEEDS_WREN   1U         // 1: 쓰기 전 WREN 필요(EEPROM/Flash), 0: FRAM(불필요)
#define EE_SR_WIP           (1u << 0)  // Status Register bit0 : WIP

// 상위 로직(헤더/레코드) 영역 정의
#define EE_HDR_ADDR         0x0000
#define EE_MAGIC            0xA5C3
#define EE_VERSION          0x01

#define DTC_BASE_ADDR       0x0100
#define DTC_SLOT_SIZE       16
#define DTC_SLOT_MAX        64

// DTC 코드(브레이크 관련 PMIC Fault 예시)
typedef enum {
    DTC_PMIC_UV = 0x11,   // Under-Voltage
    DTC_PMIC_OC = 0x12,   // Over-Current
    DTC_PMIC_OT = 0x13    // Over-Temperature
} dtc_pmic_code_t;

// 슬롯 주소 계산 매크로(읽기/쓰기 공통) 
#define DTC_ADDR_OF(idx) ( (uint16_t)(DTC_BASE_ADDR + ((uint16_t)((idx) % DTC_SLOT_MAX)) * (uint16_t)DTC_SLOT_SIZE) )

// 구조체는 1바이트 패킹(플래시/EEPROM에 그대로 저장하기 위함)
#pragma pack(push, 1)
typedef struct {
    uint16_t magic;     // 고정 매직값
    uint8_t  version;   // 데이터 포맷 버전
    uint8_t  rsv;
    uint16_t wr_idx;    // 다음에 쓸 DTC 슬롯 인덱스
    uint16_t crc;       // 헤더 무결성용, 현재 0
} ee_hdr_t;

typedef struct {
    uint32_t ts_ms;     // 기록 시각 (HAL_GetTick)
    uint8_t  pmic_code; // PMIC Fault 코드(외부에서 결정)
    uint8_t  raw[8];    // PMIC 8바이트 스냅샷
    uint16_t crc;       // (선택) 레코드 CRC, 현재 0
    uint8_t  pad;       // 정렬/확장용
} dtc_record_t;
#pragma pack(pop)


// CS 핀
HAL_StatusTypeDef EEPROM_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPort, uint16_t csPin);
HAL_StatusTypeDef EEPROM_ReadStatus(uint8_t *status);	   // 상태 레지스터 1바이트 읽기(WIP/WEL)
// SPI DMA Interrupt 방식
HAL_StatusTypeDef EEPROM_Write_DMA(uint16_t addr, const uint8_t *data, uint16_t len);
HAL_StatusTypeDef EEPROM_Read_DMA(uint16_t addr, uint8_t *data, uint16_t len);

// DMA 완료 시 main.c의 HAL 콜백에서 반드시 호출
void EEPROM_OnTxCpltIRQ(SPI_HandleTypeDef *hspi);
void EEPROM_OnRxCpltIRQ(SPI_HandleTypeDef *hspi);

// 상위 3로직 API
bool EEPROM_Probe(void);                    // 부팅 초기체크(헤더 검증/초기화)
bool EEPROM_LoadSavedData(uint16_t *last_idx); // 기존 wr_idx 복원
// 실제 DTC 레코드 읽기 API
bool DTC_ReadAt(uint16_t idx, dtc_record_t *out);      // 지정 슬롯 읽기
bool DTC_ReadLatest(dtc_record_t *out);                // 가장 최근 저장분 읽기

bool DTC_Save(uint8_t pmic_code, const uint8_t raw8[8]); // PMIC Fault 발생 시 → DTC 저장

#endif /* INC_EEPROM_H_ */
