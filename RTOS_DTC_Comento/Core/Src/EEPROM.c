/*
 * EEPROM.c
 *
 *  Created on: Aug 19, 2025
 *      Author: dydrb
 */

#include "EEPROM.h"
#include <string.h>

// HAL SPT 핸들/CS GPIO 내부 보관 포인터
static SPI_HandleTypeDef *s_hspi = NULL;
static GPIO_TypeDef *s_csPort = NULL;
static uint16_t s_csPin = 0;


// 현재 진행 중인 작업(동시 실행 방지/CS 제어)
typedef enum {
    EE_OP_IDLE = 0,
    EE_OP_WRITE,
    EE_OP_READ
} eeprom_op_t;
static volatile eeprom_op_t s_op = EE_OP_IDLE;

// 타임아웃
#define EE_TIMEOUT_SPI   100u
#define EE_TIMEOUT_WIP    10u


// CS 제어 헬퍼(CS 수동 제어)
static inline void CS_L(void) { HAL_GPIO_WritePin(s_csPort, s_csPin, GPIO_PIN_RESET); }
static inline void CS_H(void) { HAL_GPIO_WritePin(s_csPort, s_csPin, GPIO_PIN_SET); }


// 파라미터/상태 가드
static inline HAL_StatusTypeDef guard_ready(void) {
    if (!s_hspi || !s_csPort) return HAL_ERROR;   // 반드시 EEPROM_Init 먼저 호출되어야 함
    if (s_op != EE_OP_IDLE)   return HAL_BUSY;    // 동작 중인 상태에서 재진입 방지
    return HAL_OK;
}


//EEPROM_Init
HAL_StatusTypeDef EEPROM_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPort, uint16_t csPin) {
    s_hspi = hspi;
    s_csPort = csPort;
    s_csPin = csPin;
    CS_H(); // 비선택
    s_op = EE_OP_IDLE;
    return HAL_OK;
}

// DMA 유휴 대기
static void eeprom_wait_dma_idle(void){
    while (s_op != EE_OP_IDLE) {
    }
}

// 상태 레지스터 1바이트 읽기 (폴링)
// 상태 레지스터가 1바이트이고 거기에 핵심비트(WIP:Write In Progress/WEL:Write Enable Latch)이 다 들어있기 때문
// WIP - 내부 쓰기 동작중 표기 / WEL - 쓰기 허가 래치
// Status Read / Write Enable / WIP wait
HAL_StatusTypeDef EEPROM_ReadStatus(uint8_t *status) {
    if (!status) return HAL_ERROR;
    if (!s_hspi || !s_csPort) return HAL_ERROR;

    uint8_t cmd = EEPROM_CMD_RDSR;
    CS_L();
    if (HAL_SPI_Transmit(s_hspi, &cmd, 1, 100) != HAL_OK) { CS_H(); return HAL_ERROR; }
    if (HAL_SPI_Receive(s_hspi, status, 1, 100) != HAL_OK)   { CS_H(); return HAL_ERROR; }
    CS_H();
    return HAL_OK;
}

// WREN : Write Enable
static HAL_StatusTypeDef eeprom_wren(void) {
    uint8_t cmd = EEPROM_CMD_WREN;
    CS_L(); // Low
    HAL_StatusTypeDef r = HAL_SPI_Transmit(s_hspi, &cmd, 1, 100);
    CS_H(); // High
    return r;
}

// WIP=0(쓰기 완료) 될 때까지 대기. 타임아웃(ms) 지정.
static HAL_StatusTypeDef eeprom_wait_ready(uint32_t timeout_ms){
    uint32_t t0 = HAL_GetTick();
    uint8_t st;
    do {
        if (EEPROM_ReadStatus(&st) != HAL_OK) return HAL_ERROR;
        if ((st & EE_SR_WIP) == 0) return HAL_OK; // WIP=0이면 Ready
    } while (HAL_GetTick() - t0 < timeout_ms);
    return HAL_TIMEOUT;
}

// SPI DMA 전송이 끝났을때 EEPROM 드라이버가 트랜잭션을 종료하고 IDLE 상태로 돌려놓는 훅
void EEPROM_OnTxCpltIRQ(SPI_HandleTypeDef *hspi){
    if (hspi == s_hspi){	// callback이 EEPROM이 쓰는 SPI 핸들에서 온것인지 확인
        CS_H(); 			// DMA 전송 끝났으니 CS를 High(비선택)으로 올려 트랜잭션 종료
        s_op = EE_OP_IDLE;  // IDLE : 아무 작업도 진행중이지 않은 상태
    }
}

void EEPROM_OnRxCpltIRQ(SPI_HandleTypeDef *hspi){
    if (hspi == s_hspi){
        CS_H();
        s_op = EE_OP_IDLE;
    }
}



// Write (명령/주소 Polling + 데이터 DMA)
// 페이지 경계 넘으면 에러 반환
HAL_StatusTypeDef EEPROM_Write_DMA(uint16_t addr, const uint8_t *data, uint16_t len){
    if (!data || len==0 || len>EEPROM_PAGE_SIZE) return HAL_ERROR;
    if (guard_ready()!=HAL_OK) return HAL_BUSY;

    uint16_t off = addr % EEPROM_PAGE_SIZE;
    if (off + len > EEPROM_PAGE_SIZE) return HAL_ERROR; // 경계 초과되면 error

    if (eeprom_wren()!=HAL_OK) return HAL_ERROR;

    uint8_t cmd[3] = { EEPROM_CMD_WRITE, (uint8_t)(addr>>8), (uint8_t)addr };
    CS_L();
    if (HAL_SPI_Transmit(s_hspi, cmd, sizeof(cmd), EE_TIMEOUT_SPI) != HAL_OK){
        CS_H(); return HAL_ERROR;
    }

    s_op = EE_OP_WRITE;
    if (HAL_SPI_Transmit_DMA(s_hspi, (uint8_t*)data, len) != HAL_OK){
        CS_H(); s_op = EE_OP_IDLE; return HAL_ERROR;
    }
    return HAL_OK;
}


// READ (데이터만 DMA + 명령/주소는 Polling)
HAL_StatusTypeDef EEPROM_Read_DMA(uint16_t addr, uint8_t *data, uint16_t len) {
    if (!data || len == 0) return HAL_ERROR;
    if (guard_ready()!=HAL_OK) return HAL_BUSY;

       uint8_t cmd[3] = { EEPROM_CMD_READ, (uint8_t)(addr>>8), (uint8_t)addr };
       CS_L();
       if (HAL_SPI_Transmit(s_hspi, cmd, sizeof(cmd), EE_TIMEOUT_SPI) != HAL_OK){
           CS_H(); return HAL_ERROR;
       }

       s_op = EE_OP_READ;
       if (HAL_SPI_Receive_DMA(s_hspi, data, len) != HAL_OK){
           CS_H(); s_op = EE_OP_IDLE; return HAL_ERROR;
       }
       return HAL_OK;
}


// 부팅 초기체크(헤더 검증/초기화)
bool EEPROM_Probe(void){
    ee_hdr_t hdr = {0};

    (void)eeprom_wait_ready(50); // 이전 WIP 잔여 작업 마무리

    if (EEPROM_Read_DMA(EE_HDR_ADDR, (uint8_t*)&hdr, sizeof(hdr)) != HAL_OK) return false;
    eeprom_wait_dma_idle();

    if (hdr.magic != EE_MAGIC || hdr.version != EE_VERSION){
        memset(&hdr, 0, sizeof(hdr));
        hdr.magic   = EE_MAGIC;
        hdr.version = EE_VERSION;
        hdr.wr_idx  = 0;
        hdr.crc     = 0; // 추후 CRC 구현 가능

        if (eeprom_wren()!=HAL_OK) return false;
        if (EEPROM_Write_DMA(EE_HDR_ADDR, (uint8_t*)&hdr, sizeof(hdr)) != HAL_OK) return false;
        eeprom_wait_dma_idle();
        if (eeprom_wait_ready(EE_TIMEOUT_WIP) != HAL_OK) return false;
    }
    return true;
}

// 기존 저장 데이터 인덱스 로드
bool EEPROM_LoadSavedData(uint16_t *last_idx){
    if (!last_idx) return false;
    ee_hdr_t hdr;

    if (EEPROM_Read_DMA(EE_HDR_ADDR, (uint8_t*)&hdr, sizeof(hdr)) != HAL_OK) return false;
    eeprom_wait_dma_idle();

    if (hdr.magic != EE_MAGIC || hdr.version != EE_VERSION) return false;
    *last_idx = hdr.wr_idx;
    return true;
}


// 실제 DTC 레코드 읽기
bool DTC_ReadAt(uint16_t idx, dtc_record_t *out){
    if (!out) return false;
    uint16_t addr = DTC_ADDR_OF(idx);
    if (EEPROM_Read_DMA(addr, (uint8_t*)out, sizeof(*out)) != HAL_OK) return false;
    eeprom_wait_dma_idle();
    return true;
}

// 최근 Data Read
bool DTC_ReadLatest(dtc_record_t *out){
    if (!out) return false;
    ee_hdr_t hdr;
    if (EEPROM_Read_DMA(EE_HDR_ADDR, (uint8_t*)&hdr, sizeof(hdr)) != HAL_OK) return false;
    eeprom_wait_dma_idle();
    if (hdr.magic != EE_MAGIC || hdr.version != EE_VERSION) return false;
    if (hdr.wr_idx == 0) return false;  /* 아직 저장 없음 */
    uint16_t last = (uint16_t)((hdr.wr_idx - 1) % DTC_SLOT_MAX);
    return DTC_ReadAt(last, out);
}

// PMIC Fault + DTC 저장 (I2C로 읽은 PMIC 8바이트를 SPI로 저장
bool DTC_Save(uint8_t pmic_code, const uint8_t raw8[8]){
    if (!raw8) return false;

    ee_hdr_t hdr;
    if (EEPROM_Read_DMA(EE_HDR_ADDR, (uint8_t*)&hdr, sizeof(hdr)) != HAL_OK) return false;
    eeprom_wait_dma_idle();
    if (hdr.magic != EE_MAGIC || hdr.version != EE_VERSION) return false;

    uint16_t idx  = hdr.wr_idx % DTC_SLOT_MAX;
    uint16_t addr = DTC_BASE_ADDR + idx * DTC_SLOT_SIZE;

    dtc_record_t rec;
    rec.ts_ms     = HAL_GetTick();
    rec.pmic_code = pmic_code;
    memcpy(rec.raw, raw8, 8);
    rec.crc = 0;   // CRC 구현 가능
    rec.pad = 0;

    // 레코드 쓰기
    if (eeprom_wren()!=HAL_OK) return false;
    if (EEPROM_Write_DMA(addr, (uint8_t*)&rec, sizeof(rec)) != HAL_OK) return false;
    eeprom_wait_dma_idle();
    if (eeprom_wait_ready(EE_TIMEOUT_WIP) != HAL_OK) return false;

    // 헤더 wr_idx 갱신
    hdr.wr_idx = (hdr.wr_idx + 1) % DTC_SLOT_MAX;

    if (eeprom_wren()!=HAL_OK) return false;
    if (EEPROM_Write_DMA(EE_HDR_ADDR, (uint8_t*)&hdr, sizeof(hdr)) != HAL_OK) return false;
    eeprom_wait_dma_idle();
    if (eeprom_wait_ready(EE_TIMEOUT_WIP) != HAL_OK) return false;

    return true;
}



