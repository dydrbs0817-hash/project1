/*
 * DTC.h
 *
 *  Created on: Aug 20, 2025
 *      Author: dydrb
 */

#ifndef DTC_H_
#define DTC_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

#include "EEPROM.h"
#include "PMIC.h"

// DTC 코드(예시)
typedef enum {
    DTC_CODE_NONE            = 0x0000,
    DTC_CODE_BRAKE_UV        = 0x0101,
    DTC_CODE_BRAKE_OC        = 0x0102,
    DTC_CODE_BRAKE_OT        = 0x0103,
    DTC_CODE_BRAKE_OV        = 0x0104
} DTC_Code_t;

// UDS(ISO 14229) 상수
#define UDS_SVC_CLEAR_DIAG_INFO       0x14u	// DTC 클리어
#define UDS_SVC_READ_DTC_INFO         0x19u	// DTC 읽기
#define UDS_SVC_WRITE_DID             0x2Eu	// DID 쓰기
#define UDS_POS_RESP(sid)            ((uint8_t)((sid) + 0x40u))	//긍정응답 SID 계산

// 0x19 서브함수(일부)
#define UDS_19_REPORT_NUMBER_OF_DTC_BY_STATUS_MASK  0x01u	// 개수 보고
#define UDS_19_REPORT_DTC_BY_STATUS_MASK            0x02u	// 목록 보고

// NRC (부정 응답 코드)
#define UDS_NRC_SNS                       0x11u  // Service Not Supported
#define UDS_NRC_IMLOIF                    0x13u  // Incorrect Message Length or Invalid Format
#define UDS_NRC_ROOR                      0x31u  // Request Out Of Range

// 테스트용 DID(제조사 정의 영역) — 테스트용 DTC 강제 주입
#define DID_DTC_INJECT                    0xF1A0u   // 0x2E로 2바이트 DTC 코드 쓰면 저장

/* ===== 공개 API ===== */

// EEPROM 초기화/Probe까지 수행
bool DTC_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPort, uint16_t csPin);

// PMIC Fault 스냅샷 → 간단 판정 → DTC 1건 저장 (원인 프레임 8B 스냅샷 포함)
bool DTC_EvaluateAndSaveFromPmic(const PMIC_Fault8_t *volt,
                                 const PMIC_Fault8_t *curr,
                                 const PMIC_Fault8_t *temp);

// 최신 DTC 레코드 읽기
bool DTC_ReadLatestRecord(dtc_record_t *out);

// 모든 DTC 삭제(헤더 wr_idx=0 초기화)
bool DTC_ClearAll(void);

// 최신 DTC를 CAN 1프레임(8B)으로 전송(파이프라인 CAN 단계에서 사용)
bool DTC_SendLatestOverCAN(CAN_HandleTypeDef *hcan, uint16_t tx_std_id);

// UDS 단일 프레임 요청 처리(0x14/0x19/0x2E) - CAN RX IRQ에서 호출
bool DTC_HandleUdsRequest(CAN_HandleTypeDef *hcan,
                          const CAN_RxHeaderTypeDef *rxh,
                          const uint8_t req[8],
                          uint16_t tx_std_id);

#endif /* DTC_H_ */
