/*
 * DTC.c
 *
 *  Created on: Aug 20, 2025
 *      Author: dydrb
 */
#include "DTC.h"
#include <string.h>

#define ISOTP_PCI_TYPE_SF   0x00u      // PCI 상위 4비트=0이면 Single Frame
#define ISOTP_PCI_MASK      0xF0u      // 상위 4비트 마스크
#define ISOTP_LEN_MASK      0x0Fu      // 하위 4비트 = UDS 페이로드 길이
#define ISOTP_SF(len)       ((uint8_t)(ISOTP_PCI_TYPE_SF | ((len) & ISOTP_LEN_MASK))) // SF PCI 바이트 생성
#define ISOTP_IS_SF(b)      (((b) & ISOTP_PCI_MASK) == ISOTP_PCI_TYPE_SF)             // SF 여부 검사
#define ISOTP_SF_LEN(b)     ((b) & ISOTP_LEN_MASK)                                    // SF 페이로드 길이 추출

// CAN 데이터 프레임 원시 송신
static bool can_send_raw(CAN_HandleTypeDef *hcan, uint16_t tx_id,
                         const uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef tx = {0};  						// 헤더 0으로 초기화
    uint32_t mbox;
    tx.StdId = tx_id;                                   // 표준 ID
    tx.IDE   = CAN_ID_STD;                              // 표준 ID 모드
    tx.RTR   = CAN_RTR_DATA;                            // 데이터 프레임
    tx.DLC   = (len > 8) ? 8 : len;                     // 보낼 바이트 수 = 최대 8
    return (HAL_CAN_AddTxMessage(hcan, &tx, (uint8_t*)data, &mbox) == HAL_OK); // 전송 요청
}

// UDS 페이로드(≤7B)를 ISO-TP Single Frame으로 포장해서 송신
static bool isotp_send_sf(CAN_HandleTypeDef *hcan, uint16_t tx_id,
                          const uint8_t *uds, uint8_t uds_len)
{
    if (!uds || uds_len > 7) return false;             // SF는 최대 7바이트

    uint8_t frame[8] = {0};                            // CAN 데이터 버퍼
    frame[0] = ISOTP_SF(uds_len);                      // [0] = PCI(SF|len)
    for (uint8_t i = 0; i < uds_len; ++i)              // [1..] = UDS 페이로드
        frame[1 + i] = uds[i];

    // 총 보낼 바이트 수 = 1(PCI) + uds_len
    return can_send_raw(hcan, tx_id, frame, (uint8_t)(uds_len + 1)); // CAN으로 송신
}

// 부정응답(NRC) SF 생성
static bool uds_send_nrc_sf(CAN_HandleTypeDef *hcan, uint16_t tx_id,
                            uint8_t req_sid, uint8_t nrc)
{
    uint8_t uds[3] = {0x7F, req_sid, nrc};             // UDS 부정응답 페이로드
    return isotp_send_sf(hcan, tx_id, uds, 3);         // SF로 포장해 전송
}


// EEPROM(SPI) 드라이버 초기화 + 헤더 Probe
bool DTC_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPort, uint16_t csPin)
{
    if (EEPROM_Init(hspi, csPort, csPin) != HAL_OK)    // SPI 핸들/CS 핀 등록
        return false;
    if (!EEPROM_Probe())                               // 헤더(매직/버전) 점검/필요 시 초기화
        return false;
    return true;                                       // OK
}

// 고장 비트에서 DTC 코드로 간단히 매핑(우선순위: OC > OV > OT > UV)
static DTC_Code_t map_pmic_to_dtc(const PMIC_Fault8_t *volt,
                                  const PMIC_Fault8_t *curr,
                                  const PMIC_Fault8_t *temp)
{
    if (curr && (curr->bits.buck1_oc || curr->bits.buck2_oc)) return DTC_CODE_BRAKE_OC;
    if (volt && (volt->bits.buck1_ov || volt->bits.buck2_ov)) return DTC_CODE_BRAKE_OV;
    if (temp &&  temp->bits.temp_fault)                         return DTC_CODE_BRAKE_OT;
    if (volt && (volt->bits.buck1_uv || volt->bits.buck2_uv)) return DTC_CODE_BRAKE_UV;
    return DTC_CODE_NONE;
}

// 매핑된 코드와 스냅샷(8B)을 EEPROM에 1건 저장
bool DTC_EvaluateAndSaveFromPmic(const PMIC_Fault8_t *volt,
                                 const PMIC_Fault8_t *curr,
                                 const PMIC_Fault8_t *temp)
{
    DTC_Code_t code = map_pmic_to_dtc(volt, curr, temp);       // 코드 결정
    if (code == DTC_CODE_NONE) return false;

    const uint8_t *snap = NULL;                                // 저장할 8B 스냅샷 선택
    if (code == DTC_CODE_BRAKE_UV && volt)      snap = volt->raw;
    else if (code == DTC_CODE_BRAKE_OC && curr) snap = curr->raw;
    else if (temp)                                snap = temp->raw;
    if (!snap) return false;

    return DTC_Save((uint8_t)(code & 0xFF), snap);            // EEPROM에 저장(드라이버 제공)
}

// 최신 레코드 1건 읽기(있으면 true)
bool DTC_ReadLatestRecord(dtc_record_t *out)
{
    if (!out) return false;                                    // 파라미터 체크
    return DTC_ReadLatest(out);                                // 드라이버 함수 호출
}

// 모든 DTC 삭제: 헤더 wr_idx=0으로 초기화
bool DTC_ClearAll(void)
{
    ee_hdr_t hdr;                                              // 헤더 버퍼
    if (EEPROM_Read_DMA(EE_HDR_ADDR, (uint8_t*)&hdr, sizeof(hdr)) != HAL_OK)
        return false;                                          // 헤더 읽기 실패

    if (hdr.magic != EE_MAGIC || hdr.version != EE_VERSION)    // 헤더 비정상
        return EEPROM_Probe();                                 // 재초기화 시도

    hdr.wr_idx = 0;                                            // 다음 기록 인덱스=0
    hdr.crc    = 0;                                            // (미사용, 0으로)
    if (EEPROM_Write_DMA(EE_HDR_ADDR, (uint8_t*)&hdr, sizeof(hdr)) != HAL_OK)
        return false;                                          // 헤더 쓰기 실패
    return true;
}

// 최신 DTC를 1프레임으로 전송
bool DTC_SendLatestOverCAN(CAN_HandleTypeDef *hcan, uint16_t tx_std_id)
{
    dtc_record_t rec;                                          // 레코드 버퍼
    CAN_TxHeaderTypeDef tx = (CAN_TxHeaderTypeDef){0};         // CAN 헤더
    uint32_t mbox;                                             // 메일박스
    tx.StdId = tx_std_id; tx.IDE = CAN_ID_STD; tx.RTR = CAN_RTR_DATA;

    if (!DTC_ReadLatest(&rec)) {                               // 저장된 것이 없으면
        uint8_t p[2] = {0x01, 0x00};                           // 간단 알림(예: 없음)
        tx.DLC = 2;
        return (HAL_CAN_AddTxMessage(hcan, &tx, p, &mbox) == HAL_OK);
    }

    uint8_t p[8] = {                                           // 8바이트 페이로드 예시
        rec.pmic_code,                                         // [0] DTC 코드(저장 당시 1B)
        (uint8_t)(rec.ts_ms & 0xFF),                           // [1] 시간 L
        (uint8_t)((rec.ts_ms >> 8) & 0xFF),                    // [2] 시간 M
        (uint8_t)((rec.ts_ms >> 16) & 0xFF),                   // [3] 시간 H (상위는 생략)
        rec.raw[0], rec.raw[1], rec.raw[2], rec.raw[3]         // [4..7] 스냅샷 일부
    };
    tx.DLC = 8;                                                // 8바이트 전송
    return (HAL_CAN_AddTxMessage(hcan, &tx, p, &mbox) == HAL_OK);
}

//  수신 프레임 형식(ISO-TP SF)
bool DTC_HandleUdsRequest(CAN_HandleTypeDef *hcan,
                          const CAN_RxHeaderTypeDef *rxh,
                          const uint8_t req[8],                 // 수신 8B 버퍼
                          uint16_t tx_std_id)                   // 응답 ID
{
    (void)rxh;                                                 // 미사용 경고 억제
    if (!req) return false;                                    // 파라미터 방어

    uint8_t pci = req[0];                                      // [0] = PCI 바이트
    if (!ISOTP_IS_SF(pci))                                     // SF가 아니면
        return uds_send_nrc_sf(hcan, tx_std_id, 0x00, UDS_NRC_IMLOIF); // 길이/형식 오류

    uint8_t uds_len = ISOTP_SF_LEN(pci);                       // UDS 페이로드 길이(0..7)
    if (uds_len < 1)                                           // 최소 SID 1바이트는 있어야 함
        return uds_send_nrc_sf(hcan, tx_std_id, 0x00, UDS_NRC_IMLOIF);

    const uint8_t *uds = &req[1];                              // UDS 페이로드 포인터
    uint8_t sid = uds[0];                                      // [0] = SID

    switch (sid) {

    case UDS_SVC_CLEAR_DIAG_INFO: {                            // 0x14: DTC 전체 삭제
        if (uds_len != 1)                                      // 파라미터 없음이 정석
            return uds_send_nrc_sf(hcan, tx_std_id, sid, UDS_NRC_IMLOIF);
        if (!DTC_ClearAll())                                   // 내부 오류(저장소 등)
            return uds_send_nrc_sf(hcan, tx_std_id, sid, UDS_NRC_ROOR);

        uint8_t resp[1] = { UDS_POS_RESP(UDS_SVC_CLEAR_DIAG_INFO) }; // 0x54(=0x14+0x40)
        return isotp_send_sf(hcan, tx_std_id, resp, 1);        // 긍정응답
    }

    case UDS_SVC_READ_DTC_INFO: {                              // 0x19: DTC 읽기
        if (uds_len < 2)                                       // 최소 subFunction 1B 필요
            return uds_send_nrc_sf(hcan, tx_std_id, sid, UDS_NRC_IMLOIF);

        uint8_t sub = uds[1];                                  // 서브함수

        if (sub == UDS_19_REPORT_NUMBER_OF_DTC_BY_STATUS_MASK) { // 0x01: 개수 보고
            dtc_record_t rec;
            uint8_t count = DTC_ReadLatest(&rec) ? 1 : 0;      // 교육/데모: 최신 1건만 0/1
            uint8_t resp[4] = { UDS_POS_RESP(0x19), sub, 0x00, count }; // statusMask echo=0
            return isotp_send_sf(hcan, tx_std_id, resp, 4);
        }

        if (sub == UDS_19_REPORT_DTC_BY_STATUS_MASK) {         // 0x02: 목록 보고(최신 1건)
            dtc_record_t rec;
            if (!DTC_ReadLatest(&rec)) {                       // 없으면 간단 응답
                uint8_t resp[2] = { UDS_POS_RESP(0x19), sub };
                return isotp_send_sf(hcan, tx_std_id, resp, 2);
            }
            uint16_t code16 = (uint16_t)rec.pmic_code;         // 1B 코드를 2B로 확장(데모)
            uint8_t  status = 0x40;                            // testFailedSinceLastClear 가정
            uint8_t resp[5] = { UDS_POS_RESP(0x19), sub,
                                (uint8_t)(code16 >> 8), (uint8_t)code16, status };
            return isotp_send_sf(hcan, tx_std_id, resp, 5);
        }

        return uds_send_nrc_sf(hcan, tx_std_id, sid, UDS_NRC_ROOR); // 미지원 서브함수
    }

    case UDS_SVC_WRITE_DID: {                                  // 0x2E: 제조사 정의 DID 쓰기
        if (uds_len < 3)                                       // 최소 DID(2B) 필요
            return uds_send_nrc_sf(hcan, tx_std_id, sid, UDS_NRC_IMLOIF);

        uint16_t did = ((uint16_t)uds[1] << 8) | uds[2];       // DID 조합
        if (did == DID_DTC_INJECT) {                           // 0xF1A0: DTC 강제주입(테스트)
            if (uds_len < 5)                                   // 코드 2B 부족
                return uds_send_nrc_sf(hcan, tx_std_id, sid, UDS_NRC_IMLOIF);

            uint16_t code = ((uint16_t)uds[3] << 8) | uds[4];  // 코드 조합
            uint8_t  snap[8] = {0};                            // 스냅샷(테스트값 0)
            if (!DTC_Save((uint8_t)(code & 0xFF), snap))
                return uds_send_nrc_sf(hcan, tx_std_id, sid, UDS_NRC_ROOR);

            uint8_t resp[3] = { UDS_POS_RESP(UDS_SVC_WRITE_DID), uds[1], uds[2] }; // 0x6E + DID echo
            return isotp_send_sf(hcan, tx_std_id, resp, 3);
        }

        return uds_send_nrc_sf(hcan, tx_std_id, sid, UDS_NRC_ROOR);
    }

    default:
        return uds_send_nrc_sf(hcan, tx_std_id, sid, UDS_NRC_SNS);  // 미지원 서비스
    }
}
