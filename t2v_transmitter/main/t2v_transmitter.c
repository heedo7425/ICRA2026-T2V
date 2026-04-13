/*
 * ============================================================
 * T2V 송신기 (Transmitter) - IR NEC 송신 + BLE GATT 서버
 * ============================================================
 *
 * 보드    : ESP32-S3 Super Mini
 * IR 핀   : GPIO5 (IR LED 드라이버 입력)
 * 프로토콜: NEC IR (38kHz 캐리어) + BLE NimBLE GATT 서버
 *
 * [BLE GATT 서버]
 *   디바이스 이름 : "T2V-Transmitter"
 *   Service UUID  : 0x00FF
 *   Characteristic: target_address (UUID: 0xFF01)
 *     - 속성  : READ + WRITE
 *     - 초기값: 0x00
 *     - Write : "목표 주소 변경: 0xXX" 로그 출력 + 전역변수 갱신
 *
 * [키보드 입력]
 *   1 → START_GO   (0x02) — NEC 주소는 BLE로 설정된 target_address 사용
 *   2 → STOP       (0x7F)
 *   3 → START_ABORT(0x03)
 *
 * [태스크 구조]
 *   nimble_host_task : NimBLE BLE 이벤트 루프
 *   keyboard_task    : getchar()로 키 입력 → NEC 프레임 단발 송신
 * ============================================================
 */

#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* NVS: BLE 스택 내부 설정 저장에 필요 */
#include "nvs_flash.h"

/* RMT IR 송신 드라이버 */
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"

/* fcntl: stdin 논블로킹 설정용 */
#include <fcntl.h>
#include <unistd.h>

/* ESP 로그 */
#include "esp_log.h"

/* NimBLE 포트 및 FreeRTOS 연동 */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

/* NimBLE 호스트 스택 */
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "host/ble_gatt.h"

/* GAP / GATT 기본 서비스 */
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

/* ============================================================
 * 상수 정의
 * ============================================================ */

/* ── BLE ── */
#define DEVICE_NAME        "T2V-Transmitter" /* BLE 광고 디바이스 이름 */
#define SERVICE_UUID       0x00FF            /* 커스텀 서비스 UUID      */
#define CHR_UUID_TARGET    0xFF01            /* target_address 특성 UUID */

/* ── IR / RMT ── */
#define IR_TX_GPIO_NUM       5
#define IR_TX_RESOLUTION_HZ  1000000         /* 1MHz → 1틱 = 1µs */
#define IR_TX_MEM_SYMBOLS    64
#define IR_TX_QUEUE_DEPTH    4

/* ── IR 캐리어 ── */
#define IR_CARRIER_FREQ_HZ   38000
#define IR_CARRIER_DUTY      0.33f

/* ── NEC 프로토콜 타이밍 (µs) ── */
#define NEC_LEADER_HIGH_US   9000
#define NEC_LEADER_LOW_US    4500
#define NEC_BIT_PULSE_US     560
#define NEC_BIT0_SPACE_US    560
#define NEC_BIT1_SPACE_US    1690
#define NEC_END_PULSE_US     560
#define NEC_TOTAL_SYMBOLS    34      /* 1(리더) + 32(데이터) + 1(종료 펄스) */

/* ── T2V 명령어 코드 ── */
#define T2V_CMD_START_GO     0x02U
#define T2V_CMD_START_ABORT  0x03U
#define T2V_CMD_STOP         0x7FU

/* ============================================================
 * 전역 변수
 * ============================================================ */

static const char *TAG_TX  = "T2V_TX";
static const char *TAG_BLE = "T2V_BLE";

/*
 * target_address: BLE Write로 설정 가능한 목표 주소.
 * 키 입력 시 NEC 프레임의 주소 필드로 사용된다.
 * 초기값 0x00 (수신기의 vehicle_address와 일치해야 수신 처리됨).
 */
static uint8_t target_address_val = 0x00;

/* target_address 특성의 ATT 핸들 (알림 확장 시 사용) */
static uint16_t target_address_val_handle;

/* RMT 핸들 (app_main에서 초기화, keyboard_task에서 사용) */
static rmt_channel_handle_t  s_tx_channel   = NULL;
static rmt_encoder_handle_t  s_copy_encoder = NULL;

/* ============================================================
 * BLE 전방 선언
 * ============================================================ */
static void ble_app_advertise(void);

/* ============================================================
 * GATT 특성 콜백: target_address (Read / Write)
 *
 * NimBLE이 ATT Read 또는 Write 요청을 받으면 이 함수를 호출한다.
 * ============================================================ */
static int chr_access_target_address(uint16_t conn_handle,
                                     uint16_t attr_handle,
                                     struct ble_gatt_access_ctxt *ctxt,
                                     void *arg)
{
    int rc;

    switch (ctxt->op) {

        /* ── Read 요청: 현재 target_address 값 반환 ── */
        case BLE_GATT_ACCESS_OP_READ_CHR:
            ESP_LOGI(TAG_BLE, "target_address 읽기 (현재값: 0x%02X)",
                     target_address_val);
            rc = os_mbuf_append(ctxt->om,
                                &target_address_val,
                                sizeof(target_address_val));
            return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;

        /* ── Write 요청: 새 주소로 target_address 갱신 ── */
        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            /* 1바이트인지 길이 검증 */
            if (OS_MBUF_PKTLEN(ctxt->om) != sizeof(target_address_val)) {
                ESP_LOGW(TAG_BLE, "Write 길이 오류: %d bytes",
                         OS_MBUF_PKTLEN(ctxt->om));
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }

            /* mbuf에서 값 추출 후 전역변수 갱신 */
            rc = ble_hs_mbuf_to_flat(ctxt->om,
                                     &target_address_val,
                                     sizeof(target_address_val),
                                     NULL);
            if (rc != 0) {
                ESP_LOGE(TAG_BLE, "mbuf 읽기 실패: %d", rc);
                return BLE_ATT_ERR_UNLIKELY;
            }

            ESP_LOGI(TAG_BLE, "목표 주소 변경: 0x%02X", target_address_val);
            return 0;

        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

/* ============================================================
 * GATT 서비스 테이블
 *
 * 빌드 타임에 서비스/특성 구조를 선언.
 * ble_gatts_add_svcs() 호출 시 호스트에 등록된다.
 * ============================================================ */
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        /* Primary Service: 0x00FF */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(SERVICE_UUID),

        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                /* target_address 특성 (UUID: 0xFF01) */
                .uuid       = BLE_UUID16_DECLARE(CHR_UUID_TARGET),
                .access_cb  = chr_access_target_address,
                .val_handle = &target_address_val_handle,
                .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
            },
            { 0 }, /* 특성 목록 끝 */
        },
    },
    { 0 }, /* 서비스 목록 끝 */
};

/* ============================================================
 * GAP 이벤트 콜백 (연결 / 해제 / 광고 만료)
 * ============================================================ */
static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {

        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                ESP_LOGI(TAG_BLE, "BLE 연결됨 (conn_handle=%d)",
                         event->connect.conn_handle);
            } else {
                /* 연결 실패 → 광고 재시작 */
                ESP_LOGE(TAG_BLE, "BLE 연결 실패 (status=%d)",
                         event->connect.status);
                ble_app_advertise();
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            /* 연결 해제 → 광고 재시작 */
            ESP_LOGI(TAG_BLE, "BLE 연결 해제 (reason=%d)",
                     event->disconnect.reason);
            ble_app_advertise();
            break;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            /* 광고 기간 만료 → 재시작 */
            ESP_LOGI(TAG_BLE, "광고 기간 만료 — 재시작");
            ble_app_advertise();
            break;

        default:
            break;
    }
    return 0;
}

/* ============================================================
 * BLE 광고 시작
 *
 * 디바이스 이름을 광고 패킷에 포함하여
 * "T2V-Transmitter"로 탐색 가능하게 한다.
 * ============================================================ */
static void ble_app_advertise(void)
{
    struct ble_hs_adv_fields fields;
    struct ble_gap_adv_params adv_params;
    const char *name;
    int rc;

    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    name = ble_svc_gap_device_name();
    fields.name             = (uint8_t *)name;
    fields.name_len         = strlen(name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG_BLE, "광고 필드 설정 실패 (rc=%d)", rc);
        return;
    }

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; /* 연결 가능 */
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; /* 일반 탐색 */

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL,
                           BLE_HS_FOREVER,
                           &adv_params, gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG_BLE, "광고 시작 실패 (rc=%d)", rc);
    } else {
        ESP_LOGI(TAG_BLE, "BLE 광고 중 — 디바이스명: %s", name);
    }
}

/* ============================================================
 * NimBLE 호스트-컨트롤러 동기화 완료 콜백
 *
 * BLE 스택이 준비되면 호출되며, 여기서 광고를 시작한다.
 * ============================================================ */
static void ble_app_on_sync(void)
{
    int rc;
    rc = ble_hs_util_ensure_addr(0); /* 공개 주소 확인 */
    assert(rc == 0);
    ble_app_advertise();
}

/* NimBLE 비정상 리셋 콜백 */
static void ble_app_on_reset(int reason)
{
    ESP_LOGE(TAG_BLE, "NimBLE 호스트 리셋 (reason=%d)", reason);
}

/* ============================================================
 * NimBLE 호스트 태스크
 *
 * nimble_port_freertos_init()이 이 함수를 FreeRTOS 태스크로 생성.
 * nimble_port_run()은 BLE 이벤트를 처리하며 종료 시까지 블로킹.
 * ============================================================ */
static void nimble_host_task(void *param)
{
    ESP_LOGI(TAG_BLE, "NimBLE 호스트 태스크 시작");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* ============================================================
 * NEC 프레임 빌더
 *
 * 전송 순서 (LSB 먼저): 주소 → ~주소 → 명령 → ~명령
 * ============================================================ */
static void nec_build_frame(rmt_symbol_word_t *symbols,
                             uint8_t address, uint8_t command)
{
    int idx = 0;

    /* 리더 코드: 9ms HIGH + 4.5ms LOW */
    symbols[idx].duration0 = NEC_LEADER_HIGH_US;
    symbols[idx].level0    = 1;
    symbols[idx].duration1 = NEC_LEADER_LOW_US;
    symbols[idx].level1    = 0;
    idx++;

    uint8_t bytes[4] = {
        address,
        (uint8_t)~address,
        command,
        (uint8_t)~command,
    };

    for (int b = 0; b < 4; b++) {
        for (int i = 0; i < 8; i++) {
            symbols[idx].duration0 = NEC_BIT_PULSE_US;
            symbols[idx].level0    = 1;
            /* 비트 1 → 1690µs, 비트 0 → 560µs */
            symbols[idx].duration1 = ((bytes[b] >> i) & 1)
                                     ? NEC_BIT1_SPACE_US
                                     : NEC_BIT0_SPACE_US;
            symbols[idx].level1    = 0;
            idx++;
        }
    }

    /* 종료 펄스: 560µs HIGH */
    symbols[idx].duration0 = NEC_END_PULSE_US;
    symbols[idx].level0    = 1;
    symbols[idx].duration1 = 0;
    symbols[idx].level1    = 0;
}

/* ============================================================
 * NEC 프레임 1회 송신
 *
 * target_address_val을 NEC 주소로 사용하므로
 * BLE Write로 주소를 바꾸면 다음 송신부터 즉시 반영된다.
 * ============================================================ */
static void ir_send_frame(uint8_t command)
{
    /* 송신 시점의 target_address_val 스냅샷 */
    uint8_t addr = target_address_val;

    rmt_symbol_word_t nec_symbols[NEC_TOTAL_SYMBOLS];
    nec_build_frame(nec_symbols, addr, command);

    rmt_transmit_config_t tx_config = {
        .loop_count      = 0,  /* 단발 송신 */
        .flags.eot_level = 0,  /* 송신 완료 후 핀 LOW */
    };

    ESP_ERROR_CHECK(rmt_transmit(s_tx_channel,
                                 s_copy_encoder,
                                 nec_symbols,
                                 sizeof(nec_symbols),
                                 &tx_config));
    /* 송신 완료까지 최대 200ms 대기 */
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(s_tx_channel, 200));
}

/* ============================================================
 * RMT 초기화 (채널 + 38kHz 캐리어 + Copy Encoder)
 * ============================================================ */
static void rmt_init(void)
{
    rmt_tx_channel_config_t tx_cfg = {
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .gpio_num          = IR_TX_GPIO_NUM,
        .mem_block_symbols = IR_TX_MEM_SYMBOLS,
        .resolution_hz     = IR_TX_RESOLUTION_HZ,
        .trans_queue_depth = IR_TX_QUEUE_DEPTH,
        .flags = {
            .invert_out   = false,
            .with_dma     = false,
            .io_loop_back = false,
            .io_od_mode   = false,
        },
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg, &s_tx_channel));

    rmt_carrier_config_t carrier_cfg = {
        .frequency_hz              = IR_CARRIER_FREQ_HZ,
        .duty_cycle                = IR_CARRIER_DUTY,
        .flags.polarity_active_low = false,
        .flags.always_on           = false,
    };
    ESP_ERROR_CHECK(rmt_apply_carrier(s_tx_channel, &carrier_cfg));

    rmt_copy_encoder_config_t copy_enc_cfg = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_enc_cfg, &s_copy_encoder));

    ESP_ERROR_CHECK(rmt_enable(s_tx_channel));

    ESP_LOGI(TAG_TX, "RMT 초기화 완료 (GPIO%d, 38kHz 캐리어)", IR_TX_GPIO_NUM);
}

/* ============================================================
 * 키보드 입력 태스크
 *
 * getchar()는 USB Serial/JTAG VFS를 통해 동작한다.
 * (sdkconfig: CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y)
 * UART 드라이버를 설치하면 콘솔 충돌이 발생하므로 사용하지 않음.
 *
 * 송신 시 NEC 주소는 target_address_val(전역)을 그대로 사용하므로
 * BLE Write 직후 키를 누르면 새 주소로 즉시 반영된다.
 * ============================================================ */
static void keyboard_task(void *arg)
{
    /* stdin을 논블로킹으로 설정 — VFS 기본 폴링 모드에서
     * read()가 데이터 없을 때 즉시 반환하도록 보장 */
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    ESP_LOGI(TAG_TX, "----------------------------------------");
    ESP_LOGI(TAG_TX, " 키 입력 대기 중 (idf.py monitor):");
    ESP_LOGI(TAG_TX, "   1 → START_GO   (0x02)");
    ESP_LOGI(TAG_TX, "   2 → STOP       (0x7F)");
    ESP_LOGI(TAG_TX, "   3 → START_ABORT(0x03)");
    ESP_LOGI(TAG_TX, "   BLE로 목표 주소 변경 가능 (초기값: 0x%02X)",
             target_address_val);
    ESP_LOGI(TAG_TX, "----------------------------------------");

    while (1) {
        uint8_t ch;
        int n = read(STDIN_FILENO, &ch, 1);
        if (n <= 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        switch (ch) {
            case '1':
                ESP_LOGI(TAG_TX,
                         "→ START_GO 송신 (목표 주소: 0x%02X, 명령: 0x%02X)",
                         target_address_val, T2V_CMD_START_GO);
                ir_send_frame(T2V_CMD_START_GO);
                break;

            case '2':
                ESP_LOGI(TAG_TX,
                         "→ STOP 송신 (목표 주소: 0x%02X, 명령: 0x%02X)",
                         target_address_val, T2V_CMD_STOP);
                ir_send_frame(T2V_CMD_STOP);
                break;

            case '3':
                ESP_LOGI(TAG_TX,
                         "→ START_ABORT 송신 (목표 주소: 0x%02X, 명령: 0x%02X)",
                         target_address_val, T2V_CMD_START_ABORT);
                ir_send_frame(T2V_CMD_START_ABORT);
                break;

            default:
                break;
        }
    }
}

/* ============================================================
 * 애플리케이션 진입점
 * ============================================================ */
void app_main(void)
{
    esp_err_t ret;
    int rc;

    ESP_LOGI(TAG_TX,  "========================================");
    ESP_LOGI(TAG_TX,  " T2V 송신기 시작 (ESP32-S3 Super Mini)");
    ESP_LOGI(TAG_TX,  " IR 송신 핀  : GPIO%d", IR_TX_GPIO_NUM);
    ESP_LOGI(TAG_BLE, " BLE 이름   : %s",     DEVICE_NAME);
    ESP_LOGI(TAG_BLE, " 목표 주소  : 0x%02X (BLE Write로 변경 가능)",
             target_address_val);
    ESP_LOGI(TAG_TX,  "========================================");

    /* ── 1. NVS 초기화 ─────────────────────────────────────────
     * BLE 스택이 MAC 주소 등 내부 설정을 NVS에 저장하므로 필수 */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* ── 2. RMT 초기화 (채널, 캐리어, 인코더) ──────────────── */
    rmt_init();

    /* ── 4. 키보드 입력 태스크 시작 ──────────────────────────
     * BLE 연결 없이도 즉시 키 입력 → IR 송신 가능 */
    xTaskCreate(keyboard_task, "keyboard_task", 4096, NULL, 5, NULL);

    /* ── 5. NimBLE 포트 초기화 ─────────────────────────────── */
    ret = nimble_port_init();
    ESP_ERROR_CHECK(ret);

    /* ── 6. NimBLE 호스트 콜백 등록 ────────────────────────── */
    ble_hs_cfg.sync_cb  = ble_app_on_sync;   /* 동기화 완료 → 광고 시작 */
    ble_hs_cfg.reset_cb = ble_app_on_reset;  /* 비정상 리셋 처리         */

    /* ── 7. GAP / GATT 기본 서비스 초기화 ──────────────────── */
    ble_svc_gap_init();
    ble_svc_gatt_init();

    /* ── 8. 커스텀 GATT 서비스 등록 ────────────────────────── */
    rc = ble_gatts_count_cfg(gatt_svcs);  /* 필요한 ATT 속성 수 예약 */
    assert(rc == 0);
    rc = ble_gatts_add_svcs(gatt_svcs);   /* 서비스 테이블 등록       */
    assert(rc == 0);

    /* ── 9. 디바이스 이름 설정 ─────────────────────────────── */
    rc = ble_svc_gap_device_name_set(DEVICE_NAME);
    assert(rc == 0);

    /* ── 10. NimBLE 호스트 태스크 시작 ─────────────────────── */
    nimble_port_freertos_init(nimble_host_task);
}
