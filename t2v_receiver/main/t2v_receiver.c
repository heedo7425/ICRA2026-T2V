/*
 * ============================================================
 * T2V 수신기 (Receiver) - IR NEC 수신 + BLE GATT + USB CDC
 * ============================================================
 *
 * 보드    : ESP32-S3 Super Mini
 * IR 핀   : GPIO6 (VS1838B 출력, Active LOW)
 * 프로토콜: NEC IR (38kHz 캐리어) + BLE NimBLE GATT 서버 + USB CDC
 *
 * [BLE GATT 서버]
 *   디바이스 이름 : "T2V-Receiver"
 *   Service UUID  : c902d400-1809-2a94-904d-af5cbdcefe9b (128-bit)
 *   FF01 | Team Name      | Read       | "Unicorn Racing"
 *   FF02 | Car Name       | Read       | "Unicorn01"
 *   FF03 | IR NEC frames  | Read+Notify| [addr,~addr,cmd,~cmd]
 *   FF04 | Unicast addr   | Read+Write | 0x01
 *   FF05 | Multicast mask | Read+Write | 0x01
 *
 * [USB CDC]
 *   VID: 0x5455, PID: 0x1911
 *   NEC 디코딩 성공 시 4바이트 전송: [addr][~addr][cmd][~cmd]
 *   디버그 로그: UART0 (TX: IO43, 115200bps)
 *
 * [주소 필터링]
 *   유니캐스트(FF04 일치) / 멀티캐스트(0x00~0x07, FF05 마스크 비트) /
 *   브로드캐스트(0xFF) → 처리
 *
 * [태스크 구조]
 *   nimble_host_task : NimBLE BLE 이벤트 루프
 *   ir_rx_task       : RMT 수신 → NEC 디코딩 → USB 전송 → BLE Notify → 명령 처리
 * ============================================================
 */

#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "nvs_flash.h"
#include "driver/rmt_rx.h"
#include "esp_log.h"

/* ── NimBLE ── */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "host/ble_gatt.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

/* ── TinyUSB CDC ── */
#include "tinyusb.h"
#include "tinyusb_cdc_acm.h"

/* ============================================================
 * 상수 정의
 * ============================================================ */

/* ── BLE ── */
#define DEVICE_NAME   "T2V-Receiver"

/*
 * 서비스 UUID: c902d400-1809-2a94-904d-af5cbdcefe9b (128-bit)
 * NimBLE little-endian 바이트 순서:
 *   원본(big-endian): c9 02 d4 00  18 09  2a 94  90 4d  af 5c bd ce fe 9b
 *   NimBLE(LE):      9b fe ce bd  5c af  4d 90  94 2a  09 18  00 d4 02 c9
 */
static const ble_uuid128_t svc_uuid =
    BLE_UUID128_INIT(0x9b, 0xfe, 0xce, 0xbd, 0x5c, 0xaf, 0x4d, 0x90,
                     0x94, 0x2a, 0x09, 0x18, 0x00, 0xd4, 0x02, 0xc9);

#define CHR_UUID_TEAM_NAME    0xFF01
#define CHR_UUID_CAR_NAME     0xFF02
#define CHR_UUID_NEC_FRAME    0xFF03
#define CHR_UUID_UNICAST_ADDR 0xFF04
#define CHR_UUID_MCAST_MASK   0xFF05

/* ── USB CDC ── */
/*
 * VID 0x5455 / PID 0x1911: T2V 공식 USB 식별자.
 * PC 드라이버가 이 VID/PID로 T2V-Receiver를 인식한다.
 */
#define USB_VID  0x5455
#define USB_PID  0x1911

/* ── IR / RMT ── */
#define IR_RX_GPIO_NUM       6
#define IR_RX_RESOLUTION_HZ  1000000
#define IR_RMT_MEM_SYMBOLS   64

/* ── NEC 프로토콜 타이밍 (µs) ── */
#define NEC_LEADER_HIGH_US   9000
#define NEC_LEADER_LOW_US    4500
#define NEC_BIT_PULSE_US     560
#define NEC_DECODE_MARGIN_US 200
#define NEC_BIT_THRESHOLD_US 1000

/* ── RMT 신호 범위 (ns) ── */
#define RMT_MIN_NS           1000
#define RMT_MAX_NS           14000000

/* ── T2V 명령어 코드 ── */
#define T2V_CMD_START_GO     0x02
#define T2V_CMD_START_ABORT  0x03
#define T2V_CMD_STOP         0x7F
#define NEC_ADDR_BROADCAST   0xFF

#define RMT_RX_QUEUE_SIZE    1

/* ============================================================
 * 전역 변수
 * ============================================================ */

static const char *TAG_IR  = "T2V_IR";
static const char *TAG_BLE = "T2V_BLE";
static const char *TAG_USB = "T2V_USB";

/* ── BLE Characteristic 데이터 ── */
static const char team_name_val[]  = "Unicorn Racing";
static const char car_name_val[]   = "Unicorn01";
static uint8_t    nec_frame_val[4] = {0, 0, 0, 0};
static uint8_t    unicast_addr     = 0x01;
static uint8_t    mcast_mask       = 0x01;

static uint16_t nec_frame_val_handle;
static uint16_t g_conn_handle = BLE_HS_CONN_HANDLE_NONE;

/* ── USB CDC 연결 상태 ── */
static volatile bool g_usb_cdc_connected = false;

/* ── RMT 수신 ── */
static QueueHandle_t     g_rx_queue;
static rmt_symbol_word_t g_rx_symbols[IR_RMT_MEM_SYMBOLS];

/* ============================================================
 * USB CDC: 장치 디스크립터 (VID/PID 커스텀 설정)
 *
 * TinyUSB 드라이버 설치 시 이 디스크립터를 전달하면
 * PC에서 VID=0x5455, PID=0x1911 장치로 인식된다.
 * ============================================================ */
static const tusb_desc_device_t cdc_device_desc = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,           /* USB 2.0 */
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD, /* CDC용 IAD 필수 */
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,           /* 장치 버전 1.0 */
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01,
};

/* ============================================================
 * USB CDC: 라인 상태 변경 콜백 (연결 / 해제)
 *
 * PC에서 CDC 포트를 열면(DTR=1) 연결됨 이벤트 발생.
 * 포트를 닫으면(DTR=0) 연결 해제 이벤트 발생.
 * ============================================================ */
static void cdc_line_state_changed_cb(int itf, cdcacm_event_t *event)
{
    bool dtr = event->line_state_changed_data.dtr;
    g_usb_cdc_connected = dtr;

    if (dtr) {
        ESP_LOGI(TAG_USB, "USB CDC 연결됨 (포트 열림)");
    } else {
        ESP_LOGI(TAG_USB, "USB CDC 연결 해제 (포트 닫힘)");
    }
}

/* ============================================================
 * USB CDC 초기화
 *
 * TinyUSB 드라이버 설치 후 CDC ACM 클래스를 초기화한다.
 * 설정:
 *   - 커스텀 장치 디스크립터 (VID/PID)
 *   - 자동 생성 구성 디스크립터 (CDC ACM 1채널)
 *   - 라인 상태 콜백 등록
 * ============================================================ */
static void usb_cdc_init(void)
{
    /*
     * TinyUSB 드라이버 설치:
     *   device_descriptor     : 커스텀 VID/PID 디스크립터
     *   string_descriptor     : NULL → sdkconfig 기본값 사용
     *   external_phy          : false → ESP32-S3 내장 USB PHY 사용
     *   configuration_descriptor: NULL → CDC 활성화 시 자동 생성
     */
    /*
     * v2 API: tinyusb_config_t.descriptor 는 tinyusb_desc_config_t 구조체.
     *   descriptor.device : 커스텀 VID/PID 디스크립터 포인터
     *   descriptor.string / string_count : NULL/0 → Kconfig 기본값 사용
     *   descriptor.full_speed_config     : NULL → CDC 자동 생성
     */
    tinyusb_config_t tusb_cfg = {
        .port = TINYUSB_PORT_FULL_SPEED_0,
        .task = {
            .size     = 4096,
            .priority = 5,
            .xCoreID  = 0,
        },
        .descriptor = {
            .device           = &cdc_device_desc,
            .qualifier        = NULL,
            .string           = NULL,
            .string_count     = 0,
            .full_speed_config = NULL,
            .high_speed_config = NULL,
        },
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    /*
     * CDC ACM 클래스 초기화 (v2 API):
     *   cdc_port                    : TINYUSB_CDC_ACM_0 (첫 번째 CDC 포트)
     *   callback_line_state_changed : DTR 변화 시 호출 (연결/해제 감지)
     */
    tinyusb_config_cdcacm_t acm_cfg = {
        .cdc_port                     = TINYUSB_CDC_ACM_0,
        .callback_rx                  = NULL,
        .callback_rx_wanted_char      = NULL,
        .callback_line_state_changed  = cdc_line_state_changed_cb,
        .callback_line_coding_changed = NULL,
    };
    ESP_ERROR_CHECK(tinyusb_cdcacm_init(&acm_cfg));

    ESP_LOGI(TAG_USB, "USB CDC 초기화 완료 (VID=0x%04X, PID=0x%04X)",
             USB_VID, USB_PID);
}

/* ============================================================
 * USB CDC: NEC 프레임 4바이트 전송
 *
 * 형식: [addr][~addr][cmd][~cmd]
 * 예  : 주소 0x01, START_GO(0x02) → 0x01 0xFE 0x02 0xFD
 *
 * USB가 연결되지 않은 상태에서도 데이터를 쓰면 버퍼에 쌓이므로
 * 연결 상태 확인 후 전송한다.
 * ============================================================ */
static void usb_cdc_send_nec_frame(uint8_t addr, uint8_t cmd)
{
    /*
     * 프레임 형식: [SYNC_0][SYNC_1][addr][~addr][cmd][~cmd]
     * 싱크 헤더 0x55 0xAA로 프레임 경계를 식별한다.
     */
    uint8_t data[6] = {
        0x55,              /* 싱크 바이트 0 */
        0xAA,              /* 싱크 바이트 1 */
        addr,
        (uint8_t)~addr,
        cmd,
        (uint8_t)~cmd,
    };

    ESP_LOGI(TAG_USB, "USB 전송: [0x%02X, 0x%02X, 0x%02X, 0x%02X]",
             addr, (uint8_t)~addr, cmd, (uint8_t)~cmd);

    /*
     * tinyusb_cdcacm_write_queue(): TX 버퍼에 데이터 적재.
     * tinyusb_cdcacm_write_flush(): 버퍼를 USB로 실제 전송.
     * pdMS_TO_TICKS(100): 호스트가 읽을 때까지 최대 100ms 대기.
     */
    tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, data, sizeof(data));
    esp_err_t ret = tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG_USB, "USB 전송 실패 (err=%d)", ret);
    }
}

/* ============================================================
 * BLE 전방 선언
 * ============================================================ */
static void ble_app_advertise(void);

/* ============================================================
 * GATT 콜백: FF01 Team Name (Read-only)
 * ============================================================ */
static int chr_access_team_name(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    int rc = os_mbuf_append(ctxt->om, team_name_val, strlen(team_name_val));
    return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

/* ============================================================
 * GATT 콜백: FF02 Car Name (Read-only)
 * ============================================================ */
static int chr_access_car_name(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    int rc = os_mbuf_append(ctxt->om, car_name_val, strlen(car_name_val));
    return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

/* ============================================================
 * GATT 콜백: FF03 IR NEC frames (Read + Notify)
 * ============================================================ */
static int chr_access_nec_frame(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    int rc = os_mbuf_append(ctxt->om, nec_frame_val, sizeof(nec_frame_val));
    return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

/* ============================================================
 * GATT 콜백: FF04 Unicast address (Read + Write)
 * ============================================================ */
static int chr_access_unicast_addr(uint16_t conn_handle, uint16_t attr_handle,
                                   struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            rc = os_mbuf_append(ctxt->om, &unicast_addr, sizeof(unicast_addr));
            return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            if (OS_MBUF_PKTLEN(ctxt->om) != sizeof(unicast_addr)) {
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }
            rc = ble_hs_mbuf_to_flat(ctxt->om, &unicast_addr,
                                     sizeof(unicast_addr), NULL);
            if (rc != 0) { return BLE_ATT_ERR_UNLIKELY; }
            ESP_LOGI(TAG_BLE, "유니캐스트 주소 변경: 0x%02X", unicast_addr);
            return 0;
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

/* ============================================================
 * GATT 콜백: FF05 Multicast mask (Read + Write)
 * ============================================================ */
static int chr_access_mcast_mask(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            rc = os_mbuf_append(ctxt->om, &mcast_mask, sizeof(mcast_mask));
            return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            if (OS_MBUF_PKTLEN(ctxt->om) != sizeof(mcast_mask)) {
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }
            rc = ble_hs_mbuf_to_flat(ctxt->om, &mcast_mask,
                                     sizeof(mcast_mask), NULL);
            if (rc != 0) { return BLE_ATT_ERR_UNLIKELY; }
            ESP_LOGI(TAG_BLE, "멀티캐스트 마스크 변경: 0x%02X", mcast_mask);
            return 0;
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

/* ============================================================
 * GATT 서비스 테이블
 * ============================================================ */
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid      = BLE_UUID16_DECLARE(CHR_UUID_TEAM_NAME),
                .access_cb = chr_access_team_name,
                .flags     = BLE_GATT_CHR_F_READ,
            },
            {
                .uuid      = BLE_UUID16_DECLARE(CHR_UUID_CAR_NAME),
                .access_cb = chr_access_car_name,
                .flags     = BLE_GATT_CHR_F_READ,
            },
            {
                .uuid       = BLE_UUID16_DECLARE(CHR_UUID_NEC_FRAME),
                .access_cb  = chr_access_nec_frame,
                .val_handle = &nec_frame_val_handle,
                .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                .uuid      = BLE_UUID16_DECLARE(CHR_UUID_UNICAST_ADDR),
                .access_cb = chr_access_unicast_addr,
                .flags     = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
            },
            {
                .uuid      = BLE_UUID16_DECLARE(CHR_UUID_MCAST_MASK),
                .access_cb = chr_access_mcast_mask,
                .flags     = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
            },
            { 0 },
        },
    },
    { 0 },
};

/* ============================================================
 * FF03 BLE Notify 전송
 * ============================================================ */
static void ble_notify_nec_frame(uint8_t addr, uint8_t cmd)
{
    nec_frame_val[0] = addr;
    nec_frame_val[1] = (uint8_t)~addr;
    nec_frame_val[2] = cmd;
    nec_frame_val[3] = (uint8_t)~cmd;

    if (g_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }

    struct os_mbuf *om = ble_hs_mbuf_from_flat(nec_frame_val,
                                               sizeof(nec_frame_val));
    if (om == NULL) {
        ESP_LOGE(TAG_BLE, "Notify mbuf 할당 실패");
        return;
    }

    int rc = ble_gattc_notify_custom(g_conn_handle, nec_frame_val_handle, om);
    if (rc != 0) {
        ESP_LOGW(TAG_BLE, "Notify 전송 실패 (rc=%d)", rc);
    } else {
        ESP_LOGD(TAG_BLE, "FF03 Notify → [0x%02X,0x%02X,0x%02X,0x%02X]",
                 nec_frame_val[0], nec_frame_val[1],
                 nec_frame_val[2], nec_frame_val[3]);
    }
}

/* ============================================================
 * GAP 이벤트 콜백
 * ============================================================ */
static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                g_conn_handle = event->connect.conn_handle;
                ESP_LOGI(TAG_BLE, "BLE 연결됨 (conn_handle=%d)", g_conn_handle);
            } else {
                ESP_LOGE(TAG_BLE, "BLE 연결 실패 (status=%d)",
                         event->connect.status);
                ble_app_advertise();
            }
            break;
        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG_BLE, "BLE 연결 해제 (reason=%d)",
                     event->disconnect.reason);
            g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            ble_app_advertise();
            break;
        case BLE_GAP_EVENT_ADV_COMPLETE:
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
 * ============================================================ */
static void ble_app_advertise(void)
{
    struct ble_hs_adv_fields fields;
    struct ble_gap_adv_params adv_params;
    const char *name;
    int rc;

    memset(&fields, 0, sizeof(fields));
    fields.flags                 = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl            = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    name                         = ble_svc_gap_device_name();
    fields.name                  = (uint8_t *)name;
    fields.name_len              = strlen(name);
    fields.name_is_complete      = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG_BLE, "광고 필드 설정 실패 (rc=%d)", rc);
        return;
    }

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                           &adv_params, gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG_BLE, "광고 시작 실패 (rc=%d)", rc);
    } else {
        ESP_LOGI(TAG_BLE, "BLE 광고 중 — 디바이스명: %s", name);
    }
}

static void ble_app_on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);
    ble_app_advertise();
}

static void ble_app_on_reset(int reason)
{
    ESP_LOGE(TAG_BLE, "NimBLE 호스트 리셋 (reason=%d)", reason);
}

static void nimble_host_task(void *param)
{
    ESP_LOGI(TAG_BLE, "NimBLE 호스트 태스크 시작");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* ============================================================
 * RMT 수신 완료 콜백 (ISR)
 * ============================================================ */
static bool IRAM_ATTR rmt_rx_done_callback(rmt_channel_handle_t channel,
                                            const rmt_rx_done_event_data_t *edata,
                                            void *user_data)
{
    BaseType_t high_task_woken = pdFALSE;
    xQueueSendFromISR((QueueHandle_t)user_data, edata, &high_task_woken);
    return high_task_woken == pdTRUE;
}

static inline bool nec_in_range(uint32_t measured, uint32_t expected)
{
    return (measured >= expected - NEC_DECODE_MARGIN_US) &&
           (measured <= expected + NEC_DECODE_MARGIN_US);
}

/* ============================================================
 * NEC 프레임 디코딩
 * ============================================================ */
static bool nec_decode_frame(const rmt_symbol_word_t *symbols, size_t num_symbols,
                              uint8_t *address, uint8_t *command)
{
    if (num_symbols < 33) {
        ESP_LOGW(TAG_IR, "심볼 수 부족: %d개", (int)num_symbols);
        return false;
    }

    const rmt_symbol_word_t *leader = &symbols[0];
    if (leader->level0 != 1 || !nec_in_range(leader->duration0, NEC_LEADER_HIGH_US)) {
        ESP_LOGW(TAG_IR, "리더 펄스 오류: %uµs", leader->duration0);
        return false;
    }
    if (leader->level1 != 0 || !nec_in_range(leader->duration1, NEC_LEADER_LOW_US)) {
        ESP_LOGW(TAG_IR, "리더 스페이스 오류: %uµs", leader->duration1);
        return false;
    }

    uint32_t data = 0;
    for (int i = 0; i < 32; i++) {
        const rmt_symbol_word_t *s = &symbols[1 + i];
        if (s->level0 != 1 || !nec_in_range(s->duration0, NEC_BIT_PULSE_US)) {
            ESP_LOGW(TAG_IR, "비트%d 펄스 오류: %uµs", i, s->duration0);
            return false;
        }
        if (s->duration1 > NEC_BIT_THRESHOLD_US) {
            data |= (1UL << i);
        }
    }

    uint8_t addr     = (uint8_t)((data >>  0) & 0xFF);
    uint8_t addr_inv = (uint8_t)((data >>  8) & 0xFF);
    uint8_t cmd      = (uint8_t)((data >> 16) & 0xFF);
    uint8_t cmd_inv  = (uint8_t)((data >> 24) & 0xFF);

    if ((addr ^ addr_inv) != 0xFF) {
        ESP_LOGW(TAG_IR, "주소 반전 검증 실패: 0x%02X ^ 0x%02X", addr, addr_inv);
        return false;
    }
    if ((cmd ^ cmd_inv) != 0xFF) {
        ESP_LOGW(TAG_IR, "명령 반전 검증 실패: 0x%02X ^ 0x%02X", cmd, cmd_inv);
        return false;
    }

    *address = addr;
    *command = cmd;
    return true;
}

/* ============================================================
 * 주소 필터링
 *
 * true  = 처리 대상
 * false = 무시
 *
 * 우선순위:
 *   1. 브로드캐스트 (0xFF)          → 무조건 처리
 *   2. 유니캐스트 (== unicast_addr) → 처리
 *   3. 멀티캐스트 (0x00~0x07)       → mcast_mask의 해당 비트 확인
 *      예) mcast_mask=0x03, addr=0x01 → (0x03>>1)&1=1 → 처리
 * ============================================================ */
static bool addr_filter(uint8_t addr)
{
    if (addr == NEC_ADDR_BROADCAST)  { return true; }
    if (addr == unicast_addr)        { return true; }
    if (addr <= 0x07 && ((mcast_mask >> addr) & 1)) { return true; }
    return false;
}

/* ============================================================
 * T2V 명령어 처리
 * ============================================================ */
static void t2v_process_command(uint8_t address, uint8_t command)
{
    switch (command) {
        case T2V_CMD_START_GO:
            ESP_LOGI(TAG_IR, "★ START_GO 수신!    (주소: 0x%02X, 명령: 0x%02X)",
                     address, command);
            break;
        case T2V_CMD_STOP:
            ESP_LOGI(TAG_IR, "■ STOP 수신!        (주소: 0x%02X, 명령: 0x%02X)",
                     address, command);
            break;
        case T2V_CMD_START_ABORT:
            ESP_LOGI(TAG_IR, "✕ START_ABORT 수신! (주소: 0x%02X, 명령: 0x%02X)",
                     address, command);
            break;
        default:
            ESP_LOGW(TAG_IR, "알 수 없는 명령어: 0x%02X (주소: 0x%02X)",
                     command, address);
            break;
    }
}

/* ============================================================
 * IR 수신 태스크
 *
 * NEC 디코딩 성공 시 처리 순서:
 *   1. USB CDC로 4바이트 전송 (필터 무관, 항상)
 *   2. BLE FF03 Notify (필터 무관, 항상)
 *   3. 주소 필터 통과 시 명령 처리
 * ============================================================ */
static void ir_rx_task(void *arg)
{
    ESP_LOGI(TAG_IR, "IR 수신 태스크 시작 (핀: GPIO%d)", IR_RX_GPIO_NUM);

    g_rx_queue = xQueueCreate(RMT_RX_QUEUE_SIZE, sizeof(rmt_rx_done_event_data_t));
    assert(g_rx_queue != NULL);

    rmt_channel_handle_t rx_channel = NULL;
    rmt_rx_channel_config_t rx_cfg = {
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .gpio_num          = IR_RX_GPIO_NUM,
        .mem_block_symbols = IR_RMT_MEM_SYMBOLS,
        .resolution_hz     = IR_RX_RESOLUTION_HZ,
        .flags = {
            .invert_in    = true,  /* VS1838B Active LOW 보정 */
            .with_dma     = false,
            .io_loop_back = false,
        },
    };
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_cfg, &rx_channel));

    rmt_rx_event_callbacks_t cbs = { .on_recv_done = rmt_rx_done_callback };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel, &cbs, g_rx_queue));
    ESP_ERROR_CHECK(rmt_enable(rx_channel));

    rmt_receive_config_t rx_config = {
        .signal_range_min_ns = RMT_MIN_NS,
        .signal_range_max_ns = RMT_MAX_NS,
    };

    ESP_LOGI(TAG_IR, "IR 신호 대기 중... (GPIO%d)", IR_RX_GPIO_NUM);
    ESP_ERROR_CHECK(rmt_receive(rx_channel, g_rx_symbols,
                                sizeof(g_rx_symbols), &rx_config));

    while (1) {
        rmt_rx_done_event_data_t rx_data;
        if (xQueueReceive(g_rx_queue, &rx_data, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        uint8_t address, command;
        if (!nec_decode_frame(rx_data.received_symbols, rx_data.num_symbols,
                              &address, &command)) {
            ESP_LOGW(TAG_IR, "NEC 디코딩 실패 (심볼 %d개)",
                     (int)rx_data.num_symbols);
            goto next_receive;
        }

        /* ① USB CDC 전송 (주소 필터 무관, 항상) */
        usb_cdc_send_nec_frame(address, command);

        /* ② BLE FF03 Notify (주소 필터 무관, 항상) */
        ble_notify_nec_frame(address, command);

        /* ③ 주소 필터링 → 통과 시 명령 처리 */
        if (!addr_filter(address)) {
            ESP_LOGW(TAG_IR,
                     "주소 불일치 무시 (수신: 0x%02X, 유니캐스트: 0x%02X, 마스크: 0x%02X)",
                     address, unicast_addr, mcast_mask);
            goto next_receive;
        }
        t2v_process_command(address, command);

next_receive:
        ESP_ERROR_CHECK(rmt_receive(rx_channel, g_rx_symbols,
                                    sizeof(g_rx_symbols), &rx_config));
    }

    rmt_disable(rx_channel);
    rmt_del_channel(rx_channel);
    vTaskDelete(NULL);
}

/* ============================================================
 * 애플리케이션 진입점
 * ============================================================ */
void app_main(void)
{
    esp_err_t ret;
    int rc;

    ESP_LOGI(TAG_BLE, "========================================");
    ESP_LOGI(TAG_BLE, " T2V 수신기 시작 (ESP32-S3 Super Mini)");
    ESP_LOGI(TAG_IR,  " IR 수신 핀  : GPIO%d", IR_RX_GPIO_NUM);
    ESP_LOGI(TAG_BLE, " BLE 이름   : %s", DEVICE_NAME);
    ESP_LOGI(TAG_USB, " USB CDC    : VID=0x%04X, PID=0x%04X", USB_VID, USB_PID);
    ESP_LOGI(TAG_BLE, "========================================");

    /* ── 1. NVS 초기화 ── */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* ── 2. USB CDC 초기화 (NimBLE보다 먼저) ──
     * TinyUSB는 USB OTG 컨트롤러를 사용하며,
     * NimBLE은 별도의 BLE 컨트롤러를 사용하므로 충돌하지 않는다. */
    usb_cdc_init();

    /* ── 3. NimBLE 포트 초기화 ── */
    ret = nimble_port_init();
    ESP_ERROR_CHECK(ret);

    /* ── 4. 호스트 콜백 등록 ── */
    ble_hs_cfg.sync_cb  = ble_app_on_sync;
    ble_hs_cfg.reset_cb = ble_app_on_reset;

    /* ── 5. GAP / GATT 기본 서비스 초기화 ── */
    ble_svc_gap_init();
    ble_svc_gatt_init();

    /* ── 6. 커스텀 GATT 서비스 등록 ── */
    rc = ble_gatts_count_cfg(gatt_svcs);
    assert(rc == 0);
    rc = ble_gatts_add_svcs(gatt_svcs);
    assert(rc == 0);

    /* ── 7. 디바이스 이름 설정 ── */
    rc = ble_svc_gap_device_name_set(DEVICE_NAME);
    assert(rc == 0);

    /* ── 8. NimBLE 호스트 태스크 시작 ── */
    nimble_port_freertos_init(nimble_host_task);

    /* ── 9. IR 수신 태스크 시작 ── */
    xTaskCreate(ir_rx_task, "ir_rx_task", 4096, NULL, 5, NULL);
}
