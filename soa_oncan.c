/**
 * soa_oncan.c - SOA CAN transport layer
 *
 * Ported from the STM32 FDCAN version, then from the MIO ESP32-C6 TWAI version.
 * Key differences from the original:
 *   - HAL FDCAN TX FIFO  -> can_bus.c send queue (IDF 6.x new TWAI driver)
 *   - RX ISR ring buffer  -> can_bus.c RX task pushes into this file's ring buffer
 *   - HAL_FDCAN_* API    -> can_send()/can_set_extra_rx_handler() via can_bus wrapper
 *
 * 报文体用 can_frame_t (can_bus.h 里的中立类型), 因此本文件不依赖任何 TWAI 头文件。
 */
#include "soa_oncan.h"
#include "canard.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "can_bus.h"

#include <esp_log.h>

#define TAG "SoaCan"

/*
 * 订阅的 transfer-ID 超时。
 *
 * canard 只接受"期望的下一个 transfer-ID"(严格的 +1), 不匹配的帧会被**静默丢弃**
 * (canard.c 的 rxSessionUpdate: correct_tid)。唯一的绕行是超时路径:
 * rxSessionSynchronize 认为"距上一次成功接收超过 transfer_id_timeout_usec"时,
 * 允许用同一个 transfer-ID 重启接收会话。
 *
 * ⇒ 如果对端的应答 transfer-ID 不递增, 第一帧之后的所有应答都会被丢掉, 只有间隔
 *   超过这个超时的那些才收得下。现场表现就是"CAN 卡上明明有应答, 固件却次次报
 *   超时"。调小它可以让"重复 TID 的"对端也能被立刻接受。
 *
 * 默认 2000 ms = CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC, 即保持库的原始行为。
 */
#ifndef CONFIG_SOA_CAN_TRANSFER_ID_TIMEOUT_MS
#define CONFIG_SOA_CAN_TRANSFER_ID_TIMEOUT_MS 2000
#endif

#define SOA_CAN_TID_TIMEOUT_USEC_ \
    ((CanardMicrosecond)(CONFIG_SOA_CAN_TRANSFER_ID_TIMEOUT_MS) * 1000ULL)

/* ---- Per-instance constants ---- */
#define SOC_CYCLE_5MS               0x01U
#define TX_CONFIRM_EVENT            0x02U
#define RX_NOTIFICATION             0x04U

#define MAX_MSG_SERVICES_PER_INST   20U
#define MAX_SUBSCRIPTIONS_PER_INST  16U
#define CAN_TASK_STACK_SIZE         4096U   /* bytes (ESP-IDF counts stack in bytes) */

typedef struct
{
    uint8_t             src_node;
    uint8_t             des_node;
    uint16_t            port_id;
    Soa_MessageType     type;
    message_service_callback callback;
} can_message_service_t;

typedef struct
{
    uint8_t                     active;
    struct CanardInstance        canard;
    struct CanardTxQueue         tx_queue;
    uint64_t                    msg_expire;
    uint64_t                    req_expire;
    uint64_t                    rsp_expire;
    uint8_t                     msg_transfer_id;
    uint8_t                     req_transfer_id;
    uint8_t                     rsp_transfer_id;

    uint8_t                     sub_index;
    struct CanardRxSubscription  subs[MAX_SUBSCRIPTIONS_PER_INST];
    can_message_service_t        services[MAX_MSG_SERVICES_PER_INST];
} Soa_CanInstance;

static Soa_CanInstance can_inst[SOA_CAN_MAX_INSTANCES];

/* ---- CAN RX ring buffer (task context: can_bus RX task -> ring -> process) ---- */
#define CAN_RX_RING_SIZE    10U

typedef struct
{
    uint32_t ext_id;
    uint8_t  dlc;
    uint8_t  data[8];
} CanRxFrame;

static struct
{
    CanRxFrame       frames[CAN_RX_RING_SIZE];
    volatile uint8_t head;   /* written by producer */
    volatile uint8_t tail;   /* read by consumer    */
} can_rx_ring;

/* ---- Single shared CAN task & timer ---- */
static TaskHandle_t  soa_can_task_handle;
static StackType_t   soa_can_task_stack[CAN_TASK_STACK_SIZE / sizeof(StackType_t)];
static StaticTask_t  soa_can_task_tcb;
static TimerHandle_t soa_can_timer_handle;
static StaticTimer_t soa_can_timer_buf;

/* Spinlock protecting the shared canard TX queue (ESP-IDF requires a mutex). */
static portMUX_TYPE  soa_can_tx_lock = portMUX_INITIALIZER_UNLOCKED;

/* ---- node_id -> instance lookup ---- */

static Soa_CanInstance* find_instance_by_node_id(uint8_t node_id)
{
    for (uint8_t i = 0; i < SOA_CAN_MAX_INSTANCES; i++)
    {
        if (can_inst[i].active && can_inst[i].canard.node_id == node_id)
        {
            return &can_inst[i];
        }
    }
    return NULL;
}

/* ---- Shared helpers ---- */

static uint64_t get_system_tick_usec(void)
{
    TickType_t tick = xTaskGetTickCount();
    return ((uint64_t)tick * 1000000ULL) / configTICK_RATE_HZ;
}

static void* memory_allocate(void* const user_reference, const size_t size)
{
    (void)user_reference;
    return pvPortMalloc(size);
}

static void memory_deallocate(void* const user_reference,
                              const size_t size,
                              void* const pointer)
{
    (void)user_reference;
    (void)size;
    vPortFree(pointer);
}

/* ---- Per-instance internal functions ---- */

static message_service_callback find_callback(Soa_CanInstance* ci,
                                              uint8_t node_id,
                                              uint16_t port_id,
                                              Soa_MessageType type)
{
    for (size_t i = 0; i < MAX_MSG_SERVICES_PER_INST; i++)
    {
        if ((ci->services[i].callback != NULL) &&
            (ci->services[i].src_node == node_id ||
             ci->services[i].src_node == SOA_NODE_ANY) &&
            (ci->services[i].port_id == port_id) &&
            (ci->services[i].type == type))
        {
            return ci->services[i].callback;
        }
    }
    return NULL;
}

/* ---- Ring buffer helpers ---- */

static inline uint8_t ring_next(uint8_t idx)
{
    return (uint8_t)((idx + 1U) % CAN_RX_RING_SIZE);
}

static void ring_push(uint32_t ext_id, uint8_t dlc, const uint8_t* data)
{
    uint8_t next = ring_next(can_rx_ring.head);
    if (next != can_rx_ring.tail)
    {
        CanRxFrame* f = &can_rx_ring.frames[can_rx_ring.head];
        f->ext_id = ext_id;
        f->dlc    = dlc;
        for (uint8_t i = 0; i < dlc && i < 8U; i++)
        {
            f->data[i] = data[i];
        }
        can_rx_ring.head = next;
    }
}

/* ---- RX processing: one frame -> all active instances ---- */

/* CAN ID 字段位置, 与 canard.c 保持一致 (仅用于下面的诊断日志) */
#define CAN_ID_FLAG_SERVICE    (UINT32_C(1) << 25U)
#define CAN_ID_FLAG_REQUEST    (UINT32_C(1) << 24U)
#define CAN_ID_OFF_DST_NODE    7U
#define CAN_ID_OFF_SERVICE_ID  14U
#define CAN_ID_NODE_ID_MAX     0x7FU
#define CAN_ID_SERVICE_ID_MAX  0x1FFU
#define CAN_TAIL_TID_MAX       0x1FU

static void process_rx_frame(const CanRxFrame* f)
{
    struct CanardFrame frame;
    frame.extended_can_id = f->ext_id;
    frame.payload.size    = f->dlc;
    frame.payload.data    = (const void*)f->data;

    const uint64_t now = get_system_tick_usec();

    for (uint8_t i = 0; i < SOA_CAN_MAX_INSTANCES; i++)
    {
        Soa_CanInstance* ci = &can_inst[i];
        if (!ci->active) continue;

        /*
         * 诊断打点: 凡是"发给本节点的服务帧"都记一行。本节点在自己的服务里只当
         * 客户端, 所以这基本就等于"我们发出的请求的应答"。
         *
         * 为什么必须打: canard 遇到不匹配的帧 (transfer-ID 不是期望的下一个、toggle
         * 不对、没有对应订阅) 时是**静默丢弃**的 —— 既不回调也不报错。少了这行日志
         * 就分不清"对端没回"和"回了但传输层没收下", 而这两种情况现场表现完全一样
         * (固件报超时, 但 CAN 卡上能看到应答帧)。2026-09-26 就在这里迷过路。
         *
         * 流量: 每个事务 1 帧, 可以忽略。
         */
        const bool     is_service_frame  = (f->ext_id & CAN_ID_FLAG_SERVICE) != 0;
        const bool     addressed_to_self = is_service_frame &&
                                           (((f->ext_id >> CAN_ID_OFF_DST_NODE) & CAN_ID_NODE_ID_MAX) ==
                                            ci->canard.node_id);

        struct CanardRxSubscription* subscription;
        struct CanardRxTransfer rx_transfer;
        const int8_t result = canardRxAccept(&ci->canard, now,
                                             &frame, 0,
                                             &rx_transfer, &subscription);

        message_service_callback rx_cb = NULL;
        if (result == 1)
        {
            rx_cb = find_callback(
                ci,
                rx_transfer.metadata.remote_node_id,
                rx_transfer.metadata.port_id,
                (Soa_MessageType)rx_transfer.metadata.transfer_kind);
            if (rx_cb)
            {
                rx_cb((const uint8_t*)rx_transfer.payload.data,
                      rx_transfer.payload.size);
            }
            ci->canard.memory.deallocate(NULL, 0,
                rx_transfer.payload.data);
        }

        if (addressed_to_self)
        {
            const uint8_t  tail        = (f->dlc > 0U) ? f->data[f->dlc - 1U] : 0U;
            const uint16_t service_id  = (uint16_t)((f->ext_id >> CAN_ID_OFF_SERVICE_ID) & CAN_ID_SERVICE_ID_MAX);
            const uint8_t  src_node    = (uint8_t)(f->ext_id & CAN_ID_NODE_ID_MAX);
            const uint8_t  transfer_id = (uint8_t)(tail & CAN_TAIL_TID_MAX);
            const char*    kind        = (f->ext_id & CAN_ID_FLAG_REQUEST) ? "req" : "resp";

            if ((result == 1) && (rx_cb != NULL))
            {
                ESP_LOGI(TAG, "RX svc=%u %s src=%u ext_id=0x%08X tid=%u → 已交付应用",
                         (unsigned)service_id, kind, (unsigned)src_node,
                         (unsigned)f->ext_id, (unsigned)transfer_id);
            }
            else
            {
                ESP_LOGW(TAG,
                         "RX svc=%u %s src=%u ext_id=0x%08X tid=%u → **没被收货** "
                         "(canard=%d, 回调=%s)",
                         (unsigned)service_id, kind, (unsigned)src_node,
                         (unsigned)f->ext_id, (unsigned)transfer_id, (int)result,
                         (rx_cb != NULL) ? "有" : "无");
            }
        }
    }
}

static void inst_tx_flush(Soa_CanInstance* ci)
{
    for (struct CanardTxQueueItem* ti = NULL;
         (ti = canardTxPeek(&ci->tx_queue)) != NULL;)
    {
        /* Cyphal/CAN 全部用 29 位扩展帧。can_send() 会把数据拷进它自己的
           持久发送槽位, 所以这里返回后可以放心让 canardTxFree() 释放报文缓冲。 */
        can_frame_t frame = {0};
        frame.extended = true;
        frame.id       = ti->frame.extended_can_id;
        frame.dlc      = (uint8_t)ti->frame.payload.size;
        if (frame.dlc > sizeof(frame.data))
        {
            frame.dlc = sizeof(frame.data);
        }
        for (uint8_t i = 0; i < frame.dlc; i++)
        {
            frame.data[i] = ((const uint8_t*)ti->frame.payload.data)[i];
        }
        if (can_send(&frame) != 0)
        {
            break;  /* 发送槽位/队列暂时用满, 5 ms 后由周期定时器再试 */
        }
        canardTxFree(&ci->tx_queue, &ci->canard,
                     canardTxPop(&ci->tx_queue, ti));
    }
}

static void flush_all_tx(void)
{
    for (uint8_t i = 0; i < SOA_CAN_MAX_INSTANCES; i++)
    {
        if (can_inst[i].active)
        {
            inst_tx_flush(&can_inst[i]);
        }
    }
}

/* ---- CAN RX entry: called by can_bus.c RX task (single consumer, task context) ---- */
static void soa_can_on_frame(const can_frame_t* frame)
{
    if (frame == NULL || !frame->extended)
    {
        return;     /* SOA/Cyphal 只用 29 位扩展帧, 标准帧一律忽略 */
    }
    ring_push(frame->id, frame->dlc, frame->data);
    if (soa_can_task_handle)
    {
        xTaskNotify(soa_can_task_handle, RX_NOTIFICATION, eSetBits);
    }
}

/* ---- Single shared CAN task ---- */
static void soa_can_task_function(void* p)
{
    (void)p;

    xTimerStart(soa_can_timer_handle, 100);

    for (;;)
    {
        uint32_t events = 0;
        /* Block until notified by the RX feed or the 5 ms cyclic timer. */
        xTaskNotifyWait(0, 0xFFFFFFFF, &events, portMAX_DELAY);

        if ((events & SOC_CYCLE_5MS) || (events & TX_CONFIRM_EVENT))
        {
            flush_all_tx();
        }

        if (events & RX_NOTIFICATION)
        {
            while (can_rx_ring.tail != can_rx_ring.head)
            {
                process_rx_frame(&can_rx_ring.frames[can_rx_ring.tail]);
                can_rx_ring.tail = ring_next(can_rx_ring.tail);
            }
            flush_all_tx();
        }
    }
}

static void soa_can_cyclic_timer_cb(TimerHandle_t xTimer)
{
    (void)xTimer;
    if (soa_can_task_handle)
    {
        xTaskNotify(soa_can_task_handle, SOC_CYCLE_5MS, eSetBits);
    }
}

/* ======== Public API ======== */

void soa_can_init(void)
{
    struct CanardMemoryResource memory = {
        .user_reference = NULL,
        .allocate       = memory_allocate,
        .deallocate     = memory_deallocate,
    };

    can_rx_ring.head = 0;
    can_rx_ring.tail = 0;

    for (uint8_t i = 0; i < SOA_CAN_MAX_INSTANCES; i++)
    {
        Soa_CanInstance* ci = &can_inst[i];
        ci->active          = 0;
        ci->sub_index       = 0;
        ci->msg_transfer_id = 0;
        ci->req_transfer_id = 0;
        ci->rsp_transfer_id = 0;

        ci->canard   = canardInit(memory);
        ci->canard.node_id = CANARD_NODE_ID_UNSET;
        ci->tx_queue = canardTxInit(100, CANARD_MTU_CAN_CLASSIC, memory);

        for (size_t s = 0; s < MAX_MSG_SERVICES_PER_INST; s++)
        {
            ci->services[s].callback = NULL;
        }
    }

    soa_can_timer_handle = xTimerCreateStatic("soa_can_t", pdMS_TO_TICKS(10), pdTRUE, NULL,
                                              soa_can_cyclic_timer_cb,
                                              &soa_can_timer_buf);

    soa_can_task_handle = xTaskCreateStatic(soa_can_task_function, "soa_can",
                                            CAN_TASK_STACK_SIZE, NULL, 8,
                                            soa_can_task_stack,
                                            &soa_can_task_tcb);

    /* Route CAN frames from can_bus.c's RX task into the SOA CAN transport. */
    can_set_extra_rx_handler(soa_can_on_frame);
}

int8_t soa_can_create(uint8_t node_id)
{
    for (uint8_t i = 0; i < SOA_CAN_MAX_INSTANCES; i++)
    {
        if (!can_inst[i].active)
        {
            can_inst[i].canard.node_id = node_id;
            can_inst[i].active = 1;
            return SOA_OK;
        }
    }
    return SOA_NOT_OK;
}

int8_t soa_can_publish(uint8_t src_node, uint16_t object_id,
                       uint8_t* serialed_data, size_t data_len,
                       uint64_t tx_deadline_usec)
{
    Soa_CanInstance* ci = find_instance_by_node_id(src_node);
    if (ci == NULL) return SOA_NOT_OK;

    const struct CanardTransferMetadata meta = {
        .priority       = CanardPriorityNominal,
        .transfer_kind  = CanardTransferKindMessage,
        .port_id        = object_id,
        .remote_node_id = CANARD_NODE_ID_UNSET,
        .transfer_id    = ci->msg_transfer_id,
    };
    const struct CanardPayload payload = { .size = data_len, .data = serialed_data };
    ++ci->msg_transfer_id;

    portENTER_CRITICAL(&soa_can_tx_lock);
    int32_t result = canardTxPush(&ci->tx_queue, &ci->canard,
                                  tx_deadline_usec, &meta, payload,
                                  0, &ci->msg_expire);
    portEXIT_CRITICAL(&soa_can_tx_lock);
    return (result > 0) ? SOA_OK : SOA_NOT_OK;
}

int8_t soa_can_subscribe(uint8_t own_node, uint8_t from_node, Soa_MessageType type,
                            uint16_t expect_id, uint16_t max_size, message_service_callback callback)
{
    Soa_CanInstance* ci = find_instance_by_node_id(own_node);
    if (ci == NULL) return SOA_NOT_OK;

    if (ci->sub_index >= MAX_SUBSCRIPTIONS_PER_INST) return SOA_NOT_OK;

    portENTER_CRITICAL(&soa_can_tx_lock);
    int8_t result = canardRxSubscribe(&ci->canard, type, expect_id, max_size,
                                      SOA_CAN_TID_TIMEOUT_USEC_,
                                      &ci->subs[ci->sub_index++]);
    portEXIT_CRITICAL(&soa_can_tx_lock);
    if (result >= 0)
    {
        for (size_t i = 0; i < MAX_MSG_SERVICES_PER_INST; i++)
        {
            if (ci->services[i].callback == NULL)
            {
                ci->services[i].src_node  = from_node;
                ci->services[i].des_node  = own_node;
                ci->services[i].port_id   = expect_id;
                ci->services[i].type      = type;
                ci->services[i].callback  = callback;
                return SOA_OK;
            }
        }
    }
    return (result >= 0) ? SOA_OK : SOA_NOT_OK;
}

int8_t soa_can_subscribe_broadcast(uint8_t own_node, Soa_MessageType type,
                                    uint16_t expect_id, uint16_t max_size, message_service_callback callback)
{
    return soa_can_subscribe(own_node, SOA_NODE_ANY, type, expect_id, max_size, callback);
}

int8_t soa_can_request_send(uint8_t src_node, uint8_t des_node,
                            uint16_t service_id, uint8_t* serialed_data,
                            size_t data_len, uint64_t tx_deadline_usec)
{
    Soa_CanInstance* ci = find_instance_by_node_id(src_node);
    if (ci == NULL) return SOA_NOT_OK;

    const struct CanardTransferMetadata meta = {
        .priority       = CanardPriorityNominal,
        .transfer_kind  = CanardTransferKindRequest,
        .port_id        = service_id,
        .remote_node_id = des_node,
        .transfer_id    = ci->req_transfer_id,
    };
    const struct CanardPayload payload = { .size = data_len, .data = serialed_data };
    ++ci->req_transfer_id;

    portENTER_CRITICAL(&soa_can_tx_lock);
    int32_t result = canardTxPush(&ci->tx_queue, &ci->canard,
                                  tx_deadline_usec, &meta, payload,
                                  0, &ci->req_expire);
    portEXIT_CRITICAL(&soa_can_tx_lock);
    return (result > 0) ? SOA_OK : SOA_NOT_OK;
}

int8_t soa_can_response_send(uint8_t src_node, uint8_t des_node,
                             uint16_t service_id, uint8_t* serialed_data,
                             size_t data_len, uint64_t tx_deadline_usec)
{
    Soa_CanInstance* ci = find_instance_by_node_id(src_node);
    if (ci == NULL) return SOA_NOT_OK;

    const struct CanardTransferMetadata meta = {
        .priority       = CanardPriorityNominal,
        .transfer_kind  = CanardTransferKindResponse,
        .port_id        = service_id,
        .remote_node_id = des_node,
        .transfer_id    = ci->rsp_transfer_id,
    };
    const struct CanardPayload payload = { .size = data_len, .data = serialed_data };
    ++ci->rsp_transfer_id;

    portENTER_CRITICAL(&soa_can_tx_lock);
    int32_t result = canardTxPush(&ci->tx_queue, &ci->canard,
                                  tx_deadline_usec, &meta, payload,
                                  0, &ci->rsp_expire);
    portEXIT_CRITICAL(&soa_can_tx_lock);
    return (result > 0) ? SOA_OK : SOA_NOT_OK;
}

uint8_t soa_can_get_instance_count(void)
{
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < SOA_CAN_MAX_INSTANCES; i++)
    {
        if (can_inst[i].active) cnt++;
    }
    return cnt;
}
