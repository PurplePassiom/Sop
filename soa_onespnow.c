/**
 * soa_onespnow.c - SOA ESP-NOW transport layer
 *
 * 和 soa_oncan / soa_onipc / soa_onuart 同一套接口, 只是承载换成 ESP-NOW:
 *   - 广播 = 发到 FF:FF:FF:FF:FF:FF, 帧头里 des_node = 0xFF
 *   - 单播 = 发到 peer 表里登记的 MAC, 帧头里 des_node = 目标逻辑节点号
 *   - 收到帧后先过滤 magic/version, 再按 (port_id, type, src_node) 查订阅回调
 *
 * 报文格式 (本机字节序, 所有 ESP32 都是小端):
 *
 *   +------+------+---------+------+-----+-----+---------+----------+---------+
 *   | 0x53 | 0x4E | version | type | src | des | port_id | data_len | data... |
 *   +------+------+---------+------+-----+-----+---------+----------+---------+
 *      0      1       2        3     4     5    6 .. 7       8        9 ..
 *
 *   - magic "SN" 用来把 SOA 帧和 ESP-NOW 上的其它业务帧 (配网 / MCP 之类) 区分开,
 *     不匹配的帧直接丢弃, 不会打扰同频道上的其它固件。
 *   - data_len 是 payload 字节数, 总长度 = 9 + data_len。
 *   - payload 上限 SOA_ESPNOW_MAX_PAYLOAD (240)。**本层不做分片**: ESP-NOW 单帧
 *     最多 250 字节, 需要更大报文时请让上层改走 CAN (canard 会自动分片)。
 *
 * 线程模型:
 *   ESP-NOW 的接收回调跑在 WiFi 任务里, 既不能阻塞也不能在里面发帧, 所以回调只
 *   把整帧塞进队列, 真正的订阅回调由本文件自己的 soa_espnow 任务执行 —— 和
 *   soa_onipc.c 的做法一致, 应用回调因此始终在任务上下文里跑。
 *
 * WiFi 依赖:
 *   esp_now_init() 必须在 esp_wifi_start() 之后调用。本文件不负责启动 WiFi,
 *   第一次用到传输层时尝试初始化, 失败按需重试 (日志只在状态变化时打印一次)。
 */
#include "soa_onespnow.h"

#if defined(CONFIG_SOA_ESPNOW_ENABLE) && CONFIG_SOA_ESPNOW_ENABLE

#include <stdbool.h>
#include <string.h>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_wifi.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define TAG "SoaEspNow"

#define SOA_ESPNOW_MAGIC0        0x53U   /* 'S' */
#define SOA_ESPNOW_MAGIC1        0x4EU   /* 'N' */
#define SOA_ESPNOW_VERSION       1U
/* magic(2) + version + type + src + des + port_id(2) + data_len */
#define SOA_ESPNOW_HDR_LEN       9U

#define SOA_ESPNOW_QUEUE_DEPTH   8U
#define SOA_ESPNOW_TASK_STACK    4096U   /* bytes on ESP-IDF (回调全压在这个栈上) */
#define SOA_ESPNOW_TASK_PRIORITY (tskIDLE_PRIORITY + 2)

/* ---- 帧格式 ---- */

typedef struct __attribute__((packed))
{
    uint8_t  magic0;
    uint8_t  magic1;
    uint8_t  version;
    uint8_t  type;          /* Soa_MessageType */
    uint8_t  src_node;
    uint8_t  des_node;      /* SOA_ESPNOW_BROADCAST_NODE = 广播 */
    uint16_t port_id;
    uint8_t  data_len;
    uint8_t  data[SOA_ESPNOW_MAX_PAYLOAD];
} SoaEspNowFrame;

/* ---- 实例 / 订阅 / peer 表 ---- */

typedef struct
{
    uint8_t                 used;
    uint8_t                 src_node;   /* SOA_NODE_ANY = 通配 */
    Soa_MessageType         type;
    uint16_t                port_id;
    uint16_t                max_size;
    message_service_callback callback;
} SoaEspNowSub;

typedef struct
{
    uint8_t        active;
    uint8_t        node_id;
    uint8_t        sub_count;
    SoaEspNowSub   subs[SOA_ESPNOW_MAX_SUBS_PER_INST];
} SoaEspNowInstance;

typedef struct
{
    uint8_t used;
    uint8_t node_id;
    uint8_t mac[SOA_ESPNOW_MAC_LEN];
} SoaEspNowPeer;

static SoaEspNowInstance s_inst[SOA_ESPNOW_MAX_INSTANCES];
static SoaEspNowPeer     s_peer[SOA_ESPNOW_MAX_PEERS];

static const uint8_t s_broadcast_mac[SOA_ESPNOW_MAC_LEN] =
    { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

/* 队列 + 分发任务 (静态分配, 不依赖堆的长期碎片) */
static QueueHandle_t   s_queue;
static TaskHandle_t    s_task_handle;
static StaticQueue_t   s_queue_cb;
static uint8_t         s_queue_buf[SOA_ESPNOW_QUEUE_DEPTH * sizeof(SoaEspNowFrame)];
static StackType_t     s_task_stack[SOA_ESPNOW_TASK_STACK / sizeof(StackType_t)];
static StaticTask_t    s_task_tcb;

/* 运行状态 */
static bool s_inited;            /* esp_now_init() 已成功 (只做一次) */
static bool s_hw_ready;          /* 回调已注册并登记了广播 peer */
static bool s_bcast_peer_ok;     /* 广播 peer 已登记 */
static bool s_init_warned;       /* 初始化失败日志只打一次 */

/* ---- 状态与查表 ---- */

static SoaEspNowInstance* find_instance_by_node_id(uint8_t node_id)
{
    for (uint8_t i = 0; i < SOA_ESPNOW_MAX_INSTANCES; i++)
    {
        if (s_inst[i].active && s_inst[i].node_id == node_id)
        {
            return &s_inst[i];
        }
    }
    return NULL;
}

static const uint8_t* find_peer_mac(uint8_t node_id)
{
    for (uint8_t i = 0; i < SOA_ESPNOW_MAX_PEERS; i++)
    {
        if (s_peer[i].used && s_peer[i].node_id == node_id)
        {
            return s_peer[i].mac;
        }
    }
    return NULL;
}

static SoaEspNowPeer* find_peer_by_node_id(uint8_t node_id)
{
    for (uint8_t i = 0; i < SOA_ESPNOW_MAX_PEERS; i++)
    {
        if (s_peer[i].used && s_peer[i].node_id == node_id)
        {
            return &s_peer[i];
        }
    }
    return NULL;
}

static SoaEspNowPeer* alloc_peer_slot(void)
{
    for (uint8_t i = 0; i < SOA_ESPNOW_MAX_PEERS; i++)
    {
        if (!s_peer[i].used)
        {
            return &s_peer[i];
        }
    }
    return NULL;
}

static message_service_callback find_callback(SoaEspNowInstance* ci,
                                              uint8_t src_node,
                                              uint16_t port_id,
                                              Soa_MessageType type,
                                              size_t data_len)
{
    if (ci == NULL)
    {
        return NULL;
    }
    for (uint8_t i = 0; i < ci->sub_count; i++)
    {
        const SoaEspNowSub* s = &ci->subs[i];
        if (s->used == 0 || s->callback == NULL)
        {
            continue;
        }
        if (s->type != type || s->port_id != port_id)
        {
            continue;
        }
        if (s->src_node != SOA_NODE_ANY && s->src_node != src_node)
        {
            continue;
        }
        if (data_len > s->max_size)
        {
            continue;
        }
        return s->callback;
    }
    return NULL;
}

/* ---- ESP-NOW 回调 (WiFi 任务上下文) ---- */

static void espnow_recv_cb(const esp_now_recv_info_t* info,
                           const uint8_t* data, int data_len)
{
    (void)info;   /* src_addr 是 MAC; 逻辑节点号在帧头里, 不需要它 */

    if (data == NULL || data_len < (int)SOA_ESPNOW_HDR_LEN)
    {
        return;
    }

    const SoaEspNowFrame* f = (const SoaEspNowFrame*)data;
    if (f->magic0 != SOA_ESPNOW_MAGIC0 || f->magic1 != SOA_ESPNOW_MAGIC1 ||
        f->version != SOA_ESPNOW_VERSION)
    {
        return;   /* 不是 SOA 帧, 静默丢弃 */
    }
    if (f->data_len > SOA_ESPNOW_MAX_PAYLOAD ||
        (size_t)data_len != SOA_ESPNOW_HDR_LEN + (size_t)f->data_len)
    {
        ESP_LOGW(TAG, "丢弃长度异常帧: 收到 %d 字节, 帧头声明 %u",
                 data_len, (unsigned)f->data_len);
        return;
    }
    if (f->type != (uint8_t)SoaBroadcast &&
        f->type != (uint8_t)SoaRequest &&
        f->type != (uint8_t)SoaResponse)
    {
        return;
    }

    /*
     * 回调里不跑应用逻辑: 整帧入队, 由 soa_espnow 任务分发 (队列满就丢)。
     * ⚠️ 必须先把 data 拷进一个本地的完整帧再入队: data 的实际长度只有
     *    data_len 字节, 直接 xQueueSend(s_queue, f, ...) 会按 sizeof(帧) 拷贝,
     *    读到驱动缓冲区之外。
     */
    SoaEspNowFrame frame;
    memset(&frame, 0, sizeof(frame));
    memcpy(&frame, data, (size_t)data_len);

    if (s_queue == NULL || xQueueSend(s_queue, &frame, 0) != pdTRUE)
    {
        ESP_LOGD(TAG, "接收队列满, 丢弃一帧");
    }
}

static void espnow_send_cb(const esp_now_send_info_t* tx_info,
                           esp_now_send_status_t status)
{
    (void)tx_info;
    if (status != ESP_NOW_SEND_SUCCESS)
    {
        /* 广播没有 ACK, 失败在这里是常态, 只留 debug 级日志 */
        ESP_LOGD(TAG, "发送失败");
    }
}

/* ---- 分发 ---- */

static void dispatch_frame(const SoaEspNowFrame* f)
{
    if (f->des_node == SOA_ESPNOW_BROADCAST_NODE)
    {
        for (uint8_t i = 0; i < SOA_ESPNOW_MAX_INSTANCES; i++)
        {
            SoaEspNowInstance* ci = &s_inst[i];
            if (!ci->active || ci->node_id == f->src_node)
            {
                continue;
            }
            message_service_callback cb = find_callback(ci, f->src_node,
                                                        f->port_id,
                                                        (Soa_MessageType)f->type,
                                                        f->data_len);
            if (cb)
            {
                cb(f->data, f->data_len);
            }
        }
        return;
    }

    SoaEspNowInstance* ci = find_instance_by_node_id(f->des_node);
    if (ci == NULL || ci->node_id == f->src_node)
    {
        return;
    }
    message_service_callback cb = find_callback(ci, f->src_node, f->port_id,
                                                (Soa_MessageType)f->type,
                                                f->data_len);
    if (cb)
    {
        cb(f->data, f->data_len);
    }
}

static void soa_espnow_task(void* arg)
{
    (void)arg;
    SoaEspNowFrame frame;

    for (;;)
    {
        if (xQueueReceive(s_queue, &frame, portMAX_DELAY) == pdTRUE)
        {
            dispatch_frame(&frame);
        }
    }
}

/* ---- WiFi / ESP-NOW 初始化 (延迟 + 失败可重试) ---- */

static bool espnow_add_broadcast_peer(void)
{
    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(peer));
    memcpy(peer.peer_addr, s_broadcast_mac, SOA_ESPNOW_MAC_LEN);
    peer.channel = 0;              /* 0 = 跟随当前信道 */
    peer.ifidx   = WIFI_IF_STA;
    peer.encrypt = false;

    esp_err_t err = esp_now_add_peer(&peer);
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST)
    {
        ESP_LOGE(TAG, "登记广播 peer 失败: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

/**
 * @brief WiFi 是否已经 start。
 *
 * ⚠️ esp_now_init() 在 WiFi 未启动时**不是返回错误, 而是空指针 panic**
 *    (实测 IDF 6.1: soa_router_init() 在开机早期调 soa_espnow_init() →
 *     LoadProhibited @ esp_now_init)。所以每次初始化前都用这个不会崩的接口探测。
 *    esp_wifi_get_channel() 在未 init / 未 start 时只是返回错误码。
 */
static bool wifi_is_started(void)
{
    uint8_t primary = 0;
    wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
    return esp_wifi_get_channel(&primary, &second) == ESP_OK;
}

static bool espnow_ensure_ready(void)
{
    if (s_hw_ready)
    {
        return true;
    }

    if (!wifi_is_started())
    {
        if (!s_init_warned)
        {
            s_init_warned = true;
            ESP_LOGI(TAG, "ESP-NOW 尚未就绪: WiFi 未 start, 等 soa_espnow_wifi_start_sta()");
        }
        return false;
    }

    /* esp_now_init() 只调一次: 成功之后再调会返回 ESP_ERR_ESPNOW_INTERNAL */
    if (!s_inited)
    {
        esp_err_t err = esp_now_init();
        if (err != ESP_OK)
        {
            if (!s_init_warned)
            {
                s_init_warned = true;
                /* 常见原因是 WiFi 还没 start (本层不负责启动 WiFi), 之后会自动
                 * 重试, 所以这里只是提示, 不是错误。 */
                ESP_LOGI(TAG, "ESP-NOW 尚未就绪 (%s): WiFi 未启动? 后续调用会自动重试",
                         esp_err_to_name(err));
            }
            return false;
        }
        s_inited      = true;
        s_init_warned = false;
    }

    esp_err_t err = esp_now_register_recv_cb(espnow_recv_cb);
    if (err == ESP_OK)
    {
        err = esp_now_register_send_cb(espnow_send_cb);
    }
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "注册 ESP-NOW 回调失败: %s", esp_err_to_name(err));
        return false;
    }

    /* 可选: 固定信道。0 = 保持 WiFi 当前信道。
     * ⚠️ 有影响: 如果本机同时还连着 AP, 改信道会把那条连接一起挪走, 甚至断链。 */
#if defined(CONFIG_SOA_ESPNOW_CHANNEL) && (CONFIG_SOA_ESPNOW_CHANNEL > 0)
    err = esp_wifi_set_channel(CONFIG_SOA_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "设置信道 %d 失败: %s",
                 CONFIG_SOA_ESPNOW_CHANNEL, esp_err_to_name(err));
    }
#endif

    s_hw_ready      = true;
    s_bcast_peer_ok = espnow_add_broadcast_peer();

    uint8_t mac[SOA_ESPNOW_MAC_LEN] = { 0 };
    if (esp_wifi_get_mac(WIFI_IF_STA, mac) == ESP_OK)
    {
        ESP_LOGI(TAG, "ESP-NOW 就绪, 本机 STA MAC %02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
    else
    {
        ESP_LOGI(TAG, "ESP-NOW 就绪");
    }
    return true;
}

static bool ensure_broadcast_peer(void)
{
    if (s_bcast_peer_ok)
    {
        return true;
    }
    s_bcast_peer_ok = espnow_add_broadcast_peer();
    return s_bcast_peer_ok;
}

/* ---- 收发 ---- */

static int8_t espnow_send(uint8_t src_node, uint8_t des_node,
                          Soa_MessageType type, uint16_t port_id,
                          const uint8_t* serialed_data, size_t data_len)
{
    if (data_len > SOA_ESPNOW_MAX_PAYLOAD || (serialed_data == NULL && data_len > 0))
    {
        ESP_LOGW(TAG, "payload %u 超过单帧上限 %u, 拒绝发送",
                 (unsigned)data_len, (unsigned)SOA_ESPNOW_MAX_PAYLOAD);
        return SOA_NOT_OK;
    }
    if (!espnow_ensure_ready())
    {
        return SOA_NOT_OK;
    }

    SoaEspNowFrame frame;
    memset(&frame, 0, SOA_ESPNOW_HDR_LEN);
    frame.magic0   = SOA_ESPNOW_MAGIC0;
    frame.magic1   = SOA_ESPNOW_MAGIC1;
    frame.version  = SOA_ESPNOW_VERSION;
    frame.type     = (uint8_t)type;
    frame.src_node = src_node;
    frame.des_node = des_node;
    frame.port_id  = port_id;
    frame.data_len = (uint8_t)data_len;
    if (data_len > 0)
    {
        memcpy(frame.data, serialed_data, data_len);
    }

    const uint8_t* peer_mac = NULL;
    if (des_node != SOA_ESPNOW_BROADCAST_NODE)
    {
        peer_mac = find_peer_mac(des_node);
        if (peer_mac == NULL)
        {
            /* 没登记 MAC 就先广播: 接收端会按帧头 des_node 过滤, 语义不变 */
            ESP_LOGD(TAG, "节点 %u 没有 peer 映射, 回退广播", (unsigned)des_node);
        }
    }
    if (peer_mac == NULL)
    {
        if (!ensure_broadcast_peer())
        {
            return SOA_NOT_OK;
        }
        peer_mac = s_broadcast_mac;
    }

    esp_err_t err = esp_now_send(peer_mac, (const uint8_t*)&frame,
                                 SOA_ESPNOW_HDR_LEN + data_len);
    if (err != ESP_OK)
    {
        ESP_LOGD(TAG, "esp_now_send 失败: %s", esp_err_to_name(err));
        return SOA_NOT_OK;
    }
    return SOA_OK;
}

/* ======== Public API ======== */

void soa_espnow_init(void)
{
    /* 队列/任务只建一次: init 可能被重复调用 (例如 WiFi 起来后再调一次重试),
     * 静态创建不能重复, 也不能把已经成功的 esp_now_init 状态清掉。 */
    if (s_task_handle == NULL)
    {
        memset(s_inst, 0, sizeof(s_inst));
        memset(s_peer, 0, sizeof(s_peer));
        s_inited        = false;
        s_hw_ready      = false;
        s_bcast_peer_ok = false;
        s_init_warned   = false;

        s_queue = xQueueCreateStatic(SOA_ESPNOW_QUEUE_DEPTH, sizeof(SoaEspNowFrame),
                                     s_queue_buf, &s_queue_cb);
        s_task_handle = xTaskCreateStatic(soa_espnow_task, "soa_espnow",
                                          SOA_ESPNOW_TASK_STACK, NULL,
                                          SOA_ESPNOW_TASK_PRIORITY,
                                          s_task_stack, &s_task_tcb);
    }

    /* 尝试一次: WiFi 已经起来的板子 (或调用了 soa_espnow_create 之后) 立刻可用;
     * 没起来也没关系, 下面的接口会在每次调用时重试。 */
    (void)espnow_ensure_ready();
}

int8_t soa_espnow_create(uint8_t node_id)
{
    if (node_id == 0 || node_id > SOA_NODE_ID_MAX)
    {
        return SOA_NOT_OK;
    }
    if (find_instance_by_node_id(node_id) != NULL)
    {
        return SOA_OK;   /* 已存在 */
    }
    for (uint8_t i = 0; i < SOA_ESPNOW_MAX_INSTANCES; i++)
    {
        if (!s_inst[i].active)
        {
            memset(&s_inst[i], 0, sizeof(s_inst[i]));
            s_inst[i].node_id = node_id;
            s_inst[i].active  = 1;
            (void)espnow_ensure_ready();
            return SOA_OK;
        }
    }
    return SOA_NOT_OK;
}

int8_t soa_espnow_add_peer(uint8_t node_id, const uint8_t mac[SOA_ESPNOW_MAC_LEN])
{
    if (node_id == 0 || node_id > SOA_NODE_ID_MAX || mac == NULL)
    {
        return SOA_NOT_OK;
    }
    if (!espnow_ensure_ready())
    {
        return SOA_NOT_OK;
    }

    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(peer));
    memcpy(peer.peer_addr, mac, SOA_ESPNOW_MAC_LEN);
    peer.channel = 0;
    peer.ifidx   = WIFI_IF_STA;
    peer.encrypt = false;

    esp_err_t err = esp_now_add_peer(&peer);
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST)
    {
        ESP_LOGE(TAG, "登记节点 %u 的 peer 失败: %s", (unsigned)node_id,
                 esp_err_to_name(err));
        return SOA_NOT_OK;
    }

    SoaEspNowPeer* slot = find_peer_by_node_id(node_id);
    if (slot == NULL)
    {
        slot = alloc_peer_slot();
    }
    if (slot == NULL)
    {
        ESP_LOGW(TAG, "peer 表已满 (%u), 节点 %u 仍走广播",
                 (unsigned)SOA_ESPNOW_MAX_PEERS, (unsigned)node_id);
        return SOA_NOT_OK;
    }
    slot->used    = 1;
    slot->node_id = node_id;
    memcpy(slot->mac, mac, SOA_ESPNOW_MAC_LEN);
    return SOA_OK;
}

int8_t soa_espnow_remove_peer(uint8_t node_id)
{
    SoaEspNowPeer* slot = find_peer_by_node_id(node_id);
    if (slot == NULL)
    {
        return SOA_NOT_OK;
    }
    if (s_hw_ready)
    {
        (void)esp_now_del_peer(slot->mac);
    }
    memset(slot, 0, sizeof(*slot));
    return SOA_OK;
}

int8_t soa_espnow_get_mac(uint8_t mac[SOA_ESPNOW_MAC_LEN])
{
    if (mac == NULL)
    {
        return SOA_NOT_OK;
    }
    return (esp_wifi_get_mac(WIFI_IF_STA, mac) == ESP_OK) ? SOA_OK : SOA_NOT_OK;
}

int8_t soa_espnow_is_ready(void)
{
    return s_hw_ready ? SOA_OK : SOA_NOT_OK;
}

int8_t soa_espnow_wifi_start_sta(void)
{
    wifi_mode_t mode = WIFI_MODE_NULL;

    /* 已经有人初始化过 WiFi 驱动: 不重复 init, 也不改它已有的模式。
     * 但 init != start —— esp_now_init() 必须在 start 之后才能调。 */
    if (esp_wifi_get_mode(&mode) == ESP_OK)
    {
        if (!wifi_is_started())
        {
            esp_err_t start_err = esp_wifi_start();
            if (start_err != ESP_OK)
            {
                ESP_LOGE(TAG, "esp_wifi_start 失败: %s", esp_err_to_name(start_err));
                return SOA_NOT_OK;
            }
        }
        ESP_LOGI(TAG, "WiFi 已在运行 (mode=%d), 只确保 ESP-NOW 就绪", (int)mode);
        (void)espnow_ensure_ready();
        return s_hw_ready ? SOA_OK : SOA_NOT_OK;
    }

    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "esp_netif_init 失败: %s", esp_err_to_name(err));
        return SOA_NOT_OK;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "创建默认事件循环失败: %s", esp_err_to_name(err));
        return SOA_NOT_OK;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    /* 纯 ESP-NOW 节点不需要 WiFi 往 NVS 写校准数据。关掉之后本层不再依赖 NVS ——
     * 没调用过 nvs_flash_init() 的工程 (例如 MIO) 也能起来, 否则 esp_wifi_init()
     * 直接返回 ESP_ERR_NVS_NOT_INITIALIZED。 */
    cfg.nvs_enable = false;
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "esp_wifi_init 失败: %s", esp_err_to_name(err));
        return SOA_NOT_OK;
    }

    /* 纯 ESP-NOW 节点不连 AP: 用 RAM storage, 避免往 NVS 写 PHY/校准数据 */
    (void)esp_wifi_set_storage(WIFI_STORAGE_RAM);

    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_set_mode(STA) 失败: %s", esp_err_to_name(err));
        return SOA_NOT_OK;
    }

    err = esp_wifi_start();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_start 失败: %s", esp_err_to_name(err));
        return SOA_NOT_OK;
    }

    /* 未连 AP 的 STA 会进 modem sleep, ESP-NOW 收帧会丢 —— 这里保持射频常开。
     * 想省电的话调用方在空闲时自行 esp_wifi_set_ps(WIFI_PS_MIN_MODEM)。 */
    (void)esp_wifi_set_ps(WIFI_PS_NONE);

    (void)espnow_ensure_ready();
    return s_hw_ready ? SOA_OK : SOA_NOT_OK;
}

int8_t soa_espnow_publish(uint8_t src_node, uint16_t object_id,
                          uint8_t* serialed_data, size_t data_len,
                          uint64_t tx_deadline_usec)
{
    (void)tx_deadline_usec;
    if (find_instance_by_node_id(src_node) == NULL)
    {
        return SOA_NOT_OK;
    }
    return espnow_send(src_node, SOA_ESPNOW_BROADCAST_NODE, SoaBroadcast,
                       object_id, serialed_data, data_len);
}

int8_t soa_espnow_subscribe(uint8_t own_node, uint8_t from_node, Soa_MessageType type,
                            uint16_t expect_id, uint16_t max_size,
                            message_service_callback callback)
{
    SoaEspNowInstance* ci = find_instance_by_node_id(own_node);
    if (ci == NULL || callback == NULL)
    {
        return SOA_NOT_OK;
    }
    if (ci->sub_count >= SOA_ESPNOW_MAX_SUBS_PER_INST)
    {
        return SOA_NOT_OK;
    }

    SoaEspNowSub* sub = &ci->subs[ci->sub_count++];
    sub->used     = 1;
    sub->src_node = from_node;
    sub->type     = type;
    sub->port_id  = expect_id;
    sub->max_size = max_size;
    sub->callback = callback;
    return SOA_OK;
}

int8_t soa_espnow_subscribe_broadcast(uint8_t own_node, Soa_MessageType type,
                                      uint16_t expect_id, uint16_t max_size,
                                      message_service_callback callback)
{
    return soa_espnow_subscribe(own_node, SOA_NODE_ANY, type, expect_id,
                                max_size, callback);
}

int8_t soa_espnow_request_send(uint8_t src_node, uint8_t des_node,
                               uint16_t service_id, uint8_t* serialed_data,
                               size_t data_len, uint64_t tx_deadline_usec)
{
    (void)tx_deadline_usec;
    if (find_instance_by_node_id(src_node) == NULL)
    {
        return SOA_NOT_OK;
    }
    return espnow_send(src_node, des_node, SoaRequest, service_id,
                       serialed_data, data_len);
}

int8_t soa_espnow_response_send(uint8_t src_node, uint8_t des_node,
                                uint16_t service_id, uint8_t* serialed_data,
                                size_t data_len, uint64_t tx_deadline_usec)
{
    (void)tx_deadline_usec;
    if (find_instance_by_node_id(src_node) == NULL)
    {
        return SOA_NOT_OK;
    }
    return espnow_send(src_node, des_node, SoaResponse, service_id,
                       serialed_data, data_len);
}

uint8_t soa_espnow_get_instance_count(void)
{
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < SOA_ESPNOW_MAX_INSTANCES; i++)
    {
        if (s_inst[i].active)
        {
            cnt++;
        }
    }
    return cnt;
}

#else /* !CONFIG_SOA_ESPNOW_ENABLE */

/*
 * 没开 CONFIG_SOA_ESPNOW_ENABLE 时提供空实现, 让 soa_router.c 可以无条件调用,
 * 不必在每个调用点加 #if —— 和 soa_onuart.c 的处理方式一致。
 * 这样也能避免在“没有 WiFi 的板子”上把 esp_now 链接进固件。
 */
void soa_espnow_init(void)
{
}

int8_t soa_espnow_create(uint8_t node_id)
{
    (void)node_id;
    return SOA_NOT_OK;
}

int8_t soa_espnow_add_peer(uint8_t node_id, const uint8_t mac[SOA_ESPNOW_MAC_LEN])
{
    (void)node_id;
    (void)mac;
    return SOA_NOT_OK;
}

int8_t soa_espnow_remove_peer(uint8_t node_id)
{
    (void)node_id;
    return SOA_NOT_OK;
}

int8_t soa_espnow_get_mac(uint8_t mac[SOA_ESPNOW_MAC_LEN])
{
    (void)mac;
    return SOA_NOT_OK;
}

int8_t soa_espnow_is_ready(void)
{
    return SOA_NOT_OK;
}

int8_t soa_espnow_wifi_start_sta(void)
{
    /* 传输层没编进来 (CONFIG_SOA_ESPNOW_ENABLE=n), 不要替调用方启动 WiFi */
    return SOA_NOT_OK;
}

int8_t soa_espnow_publish(uint8_t src_node, uint16_t object_id,
                          uint8_t* serialed_data, size_t data_len,
                          uint64_t tx_deadline_usec)
{
    (void)src_node;
    (void)object_id;
    (void)serialed_data;
    (void)data_len;
    (void)tx_deadline_usec;
    return SOA_NOT_OK;
}

int8_t soa_espnow_subscribe(uint8_t own_node, uint8_t from_node, Soa_MessageType type,
                            uint16_t expect_id, uint16_t max_size,
                            message_service_callback callback)
{
    (void)own_node;
    (void)from_node;
    (void)type;
    (void)expect_id;
    (void)max_size;
    (void)callback;
    return SOA_NOT_OK;
}

int8_t soa_espnow_subscribe_broadcast(uint8_t own_node, Soa_MessageType type,
                                      uint16_t expect_id, uint16_t max_size,
                                      message_service_callback callback)
{
    (void)own_node;
    (void)type;
    (void)expect_id;
    (void)max_size;
    (void)callback;
    return SOA_NOT_OK;
}

int8_t soa_espnow_request_send(uint8_t src_node, uint8_t des_node,
                               uint16_t service_id, uint8_t* serialed_data,
                               size_t data_len, uint64_t tx_deadline_usec)
{
    (void)src_node;
    (void)des_node;
    (void)service_id;
    (void)serialed_data;
    (void)data_len;
    (void)tx_deadline_usec;
    return SOA_NOT_OK;
}

int8_t soa_espnow_response_send(uint8_t src_node, uint8_t des_node,
                                uint16_t service_id, uint8_t* serialed_data,
                                size_t data_len, uint64_t tx_deadline_usec)
{
    (void)src_node;
    (void)des_node;
    (void)service_id;
    (void)serialed_data;
    (void)data_len;
    (void)tx_deadline_usec;
    return SOA_NOT_OK;
}

uint8_t soa_espnow_get_instance_count(void)
{
    return 0;
}

#endif /* CONFIG_SOA_ESPNOW_ENABLE */
