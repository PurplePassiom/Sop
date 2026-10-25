#include "soa_onipc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>

#define MAX_IPC_SUBSCRIPTIONS_PER_INST  16U
#define MAX_IPC_SERVICES_PER_INST       20U
#define IPC_MSG_MAX_PAYLOAD             32U
#define IPC_QUEUE_DEPTH                 16U
/* ⚠️ 这个栈要跑**所有 IPC 节点的接收回调** —— subscribe 注册的回调是在传输任务里
 *    **同步**执行的, 因此回调里的应用逻辑和 ESP_LOG 全都压在这个栈上。
 *
 *    2048 字节不够: MIO 时代 (2026-09-21) 在回调里加调试日志后爆过栈 ——
 *        ***ERROR*** A stack overflow in task ipc has been detected.
 *    一次 ESP_LOG 的格式化 + UART 写出就要吃掉不少栈。
 *    这里与 soa_oncan 的 CAN_TASK_STACK_SIZE 对齐 (同样要跑回调)。 */
#define IPC_TASK_STACK_SIZE             4096U   /* bytes on ESP-IDF */
#define IPC_TASK_PRIORITY               (tskIDLE_PRIORITY + 2)
#define IPC_BROADCAST_NODE              0xFFU

typedef struct
{
    uint16_t        port_id;
    Soa_MessageType type;
    uint16_t        max_size;
} ipc_subscription_t;

typedef struct
{
    uint8_t             src_node;
    uint8_t             des_node;
    uint16_t            port_id;
    Soa_MessageType     type;
    message_service_callback callback;
} ipc_message_service_t;

typedef struct
{
    uint8_t                 active;
    uint8_t                 node_id;
    uint8_t                 sub_index;
    ipc_subscription_t      subs[MAX_IPC_SUBSCRIPTIONS_PER_INST];
    ipc_message_service_t   services[MAX_IPC_SERVICES_PER_INST];
} Soa_IpcInstance;

/* Queue item: metadata + data copy */
typedef struct
{
    uint8_t         src_node;
    uint8_t         des_node;   /* IPC_BROADCAST_NODE = broadcast */
    uint16_t        port_id;
    Soa_MessageType type;
    uint16_t        data_len;
    uint8_t         data[IPC_MSG_MAX_PAYLOAD];
} IpcQueueItem;

static Soa_IpcInstance ipc_inst[SOA_IPC_MAX_INSTANCES];
static QueueHandle_t   ipc_queue;
static TaskHandle_t    ipc_task_handle;
static StaticQueue_t   ipc_queue_cb;
static uint8_t         ipc_queue_buf[IPC_QUEUE_DEPTH * sizeof(IpcQueueItem)];
static StackType_t     ipc_task_stack[IPC_TASK_STACK_SIZE / sizeof(StackType_t)];
static StaticTask_t    ipc_task_tcb;

/* ---- node_id → instance lookup ---- */

static Soa_IpcInstance* find_instance_by_node_id(uint8_t node_id)
{
    for (uint8_t i = 0; i < SOA_IPC_MAX_INSTANCES; i++)
    {
        if (ipc_inst[i].active && ipc_inst[i].node_id == node_id)
        {
            return &ipc_inst[i];
        }
    }
    return NULL;
}

static message_service_callback find_callback(Soa_IpcInstance* ci,
                                              uint8_t src_node,
                                              uint16_t port_id,
                                              Soa_MessageType type)
{
    for (size_t i = 0; i < MAX_IPC_SERVICES_PER_INST; i++)
    {
        if ((ci->services[i].callback != NULL) &&
            (ci->services[i].src_node == src_node ||
             ci->services[i].src_node == SOA_NODE_ANY) &&
            (ci->services[i].port_id  == port_id) &&
            (ci->services[i].type     == type))
        {
            return ci->services[i].callback;
        }
    }
    return NULL;
}

static int has_subscription(Soa_IpcInstance* ci, uint16_t port_id,
                            Soa_MessageType type, size_t data_len)
{
    for (uint8_t i = 0; i < ci->sub_index; i++)
    {
        if (ci->subs[i].port_id == port_id &&
            ci->subs[i].type    == type &&
            data_len <= ci->subs[i].max_size)
        {
            return 1;
        }
    }
    return 0;
}

/* ---- Enqueue helper (non-blocking from caller context) ---- */

static int8_t enqueue_msg(uint8_t src_node, uint8_t des_node,
                          uint16_t port_id, Soa_MessageType type,
                          const uint8_t* data, size_t data_len)
{
    if (ipc_queue == NULL) return SOA_NOT_OK;
    if (data_len > IPC_MSG_MAX_PAYLOAD) return SOA_NOT_OK;

    IpcQueueItem item;
    item.src_node = src_node;
    item.des_node = des_node;
    item.port_id  = port_id;
    item.type     = type;
    item.data_len = (uint16_t)data_len;
    memcpy(item.data, data, data_len);

    if (xQueueSend(ipc_queue, &item, 0) != pdTRUE)
    {
        return SOA_NOT_OK; /* queue full, drop */
    }
    return SOA_OK;
}

/* ---- Dispatch: process one dequeued message ---- */

static void dispatch_broadcast(const IpcQueueItem* item)
{
    for (uint8_t i = 0; i < SOA_IPC_MAX_INSTANCES; i++)
    {
        Soa_IpcInstance* ci = &ipc_inst[i];
        if (!ci->active || ci->node_id == item->src_node) continue;

        if (!has_subscription(ci, item->port_id, SoaBroadcast, item->data_len))
            continue;

        message_service_callback cb = find_callback(ci, item->src_node,
                                                    item->port_id, SoaBroadcast);
        if (cb)
        {
            cb(item->data, item->data_len);
        }
    }
}

static void dispatch_unicast(const IpcQueueItem* item)
{
    Soa_IpcInstance* ci = find_instance_by_node_id(item->des_node);
    if (ci == NULL) return;

    message_service_callback cb = find_callback(ci, item->src_node,
                                                item->port_id, item->type);
    if (cb)
    {
        cb(item->data, item->data_len);
    }
}

/* ---- IPC background task ---- */

static void soa_ipc_task(void* arg)
{
    (void)arg;
    IpcQueueItem item;

    for (;;)
    {
        if (xQueueReceive(ipc_queue, &item, portMAX_DELAY) == pdTRUE)
        {
            if (item.des_node == IPC_BROADCAST_NODE)
            {
                dispatch_broadcast(&item);
            }
            else
            {
                dispatch_unicast(&item);
            }
        }
    }
}

/* ======== Public API ======== */

void soa_ipc_init(void)
{
    memset(ipc_inst, 0, sizeof(ipc_inst));

    ipc_queue = xQueueCreateStatic(IPC_QUEUE_DEPTH, sizeof(IpcQueueItem),
                                   ipc_queue_buf, &ipc_queue_cb);

    ipc_task_handle = xTaskCreateStatic(soa_ipc_task, "ipc",
                                        IPC_TASK_STACK_SIZE,
                                        NULL, IPC_TASK_PRIORITY,
                                        ipc_task_stack, &ipc_task_tcb);
}

int8_t soa_ipc_create(uint8_t node_id)
{
    if (find_instance_by_node_id(node_id) != NULL)
    {
        return SOA_OK; /* already exists */
    }
    for (uint8_t i = 0; i < SOA_IPC_MAX_INSTANCES; i++)
    {
        if (!ipc_inst[i].active)
        {
            ipc_inst[i].node_id   = node_id;
            ipc_inst[i].sub_index = 0;
            ipc_inst[i].active    = 1;
            return SOA_OK;
        }
    }
    return SOA_NOT_OK;
}
#if 1
int8_t soa_ipc_publish(uint8_t src_node, uint16_t object_id,
                       uint8_t* serialed_data, size_t data_len,
                       uint64_t tx_deadline_usec)
{
    (void)tx_deadline_usec;
    if (find_instance_by_node_id(src_node) == NULL) return SOA_NOT_OK;
    return enqueue_msg(src_node, IPC_BROADCAST_NODE,
                       object_id, SoaBroadcast, serialed_data, data_len);
}
#endif

int8_t soa_ipc_subscribe(uint8_t own_node, uint8_t from_node, Soa_MessageType type,
                            uint16_t expect_id, uint16_t max_size, message_service_callback callback)
{
    Soa_IpcInstance* ci = find_instance_by_node_id(own_node);
    if (ci == NULL) return SOA_NOT_OK;

    if (ci->sub_index >= MAX_IPC_SUBSCRIPTIONS_PER_INST) return SOA_NOT_OK;

    /* Store subscription filter */
    ci->subs[ci->sub_index].port_id  = expect_id;
    ci->subs[ci->sub_index].type     = type;
    ci->subs[ci->sub_index].max_size = max_size;

    /* Store callback in service table */
    for (size_t i = 0; i < MAX_IPC_SERVICES_PER_INST; i++)
    {
        if (ci->services[i].callback == NULL)
        {
            ci->services[i].src_node = from_node;
            ci->services[i].des_node = own_node;
            ci->services[i].port_id  = expect_id;
            ci->services[i].type     = type;
            ci->services[i].callback = callback;
            break;
        }
    }

    ci->sub_index++;
    return SOA_OK;
}

int8_t soa_ipc_subscribe_broadcast(uint8_t own_node, Soa_MessageType type, uint16_t expect_id, uint16_t max_size, message_service_callback callback)
{
    Soa_IpcInstance* ci = find_instance_by_node_id(own_node);
    if (ci == NULL) return SOA_NOT_OK;

    if (ci->sub_index >= MAX_IPC_SUBSCRIPTIONS_PER_INST) return SOA_NOT_OK;

    ci->subs[ci->sub_index].port_id  = expect_id;
    ci->subs[ci->sub_index].type     = type;
    ci->subs[ci->sub_index].max_size = max_size;

    for (size_t i = 0; i < MAX_IPC_SERVICES_PER_INST; i++)
    {
        if (ci->services[i].callback == NULL)
        {
            ci->services[i].src_node = SOA_NODE_ANY;
            ci->services[i].des_node = own_node;
            ci->services[i].port_id  = expect_id;
            ci->services[i].type     = type;
            ci->services[i].callback = callback;
            break;
        }
    }

    ci->sub_index++;
    return SOA_OK;
}

int8_t soa_ipc_request_send(uint8_t src_node, uint8_t des_node,
                            uint16_t service_id, uint8_t* serialed_data,
                            size_t data_len, uint64_t tx_deadline_usec)
{
    (void)tx_deadline_usec;
    if (find_instance_by_node_id(src_node) == NULL) return SOA_NOT_OK;
    return enqueue_msg(src_node, des_node,
                       service_id, SoaRequest, serialed_data, data_len);
}

int8_t soa_ipc_response_send(uint8_t src_node, uint8_t des_node,
                             uint16_t service_id, uint8_t* serialed_data,
                             size_t data_len, uint64_t tx_deadline_usec)
{
    (void)tx_deadline_usec;
    if (find_instance_by_node_id(src_node) == NULL) return SOA_NOT_OK;
    return enqueue_msg(src_node, des_node,
                       service_id, SoaResponse, serialed_data, data_len);
}

uint8_t soa_ipc_get_instance_count(void)
{
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < SOA_IPC_MAX_INSTANCES; i++)
    {
        if (ipc_inst[i].active) cnt++;
    }
    return cnt;
}
