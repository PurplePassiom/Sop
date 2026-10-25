#include "soa_router.h"
#include "soa_oncan.h"
#include "soa_onipc.h"
#include "soa_onuart.h"
#include "soa_onespnow.h"
#include <string.h>

/* Default TX deadline when routing hides transport details (microseconds) */
#define SOA_DEFAULT_TX_DEADLINE_USEC    0U

/* ---------- route table (merged from soa_router_cfg.c) -------------------- */

typedef struct {
    uint8_t      node_id1;
    uint8_t      node_id2;
    Soa_PathType path;
} Soa_RouteEntry;

static Soa_RouteEntry route_table[SOA_MAX_ROUTES];
static uint8_t        route_count = 0;

static Soa_PathType soa_router_get_route_path(uint8_t src_node, uint8_t des_node)
{
    for (uint8_t i = 0; i < route_count; i++)
    {
        if ((route_table[i].node_id1 == src_node && route_table[i].node_id2 == des_node) ||
            (route_table[i].node_id1 == des_node && route_table[i].node_id2 == src_node))
        {
            return route_table[i].path;
        }
    }
    return SOA_PATH_UNKNOWN;
}

int8_t soa_router_add_route(uint8_t node_id1, uint8_t node_id2,
                            Soa_PathType path)
{
    /* update if route already exists (bidirectional) */
    for (uint8_t i = 0; i < route_count; i++)
    {
        if ((route_table[i].node_id1 == node_id1 &&
             route_table[i].node_id2 == node_id2) ||
            (route_table[i].node_id1 == node_id2 &&
             route_table[i].node_id2 == node_id1))
        {
            route_table[i].path = path;
            return 0;
        }
    }
    if (route_count >= SOA_MAX_ROUTES)
    {
        return -1;
    }
    route_table[route_count].node_id1 = node_id1;
    route_table[route_count].node_id2 = node_id2;
    route_table[route_count].path     = path;
    route_count++;
    return 0;
}

/* ---------- router -------------------------------------------------------- */

void soa_router_init(void)
{
    memset(route_table, 0, sizeof(route_table));
    route_count = 0;
    soa_can_init();
    soa_ipc_init();
    soa_uart_init();
    soa_espnow_init();
}

#if 1
int8_t soa_router_publish(uint8_t source_node, uint16_t object_id,
                          uint8_t* serialed_data, size_t data_len)
{
    /* Broadcast: send on Master */
    int8_t r1 = soa_can_publish(source_node, object_id,
                                serialed_data, data_len,
                                SOA_DEFAULT_TX_DEADLINE_USEC);
    int8_t r2 = soa_ipc_publish(source_node, object_id,
                                serialed_data, data_len,
                                SOA_DEFAULT_TX_DEADLINE_USEC);
    int8_t r3 = soa_uart_publish(source_node, object_id,
                                    serialed_data, data_len,
                                    SOA_DEFAULT_TX_DEADLINE_USEC);
    int8_t r4 = soa_espnow_publish(source_node, object_id,
                                   serialed_data, data_len,
                                   SOA_DEFAULT_TX_DEADLINE_USEC);
    return (r1 == SOA_OK || r2 == SOA_OK || r3 == SOA_OK || r4 == SOA_OK) ? SOA_OK : SOA_NOT_OK;
}
#endif
int8_t soa_router_subscribe(uint8_t own_node, uint8_t from_node, Soa_MessageType type,
                            uint16_t expect_id, uint16_t max_size, message_service_callback callback)
{
    Soa_PathType path = soa_router_get_route_path(own_node, from_node);

    switch (path)
    {
    case SOA_PATH_CAN:
        return soa_can_subscribe(own_node, from_node, type, expect_id, max_size, callback);
    case SOA_PATH_IPC:
        return soa_ipc_subscribe(own_node, from_node, type, expect_id, max_size, callback);
    case SOA_PATH_UART:
        return soa_uart_subscribe(own_node, from_node, type, expect_id, max_size, callback);
    case SOA_PATH_ESPNOW:
        return soa_espnow_subscribe(own_node, from_node, type, expect_id, max_size, callback);
    default:
        return SOA_NOT_OK;
    }
}

int8_t soa_router_subscribe_broadcast(uint8_t own_node, Soa_MessageType type, uint16_t expect_id, uint16_t max_size, message_service_callback callback)
{
    int8_t r1 = soa_can_subscribe_broadcast(own_node, type, expect_id, max_size, callback);
    int8_t r2 = soa_ipc_subscribe_broadcast(own_node, type, expect_id, max_size, callback);
    int8_t r3 = soa_uart_subscribe_broadcast(own_node, type, expect_id, max_size, callback);
    int8_t r4 = soa_espnow_subscribe_broadcast(own_node, type, expect_id, max_size, callback);
    return (r1 == SOA_OK || r2 == SOA_OK || r3 == SOA_OK || r4 == SOA_OK) ? SOA_OK : SOA_NOT_OK;
}

int8_t soa_router_request_send(uint8_t src_node, uint8_t des_node,
                               uint16_t service_id, uint8_t* serialed_data,
                               size_t data_len)
{
    Soa_PathType path = soa_router_get_route_path(src_node, des_node);

    switch (path)
    {
    case SOA_PATH_CAN:
        return soa_can_request_send(src_node, des_node,
                                    service_id, serialed_data, data_len,
                                    SOA_DEFAULT_TX_DEADLINE_USEC);
    case SOA_PATH_IPC:
        return soa_ipc_request_send(src_node, des_node,
                                    service_id, serialed_data, data_len,
                                    SOA_DEFAULT_TX_DEADLINE_USEC);
    case SOA_PATH_UART:
        return soa_uart_request_send(src_node, des_node,
                                     service_id, serialed_data, data_len,
                                     SOA_DEFAULT_TX_DEADLINE_USEC);
    case SOA_PATH_ESPNOW:
        return soa_espnow_request_send(src_node, des_node,
                                       service_id, serialed_data, data_len,
                                       SOA_DEFAULT_TX_DEADLINE_USEC);
    default:
        return SOA_NOT_OK;
    }
}

int8_t soa_router_response_send(uint8_t src_node, uint8_t des_node,
                                uint16_t service_id, uint8_t* serialed_data,
                                size_t data_len)
{
    Soa_PathType path = soa_router_get_route_path(src_node, des_node);

    switch (path)
    {
    case SOA_PATH_CAN:
        return soa_can_response_send(src_node, des_node,
                                     service_id, serialed_data, data_len,
                                     SOA_DEFAULT_TX_DEADLINE_USEC);
    case SOA_PATH_IPC:
        return soa_ipc_response_send(src_node, des_node,
                                     service_id, serialed_data, data_len,
                                     SOA_DEFAULT_TX_DEADLINE_USEC);
    case SOA_PATH_UART:
        return soa_uart_response_send(src_node, des_node,
                                      service_id, serialed_data, data_len,
                                      SOA_DEFAULT_TX_DEADLINE_USEC);
    case SOA_PATH_ESPNOW:
        return soa_espnow_response_send(src_node, des_node,
                                        service_id, serialed_data, data_len,
                                        SOA_DEFAULT_TX_DEADLINE_USEC);
    default:
        return SOA_NOT_OK;
    }
}
