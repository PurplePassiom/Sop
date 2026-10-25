#include "soa.h"
#include "soa_router.h"
#include "soa_oncan.h"
#include "soa_onipc.h"
#include "soa_onuart.h"
#include "soa_onespnow.h"

void soa_init(void)
{
    /* initialize router (which initializes all transports) */
    soa_router_init();
}

void soa_create_node(uint8_t node_id, Soa_NodeType node_type)
{
    if (node_type == SOA_NODE_CAN)
    {
        soa_can_create(node_id);
    }
    else if (node_type == SOA_NODE_IPC)
    {
        soa_ipc_create(node_id);
    }
    else if (node_type == SOA_NODE_UART)
    {
        soa_uart_create(node_id);
    }
    else if (node_type == SOA_NODE_ESPNOW)
    {
        soa_espnow_create(node_id);
    }
}

void soa_create_route_path(uint8_t node_id1, uint8_t node_id2,
                           Soa_PathType path)
{
    soa_router_add_route(node_id1, node_id2, path);
}
#if 1
int8_t soa_publish(uint8_t source_node, uint16_t object_id,
                   uint8_t* serialed_data, size_t data_len)
{
    return soa_router_publish(source_node, object_id,
                              serialed_data, data_len);
}
#endif

int8_t soa_subscribe(uint8_t own_node, uint8_t from_node, Soa_MessageType type,
                     uint16_t expect_id, uint16_t max_size, message_service_callback callback)
{
    return soa_router_subscribe(own_node, from_node, type, expect_id, max_size, callback);
}

int8_t soa_subscribe_broadcast(uint8_t own_node, Soa_MessageType type, uint16_t expect_id, uint16_t max_size, message_service_callback callback)
{
    return soa_router_subscribe_broadcast(own_node, type, expect_id, max_size, callback);
}

int8_t soa_request_send(uint8_t src_node, uint8_t des_node,
                        uint16_t service_id, uint8_t* serialed_data,
                        size_t data_len)
{
    return soa_router_request_send(src_node, des_node,
                                   service_id, serialed_data, data_len);
}

int8_t soa_response_send(uint8_t src_node, uint8_t des_node,
                         uint16_t service_id, uint8_t* serialed_data,
                         size_t data_len)
{
    return soa_router_response_send(src_node, des_node,
                                    service_id, serialed_data, data_len);
}
