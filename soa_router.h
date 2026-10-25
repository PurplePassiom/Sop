#ifndef _SOA_ROUTER_H_
#define _SOA_ROUTER_H_

#include <stdint.h>
#include <stddef.h>
#include "soa_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SOA_MAX_ROUTES  16
#define SOA_MAX_NODES   8

typedef enum {
    SOA_NODE_CAN     = 0,
    SOA_NODE_IPC     = 1,
    SOA_NODE_UART    = 2,
    SOA_NODE_ESPNOW  = 3,
    SOA_NODE_UNKNOWN = 0xFF,
} Soa_NodeType;

typedef enum {
    SOA_PATH_CAN     = 0,
    SOA_PATH_IPC     = 1,
    SOA_PATH_UART    = 2,
    SOA_PATH_ESPNOW  = 3,
    SOA_PATH_UNKNOWN = 0xFF,
} Soa_PathType;

/**
 * @brief Initialize the router (and all transport layers).
 */
void soa_router_init(void);

#if 1
/**
 * @brief Publish via the transport determined by source node's route.
 */
int8_t soa_router_publish(uint8_t source_node, uint16_t object_id,
                          uint8_t* serialed_data, size_t data_len);
#endif

/**
 * @brief Subscribe on the transport determined by (own_node, from_node) route.
 */
int8_t soa_router_subscribe(uint8_t own_node, uint8_t from_node, Soa_MessageType type,
                            uint16_t expect_id, uint16_t max_size, message_service_callback callback);

int8_t soa_router_subscribe_broadcast(uint8_t own_node, Soa_MessageType type, uint16_t expect_id, uint16_t max_size, message_service_callback callback);
/**
 * @brief Send a request via the transport determined by (src, des) route.
 */
int8_t soa_router_request_send(uint8_t src_node, uint8_t des_node,
                               uint16_t service_id, uint8_t* serialed_data,
                               size_t data_len);

/**
 * @brief Send a response via the transport determined by (src, des) route.
 */
int8_t soa_router_response_send(uint8_t src_node, uint8_t des_node,
                                uint16_t service_id, uint8_t* serialed_data,
                                size_t data_len);

/**
 * @brief Add or update a route between two nodes.
 * @param node_id1  First node.
 * @param node_id2  Second node.
 * @param path      Transport path to use for this pair.
 * @return 0 on success, -1 if table is full.
 */
int8_t soa_router_add_route(uint8_t node_id1, uint8_t node_id2,
                            Soa_PathType path);

#ifdef __cplusplus
}
#endif

#endif /* _SOA_ROUTER_H_ */
