#ifndef _SOA_H_
#define _SOA_H_

#include "soa_types.h"
#include "soa_router.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize SOA framework (router, all transports, default routes).
 */
void soa_init(void);

void soa_create_node(uint8_t node_id, Soa_NodeType node_type);
/**
 * @brief Configure a route path between two nodes.
 * @param node_id1  First node.
 * @param node_id2  Second node.
 * @param path      Transport type (SOA_PATH_CAN / SOA_PATH_IPC / SOA_PATH_UART /
 *                  SOA_PATH_ESPNOW).
 */
void soa_create_route_path(uint8_t node_id1, uint8_t node_id2,
                           Soa_PathType path);

#if 1
/**
 * @brief Publish a broadcast message from source_node.
 */
int8_t soa_publish(uint8_t source_node, uint16_t object_id,
                   uint8_t* serialed_data, size_t data_len);
#endif

/**
 * @brief Subscribe to messages from from_node on the transport associated with own_node.
 */
int8_t soa_subscribe(uint8_t own_node, uint8_t from_node, Soa_MessageType type,
                     uint16_t expect_id, uint16_t max_size, message_service_callback callback);

int8_t soa_subscribe_broadcast(uint8_t own_node, Soa_MessageType type, uint16_t expect_id, uint16_t max_size, message_service_callback callback);
/**
 * @brief Send a service request from src_node to des_node.
 */
int8_t soa_request_send(uint8_t src_node, uint8_t des_node,
                        uint16_t service_id, uint8_t* serialed_data,
                        size_t data_len);

/**
 * @brief Send a service response from src_node to des_node.
 */
int8_t soa_response_send(uint8_t src_node, uint8_t des_node,
                         uint16_t service_id, uint8_t* serialed_data,
                         size_t data_len);

#ifdef __cplusplus
}
#endif

#endif /* _SOA_H_ */