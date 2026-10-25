#ifndef _SOA_ONCAN_H_
#define _SOA_ONCAN_H_

#include "soa_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum number of CAN transport instances (canard nodes sharing the bus) */
#define SOA_CAN_MAX_INSTANCES   1U

/**
 * @brief Initialize ALL CAN transport instances.
 *        Must be called once at startup before any other soa_can_* API.
 */
void soa_can_init(void);

/**
 * @brief Create a canard node on CAN.
 * @param node_id  The Cyphal node ID to assign (mapped to an internal instance).
 * @return SOA_OK on success.
 */
int8_t soa_can_create(uint8_t node_id);

#if 1
/**
 * @brief Publish a broadcast message on CAN.
 */
int8_t soa_can_publish(uint8_t src_node, uint16_t object_id,
                       uint8_t* serialed_data, size_t data_len,
                       uint64_t tx_deadline_usec);
#endif
/**
 * @brief Subscribe to a message/service on CAN.
 */
int8_t soa_can_subscribe(uint8_t own_node, uint8_t from_node, Soa_MessageType type,
                            uint16_t expect_id, uint16_t max_size, message_service_callback callback);

int8_t soa_can_subscribe_broadcast(uint8_t own_node, Soa_MessageType type,
                                    uint16_t expect_id, uint16_t max_size, message_service_callback callback);

/**
 * @brief Send a service request on CAN.
 */
int8_t soa_can_request_send(uint8_t src_node, uint8_t des_node,
                            uint16_t service_id, uint8_t* serialed_data,
                            size_t data_len, uint64_t tx_deadline_usec);

/**
 * @brief Send a service response on CAN.
 */
int8_t soa_can_response_send(uint8_t src_node, uint8_t des_node,
                             uint16_t service_id, uint8_t* serialed_data,
                             size_t data_len, uint64_t tx_deadline_usec);

/**
 * @brief Get the number of active CAN instances.
 */
uint8_t soa_can_get_instance_count(void);

#ifdef __cplusplus
}
#endif

#endif /* _SOA_ONCAN_H_ */
