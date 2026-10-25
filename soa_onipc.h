#ifndef _SOA_ONIPC_H_
#define _SOA_ONIPC_H_

#include "soa_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SOA_IPC_MAX_INSTANCES   4U

void soa_ipc_init(void);

int8_t soa_ipc_create(uint8_t node_id);
#if 1
int8_t soa_ipc_publish(uint8_t src_node, uint16_t object_id,
                       uint8_t* serialed_data, size_t data_len,
                       uint64_t tx_deadline_usec);
#endif
int8_t soa_ipc_subscribe(uint8_t own_node, uint8_t from_node, Soa_MessageType type,
                            uint16_t expect_id, uint16_t max_size, message_service_callback callback);

int8_t soa_ipc_subscribe_broadcast(uint8_t own_node, Soa_MessageType type, uint16_t expect_id, uint16_t max_size, message_service_callback callback);

int8_t soa_ipc_request_send(uint8_t src_node, uint8_t des_node,
                            uint16_t service_id, uint8_t* serialed_data,
                            size_t data_len, uint64_t tx_deadline_usec);

int8_t soa_ipc_response_send(uint8_t src_node, uint8_t des_node,
                             uint16_t service_id, uint8_t* serialed_data,
                             size_t data_len, uint64_t tx_deadline_usec);

uint8_t soa_ipc_get_instance_count(void);

#ifdef __cplusplus
}
#endif

#endif /* _SOA_ONIPC_H_ */
