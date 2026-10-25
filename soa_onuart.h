#ifndef _SOA_ONUART_H_
#define _SOA_ONUART_H_

#include "soa_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum number of UART transport instances (logical nodes on the same UART) */
#define SOA_UART_MAX_INSTANCES  4U

/**
 * @brief Initialize the UART transport layer.
 *        Must be called once at startup before any other soa_uart_* API.
 */
void soa_uart_init(void);

/**
 * @brief Register a logical node on the UART transport.
 * @param node_id  Logical node ID to create.
 * @return SOA_OK on success.
 */
int8_t soa_uart_create(uint8_t node_id);

/**
 * @brief Publish a broadcast message over UART.
 * @param src_node      Sending node ID.
 * @param object_id     Port/subject ID.
 * @param serialed_data Serialized payload.
 * @param data_len      Payload length in bytes.
 * @return SOA_OK on success.
 */
int8_t soa_uart_publish(uint8_t src_node, uint16_t object_id,
                        uint8_t *serialed_data, size_t data_len,
                        uint64_t tx_deadline_usec);

/**
 * @brief Subscribe to a message/service on UART.
 * @param own_node   Receiving node ID (must be registered via soa_uart_create).
 * @param from_node  Expected sender node ID, or SOA_NODE_ANY for wildcard.
 * @param type       Message type (SoaBroadcast / SoaRequest / SoaResponse).
 * @param expect_id  Port/service ID to match.
 * @param max_size   Maximum expected payload size.
 * @param callback   Callback invoked on frame receipt.
 * @return SOA_OK on success.
 */
int8_t soa_uart_subscribe(uint8_t own_node, uint8_t from_node, Soa_MessageType type,
                          uint16_t expect_id, uint16_t max_size,
                          message_service_callback callback);

int8_t soa_uart_subscribe_broadcast(uint8_t own_node, Soa_MessageType type, uint16_t expect_id, uint16_t max_size,
                                    message_service_callback callback);

/**
 * @brief Send a service request over UART.
 */
int8_t soa_uart_request_send(uint8_t src_node, uint8_t des_node,
                             uint16_t service_id, uint8_t *serialed_data,
                             size_t data_len, uint64_t tx_deadline_usec);

/**
 * @brief Send a service response over UART.
 */
int8_t soa_uart_response_send(uint8_t src_node, uint8_t des_node,
                              uint16_t service_id, uint8_t *serialed_data,
                              size_t data_len, uint64_t tx_deadline_usec);

/**
 * @brief Return the number of active UART instances.
 */
uint8_t soa_uart_get_instance_count(void);

#ifdef __cplusplus
}
#endif

#endif /* _SOA_ONUART_H_ */
