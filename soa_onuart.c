/**
 * soa_onuart.c - SOA UART transport stub
 *
 * The UART transport is not ported.  All APIs are provided as no-op stubs so the
 * router and the HUI node can link; every call returns SOA_NOT_OK except init.
 *
 * 本板 (AI 音箱) 与中央 ECU 之间**只走 CAN** (见 main/soa/hui.h 与
 * main/soa/port/can_bus.h)，所以 UART 传输层当前不需要实现。
 * soa_router_init() 仍会照常调用 soa_uart_init()，这里保持空实现即可。
 *
 * 若将来真的要用 UART (旧方案: 帧协议 SOF 0xA5 0x5A + CRC8/MAXIM，
 * 见 06_TIO)，再在 ESP-IDF UART 驱动上补这套帧协议。
 */
#include "soa_onuart.h"

void soa_uart_init(void)
{
    /* nothing to initialize */
}

int8_t soa_uart_create(uint8_t node_id)
{
    (void)node_id;
    return SOA_NOT_OK;
}

int8_t soa_uart_publish(uint8_t src_node, uint16_t object_id,
                        uint8_t *serialed_data, size_t data_len,
                        uint64_t tx_deadline_usec)
{
    (void)src_node; (void)object_id; (void)serialed_data;
    (void)data_len; (void)tx_deadline_usec;
    return SOA_NOT_OK;
}

int8_t soa_uart_subscribe(uint8_t own_node, uint8_t from_node, Soa_MessageType type,
                          uint16_t expect_id, uint16_t max_size,
                          message_service_callback callback)
{
    (void)own_node; (void)from_node; (void)type;
    (void)expect_id; (void)max_size; (void)callback;
    return SOA_NOT_OK;
}

int8_t soa_uart_subscribe_broadcast(uint8_t own_node, Soa_MessageType type,
                                    uint16_t expect_id, uint16_t max_size,
                                    message_service_callback callback)
{
    (void)own_node; (void)type; (void)expect_id;
    (void)max_size; (void)callback;
    return SOA_NOT_OK;
}

int8_t soa_uart_request_send(uint8_t src_node, uint8_t des_node,
                             uint16_t service_id, uint8_t *serialed_data,
                             size_t data_len, uint64_t tx_deadline_usec)
{
    (void)src_node; (void)des_node; (void)service_id;
    (void)serialed_data; (void)data_len; (void)tx_deadline_usec;
    return SOA_NOT_OK;
}

int8_t soa_uart_response_send(uint8_t src_node, uint8_t des_node,
                              uint16_t service_id, uint8_t *serialed_data,
                              size_t data_len, uint64_t tx_deadline_usec)
{
    (void)src_node; (void)des_node; (void)service_id;
    (void)serialed_data; (void)data_len; (void)tx_deadline_usec;
    return SOA_NOT_OK;
}

uint8_t soa_uart_get_instance_count(void)
{
    return 0;
}
