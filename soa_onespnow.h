#ifndef _SOA_ONESPNOW_H_
#define _SOA_ONESPNOW_H_

#include "soa_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 同一块板子上最多几个逻辑节点挂在 ESP-NOW 上 (与 onipc/onuart 保持一致) */
#define SOA_ESPNOW_MAX_INSTANCES     4U
/* node_id -> MAC 映射表大小 (不含广播地址) */
#define SOA_ESPNOW_MAX_PEERS         8U
/* 每个逻辑节点最多几条订阅 */
#define SOA_ESPNOW_MAX_SUBS_PER_INST 16U
/* 单帧 payload 上限: 250 (ESP_NOW_MAX_DATA_LEN) - 9 字节帧头 - 1 字节余量 */
#define SOA_ESPNOW_MAX_PAYLOAD       240U
#define SOA_ESPNOW_MAC_LEN           6U
/* 广播用的逻辑节点号 (帧头 des_node), 同时就是 SOA_NODE_ANY */
#define SOA_ESPNOW_BROADCAST_NODE    SOA_NODE_ANY

/**
 * @brief 初始化 ESP-NOW 传输层 (清空实例/peer 表, 建接收队列与分发任务)。
 *
 * @note 这里**不会**启动 WiFi。esp_now_init() 要求调用前 WiFi 已经 start,
 *       所以第一次真正用到传输层时才尝试初始化 (见 soa_espnow_is_ready())。
 *       若启动顺序不确定, 之后再调用一次本函数或任意一个发送/建节点接口即可,
 *       失败会自动重试, 日志只在状态变化时打印一次。
 *
 * @note 本层独占 esp_now_init()/esp_now_add_peer()。同一固件里不要再有第二个
 *       模块初始化 ESP-NOW; 若它已经占用了接收回调, SOA 帧不会再被分发到这里。
 */
void soa_espnow_init(void);

/**
 * @brief 在 ESP-NOW 上注册一个本地逻辑节点。
 * @param node_id 逻辑节点号 (1..127)。
 * @return SOA_OK on success.
 */
int8_t soa_espnow_create(uint8_t node_id);

/**
 * @brief 登记 (node_id -> MAC) 映射, 让发往该节点的报文走单播。
 *
 * 未登记的节点一律走 ESP-NOW 广播 (帧头 des_node 仍然是目标节点号), 因此
 * “先能通, 再配单播” 的调试顺序是可行的, 只是广播没有 ACK 也更容易被干扰。
 *
 * @param node_id 对端逻辑节点号。
 * @param mac     对端 STA MAC (6 字节, 可用 soa_espnow_get_mac() 在对端读出)。
 * @return SOA_OK on success.
 */
int8_t soa_espnow_add_peer(uint8_t node_id, const uint8_t mac[SOA_ESPNOW_MAC_LEN]);

/**
 * @brief 删除上面登记的映射 (被删除后发往该节点会回退到广播)。
 */
int8_t soa_espnow_remove_peer(uint8_t node_id);

/**
 * @brief 读取本机 STA MAC —— 给对端配 peer 表用。
 * @note 需要 WiFi 已启动; 未启动时返回 SOA_NOT_OK。
 */
int8_t soa_espnow_get_mac(uint8_t mac[SOA_ESPNOW_MAC_LEN]);

/**
 * @brief ESP-NOW 是否已经初始化成功 (WiFi 已启动且 esp_now_init() 通过)。
 */
int8_t soa_espnow_is_ready(void);

/**
 * @brief 可选辅助: 把 WiFi 起成 STA 模式 (不连 AP), 让 ESP-NOW 能用。
 *
 * 只做射频需要的最小动作: esp_netif_init + 默认事件循环 + esp_wifi_init +
 * set_storage(RAM) + set_mode(STA) + start, 然后初始化 ESP-NOW。
 *   - 不连接任何 AP, 不启动 DHCP 客户端;
 *   - storage 用 RAM, 不往 NVS 写 PHY/校准数据 (纯 ESP-NOW 节点常常没有可写 NVS);
 *   - 关闭 modem sleep (未连 AP 时它会丢帧)。
 *
 * 幂等: 若 WiFi 驱动已经初始化 (例如应用自己起了 WiFi), 本函数**不会**重 init、
 * 也不会改它的模式, 只确保 ESP-NOW 就绪。
 *
 * @note 应用自己管理 WiFi (连 AP / AP 配网 / 省电策略) 时不要调用本函数。
 * @return SOA_OK = WiFi 已启动且 ESP-NOW 就绪; SOA_NOT_OK 否则。
 */
int8_t soa_espnow_wifi_start_sta(void);

/**
 * @brief 在 ESP-NOW 上广播一条消息 (des_node = 0xFF)。
 */
int8_t soa_espnow_publish(uint8_t src_node, uint16_t object_id,
                          uint8_t* serialed_data, size_t data_len,
                          uint64_t tx_deadline_usec);

/**
 * @brief 在 ESP-NOW 上订阅 (from_node 可以是具体节点, 也可以是 SOA_NODE_ANY)。
 */
int8_t soa_espnow_subscribe(uint8_t own_node, uint8_t from_node, Soa_MessageType type,
                            uint16_t expect_id, uint16_t max_size,
                            message_service_callback callback);

int8_t soa_espnow_subscribe_broadcast(uint8_t own_node, Soa_MessageType type,
                                      uint16_t expect_id, uint16_t max_size,
                                      message_service_callback callback);

/**
 * @brief 发送服务请求 (单播; 没有 peer 映射时回退广播)。
 */
int8_t soa_espnow_request_send(uint8_t src_node, uint8_t des_node,
                               uint16_t service_id, uint8_t* serialed_data,
                               size_t data_len, uint64_t tx_deadline_usec);

/**
 * @brief 发送服务应答 (单播; 没有 peer 映射时回退广播)。
 */
int8_t soa_espnow_response_send(uint8_t src_node, uint8_t des_node,
                                uint16_t service_id, uint8_t* serialed_data,
                                size_t data_len, uint64_t tx_deadline_usec);

/**
 * @brief 当前注册的逻辑节点数。
 */
uint8_t soa_espnow_get_instance_count(void);

#ifdef __cplusplus
}
#endif

#endif /* _SOA_ONESPNOW_H_ */
