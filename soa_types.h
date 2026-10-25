#ifndef _SOA_TYPES_H_
#define _SOA_TYPES_H_

#include <stdint.h>
#include <stddef.h>

#define SOA_OK      0
#define SOA_NOT_OK  1

/* ---------------------------------------------------------------------------
 * node_id 空间: 1 ~ 127 (7 bit)。SOA_NODE_ANY = 0xFF 是通配符, 不是合法节点号。
 *
 * 号段规划 (见 MIO架构设计.md §4.6/§4.7):
 *   1  ~ 63    功能节点 —— 抽象某个控制器的节点 (下面这些)
 *   64 ~ 127   诊断节点 —— 一个硬件布置一个诊断服务, 用独立 node_id
 *     ⚠️ 诊断不复用功能节点的 node_id, 也不做成功能节点里的一个服务。
 *
 * HUI_NODE_ID = 6 就是本板 (AI 音箱): 它以 dsdl_CenterAndHui.* 这套服务与
 * 中央 ECU 对接, 详见 main/soa/hui.h。
 * ------------------------------------------------------------------------- */
#define MASTER_NODE_ID      2u
#define DISPLAY_NODE_ID     3u
#define BCM_NODE_ID         4u
#define MCU_NODE_ID         5u
#define HUI_NODE_ID         6u
#define BMS_NODE_ID         7u

#define SOA_NODE_ID_MAX     127u    /* node_id 上限 */
#define SOA_NODE_ANY        0xFFu   /* wildcard: match any source node */

typedef enum {
    SoaBroadcast = 0,
    SoaResponse  = 1,
    SoaRequest   = 2,
} Soa_MessageType;

typedef void (*message_service_callback)(const uint8_t* data, size_t data_len);

#endif /* _SOA_TYPES_H_ */
