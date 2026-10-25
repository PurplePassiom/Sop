/**
 * @file can_bus.h
 * @brief CAN (TWAI) 总线驱动接口 —— 从 MIO 工程移植, 板子解耦 + 用 IDF 6.x 新驱动
 *
 * 与 MIO 版本的差异:
 *   - MIO 版把 PIN_CAN_TX / PIN_CAN_RX / PIN_CAN_STB_EN 写死在 hw/io_config.h 里,
 *     属于「核心代码依赖具体板子」。移植到 xiaozhi 后改为由**板级**通过
 *     can_bus_config_t 传入 (引脚仍写在板子的 config.h 里), 核心代码不认识任何板子。
 *   - MIO 版的收发器待机控制走 io_set_can_standby(); 这里直接驱动 STB GPIO。
 *   - 新增 bitrate 字段 (MIO 固定 500 kbps)。
 *   - **改用 IDF 6.x 的新 TWAI 驱动 (`esp_twai.h`)**, 不再用 legacy `driver/twai.h`。
 *     三处形态变化:
 *       1) 报文类型用本文件自己的 `can_frame_t`, 上层不必 include 任何 TWAI 头文件
 *          (legacy 的 `twai_message_t` 已是 deprecated 类型);
 *       2) 接收是**中断回调**驱动 —— 新驱动**没有**「任务里阻塞收包」的 API。
 *          所以本层在 ISR 里把帧塞进队列, 再由自己的接收任务派发给上层回调,
 *          上层回调因此仍在**任务上下文**执行 (可以打日志 / 用 xTaskNotify),
 *          与 MIO 版的约定完全一致;
 *       3) 波特率直接给数字 (legacy 要传 TWAI_TIMING_CONFIG_500KBITS() 位时序宏)。
 *
 * 引脚只由板级 config.h 提供, 见 main/boards/<board>/config.h 里的 CAN_* 宏。
 *
 * 总线待机 (留给以后做低功耗/总线静默): can_enter_standby() / can_exit_standby()
 * 分别对应新驱动的 twai_node_disable() / twai_node_enable()。
 * ⚠️ 协同休眠已从本工程移除, 这两个函数**当前没有任何调用者**。
 */

#ifndef CAN_BUS_H
#define CAN_BUS_H

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 常见 CAN 波特率 */
#define CAN_BUS_BITRATE_125K    125000U
#define CAN_BUS_BITRATE_250K    250000U
#define CAN_BUS_BITRATE_500K    500000U
#define CAN_BUS_BITRATE_1M      1000000U

/**
 * @brief 一帧经典 CAN 报文 (最多 8 字节数据)。
 *
 * 刻意不暴露驱动类型: 上层 (soa_oncan 等) 只认这个结构, 换驱动不用改上层。
 */
typedef struct
{
    uint32_t id;        /**< 仲裁 ID: 11 位标准帧 / 29 位扩展帧 (纯 ID, 不带标志位) */
    uint8_t  dlc;       /**< 数据字节数 (0..8) */
    bool     extended;  /**< true = 29 位扩展帧, false = 11 位标准帧 */
    bool     rtr;       /**< true = 远程帧 (无数据段) */
    uint8_t  data[8];   /**< 数据, 仅前 dlc 字节有效 */
} can_frame_t;

/**
 * @brief 板级 CAN 参数。由板级初始化填充后传给 can_init()。
 *
 * 板级代码 (main/boards 下对应板子的 .cc) 有 config.h 的可见性, 核心代码没有,
 * 因此引脚一律走这里传入, 不要在核心代码里 include 某个板子的 config.h。
 */
typedef struct
{
    gpio_num_t tx_gpio;             /**< CAN TX  -> 收发器 TXD */
    gpio_num_t rx_gpio;             /**< CAN RX  <- 收发器 RXD */
    gpio_num_t stb_gpio;            /**< 收发器 STB, 没有就填 GPIO_NUM_NC */
    int        stb_standby_level;   /**< STB 处于待机时的电平 (常规收发器为 1) */
    uint32_t   bitrate;             /**< 波特率, 用 CAN_BUS_BITRATE_* 或 0 取 500k */
} can_bus_config_t;

/**
 * @brief 总线健康状态 (新硬件 bring-up 时很有用: 能区分「没接对 / 缺终端电阻 / 速率错」)
 */
typedef enum
{
    CAN_BUS_STATE_OK      = 0,  /**< Error active: 通信正常 */
    CAN_BUS_STATE_WARNING,      /**< Error warning: 错误计数偏高 (>=96) */
    CAN_BUS_STATE_PASSIVE,      /**< Error passive: 被动错误 (>=128) */
    CAN_BUS_STATE_BUS_OFF,      /**< Bus-off: 掉线 */
    CAN_BUS_STATE_UNKNOWN,      /**< 驱动未启动 / 未知 */
} can_bus_state_t;

/* ------------------------------ 收发回调 ------------------------------ */

/**
 * @brief CAN 报文接收回调。
 * @note 在**接收任务上下文**中调用 (不是 ISR) —— 可以打日志、可以用非 ISR 的 FreeRTOS API。
 */
typedef void (*can_rx_handler_t)(const can_frame_t *frame);

/** @brief 设置 CAN 报文接收回调 (can_init 前后均可设置) */
void can_set_rx_handler(can_rx_handler_t handler);

/**
 * @brief 额外注册一个原始帧处理器 (在 can_rx_handler 之后被调用)。
 *        用于让 SOA 传输层 (soa_oncan) 消费 CAN 帧, 避免与调试回调抢报文。
 */
typedef void (*can_extra_rx_handler_t)(const can_frame_t *frame);
void can_set_extra_rx_handler(can_extra_rx_handler_t handler);

/* ------------------------------ 生命周期 ------------------------------ */

/**
 * @brief 建立并启用 TWAI 节点 (接受所有报文), 并创建接收任务。
 *        同时将收发器退出待机 (STB 拉到正常电平)。
 *
 * @param cfg  板级参数, 不可为 NULL
 * @return ESP_OK 成功
 */
esp_err_t can_init(const can_bus_config_t *cfg);

/**
 * @brief 发送一帧 (任务上下文调用)。发送队列满时最多等 CAN_TX_TIMEOUT_MS。
 * @return ESP_OK = 已排入发送队列
 */
esp_err_t can_send(const can_frame_t *frame);

/** @brief 驱动是否已建立且当前处于启用状态 */
bool can_is_started(void);

/** @brief 当前总线健康状态 */
can_bus_state_t can_get_bus_state(void);

/* ============== 总线待机 (留给以后做低功耗, 当前无调用者) ============== */

/**
 * @brief 进入总线待机: 先让收发器进 standby, 再关断 TWAI 节点。
 *        此时总线上任何活动都不会再被本节点接收。
 * @return ESP_OK 成功
 */
esp_err_t can_enter_standby(void);

/**
 * @brief 退出总线待机: 重新启用 TWAI 节点并把收发器拉回正常模式。
 *        唤醒后必须调用, 否则节点收不到任何报文。
 * @return ESP_OK 成功
 */
esp_err_t can_exit_standby(void);

/** @brief 当前是否处于总线待机 */
bool can_is_in_standby(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_BUS_H */
