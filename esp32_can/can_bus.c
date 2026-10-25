/**
 * @file can_bus.c
 * @brief CAN (TWAI) 总线驱动实现 —— 从 MIO 工程 (ESP32-C6) 移植, 用 IDF 6.x 新 TWAI 驱动
 *
 * 结构 (被新驱动的形态决定, 说明见 can_bus.h):
 *
 *   RX:  硬件中断 -> on_rx_done (ISR) -> twai_node_receive_from_isr
 *                 -> xQueueSendFromISR -> can_rx_task (任务上下文)
 *                 -> can_rx_handler / can_extra_rx_handler
 *
 *   TX:  can_send (任务) -> 占用一个持久 TX 槽 (s_tx[]) -> twai_node_transmit
 *                 -> on_tx_done (ISR) 归还槽位
 *
 * ⚠️ 为什么 TX 需要「持久槽位」:
 *   新驱动的 twai_node_transmit() **不拷贝**数据 —— 它只把 `const twai_frame_t*`
 *   存进自己的队列 (见 esp_twai_onchip.c: `twai_ctx->p_curr_tx[0] = frame;` 与
 *   twai_frame_queue_push_safe(..., frame, ...)), 等硬件发完才通过 on_tx_done
 *   把同一个指针还回来。legacy 的 twai_transmit() 是拷贝语义, 所以 MIO 版可以
 *   传栈上的 twai_message_t 然后立刻释放 canard 的缓冲。
 *   如果这里照抄, 就会出现「驱动还持有指针, 调用方已经 free 了」的 use-after-free。
 *   因此本层准备固定数量的槽位 (frame + 数据缓冲都是静态存储), 发完才归还;
 *   槽位用尽就返回 ESP_ERR_TIMEOUT, 让上层 (soa_oncan 的 inst_tx_flush) 稍后重试。
 *
 * ⚠️ 回调上下文: 所有 on_xxx 回调都在 ISR 里跑 (禁止阻塞 API / 禁止 printf 式日志),
 *   所以真正的日志和总线状态汇报都放在接收任务里做。
 */

#include "can_bus.h"

#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_log.h"
#include "esp_err.h"

#include <inttypes.h>
#include <string.h>

#define TAG "can"

#define CAN_RX_TASK_STACK_SIZE   4096
#define CAN_RX_TASK_PRIORITY     5
#define CAN_TX_TIMEOUT_MS        10    /* 驱动发送队列满时的等待上限 */
#define CAN_TX_QUEUE_DEPTH       8     /* 驱动内部发送队列深度 */
#define CAN_TX_SLOTS             4     /* 本层持久发送槽位数 (见文件头说明) */
#define CAN_RX_QUEUE_DEPTH       16
#define CAN_STATE_POLL_MS        200   /* 接收任务超时: 顺带汇报状态变化 / 自动 bus-off 恢复 */

/** 总线掉到 bus-off 后自动尝试恢复 (CAN 节点的常规做法; 置 0 关闭) */
#define CAN_AUTO_RECOVER_ON_BUS_OFF  1
#define CAN_RECOVER_RETRY_MS         1000

/* ---------- 持久发送槽位 ---------- */
typedef struct
{
    twai_frame_t frame;                      /* 驱动会长期持有这个指针, 必须是静态存储 */
    uint8_t      buf[TWAI_FRAME_MAX_LEN];    /* 数据也必须是静态存储 */
    volatile bool in_use;
} can_tx_slot_t;

static can_tx_slot_t s_tx[CAN_TX_SLOTS];
static portMUX_TYPE  s_tx_lock = portMUX_INITIALIZER_UNLOCKED;

/* ---------- 运行状态 ---------- */
static twai_node_handle_t     s_node    = NULL;
static QueueHandle_t          s_rx_queue = NULL;
static TaskHandle_t           s_rx_task  = NULL;
static bool                   s_started = false;   /* 节点已建立且当前启用 */
static bool                   s_standby = false;   /* 处于待机 (收发器 standby + 节点 disable) */

static can_rx_handler_t       s_rx_handler = NULL;
static can_extra_rx_handler_t s_extra_rx_handler = NULL;

static gpio_num_t             s_stb_gpio = GPIO_NUM_NC;
static int                    s_stb_standby_level = 1;

static volatile can_bus_state_t s_bus_state = CAN_BUS_STATE_UNKNOWN;
static volatile uint32_t        s_state_change_cnt = 0;   /* ISR 置位: 有状态变化待汇报 */
static volatile uint32_t        s_rx_dropped = 0;         /* ISR 置位: 接收队列满丢帧数 */

/* ===================== 小工具 ===================== */

/** 把 STB 拉到指定电平 (GPIO_NUM_NC 表示本板没有 STB 控制脚) */
static void can_set_stb_level(bool standby)
{
    if (s_stb_gpio == GPIO_NUM_NC) {
        return;
    }
    gpio_set_level(s_stb_gpio, standby ? s_stb_standby_level : !s_stb_standby_level);
}

static can_bus_state_t can_map_state(twai_error_state_t st)
{
    switch (st) {
        case TWAI_ERROR_ACTIVE:  return CAN_BUS_STATE_OK;
        case TWAI_ERROR_WARNING: return CAN_BUS_STATE_WARNING;
        case TWAI_ERROR_PASSIVE: return CAN_BUS_STATE_PASSIVE;
        case TWAI_ERROR_BUS_OFF: return CAN_BUS_STATE_BUS_OFF;
        default:                 return CAN_BUS_STATE_UNKNOWN;
    }
}

static const char *can_state_name(can_bus_state_t st)
{
    switch (st) {
        case CAN_BUS_STATE_OK:      return "OK(error-active)";
        case CAN_BUS_STATE_WARNING: return "WARNING";
        case CAN_BUS_STATE_PASSIVE: return "PASSIVE";
        case CAN_BUS_STATE_BUS_OFF: return "BUS-OFF";
        default:                    return "UNKNOWN";
    }
}

/* ===================== ISR 回调 ===================== */

/**
 * 收到一帧。**在 ISR 里跑**: 只做拷贝 + 入队, 不做任何阻塞/日志。
 * @note 若将来打开 CONFIG_TWAI_ISR_CACHE_SAFE=y, 本函数必须加 IRAM_ATTR,
 *       否则 twai_node_register_event_callbacks() 会直接返回 ESP_ERR_INVALID_ARG。
 */
static bool can_on_rx_done(twai_node_handle_t handle,
                           const twai_rx_done_event_data_t *edata, void *user_ctx)
{
    (void)edata; (void)user_ctx;

    uint8_t      raw[TWAI_FRAME_MAX_LEN];
    twai_frame_t rx = {
        .buffer     = raw,
        .buffer_len = TWAI_FRAME_MAX_LEN,
    };

    if (twai_node_receive_from_isr(handle, &rx) != ESP_OK) {
        return false;
    }

    can_frame_t frame = { 0 };
    frame.id       = rx.header.id;
    frame.extended = rx.header.ide != 0;
    frame.rtr      = rx.header.rtr != 0;

    /* 经典 CAN 的 dlc 就是字节数; 只有 FD 帧才需要 twaifd_dlc2len() 换算。
       本板跑的是经典 CAN (Cyphal/CAN 也是经典帧)。 */
    uint32_t len = rx.header.fdf ? twaifd_dlc2len(rx.header.dlc) : rx.header.dlc;
    if (len > sizeof(frame.data)) {
        len = sizeof(frame.data);
    }
    frame.dlc = (uint8_t)len;
    if (!frame.rtr && len > 0) {
        memcpy(frame.data, raw, len);
    }

    BaseType_t woken = pdFALSE;
    if (xQueueSendFromISR(s_rx_queue, &frame, &woken) != pdTRUE) {
        s_rx_dropped++;     /* 队列满: 丢帧 (并把唤醒令牌让给别人, 不能阻塞 ISR) */
        return woken == pdTRUE;
    }
    return woken == pdTRUE;
}

/** 一帧发送完成 (成功或失败都算): 归还对应的持久槽位。在 ISR 里跑。 */
static bool can_on_tx_done(twai_node_handle_t handle,
                           const twai_tx_done_event_data_t *edata, void *user_ctx)
{
    (void)handle; (void)user_ctx;

    if (edata->done_tx_frame == NULL) {
        return false;
    }
    for (int i = 0; i < CAN_TX_SLOTS; i++) {
        if (edata->done_tx_frame == &s_tx[i].frame) {
            portENTER_CRITICAL_ISR(&s_tx_lock);
            s_tx[i].in_use = false;
            portEXIT_CRITICAL_ISR(&s_tx_lock);
            break;
        }
    }
    return false;
}

/** 总线错误状态变化 (error-active/warning/passive/bus-off)。在 ISR 里跑: 只记录, 任务里打印。 */
static bool can_on_state_change(twai_node_handle_t handle,
                                const twai_state_change_event_data_t *edata, void *user_ctx)
{
    (void)handle; (void)user_ctx;
    s_bus_state = can_map_state(edata->new_sta);
    s_state_change_cnt++;
    return false;
}

/* ===================== 接收任务 ===================== */

static void can_rx_task(void *arg)
{
    (void)arg;
    can_frame_t frame;
#if CAN_AUTO_RECOVER_ON_BUS_OFF
    TickType_t last_recover = 0;
#endif

    for (;;)
    {
        if (xQueueReceive(s_rx_queue, &frame, pdMS_TO_TICKS(CAN_STATE_POLL_MS)) == pdTRUE) {
            /* 上层回调在**任务上下文**里跑, 所以它们可以自由打日志 / 用 FreeRTOS API */
            if (s_rx_handler != NULL) {
                s_rx_handler(&frame);
            }
            if (s_extra_rx_handler != NULL) {
                s_extra_rx_handler(&frame);
            }
        }

        /* --- 以下都只在超时/空闲路径里做, 不影响收包延迟 --- */

        if (s_state_change_cnt != 0) {
            s_state_change_cnt = 0;
            const can_bus_state_t st = s_bus_state;
            if (st == CAN_BUS_STATE_OK) {
                ESP_LOGI(TAG, "总线状态: %s", can_state_name(st));
            } else {
                ESP_LOGW(TAG, "总线状态: %s", can_state_name(st));
            }
        }

        if (s_rx_dropped != 0) {
            ESP_LOGW(TAG, "接收队列满, 已丢 %" PRIu32 " 帧 (提高优先级/减少上层阻塞)", s_rx_dropped);
            s_rx_dropped = 0;
        }

#if CAN_AUTO_RECOVER_ON_BUS_OFF
        /* bus-off 后驱动不会自己回来, 必须显式 recover。
           ⚠️ 待机时节点也是 bus-off 状态 (那是我们主动 disable 的), 不能去 recover。 */
        if (!s_standby && s_bus_state == CAN_BUS_STATE_BUS_OFF) {
            const TickType_t now = xTaskGetTickCount();
            if ((now - last_recover) >= pdMS_TO_TICKS(CAN_RECOVER_RETRY_MS)) {
                last_recover = now;
                if (twai_node_recover(s_node) == ESP_OK) {
                    ESP_LOGW(TAG, "从 BUS-OFF 发起恢复");
                }
            }
        }
#endif
    }
}

/* ===================== 对外 API ===================== */

void can_set_rx_handler(can_rx_handler_t handler)
{
    s_rx_handler = handler;
}

void can_set_extra_rx_handler(can_extra_rx_handler_t handler)
{
    s_extra_rx_handler = handler;
}

esp_err_t can_init(const can_bus_config_t *cfg)
{
    if (s_started) {
        return ESP_OK;
    }
    if (cfg == NULL) {
        ESP_LOGE(TAG, "can_init: cfg 不能为 NULL");
        return ESP_ERR_INVALID_ARG;
    }

    s_stb_gpio = cfg->stb_gpio;
    s_stb_standby_level = cfg->stb_standby_level ? 1 : 0;

    /* 1. 收发器 STB: 输出, 先放到「正常通信」电平 */
    if (s_stb_gpio != GPIO_NUM_NC) {
        gpio_config_t stb_conf = {
            .pin_bit_mask = 1ULL << s_stb_gpio,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        esp_err_t gpio_err = gpio_config(&stb_conf);
        if (gpio_err != ESP_OK) {
            ESP_LOGE(TAG, "STB (GPIO%d) 配置失败: %s", s_stb_gpio, esp_err_to_name(gpio_err));
            return gpio_err;
        }
        can_set_stb_level(false);
    }

    /* 2. 接收队列: 必须早于 enable, 否则第一帧进来时还没有队列可放 */
    s_rx_queue = xQueueCreate(CAN_RX_QUEUE_DEPTH, sizeof(can_frame_t));
    if (s_rx_queue == NULL) {
        ESP_LOGE(TAG, "创建 CAN 接收队列失败");
        return ESP_ERR_NO_MEM;
    }

    /* 3. 建立节点。波特率直接给数字; 时钟源 0 = 用驱动的默认值 (APB) */
    const uint32_t bitrate = cfg->bitrate ? cfg->bitrate : CAN_BUS_BITRATE_500K;
    twai_onchip_node_config_t node_cfg = {
        .io_cfg = {
            .tx                = cfg->tx_gpio,
            .rx                = cfg->rx_gpio,
            .quanta_clk_out    = GPIO_NUM_NC,   /* 不需要时钟输出脚 */
            .bus_off_indicator = GPIO_NUM_NC,   /* 不需要 bus-off 指示脚 */
        },
        .clk_src      = TWAI_CLK_SRC_DEFAULT,
        .bit_timing   = { .bitrate = bitrate, .sp_permill = 0, .ssp_permill = 0 },
        .data_timing  = { .bitrate = 0 },       /* 经典 CAN, 不用 FD 数据段时序 */
        .timestamp_resolution_hz = 0,           /* 关掉时间戳功能 */
        .fail_retry_cnt = -1,                   /* 失败无限重发 (与 legacy 默认一致) */
        .tx_queue_depth = CAN_TX_QUEUE_DEPTH,
        .intr_priority  = 0,                    /* 0 = 驱动默认优先级 */
        .flags = {
            .enable_self_test   = 0,
            .enable_loopback    = 0,
            .enable_listen_only = 0,
            .no_receive_rtr     = 0,
            .sleep_allow_pd     = 0,
            .enable_scheduled_tx = 0,
        },
    };

    esp_err_t err = twai_new_node_onchip(&node_cfg, &s_node);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "建立 TWAI 节点失败: %s", esp_err_to_name(err));
        goto fail;
    }

    /* 4. 接收全部报文 (等价于 legacy 的 TWAI_FILTER_CONFIG_ACCEPT_ALL)。
        ⚠️ id=0 / mask=0 在驱动里是 "full_open", 会同时放行标准帧和扩展帧 ——
        SOA 用的全是 29 位扩展帧, 这里千万不能配成 std-only。
        过滤器只能在节点未启用时配置。 */
    const twai_mask_filter_config_t filter = {
        .id   = 0,
        .mask = 0,
        .is_ext = 0,
        .no_classic = 0,
        .no_fd = 0,
        .dual_filter = 0,
    };
    err = twai_node_config_mask_filter(s_node, 0, &filter);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "配置接收过滤器失败: %s", esp_err_to_name(err));
        goto fail;
    }

    /* 5. 注册回调 (同样只能在节点未启用时注册) */
    const twai_event_callbacks_t cbs = {
        .on_tx_done      = can_on_tx_done,
        .on_rx_done      = can_on_rx_done,
        .on_state_change = can_on_state_change,
        .on_error        = NULL,
    };
    err = twai_node_register_event_callbacks(s_node, &cbs, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "注册 TWAI 回调失败: %s", esp_err_to_name(err));
        goto fail;
    }

    /* 6. 启动 */
    err = twai_node_enable(s_node);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "启用 TWAI 节点失败: %s", esp_err_to_name(err));
        goto fail;
    }
    s_started = true;
    s_standby = false;
    s_bus_state = CAN_BUS_STATE_OK;

    /* 7. 接收任务 */
    if (xTaskCreate(can_rx_task, "can_rx", CAN_RX_TASK_STACK_SIZE, NULL,
                    CAN_RX_TASK_PRIORITY, &s_rx_task) != pdPASS) {
        ESP_LOGE(TAG, "CAN 接收任务创建失败");
        s_rx_task = NULL;
        err = ESP_ERR_NO_MEM;
        (void)twai_node_disable(s_node);
        s_started = false;
        goto fail;
    }

    ESP_LOGI(TAG, "CAN (TWAI) 初始化完成: TX=GPIO%d RX=GPIO%d STB=%d bitrate=%" PRIu32,
             cfg->tx_gpio, cfg->rx_gpio, cfg->stb_gpio, bitrate);
    return ESP_OK;

fail:
    if (s_node != NULL) {
        (void)twai_node_delete(s_node);
        s_node = NULL;
    }
    if (s_rx_queue != NULL) {
        vQueueDelete(s_rx_queue);
        s_rx_queue = NULL;
    }
    return err;
}

esp_err_t can_send(const can_frame_t *frame)
{
    if (!s_started || s_node == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (frame == NULL || frame->dlc > TWAI_FRAME_MAX_LEN) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 取一个空闲的持久槽位 (驱动只持有指针, 数据必须活到 on_tx_done) */
    can_tx_slot_t *slot = NULL;
    portENTER_CRITICAL(&s_tx_lock);
    for (int i = 0; i < CAN_TX_SLOTS; i++) {
        if (!s_tx[i].in_use) {
            s_tx[i].in_use = true;
            slot = &s_tx[i];
            break;
        }
    }
    portEXIT_CRITICAL(&s_tx_lock);

    if (slot == NULL) {
        /* 上一批还在发 (总线没应答会一直重发): 让上层稍后重试, 不要在这里等 */
        return ESP_ERR_TIMEOUT;
    }

    const uint8_t len = frame->dlc;
    if (len > 0 && !frame->rtr) {
        memcpy(slot->buf, frame->data, len);
    }
    slot->frame.header.id    = frame->id;
    slot->frame.header.dlc   = len;
    slot->frame.header.ide   = frame->extended ? 1u : 0u;
    slot->frame.header.rtr   = frame->rtr ? 1u : 0u;
    slot->frame.header.fdf   = 0;    /* 经典 CAN 帧 */
    slot->frame.header.brs   = 0;
    slot->frame.header.esi   = 0;
    slot->frame.header.timestamp = 0;
    slot->frame.buffer       = slot->buf;
    slot->frame.buffer_len   = len;
    slot->frame.tx_queue_priority = 0;

    const esp_err_t err = twai_node_transmit(s_node, &slot->frame, CAN_TX_TIMEOUT_MS);
    if (err != ESP_OK) {
        portENTER_CRITICAL(&s_tx_lock);
        slot->in_use = false;
        portEXIT_CRITICAL(&s_tx_lock);
    }
    return err;
}

bool can_is_started(void)
{
    return s_started;
}

can_bus_state_t can_get_bus_state(void)
{
    return s_bus_state;
}

/* ============== 总线待机 (留给以后做低功耗, 当前无调用者) ============== */

esp_err_t can_enter_standby(void)
{
    if (!s_started || s_standby) {
        return ESP_OK;
    }

    /* 顺序不可颠倒: 先让收发器静默, 再关断控制器,
       否则关控制器的瞬间可能把半截帧发到总线上 */
    can_set_stb_level(true);

    esp_err_t err = twai_node_disable(s_node);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "TWAI 停止失败: %s", esp_err_to_name(err));
        return err;
    }

    s_started = false;
    s_standby = true;
    s_bus_state = CAN_BUS_STATE_UNKNOWN;   /* disable 后驱动也报 bus-off, 不当作真实故障 */
    ESP_LOGI(TAG, "CAN 进入待机 (收发器 standby + TWAI 停止)");
    return ESP_OK;
}

esp_err_t can_exit_standby(void)
{
    if (s_node == NULL || !s_standby) {
        return ESP_OK;
    }

    esp_err_t err = twai_node_enable(s_node);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "TWAI 重启失败: %s", esp_err_to_name(err));
        return err;
    }

    /* 收发器回到正常通信模式 */
    can_set_stb_level(false);

    s_started = true;
    s_standby = false;
    s_bus_state = CAN_BUS_STATE_OK;
    ESP_LOGI(TAG, "CAN 退出待机 (TWAI 重启 + 收发器正常)");
    return ESP_OK;
}

bool can_is_in_standby(void)
{
    return s_standby;
}
