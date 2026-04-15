#include "comm.h"

#include <string.h>

#include "bsp_dwt.h"
#include "bsp_usart.h"

typedef struct
{
    USARTInstance *uart;
    uint8_t rx_buf[COMM_RX_BUF_SIZE];
    uint8_t tx_buf[COMM_TX_BUF_SIZE];
    uint8_t rx_updated;
    uint8_t online;
    uint32_t last_feed_ms;
    CommRxCallback_t rx_cb;
} CommInstance_t;

static CommInstance_t comm_instance;

static void CommRestartRxInternal(void);
static void CommHandleOffline(CommInstance_t *comm);
static void CommFeedWatchdogInternal(void);

static uint32_t CommGetTimelineMs(void)
{
    return (uint32_t)DWT_GetTimeline_ms();
}

static void CommUartRxCallback(void)
{
    if (comm_instance.uart == NULL) {
        return;
    }

    memcpy(comm_instance.rx_buf, comm_instance.uart->recv_buff, sizeof(comm_instance.rx_buf));
    comm_instance.rx_updated = 1;
    CommFeedWatchdogInternal();

    if (comm_instance.rx_cb != NULL) {
        comm_instance.rx_cb(comm_instance.rx_buf);
    }
}

static void CommRestartRxInternal(void)
{
    if (comm_instance.uart != NULL) {
        USARTServiceInit(comm_instance.uart);
    }
}

static void CommHandleOffline(CommInstance_t *comm)
{
    memset(comm->rx_buf, 0, sizeof(comm->rx_buf));
    comm->rx_updated = 0;
    comm->online = 0;

    if (comm->uart != NULL) {
        USARTServiceInit(comm->uart);
    }
}

void CommInit(void)
{
    USART_Init_Config_s uart_config;

    if (comm_instance.uart != NULL) {
        return;
    }

    memset(&comm_instance, 0, sizeof(comm_instance));
    memset(&uart_config, 0, sizeof(uart_config));

    uart_config.recv_buff_size = COMM_RX_BUF_SIZE;
    uart_config.usart_handle = COMM_UART_HANDLE;
    uart_config.module_callback = CommUartRxCallback;
    comm_instance.uart = USARTRegister(&uart_config);
    CommFeedWatchdogInternal();
}

uint8_t CommSendDMA(const uint8_t *data, uint16_t len)
{
    if (comm_instance.uart == NULL || data == NULL) {
        return 0;
    }

    if (len == 0 || len > COMM_TX_BUF_SIZE) {
        return 0;
    }

    if (COMM_UART_HANDLE->gState != HAL_UART_STATE_READY) {
        return 0;
    }

    memcpy(comm_instance.tx_buf, data, len);
    USARTSend(comm_instance.uart, comm_instance.tx_buf, len, USART_TRANSFER_DMA);
    return 1;
}

const uint8_t *CommGetRxBuffer(void)
{
    return comm_instance.rx_buf;
}

uint8_t CommHasNewData(void)
{
    return comm_instance.rx_updated;
}

void CommClearRxFlag(void)
{
    comm_instance.rx_updated = 0;
}

void CommSetRxCallback(CommRxCallback_t cb)
{
    comm_instance.rx_cb = cb;
}

static void CommFeedWatchdogInternal(void)
{
    comm_instance.last_feed_ms = CommGetTimelineMs();
    comm_instance.online = 1;
}

uint8_t CommIsOnline(void)
{
    uint32_t now_ms;

    if (comm_instance.uart == NULL) {
        return 0;
    }

    if (!comm_instance.online) {
        return 0;
    }

    now_ms = CommGetTimelineMs();
    if ((now_ms - comm_instance.last_feed_ms) > COMM_WATCHDOG_TIMEOUT_MS) {
        CommHandleOffline(&comm_instance);
        return 0;
    }

    return 1;
}

void CommRestartRx(void)
{
    CommRestartRxInternal();
}
