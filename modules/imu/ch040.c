#include "ch040.h"

static DaemonInstance *ch040_daemon_instance;
static Ch040UsartInstance *Ch040Usart;


static void crc16_update(uint16_t *currectCrc, const uint8_t *src, uint32_t lengthInBytes)
{
    uint32_t crc = *currectCrc;
    uint32_t j;
    for (j = 0; j < lengthInBytes; ++j) {
        uint32_t i;
        uint32_t byte = src[j];
        crc ^= byte << 8;
        for (i = 0; i < 8; ++i) {
            uint32_t temp = crc << 1;
            if (crc & 0x8000) {
                temp ^= 0x1021;
            }
            crc = temp;
        }
    }
    *currectCrc = crc;
}

static void Ch040_callback()
{
    static uint8_t *recv_buff;
    recv_buff = Ch040Usart->Ch040_usart->recv_buff;
    if (recv_buff == NULL) // 空数据包，则不作任何处理
        return;
    // 解析数据
    if ((recv_buff[0] != 0x5A) || (recv_buff[1] != 0xA5)) // 帧头错误
        return;
    else {
        static int16_t payload_len;
        static uint16_t crc;
        crc         = 0;
        payload_len = recv_buff[2] + (recv_buff[3] << 8);
        crc16_update(&crc, recv_buff, 4);
        crc16_update(&crc, recv_buff + 6, payload_len);
        if (recv_buff[4] != (crc & 0xFF) || recv_buff[5] != (crc >> 8)) // CRC校验错误
            return;
        else {
            int offset                       = 6; /* Payload start at buf[6] */
            Ch040Usart->ch040_data.tag       = U1(recv_buff + offset + 0);
            Ch040Usart->ch040_data.pressure  = R4(recv_buff + offset + 4);
            Ch040Usart->ch040_data.timestamp = U4(recv_buff + offset + 8);
            Ch040Usart->ch040_data.acc[0]    = R4(recv_buff + offset + 12);
            Ch040Usart->ch040_data.acc[1]    = R4(recv_buff + offset + 16);
            Ch040Usart->ch040_data.acc[2]    = R4(recv_buff + offset + 20);
            Ch040Usart->ch040_data.gyr[0]    = R4(recv_buff + offset + 24);
            Ch040Usart->ch040_data.gyr[1]    = R4(recv_buff + offset + 28);
            Ch040Usart->ch040_data.gyr[2]    = R4(recv_buff + offset + 32);
            Ch040Usart->ch040_data.mag[0]    = R4(recv_buff + offset + 36);
            Ch040Usart->ch040_data.mag[1]    = R4(recv_buff + offset + 40);
            Ch040Usart->ch040_data.mag[2]    = R4(recv_buff + offset + 44);
            Ch040Usart->ch040_data.eul[0]    = R4(recv_buff + offset + 48);
            Ch040Usart->ch040_data.eul[1]    = R4(recv_buff + offset + 52);
            Ch040Usart->ch040_data.eul[2]    = R4(recv_buff + offset + 56);
            Ch040Usart->ch040_data.quat[0]   = R4(recv_buff + offset + 60);
            Ch040Usart->ch040_data.quat[1]   = R4(recv_buff + offset + 64);
            Ch040Usart->ch040_data.quat[2]   = R4(recv_buff + offset + 68);
            Ch040Usart->ch040_data.quat[3]   = R4(recv_buff + offset + 72);
        }
    }
}

Ch040UsartInstance *Ch040_USART_Init(void)
{
    Ch040UsartInstance *instance = (Ch040UsartInstance *)malloc(sizeof(Ch040UsartInstance));
    memset(instance, 0, sizeof(Ch040UsartInstance));

    USART_Init_Config_s CH040Data;
    CH040Data.module_callback = Ch040_callback;
    CH040Data.recv_buff_size  = 82;
    CH040Data.usart_handle    = &huart1;
    instance->Ch040_usart   = USARTRegister(&CH040Data);
    if (instance->Ch040_usart == NULL) {
        // USART注册失败
        free(instance);
        return NULL;
    }
    Ch040Usart = instance;
    return instance;
}



static void DecodeCh040(CANInstance *_instance)
{
}
Ch040CanInstance *Ch040_Init(void)
{
    Ch040CanInstance *instance = (Ch040CanInstance *)malloc(sizeof(Ch040CanInstance));
    memset(instance, 0, sizeof(Ch040CanInstance));

    CAN_Init_Config_s can_config = {
        .can_handle          = &hcan2,
        .rx_id               = 0x33,
        .can_module_callback = DecodeCh040,
        .id                  = instance, // 将instance的地址传递给can_instance的id,以便在回调函数中使用
    };

    instance->can_ins = CANRegister(&can_config);

    // 守护进程配置
    Daemon_Init_Config_s daemon_config = {
        .reload_count = 50, // 50*10ms=500ms
        .owner_id     = instance,
        .callback     = NULL, // 可以在此处添加异常处理函数
    };
    ch040_daemon_instance = DaemonRegister(&daemon_config);

    return instance;
}