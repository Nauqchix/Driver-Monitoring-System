#include "Update_firmware.h"
#include "stm32f4xx_hal.h"
#include <string.h>

/* ===================== USER CONFIG ===================== */
extern UART_HandleTypeDef huart1;
#define BL_UART        huart1

#define CHUNK_MAX      256
#define UART_TIMEOUT   5000

/* ===================== PROTOCOL ===================== */
#define MAGIC0 0xB1
#define MAGIC1 0x0A

#define TYPE_START 1
#define TYPE_DATA  2
#define TYPE_END   3

#define ACK  0x79
#define NACK 0x1F

typedef struct __attribute__((packed)) {
    uint8_t  m0;
    uint8_t  m1;
    uint8_t  type;
    uint8_t  seq;
    uint16_t len;     // little-endian
    uint16_t crc16;   // CRC16-CCITT on (type,seq,len,payload)
} BL_Header;

/* ===================== LOW-LEVEL UART ===================== */
static HAL_StatusTypeDef uart_read_exact(uint8_t *buf, uint32_t n, uint32_t timeout_ms)
{
    return HAL_UART_Receive(&BL_UART, buf, n, timeout_ms);
}

static void uart_send_byte(uint8_t b)
{
    HAL_UART_Transmit(&BL_UART, &b, 1, 100);
}

/* ===================== CRC16-CCITT (0x1021, init 0xFFFF) ===================== */
//static uint16_t crc16_ccitt_update(uint16_t crc, uint8_t data)
//{
//    crc ^= (uint16_t)data << 8;
//    for (int i = 0; i < 8; i++) {
//        crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
//    }
//    return crc;
//}

//static uint16_t crc16_ccitt(const uint8_t *data, uint32_t len)
//{
//    uint16_t crc = 0xFFFF;
//    for (uint32_t i = 0; i < len; i++) crc = crc16_ccitt_update(crc, data[i]);
//    return crc;
//}
//
//static uint16_t packet_crc16(const BL_Header *h, const uint8_t *payload)
//{
//    uint16_t crc = 0xFFFF;
//    crc = crc16_ccitt_update(crc, h->type);
//    crc = crc16_ccitt_update(crc, h->seq);
//    crc = crc16_ccitt_update(crc, (uint8_t)(h->len & 0xFF));
//    crc = crc16_ccitt_update(crc, (uint8_t)(h->len >> 8));
//    for (uint32_t i = 0; i < h->len; i++) crc = crc16_ccitt_update(crc, payload[i]);
//    return crc;
//}

/* ===================== CRC32 (MPEG-2 style, no reflect) ===================== */
static uint32_t crc32_mpeg2(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= ((uint32_t)data[i]) << 24;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80000000U) ? (crc << 1) ^ 0x04C11DB7U : (crc << 1);
        }
    }
    return crc;
}

/* ===================== FLASH SIZE (auto-detect) ===================== */
static uint32_t flash_end_addr(void)
{
    // FLASHSIZE_BASE holds flash size in KB (STM32)
    uint16_t kb = *(volatile uint16_t*)FLASHSIZE_BASE;
    return 0x08000000U + ((uint32_t)kb * 1024U) - 1U;
}

/* ===================== SECTOR MAPPING ===================== */
static const uint32_t sector_addr[] = {
    0x08000000, // S0  16K
    0x08004000, // S1  16K
    0x08008000, // S2  16K
    0x0800C000, // S3  16K
    0x08010000, // S4  64K
    0x08020000, // S5  128K
    0x08040000, // S6  128K
    0x08060000, // S7  128K
    0x08080000, // S8  128K
    0x080A0000, // S9  128K
    0x080C0000, // S10 128K
    0x080E0000, // S11 128K
    0x08100000  // end marker
};

static uint32_t addr_to_sector(uint32_t addr)
{
    // returns sector index 0..11
    for (uint32_t i = 0; i < 12; i++) {
        if (addr < sector_addr[i + 1]) return i;
    }
    return 11;
}

static uint32_t sector_index_to_hal(uint32_t idx)
{
    static const uint32_t map[] = {
        FLASH_SECTOR_0, FLASH_SECTOR_1, FLASH_SECTOR_2, FLASH_SECTOR_3,
        FLASH_SECTOR_4, FLASH_SECTOR_5, FLASH_SECTOR_6, FLASH_SECTOR_7,
        FLASH_SECTOR_8, FLASH_SECTOR_9, FLASH_SECTOR_10, FLASH_SECTOR_11
    };
    return map[(idx <= 11) ? idx : 11];
}

/* ===================== FLASH ERASE / WRITE ===================== */
HAL_StatusTypeDef erase_app(uint32_t fw_size)
{
    if (fw_size == 0) return HAL_ERROR;

    uint32_t end = APP_ADDR + fw_size - 1U;
    uint32_t flash_end = flash_end_addr();
    if (end > flash_end) return HAL_ERROR;

    uint32_t first_idx = addr_to_sector(APP_ADDR); // should be 2
    uint32_t last_idx  = addr_to_sector(end);

    FLASH_EraseInitTypeDef erase = {0};
    uint32_t sector_error = 0;

    erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    erase.Sector       = sector_index_to_hal(first_idx);
    erase.NbSectors    = (last_idx - first_idx) + 1U;

    HAL_FLASH_Unlock();
    HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&erase, &sector_error);
    HAL_FLASH_Lock();

    return st;
}

HAL_StatusTypeDef write_app(uint32_t address, const uint8_t *data, uint32_t length)
{
    if (length == 0) return HAL_OK;
    if (address < APP_ADDR) return HAL_ERROR;

    uint32_t flash_end = flash_end_addr();
    if ((address + length - 1U) > flash_end) return HAL_ERROR;

    HAL_FLASH_Unlock();
    HAL_StatusTypeDef st = HAL_OK;

    for (uint32_t i = 0; i < length; i += 4) {
        uint32_t word = 0xFFFFFFFFU;
        uint32_t remain = length - i;
        uint32_t take = (remain >= 4) ? 4 : remain;

        for (uint32_t b = 0; b < take; b++) {
            ((uint8_t*)&word)[b] = data[i + b];
        }

        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + i, word);
        if (st != HAL_OK) break;
    }

    HAL_FLASH_Lock();
    return st;
}

int verify_app(uint32_t address, const uint8_t *data, uint32_t length)
{
    const uint8_t *p = (const uint8_t*)address;
    for (uint32_t i = 0; i < length; i++) {
        if (p[i] != data[i]) return -1;
    }
    return 0;
}

/* ===================== RECEIVE ONE PACKET ===================== */
static int recv_packet(BL_Header *h, uint8_t *payload, uint32_t payload_max)
{

    uint8_t b = 0;
    while (1) {
        if (uart_read_exact(&b, 1, UART_TIMEOUT) != HAL_OK) return -1;
        if (b == MAGIC0) {
            uint8_t b2 = 0;
            if (uart_read_exact(&b2, 1, UART_TIMEOUT) != HAL_OK) return -1;
            if (b2 == MAGIC1) { h->m0 = b; h->m1 = b2; break; }
        }
    }

    if (uart_read_exact(((uint8_t*)h) + 2, sizeof(BL_Header) - 2, UART_TIMEOUT) != HAL_OK) return -1;

    if (h->len > payload_max) return -2;

    if (h->len > 0) {
        if (uart_read_exact(payload, h->len, UART_TIMEOUT) != HAL_OK) return -1;
    }

    // CRC16 check
    uint16_t c = packet_crc16(h, payload);
    if (c != h->crc16) return -3;

    return 0;
}



/* ===================== MAIN UPDATE FUNCTION ===================== */
void bootloader_update(void)
{
    uint8_t payload[CHUNK_MAX];

    uint32_t fw_size = 0;
    uint32_t fw_crc  = 0;

    uint32_t written = 0;
    uint32_t addr = APP_ADDR;
    uint8_t expect_seq = 0;

    if (HAL_UART_Receive(&huart1, (uint8_t*)&fw_size, 4, 3000) != HAL_OK)
        {
    		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
            return;
        }

    while (1)
    {
        BL_Header h;
        int r = recv_packet(&h, payload, CHUNK_MAX);
        if (r != 0) { uart_send_byte(NACK); continue; }

        if (h.type == TYPE_START)
        {
            if (h.len != 8) { uart_send_byte(NACK); continue; }

            // little-endian parse
            memcpy(&fw_size, &payload[0], 4);
            memcpy(&fw_crc,  &payload[4], 4);

            // basic sanity
            if (fw_size == 0) { uart_send_byte(NACK); continue; }

            if (erase_app(fw_size) != HAL_OK) { uart_send_byte(NACK); continue; }

            // reset state
            written = 0;
            addr = APP_ADDR;
            expect_seq = 0;

            uart_send_byte(ACK);
        }
        else if (h.type == TYPE_DATA)
        {
            if (fw_size == 0) { uart_send_byte(NACK); continue; }
            if (h.seq != expect_seq) { uart_send_byte(NACK); continue; }
            if (written + h.len > fw_size) { uart_send_byte(NACK); continue; }

            if (write_app(addr, payload, h.len) != HAL_OK) { uart_send_byte(NACK); continue; }
            if (verify_app(addr, payload, h.len) != 0)     { uart_send_byte(NACK); continue; }

            addr += h.len;
            written += h.len;
            expect_seq++;

            uart_send_byte(ACK);
        }
        else if (h.type == TYPE_END)
        {
            if (fw_size == 0) { uart_send_byte(NACK); continue; }
            if (written != fw_size) { uart_send_byte(NACK); continue; }

            uint32_t calc = crc32_mpeg2((const uint8_t*)APP_ADDR, fw_size);
            if (calc != fw_crc) { uart_send_byte(NACK); continue; }

            uart_send_byte(ACK);

            HAL_Delay(50);
            jump_to_app();

        }
        else
        {
            uart_send_byte(NACK);
        }
    }
}
