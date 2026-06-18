/*
 * jmraidctl - JMB39x/JMS56x RAID controller management tool
 *
 * Copyright (C) 2010 Werner Johansson <wj@xnk.nu> (original JMraidcon)
 * Copyright (C) 2024 Extended command support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <scsi/sg.h>
#include <asm/byteorder.h>
#include "jm_crc.h"
#include "sata_xor.h"
#include "jmraid_commands.h"

/* ATA command for TRIM */
#define ATA_DATA_SET_MANAGEMENT 0x06

/*
 * Delay between sector I/O operations.
 * Without this delay, rapid successive writes to the JMB communication sector
 * can cause ATA timeouts and lock up the controller. In this state, the
 * controller disappears from BIOS and a forced power off / power on cycle
 * is required to recover (a soft reboot is not sufficient).
 * 50ms is conservative; the exact minimum safe value is unknown.
 */
#define IO_DELAY_US 50000

#define SECTORSIZE 512
#define READ_CMD 0x28
#define WRITE_CMD 0x2a
#define RW_CMD_LEN 10
#define JM_RAID_WAKEUP_CMD 0x197b0325

static sg_io_hdr_t io_hdr;
static uint8_t rwCmdBlk[RW_CMD_LEN] = {READ_CMD, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x00, 0x00, 0x01, 0x00};
static uint32_t g_cmdNum = 1;

static uint32_t Do_JM_Cmd(int fd, uint32_t* cmd, uint32_t* resp)
{
    uint32_t myCRC = JM_CRC(cmd, 0x7f);
    cmd[0x7f] = __cpu_to_le32(myCRC);
    SATA_XOR(cmd);

    io_hdr.dxfer_direction = SG_DXFER_TO_DEV;
    rwCmdBlk[0] = WRITE_CMD;
    io_hdr.dxferp = cmd;
    ioctl(fd, SG_IO, &io_hdr);
    usleep(IO_DELAY_US);

    io_hdr.dxfer_direction = SG_DXFER_FROM_DEV;
    rwCmdBlk[0] = READ_CMD;
    io_hdr.dxferp = resp;
    ioctl(fd, SG_IO, &io_hdr);
    usleep(IO_DELAY_US);

    SATA_XOR(resp);
    myCRC = JM_CRC(resp, 0x7f);
    return (myCRC == __le32_to_cpu(resp[0x7f])) ? 0 : 1;
}

static void send_cmd(int fd, uint32_t scrambled_cmd, uint8_t* cmd, uint32_t len, uint8_t* resp)
{
    uint8_t buf[SECTORSIZE];
    memset(buf, 0, SECTORSIZE);
    memcpy(buf + 0x08, cmd, len);
    ((uint32_t*)buf)[0] = __cpu_to_le32(scrambled_cmd);
    ((uint32_t*)buf)[1] = __cpu_to_le32(g_cmdNum++);
    Do_JM_Cmd(fd, (uint32_t*)buf, (uint32_t*)resp);
}

static uint8_t g_probeBuf[SECTORSIZE];

static int init_controller(int fd)
{
    uint32_t *probeBuf32 = (uint32_t *)g_probeBuf;

    memset(g_probeBuf, 0, SECTORSIZE);
    probeBuf32[0] = __cpu_to_le32(JM_RAID_WAKEUP_CMD);
    probeBuf32[0x1f8 >> 2] = __cpu_to_le32(0x10eca1db);
    for (uint32_t i = 0x10; i < 0x1f8; i++)
        g_probeBuf[i] = i & 0xff;

    io_hdr.dxfer_direction = SG_DXFER_TO_DEV;
    rwCmdBlk[0] = WRITE_CMD;
    io_hdr.dxferp = g_probeBuf;

    uint32_t wakeup_vals[] = {0x3c75a80b, 0x0388e337, 0x689705f3, 0xe00c523a};
    for (int i = 0; i < 4; i++) {
        probeBuf32[1] = __cpu_to_le32(wakeup_vals[i]);
        uint32_t crc = JM_CRC(probeBuf32, 0x1fc >> 2);
        probeBuf32[0x1fc >> 2] = __cpu_to_le32(crc);
        ioctl(fd, SG_IO, &io_hdr);
        usleep(IO_DELAY_US);
    }
    return 0;
}

static int init_controller_at_lba(int fd, uint8_t lba)
{
    uint32_t *probeBuf32 = (uint32_t *)g_probeBuf;
    uint8_t saved_lba = rwCmdBlk[5];
    uint8_t readbuf[SECTORSIZE];

    rwCmdBlk[5] = lba;

    io_hdr.dxfer_direction = SG_DXFER_FROM_DEV;
    rwCmdBlk[0] = READ_CMD;
    io_hdr.dxferp = readbuf;
    ioctl(fd, SG_IO, &io_hdr);
    usleep(IO_DELAY_US);

    int nonempty = 0;
    for (int i = 0; i < SECTORSIZE; i++) {
        if (readbuf[i] != 0) {
            nonempty = 1;
            break;
        }
    }

    if (nonempty) {
        memset(g_probeBuf, 0, SECTORSIZE);
        io_hdr.dxfer_direction = SG_DXFER_TO_DEV;
        rwCmdBlk[0] = WRITE_CMD;
        io_hdr.dxferp = g_probeBuf;
        ioctl(fd, SG_IO, &io_hdr);
        usleep(IO_DELAY_US);
    }

    memset(g_probeBuf, 0, SECTORSIZE);
    probeBuf32[0] = __cpu_to_le32(JM_RAID_WAKEUP_CMD);
    probeBuf32[0x1f8 >> 2] = __cpu_to_le32(0x10eca1db);
    for (uint32_t i = 0x10; i < 0x1f8; i++)
        g_probeBuf[i] = i & 0xff;

    io_hdr.dxfer_direction = SG_DXFER_TO_DEV;
    rwCmdBlk[0] = WRITE_CMD;
    io_hdr.dxferp = g_probeBuf;

    uint32_t wakeup_vals[] = {0x3c75a80b, 0x0388e337, 0x689705f3, 0xe00c523a};
    for (int i = 0; i < 4; i++) {
        probeBuf32[1] = __cpu_to_le32(wakeup_vals[i]);
        uint32_t crc = JM_CRC(probeBuf32, 0x1fc >> 2);
        probeBuf32[0x1fc >> 2] = __cpu_to_le32(crc);
        ioctl(fd, SG_IO, &io_hdr);
        usleep(IO_DELAY_US);
    }

    rwCmdBlk[5] = saved_lba;
    return 0;
}

static uint32_t g_ata_cmd_id = 1;

static void reset_ata_cmd_id(void)
{
    g_ata_cmd_id = 1;
}

static int jmb_identify_disk(int fd, uint32_t scrambled_cmd, int port, uint8_t lba_sector)
{
    uint8_t buf[SECTORSIZE];
    uint8_t resp[SECTORSIZE];
    uint32_t *buf32 = (uint32_t *)buf;

    memset(buf, 0, SECTORSIZE);
    buf32[0] = __cpu_to_le32(scrambled_cmd);
    buf32[1] = __cpu_to_le32(g_ata_cmd_id++);

    uint8_t *cmd = buf + 8;
    cmd[0] = 0x00;
    cmd[1] = 0x02;
    cmd[2] = 0x02;
    cmd[3] = 0xff;
    cmd[4] = port;
    cmd[8] = port;

    uint32_t crc = JM_CRC(buf32, 0x7f);
    buf32[0x7f] = __cpu_to_le32(crc);
    SATA_XOR(buf32);

    uint8_t saved_lba = rwCmdBlk[5];
    rwCmdBlk[5] = lba_sector;

    io_hdr.dxfer_direction = SG_DXFER_TO_DEV;
    rwCmdBlk[0] = WRITE_CMD;
    io_hdr.dxferp = buf;
    ioctl(fd, SG_IO, &io_hdr);
    usleep(IO_DELAY_US);

    io_hdr.dxfer_direction = SG_DXFER_FROM_DEV;
    rwCmdBlk[0] = READ_CMD;
    io_hdr.dxferp = resp;
    ioctl(fd, SG_IO, &io_hdr);
    usleep(IO_DELAY_US);

    rwCmdBlk[5] = saved_lba;

    SATA_XOR((uint32_t *)resp);
    crc = JM_CRC((uint32_t *)resp, 0x7f);
    if (crc != __le32_to_cpu(((uint32_t *)resp)[0x7f])) {
        return 1;
    }

    if (resp[16] < ' ') {
        return 1;
    }

    return 0;
}

static uint32_t Do_JM_Cmd_LBA(int fd, uint32_t* cmd, uint32_t* resp, uint8_t lba)
{
    uint32_t myCRC = JM_CRC(cmd, 0x7f);
    cmd[0x7f] = __cpu_to_le32(myCRC);
    SATA_XOR(cmd);

    uint8_t saved_lba = rwCmdBlk[5];
    rwCmdBlk[5] = lba;

    io_hdr.dxfer_direction = SG_DXFER_TO_DEV;
    rwCmdBlk[0] = WRITE_CMD;
    io_hdr.dxferp = cmd;
    ioctl(fd, SG_IO, &io_hdr);
    usleep(IO_DELAY_US);

    io_hdr.dxfer_direction = SG_DXFER_FROM_DEV;
    rwCmdBlk[0] = READ_CMD;
    io_hdr.dxferp = resp;
    ioctl(fd, SG_IO, &io_hdr);
    usleep(IO_DELAY_US);

    rwCmdBlk[5] = saved_lba;

    SATA_XOR(resp);
    myCRC = JM_CRC(resp, 0x7f);
    return (myCRC == __le32_to_cpu(resp[0x7f])) ? 0 : 1;
}

static int cmd_ata_passthru_out(int fd, uint32_t scrambled_cmd, int port,
                                uint8_t command, uint8_t features, uint8_t sector_count,
                                const uint8_t *data_out, unsigned data_out_size,
                                uint8_t lba_sector)
{
    uint8_t buf[SECTORSIZE];
    uint8_t resp[SECTORSIZE];
    uint32_t *buf32 = (uint32_t *)buf;

    memset(buf, 0, SECTORSIZE);
    buf32[0] = __cpu_to_le32(scrambled_cmd);
    buf32[1] = __cpu_to_le32(g_ata_cmd_id++);

    uint8_t *cmd = buf + 8;
    cmd[0] = 0x00;
    cmd[1] = JM_CMD_TYPE_SATA;
    cmd[2] = JM_SATA_ATA_PASSTHRU;
    cmd[3] = 0xff;
    cmd[4] = port;
    cmd[5] = 0x03;
    cmd[7] = 0xe0;
    cmd[10] = features;
    cmd[12] = sector_count;
    cmd[20] = 0xa0;
    cmd[22] = command;

    if (data_out && data_out_size > 0) {
        unsigned max_data = SECTORSIZE - 32 - 4;
        if (data_out_size > max_data)
            data_out_size = max_data;
        memcpy(buf + 32, data_out, data_out_size);
    }

    if (Do_JM_Cmd_LBA(fd, buf32, (uint32_t *)resp, lba_sector) != 0) {
        fprintf(stderr, "CRC error in response\n");
        return 1;
    }

    uint8_t status = resp[31];
    if (status == 0x00) {
        fprintf(stderr, "ATA passthrough not supported (status=0x00)\n");
        fprintf(stderr, "Note: JMS56x controllers do not support DATA OUT commands.\n");
        return 1;
    }
    if ((status & 0xc1) != 0x40) {
        if (status & 0x01) {
            fprintf(stderr, "ATA command failed with error (status=0x%02x)\n", status);
        } else {
            fprintf(stderr, "ATA command failed (status=0x%02x)\n", status);
        }
        return 1;
    }

    return 0;
}

static int cmd_trim(int fd, uint32_t scrambled_cmd, int port, uint64_t lba, uint16_t count, uint8_t lba_sector)
{
    uint8_t trim_data[512];
    memset(trim_data, 0, sizeof(trim_data));

    trim_data[0] = lba & 0xff;
    trim_data[1] = (lba >> 8) & 0xff;
    trim_data[2] = (lba >> 16) & 0xff;
    trim_data[3] = (lba >> 24) & 0xff;
    trim_data[4] = (lba >> 32) & 0xff;
    trim_data[5] = (lba >> 40) & 0xff;
    trim_data[6] = count & 0xff;
    trim_data[7] = (count >> 8) & 0xff;

    printf("Sending TRIM: LBA=%llu, count=%u sectors to port %d\n",
           (unsigned long long)lba, count, port);

    init_controller_at_lba(fd, lba_sector);
    reset_ata_cmd_id();

    if (jmb_identify_disk(fd, scrambled_cmd, port, lba_sector) != 0) {
        fprintf(stderr, "No device at port %d\n", port);
        return 1;
    }

    int ret = cmd_ata_passthru_out(fd, scrambled_cmd, port,
                                   ATA_DATA_SET_MANAGEMENT,
                                   0x01, 0x01,
                                   trim_data, 512,
                                   lba_sector);

    if (ret == 0) {
        printf("TRIM command completed successfully\n");
    }
    return ret;
}

static void swap_bytes(uint8_t *data, uint32_t size)
{
    while (size > 1) {
        uint8_t temp = data[0];
        data[0] = data[1];
        data[1] = temp;
        data += 2;
        size -= 2;
    }
}

static uint32_t read_u32_le(const uint8_t *p)
{
    return p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24);
}

static uint16_t read_u16_le(const uint8_t *p)
{
    return p[0] | (p[1] << 8);
}

/* Get chip/firmware info: command {0x01, 0x01} */
static int cmd_info(int fd, uint32_t scrambled_cmd)
{
    uint8_t cmd[] = {0x00, 0x01, 0x01, 0xff};
    uint8_t resp[SECTORSIZE];

    send_cmd(fd, scrambled_cmd, cmd, sizeof(cmd), resp);

    if (resp[0x0B] != 0x00) {
        fprintf(stderr, "Controller info not available (command not supported)\n");
        return 1;
    }

    const uint8_t *p = resp + 0x0C;

    printf("Controller Information\n");
    printf("======================\n");
    printf("Firmware version = %02d.%02d.%02d.%02d\n",
           p[3], p[2], p[1], p[0]);

    char product_name[0x21];
    char manufacturer[0x21];
    memcpy(product_name, p + 0x14, 0x20);
    product_name[0x20] = '\0';
    memcpy(manufacturer, p + 0x34, 0x20);
    manufacturer[0x20] = '\0';

    for (int i = 0x1f; i >= 0 && product_name[i] == ' '; i--)
        product_name[i] = '\0';
    for (int i = 0x1f; i >= 0 && manufacturer[i] == ' '; i--)
        manufacturer[i] = '\0';

    printf("Product name     = %s\n", product_name);
    printf("Manufacturer     = %s\n", manufacturer);
    printf("Serial number    = %u\n", read_u32_le(p + 0xA0));

    return 0;
}

static const char *get_sata_port_type_text(uint8_t port_type)
{
    switch (port_type) {
        case 0x00: return "No Device";
        case 0x01: return "Hard Disk";
        case 0x02: return "RAID Disk";
        case 0x03: return "Optical Drive";
        case 0x04: return "Bad Port";
        case 0x05: return "Skip";
        case 0x06: return "Off";
        case 0x07: return "Host";
        default: return "Unknown";
    }
}

static const char *get_sata_speed_text(uint8_t speed)
{
    switch (speed) {
        case 0x00: return "No Connection";
        case 0x01: return "1.5 Gb/s";
        case 0x02: return "3.0 Gb/s";
        case 0x03: return "6.0 Gb/s";
        default: return "Unknown";
    }
}

/* Get SATA info (all ports): command {0x02, 0x01} */
static int cmd_disks(int fd, uint32_t scrambled_cmd)
{
    uint8_t cmd[] = {0x00, 0x02, 0x01, 0xff};
    uint8_t resp[SECTORSIZE];

    send_cmd(fd, scrambled_cmd, cmd, sizeof(cmd), resp);

    if (resp[0x0B] != 0x00) {
        fprintf(stderr, "SATA info not available (command not supported)\n");
        return 1;
    }

    const uint8_t *p = resp + 0x0C + 0x04;

    printf("SATA Port Information\n");
    printf("=====================\n\n");

    for (int i = 0; i < 5; i++) {
        uint8_t port_type = p[0x48];

        printf("Port %d: %s\n", i, get_sata_port_type_text(port_type));

        if (port_type == 0x01 || port_type == 0x02) {
            char model_name[0x29];
            char serial_number[0x15];

            memcpy(model_name, p, 0x28);
            model_name[0x28] = '\0';
            swap_bytes((uint8_t*)model_name, 0x28);

            memcpy(serial_number, p + 0x28, 0x14);
            serial_number[0x14] = '\0';
            swap_bytes((uint8_t*)serial_number, 0x14);

            for (int j = 0x27; j >= 0 && model_name[j] == ' '; j--)
                model_name[j] = '\0';
            for (int j = 0x13; j >= 0 && serial_number[j] == ' '; j--)
                serial_number[j] = '\0';

            uint64_t capacity = (uint64_t)read_u32_le(p + 0x3C) * 32 * 1024 * 1024;
            uint8_t speed = p[0x4A];

            printf("  Model:    %s\n", model_name);
            printf("  Serial:   %s\n", serial_number);
            printf("  Capacity: %.2f GB\n", (double)capacity / (1024.0 * 1024.0 * 1024.0));
            printf("  Speed:    %s\n", get_sata_speed_text(speed));

            if (port_type == 0x02) {
                printf("  RAID idx: %d, Member: %d\n", p[0x42], p[0x43]);
            }
        }
        printf("\n");
        p += 0x50;
    }

    return 0;
}

static const char *get_smart_attr_name(uint8_t id)
{
    switch (id) {
        case 0x01: return "Raw_Read_Error_Rate";
        case 0x02: return "Throughput_Performance";
        case 0x03: return "Spin_Up_Time";
        case 0x04: return "Start_Stop_Count";
        case 0x05: return "Reallocated_Sector_Ct";
        case 0x07: return "Seek_Error_Rate";
        case 0x08: return "Seek_Time_Performance";
        case 0x09: return "Power_On_Hours";
        case 0x0A: return "Spin_Retry_Count";
        case 0x0B: return "Calibration_Retry_Count";
        case 0x0C: return "Power_Cycle_Count";
        case 0xB7: return "SATA_Downshift_Count";
        case 0xB8: return "End-to-End_Error";
        case 0xBB: return "Reported_Uncorrect";
        case 0xBC: return "Command_Timeout";
        case 0xBD: return "High_Fly_Writes";
        case 0xBE: return "Airflow_Temperature_Cel";
        case 0xBF: return "G-Sense_Error_Rate";
        case 0xC0: return "Power-Off_Retract_Count";
        case 0xC1: return "Load_Cycle_Count";
        case 0xC2: return "Temperature_Celsius";
        case 0xC3: return "Hardware_ECC_Recovered";
        case 0xC4: return "Reallocation_Event_Ct";
        case 0xC5: return "Current_Pending_Sector";
        case 0xC6: return "Offline_Uncorrectable";
        case 0xC7: return "UDMA_CRC_Error_Count";
        case 0xC8: return "Write_Error_Rate";
        case 0xF0: return "Head_Flying_Hours";
        case 0xF1: return "Total_LBAs_Written";
        case 0xF2: return "Total_LBAs_Read";
        default: return "Unknown_Attribute";
    }
}

/*
 * ATA passthrough with DATA IN (for SMART read)
 * Copied from smartmontools dev_jmb39x_raid.cpp ata_pass_through()
 *
 * Command format (24 bytes):
 *   [0-3]  = {0x00, 0x02, 0x03, 0xff}  - JMB header
 *   [4]    = port
 *   [5]    = ata_read_size: 0x02 for DATA IN
 *   [6]    = 0x00
 *   [7]    = ata_read_addr: 0xe0 for DATA IN
 *   [8-9]  = 0x00
 *   [10]   = features (low)
 *   [11]   = features (high, for 48-bit)
 *   [12]   = sector_count (low)
 *   [13]   = sector_count (high)
 *   [14]   = lba_low (low)
 *   [15]   = lba_low (high)
 *   [16]   = lba_mid (low)
 *   [17]   = lba_mid (high)
 *   [18]   = lba_high (low)
 *   [19]   = lba_high (high)
 *   [20]   = device (0xa0)
 *   [21]   = 0x00
 *   [22]   = command
 *   [23]   = status (returned)
 *
 * Response: status at offset 31, data at offset 32 (464 bytes max)
 */
static int cmd_ata_passthru_in(int fd, uint32_t scrambled_cmd, int port,
                               uint8_t command, uint8_t features,
                               uint8_t lba_mid, uint8_t lba_high,
                               uint8_t *data_out, unsigned data_out_size)
{
    uint8_t cmd[24];
    uint8_t resp[SECTORSIZE];

    memset(cmd, 0, sizeof(cmd));

    /* JMB command header */
    cmd[0] = 0x00;
    cmd[1] = JM_CMD_TYPE_SATA;       /* 0x02 */
    cmd[2] = JM_SATA_ATA_PASSTHRU;   /* 0x03 */
    cmd[3] = 0xff;
    cmd[4] = port;
    cmd[5] = 0x02;                   /* ata_read_size: DATA IN */
    cmd[6] = 0x00;
    cmd[7] = 0xe0;                   /* ata_read_addr: 0xe0 for DATA IN */
    cmd[8] = 0x00;
    cmd[9] = 0x00;

    /* ATA registers (low byte, high byte for 48-bit) */
    cmd[10] = features;              /* features low */
    cmd[11] = 0x00;                  /* features high */
    cmd[12] = 0x00;                  /* sector_count low */
    cmd[13] = 0x00;                  /* sector_count high */
    cmd[14] = 0x00;                  /* lba_low low */
    cmd[15] = 0x00;                  /* lba_low high */
    cmd[16] = lba_mid;               /* lba_mid low (0x4F for SMART) */
    cmd[17] = 0x00;                  /* lba_mid high */
    cmd[18] = lba_high;              /* lba_high low (0xC2 for SMART) */
    cmd[19] = 0x00;                  /* lba_high high */
    cmd[20] = 0xa0;                  /* device */
    cmd[21] = 0x00;
    cmd[22] = command;               /* command (0xB0 for SMART) */
    cmd[23] = 0x00;                  /* status returned here */

    send_cmd(fd, scrambled_cmd, cmd, sizeof(cmd), resp);

    /* Check ATA status at response offset 31 (8 header + 23 cmd offset) */
    uint8_t status = resp[31];
    if (status == 0x00) {
        fprintf(stderr, "No device connected to port %d\n", port);
        return 1;
    }
    if ((status & 0xc1) != 0x40) {  /* !(!BSY && DRDY && !ERR) */
        fprintf(stderr, "SMART command failed (status=0x%02x)\n", status);
        return 1;
    }

    /* Copy data from response offset 32 (max 464 bytes due to JMB truncation) */
    unsigned to_copy = data_out_size;
    if (to_copy > 464)
        to_copy = 464;
    memcpy(data_out, resp + 32, to_copy);

    return 0;
}

/* Read SMART attributes from a disk */
static int cmd_smart(int fd, uint32_t scrambled_cmd, int port)
{
    uint8_t smart_data[512];
    uint8_t thresh_data[512];

    memset(smart_data, 0, sizeof(smart_data));
    memset(thresh_data, 0, sizeof(thresh_data));

    printf("Reading SMART data from port %d...\n\n", port);

    /* SMART READ DATA: features=0xD0, lba_mid=0x4F, lba_high=0xC2, cmd=0xB0 */
    if (cmd_ata_passthru_in(fd, scrambled_cmd, port,
                            0xB0, 0xD0, 0x4F, 0xC2,
                            smart_data, 464) != 0) {
        fprintf(stderr, "Failed to read SMART data\n");
        return 1;
    }

    /* SMART READ THRESHOLDS: features=0xD1 */
    if (cmd_ata_passthru_in(fd, scrambled_cmd, port,
                            0xB0, 0xD1, 0x4F, 0xC2,
                            thresh_data, 464) != 0) {
        fprintf(stderr, "Failed to read SMART thresholds\n");
        return 1;
    }

    printf("ID# ATTRIBUTE_NAME          VALUE WORST THRESH RAW\n");

    /* Parse SMART attributes (30 attributes, 12 bytes each, starting at offset 2) */
    /* Note: JMB39x truncates data to 464 bytes, so we can read ~38 attributes max */
    const uint8_t *attr = smart_data + 2;
    const uint8_t *thresh = thresh_data + 2;
    for (int i = 0; i < 30 && (attr - smart_data) < 460; i++) {
        uint8_t id = attr[0];
        if (id == 0)
            break;

        uint8_t value = attr[3];
        uint8_t worst = attr[4];
        uint8_t threshold = thresh[1];  /* threshold at offset 1 in each entry */
        uint64_t raw = (uint64_t)attr[5] | ((uint64_t)attr[6] << 8) |
                      ((uint64_t)attr[7] << 16) | ((uint64_t)attr[8] << 24) |
                      ((uint64_t)attr[9] << 32) | ((uint64_t)attr[10] << 40);

        printf("%3u %-24s %3u   %3u    %3u   %llu\n",
               id, get_smart_attr_name(id), value, worst, threshold,
               (unsigned long long)raw);

        attr += 12;
        thresh += 12;
    }

    return 0;
}

static int cmd_alarm_mute(int fd, uint32_t scrambled_cmd)
{
    uint8_t cmd[] = {0x00, JM_CMD_TYPE_RAID, JM_PARAM_ALARM_MUTE, 0xff};
    uint8_t resp[SECTORSIZE];

    send_cmd(fd, scrambled_cmd, cmd, sizeof(cmd), resp);

    if (resp[0x0B] == 0x00) {
        printf("Alarm muted\n");
        return 0;
    }
    fprintf(stderr, "Failed (status: 0x%02x)\n", resp[0x0B]);
    return 1;
}

static int cmd_identify(int fd, uint32_t scrambled_cmd, int disk, int stop)
{
    uint8_t cmd[0x44];
    uint8_t resp[SECTORSIZE];

    memset(cmd, 0, sizeof(cmd));
    cmd[0] = 0x00;
    cmd[1] = JM_CMD_TYPE_SATA;
    cmd[2] = JM_SATA_IDENTIFY_DISK;
    cmd[3] = 0xff;
    cmd[4] = disk;

    if (!stop) {
        cmd[5] = 0x04;
        cmd[5 + 0x20] = 0x0a;
    }

    send_cmd(fd, scrambled_cmd, cmd, sizeof(cmd), resp);

    if (resp[0x0B] == 0x00) {
        printf("Disk %d identification %s\n", disk, stop ? "stopped" : "started (LED blinking)");
        return 0;
    }
    fprintf(stderr, "Failed (status: 0x%02x)\n", resp[0x0B]);
    return 1;
}

static int cmd_standby_timer_get(int fd, uint32_t scrambled_cmd)
{
    uint8_t cmd[] = {0x00, 0x03, 0x02, 0xff, 0x00};
    uint8_t resp[SECTORSIZE];

    send_cmd(fd, scrambled_cmd, cmd, sizeof(cmd), resp);

    const uint8_t *info = resp + 0x0C;
    uint16_t timer_raw = info[0x66] | (info[0x67] << 8);
    uint16_t timer_secs = timer_raw * 10;

    if (timer_secs > 0)
        printf("Standby timer: %d seconds\n", timer_secs);
    else
        printf("Standby timer: disabled\n");
    return 0;
}

static int cmd_standby_timer_set(int fd, uint32_t scrambled_cmd, int timer_secs)
{
    uint8_t cmd[0x54];
    uint8_t resp[SECTORSIZE];

    memset(cmd, 0, sizeof(cmd));
    cmd[0] = 0x00;
    cmd[1] = JM_CMD_TYPE_RAID;
    cmd[2] = JM_RAID_MODIFY_PARAM;
    cmd[3] = 0xff;
    cmd[4] = 0;
    cmd[5] = JM_PARAM_STANDBY_TIMER;

    uint16_t timer_val = timer_secs / 10;
    cmd[0x52] = timer_val & 0xff;
    cmd[0x53] = (timer_val >> 8) & 0xff;

    send_cmd(fd, scrambled_cmd, cmd, sizeof(cmd), resp);

    if (resp[0x0B] == 0x00) {
        if (timer_secs > 0)
            printf("Standby timer set to %d seconds\n", timer_secs);
        else
            printf("Standby timer disabled\n");
        return 0;
    }
    fprintf(stderr, "Failed (status: 0x%02x)\n", resp[0x0B]);
    return 1;
}

static int cmd_rebuild_priority(int fd, uint32_t scrambled_cmd, int priority)
{
    uint8_t cmd[0x54];
    uint8_t resp[SECTORSIZE];
    uint16_t pval;

    switch (priority) {
        case 1: pval = JM_REBUILD_PRIORITY_HIGHEST; break;
        case 2: pval = JM_REBUILD_PRIORITY_HIGH; break;
        case 3: pval = JM_REBUILD_PRIORITY_MEDIUM; break;
        case 4: pval = JM_REBUILD_PRIORITY_LOW; break;
        case 5: pval = JM_REBUILD_PRIORITY_LOWEST; break;
        default:
            fprintf(stderr, "Priority must be 1-5\n");
            return 1;
    }

    memset(cmd, 0, sizeof(cmd));
    cmd[0] = 0x00;
    cmd[1] = JM_CMD_TYPE_RAID;
    cmd[2] = JM_RAID_MODIFY_PARAM;
    cmd[3] = 0xff;
    cmd[4] = 0;
    cmd[5] = JM_PARAM_REBUILD_PRIORITY;
    cmd[0x50] = pval & 0xff;
    cmd[0x51] = (pval >> 8) & 0xff;

    send_cmd(fd, scrambled_cmd, cmd, sizeof(cmd), resp);

    const char *names[] = {"", "Highest", "High", "Medium", "Low", "Lowest"};
    if (resp[0x0B] == 0x00) {
        printf("Rebuild priority set to %s\n", names[priority]);
        return 0;
    }
    fprintf(stderr, "Failed (status: 0x%02x)\n", resp[0x0B]);
    return 1;
}

static int cmd_rebuild_status(int fd, uint32_t scrambled_cmd)
{
    uint8_t cmd[] = {0x00, 0x03, 0x02, 0xff, 0x00};
    uint8_t resp[SECTORSIZE];

    send_cmd(fd, scrambled_cmd, cmd, sizeof(cmd), resp);

    const uint8_t *info = resp + 0x0C;
    uint8_t state = info[0x46];

    const char *states[] = {"Broken", "Degraded", "Rebuilding", "Normal", "Expansion", "Backup"};
    printf("RAID state: %s\n", state < 6 ? states[state] : "Unknown");

    if (state == 0x02) {
        uint32_t progress = read_u32_le(info + 0x60);
        uint32_t capacity = read_u32_le(info + 0x40);
        if (capacity > 0) {
            int pct = (int)((uint64_t)progress * 100 / capacity);
            printf("Rebuild progress: %d%%\n", pct);
        }
    }
    return 0;
}

static const char *get_raid_level_text(uint8_t level)
{
    switch (level) {
        case 0: return "RAID 0";
        case 1: return "RAID 1";
        case 2: return "JBOD / LARGE";
        case 3: return "RAID 3";
        case 4: return "CLONE";
        case 5: return "RAID 5";
        case 6: return "RAID 10";
        default: return "?";
    }
}

static const char *get_raid_state_text(uint8_t state)
{
    switch (state) {
        case 0: return "Broken";
        case 1: return "Degraded";
        case 2: return "Rebuilding";
        case 3: return "Normal";
        case 4: return "Expansion";
        case 5: return "Backup";
        default: return "?";
    }
}

static const char *get_rebuild_priority_text(uint16_t priority)
{
    if (priority <= 0x0600) return "Highest";
    if (priority <= 0x0c00) return "High";
    if (priority <= 0x1800) return "Medium";
    if (priority <= 0x3000) return "Low";
    return "Lowest";
}

static int cmd_status(int fd, uint32_t scrambled_cmd, const char *controller_type)
{
    uint8_t resp[SECTORSIZE];

    printf("Using %s\n\n", strcmp(controller_type, "jmb39x") == 0 ? "JMB39x" : "JMS56x");

    uint8_t cmd_raid[] = {0x00, 0x03, 0x02, 0xff, 0x00};
    send_cmd(fd, scrambled_cmd, cmd_raid, sizeof(cmd_raid), resp);

    const uint8_t *p = resp + 0x10;

    uint8_t port_state = p[0x40];
    if (port_state == 0) {
        printf("Port state = 0 (no RAID configured)\n");
        return 0;
    }

    char model_name[0x29];
    char serial_number[0x15];
    memcpy(model_name, p, 0x28);
    model_name[0x28] = '\0';
    swap_bytes((uint8_t*)model_name, 0x28);
    memcpy(serial_number, p + 0x28, 0x14);
    serial_number[0x14] = '\0';
    swap_bytes((uint8_t*)serial_number, 0x14);

    uint8_t level = p[0x50];
    uint64_t capacity = (uint64_t)read_u32_le(p + 0x3c) * 32 * 1024 * 1024;
    uint8_t state = p[0x42];
    uint8_t member_count = p[0x51];
    uint16_t rebuild_priority = read_u16_le(p + 0x60);
    uint16_t standby_timer = read_u16_le(p + 0x62) * 10;
    char password[9];
    memcpy(password, p + 0x78, 8);
    password[8] = '\0';
    uint64_t rebuild_progress = (uint64_t)read_u32_le(p + 0x5c) * 32 * 1024 * 1024;

    printf("Model name       = %s\n", model_name);
    printf("Serial number    = %s\n", serial_number);
    printf("Port state       = %d \n", port_state);
    printf("Level            = %d (%s)\n", level, get_raid_level_text(level));
    printf("Capacity         = %.2f GB\n", (double)capacity / (1024.0 * 1024.0 * 1024.0));
    printf("State            = %d (%s)\n", state, get_raid_state_text(state));
    printf("Member count     = %d\n", member_count);
    printf("Rebuild priority = %d (%s)\n", rebuild_priority, get_rebuild_priority_text(rebuild_priority));
    printf("Standby timer    = %d sec\n", standby_timer);
    printf("Password         = %s\n", password);
    printf("Rebuild progress = %.2f %%\n", capacity ? (double)rebuild_progress * 100.0 / capacity : 0.0);

    const uint8_t *mp = p + 0xa0;
    for (int i = 0; i < member_count && i < 5; i++) {
        printf("\nMember %d\n\n", i);
        printf("  Ready         = %d\n", mp[0x00]);
        printf("  LBA48 support = %d\n", mp[0x04]);
        printf("  SATA port     = %d\n", mp[0x07]);
        printf("  SATA page     = %d\n", mp[0x06]);
        printf("  SATA base     = %d\n", read_u32_le(mp + 0x08));
        uint64_t member_size = (uint64_t)read_u32_le(mp + 0x0c) * 32 * 1024 * 1024;
        printf("  SATA size     = %.2f GB\n", (double)member_size / (1024.0 * 1024.0 * 1024.0));
        mp += 0x20;
    }

    return 0;
}

static void usage(const char *prog)
{
    printf("jmraidctl - JMB39x/JMS56x RAID controller management tool\n\n");
    printf("Usage: %s /dev/sgX <jmb39x|jms56x> [command] [args...]\n\n", prog);
    printf("If no command is given, displays RAID status.\n\n");
    printf("Commands:\n");
    printf("  status               Show RAID status (default)\n");
    printf("  info                 Show controller chip/firmware info (JMB39x)\n");
    printf("  disks                Show all SATA ports and disks (JMB39x)\n");
    printf("  smart <P>            Show SMART attributes for port P\n");
    printf("  alarm-mute (sa)      Mute alarm buzzer\n");
    printf("  identify (id) <P>    Blink LED on disk P (0-4)\n");
    printf("  identify <P> stop    Stop LED blinking\n");
    printf("  standby [secs]       Get/set standby timer (0=disable)\n");
    printf("  rebuild-priority <N> Set rebuild priority (1=highest..5=lowest)\n");
    printf("  rebuild-status       Show rebuild progress\n");
    printf("  trim <P> <lba> <cnt> Send TRIM command (JMB39x only)\n");
    printf("  -h, --help           Show this help\n");
}

int main(int argc, char *argv[])
{
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            usage(argv[0]);
            return 0;
        }
    }

    if (argc < 3) {
        usage(argv[0]);
        return 1;
    }

    int fd = open(argv[1], O_RDWR);
    if (fd < 0) {
        perror("Cannot open device");
        return 1;
    }

    int k;
    if (ioctl(fd, SG_GET_VERSION_NUM, &k) < 0 || k < 30000) {
        fprintf(stderr, "%s is not an sg device\n", argv[1]);
        close(fd);
        return 1;
    }

    uint32_t scrambled_cmd;
    if (strcmp(argv[2], "jms56x") == 0) {
        scrambled_cmd = 0x197b0562;
    } else if (strcmp(argv[2], "jmb39x") == 0) {
        scrambled_cmd = 0x197b0322;
    } else {
        fprintf(stderr, "Unknown controller type: %s\n", argv[2]);
        close(fd);
        return 1;
    }

    uint8_t sense_buffer[32];
    memset(&io_hdr, 0, sizeof(sg_io_hdr_t));
    io_hdr.interface_id = 'S';
    io_hdr.cmd_len = sizeof(rwCmdBlk);
    io_hdr.mx_sb_len = sizeof(sense_buffer);
    io_hdr.dxfer_len = SECTORSIZE;
    io_hdr.cmdp = rwCmdBlk;
    io_hdr.sbp = sense_buffer;
    io_hdr.timeout = 3000;

    init_controller(fd);

    int ret = 0;

    if (argc < 4) {
        ret = cmd_status(fd, scrambled_cmd, argv[2]);
        close(fd);
        return ret;
    }

    const char *cmd = argv[3];

    if (strcmp(cmd, "status") == 0) {
        ret = cmd_status(fd, scrambled_cmd, argv[2]);
    }
    else if (strcmp(cmd, "info") == 0) {
        ret = cmd_info(fd, scrambled_cmd);
    }
    else if (strcmp(cmd, "disks") == 0) {
        ret = cmd_disks(fd, scrambled_cmd);
    }
    else if (strcmp(cmd, "smart") == 0) {
        if (argc < 5) {
            fprintf(stderr, "Usage: smart <port>\n");
            ret = 1;
        } else {
            ret = cmd_smart(fd, scrambled_cmd, atoi(argv[4]));
        }
    }
    else if (strcmp(cmd, "alarm-mute") == 0 || strcmp(cmd, "sa") == 0) {
        ret = cmd_alarm_mute(fd, scrambled_cmd);
    }
    else if (strcmp(cmd, "identify") == 0 || strcmp(cmd, "id") == 0) {
        if (argc < 5) {
            fprintf(stderr, "Usage: identify <disk> [stop]\n");
            ret = 1;
        } else {
            int disk = atoi(argv[4]);
            int stop = (argc > 5 && strcmp(argv[5], "stop") == 0);
            ret = cmd_identify(fd, scrambled_cmd, disk, stop);
        }
    }
    else if (strcmp(cmd, "standby") == 0 || strcmp(cmd, "st") == 0) {
        if (argc < 5) {
            ret = cmd_standby_timer_get(fd, scrambled_cmd);
        } else {
            ret = cmd_standby_timer_set(fd, scrambled_cmd, atoi(argv[4]));
        }
    }
    else if (strcmp(cmd, "rebuild-priority") == 0 || strcmp(cmd, "sp") == 0) {
        if (argc < 5) {
            fprintf(stderr, "Usage: rebuild-priority <1-5>\n");
            ret = 1;
        } else {
            ret = cmd_rebuild_priority(fd, scrambled_cmd, atoi(argv[4]));
        }
    }
    else if (strcmp(cmd, "rebuild-status") == 0 || strcmp(cmd, "gr") == 0) {
        ret = cmd_rebuild_status(fd, scrambled_cmd);
    }
    else if (strcmp(cmd, "trim") == 0) {
        if (argc < 7) {
            fprintf(stderr, "Usage: trim <port> <lba> <count>\n");
            ret = 1;
        } else {
            int port = atoi(argv[4]);
            uint64_t lba = strtoull(argv[5], NULL, 0);
            uint16_t count = atoi(argv[6]);
            uint8_t lba_sector = (argc > 7) ? atoi(argv[7]) : 250;
            ret = cmd_trim(fd, scrambled_cmd, port, lba, count, lba_sector);
        }
    }
    else {
        fprintf(stderr, "Unknown command: %s\n", cmd);
        usage(argv[0]);
        ret = 1;
    }

    close(fd);
    return ret;
}
