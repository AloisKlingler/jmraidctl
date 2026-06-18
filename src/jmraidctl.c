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

    io_hdr.dxfer_direction = SG_DXFER_FROM_DEV;
    rwCmdBlk[0] = READ_CMD;
    io_hdr.dxferp = resp;
    ioctl(fd, SG_IO, &io_hdr);

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
    }
    return 0;
}

/*
 * Initialize controller at a specific LBA for ATA passthrough
 * Different from RAID management which uses LBA 254
 *
 * This follows the smartmontools sequence:
 * 1. Read original sector (to check state)
 * 2. If non-zero, write zeros to reset state
 * 3. Write 4 wakeup sectors
 */
static int init_controller_at_lba(int fd, uint8_t lba)
{
    uint32_t *probeBuf32 = (uint32_t *)g_probeBuf;
    uint8_t saved_lba = rwCmdBlk[5];
    uint8_t readbuf[SECTORSIZE];

    /* Set LBA for all operations */
    rwCmdBlk[5] = lba;

    /* First read the current sector content */
    io_hdr.dxfer_direction = SG_DXFER_FROM_DEV;
    rwCmdBlk[0] = READ_CMD;
    io_hdr.dxferp = readbuf;
    ioctl(fd, SG_IO, &io_hdr);

    /* Check if sector is non-empty */
    int nonempty = 0;
    for (int i = 0; i < SECTORSIZE; i++) {
        if (readbuf[i] != 0) {
            nonempty = 1;
            break;
        }
    }

    /* If non-empty, zero it to reset controller state */
    if (nonempty) {
        memset(g_probeBuf, 0, SECTORSIZE);
        io_hdr.dxfer_direction = SG_DXFER_TO_DEV;
        rwCmdBlk[0] = WRITE_CMD;
        io_hdr.dxferp = g_probeBuf;
        ioctl(fd, SG_IO, &io_hdr);
    }

    /* Now write the wakeup sequence (without XOR, matching smartmontools) */
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
    }

    /* Restore LBA */
    rwCmdBlk[5] = saved_lba;
    return 0;
}

/* Command ID counter for ATA passthrough at sector 250 */
static uint32_t g_ata_cmd_id = 1;

/* Reset ATA command ID counter - call after init_controller_at_lba */
static void reset_ata_cmd_id(void)
{
    g_ata_cmd_id = 1;
}

/*
 * Run JMB identify disk command to verify connection
 * Must be called after init_controller_at_lba()
 */

static int jmb_identify_disk(int fd, uint32_t scrambled_cmd, int port, uint8_t lba_sector)
{
    uint8_t buf[SECTORSIZE];
    uint8_t resp[SECTORSIZE];
    uint32_t *buf32 = (uint32_t *)buf;

    memset(buf, 0, SECTORSIZE);

    /* Header */
    buf32[0] = __cpu_to_le32(scrambled_cmd);
    buf32[1] = __cpu_to_le32(g_ata_cmd_id++);

    /* JMB identify disk command at offset 8 */
    /* {0x00, 0x02, 0x02, 0xff, port, 0x00,0x00,0x00, port, ...} */
    uint8_t *cmd = buf + 8;
    cmd[0] = 0x00;
    cmd[1] = 0x02;  /* JMS56x: 0x02 */
    cmd[2] = 0x02;
    cmd[3] = 0xff;
    cmd[4] = port;
    cmd[5] = 0x00;
    cmd[6] = 0x00;
    cmd[7] = 0x00;
    cmd[8] = port;
    /* rest is zeros */

    /* CRC + XOR */
    uint32_t crc = JM_CRC(buf32, 0x7f);
    buf32[0x7f] = __cpu_to_le32(crc);
    SATA_XOR(buf32);

    /* Set LBA */
    uint8_t saved_lba = rwCmdBlk[5];
    rwCmdBlk[5] = lba_sector;

    /* Send */
    io_hdr.dxfer_direction = SG_DXFER_TO_DEV;
    rwCmdBlk[0] = WRITE_CMD;
    io_hdr.dxferp = buf;
    ioctl(fd, SG_IO, &io_hdr);

    /* Read response */
    io_hdr.dxfer_direction = SG_DXFER_FROM_DEV;
    rwCmdBlk[0] = READ_CMD;
    io_hdr.dxferp = resp;
    ioctl(fd, SG_IO, &io_hdr);

    rwCmdBlk[5] = saved_lba;

    /* De-XOR and check CRC */
    SATA_XOR((uint32_t *)resp);
    crc = JM_CRC((uint32_t *)resp, 0x7f);
    if (crc != __le32_to_cpu(((uint32_t *)resp)[0x7f])) {
        fprintf(stderr, "JMB identify: CRC error\n");
        return 1;
    }

    /* Check for device model string at offset 16 */
    if (resp[16] < ' ') {
        fprintf(stderr, "JMB identify: No device at port %d\n", port);
        return 1;
    }

    return 0;
}

/*
 * Low-level JMB39x command at specific LBA sector
 */
static uint32_t Do_JM_Cmd_LBA(int fd, uint32_t* cmd, uint32_t* resp, uint8_t lba)
{
    uint32_t myCRC = JM_CRC(cmd, 0x7f);
    cmd[0x7f] = __cpu_to_le32(myCRC);
    SATA_XOR(cmd);

    /* Temporarily change LBA in command block */
    uint8_t saved_lba = rwCmdBlk[5];
    rwCmdBlk[5] = lba;

    io_hdr.dxfer_direction = SG_DXFER_TO_DEV;
    rwCmdBlk[0] = WRITE_CMD;
    io_hdr.dxferp = cmd;
    ioctl(fd, SG_IO, &io_hdr);

    io_hdr.dxfer_direction = SG_DXFER_FROM_DEV;
    rwCmdBlk[0] = READ_CMD;
    io_hdr.dxferp = resp;
    ioctl(fd, SG_IO, &io_hdr);

    /* Restore original LBA */
    rwCmdBlk[5] = saved_lba;

    SATA_XOR(resp);
    myCRC = JM_CRC(resp, 0x7f);
    return (myCRC == __le32_to_cpu(resp[0x7f])) ? 0 : 1;
}

/*
 * Send ATA passthrough command with DATA OUT
 * Based on smartmontools jmb39x protocol analysis
 *
 * Uses sector 250 (0xfa) for ATA passthrough, matching smartmontools behavior.
 *
 * Protocol structure (512-byte sector):
 *   Bytes 0-7:   Header (magic, command number)
 *   Bytes 8-31:  ATA command descriptor (24 bytes)
 *   Bytes 32-507: Data area (for DATA OUT, up to 476 bytes)
 *   Bytes 508-511: CRC
 *
 * ATA command descriptor:
 *   [0] = 0x00
 *   [1] = 0x02 (JM_CMD_TYPE_SATA)
 *   [2] = 0x03 (JM_SATA_ATA_PASSTHRU)
 *   [3] = 0xff
 *   [4] = port (0 or 1)
 *   [5] = ata_read_size: 0x01=NO DATA, 0x02=DATA IN, 0x03=DATA OUT
 *   [6] = 0x00
 *   [7] = ata_read_addr: 0x00 for NO DATA, 0xe0 for DATA IN/OUT
 *   [8-9] = 0x00
 *   [10] = features (low)
 *   [11] = features (high, for 48-bit)
 *   [12] = sector_count (low)
 *   [13] = sector_count (high, for 48-bit)
 *   [14] = lba_low (low)
 *   [15] = lba_low (high)
 *   [16] = lba_mid (low)
 *   [17] = lba_mid (high)
 *   [18] = lba_high (low)
 *   [19] = lba_high (high)
 *   [20] = device (0xa0)
 *   [21] = 0x00
 *   [22] = command
 *   [23] = status (returned)
 */
static int cmd_ata_passthru_out(int fd, uint32_t scrambled_cmd, int port,
                                uint8_t command, uint8_t features, uint8_t sector_count,
                                const uint8_t *data_out, unsigned data_out_size,
                                uint8_t lba_sector)
{
    uint8_t buf[SECTORSIZE];
    uint8_t resp[SECTORSIZE];
    uint32_t *buf32 = (uint32_t *)buf;

    memset(buf, 0, SECTORSIZE);

    /* Header - use same command ID counter as identify */
    buf32[0] = __cpu_to_le32(scrambled_cmd);
    buf32[1] = __cpu_to_le32(g_ata_cmd_id++);

    /* ATA command descriptor at offset 8 */
    uint8_t *cmd = buf + 8;

    cmd[0] = 0x00;
    cmd[1] = JM_CMD_TYPE_SATA;
    cmd[2] = JM_SATA_ATA_PASSTHRU;
    cmd[3] = 0xff;
    cmd[4] = port;
    cmd[5] = 0x03;  /* ata_read_size: 0x03 = DATA OUT */
    cmd[6] = 0x00;
    cmd[7] = 0xe0;  /* ata_read_addr: 0xe0 for DATA OUT */
    cmd[8] = 0x00;
    cmd[9] = 0x00;
    cmd[10] = features;      /* features low */
    cmd[11] = 0x00;          /* features high */
    cmd[12] = sector_count;  /* sector count low */
    cmd[13] = 0x00;          /* sector count high */
    cmd[14] = 0x00;          /* lba_low */
    cmd[15] = 0x00;
    cmd[16] = 0x00;          /* lba_mid */
    cmd[17] = 0x00;
    cmd[18] = 0x00;          /* lba_high */
    cmd[19] = 0x00;
    cmd[20] = 0xa0;          /* device */
    cmd[21] = 0x00;
    cmd[22] = command;
    cmd[23] = 0x00;          /* status (returned) */

    /* Embed data at offset 32 (after 8-byte header + 24-byte command) */
    if (data_out && data_out_size > 0) {
        unsigned max_data = SECTORSIZE - 32 - 4;  /* 476 bytes max */
        if (data_out_size > max_data)
            data_out_size = max_data;
        memcpy(buf + 32, data_out, data_out_size);
    }

    /* Use specific LBA sector for ATA passthrough */
    if (Do_JM_Cmd_LBA(fd, buf32, (uint32_t *)resp, lba_sector) != 0) {
        fprintf(stderr, "CRC error in response\n");
        return 1;
    }

    /* Check ATA status register at offset 31 (8 + 23) */
    uint8_t status = resp[31];
    if (status == 0x00) {
        fprintf(stderr, "ATA passthrough not supported (status=0x00)\n");
        fprintf(stderr, "Note: JMS56x controllers do not support DATA OUT commands like TRIM.\n");
        fprintf(stderr, "Use JMB39x controllers for TRIM support.\n");
        return 1;
    }
    if ((status & 0xc1) != 0x40) {  /* !(!BSY && DRDY && !ERR) */
        if (status & 0x01) {
            fprintf(stderr, "ATA command failed with error (status=0x%02x)\n", status);
            fprintf(stderr, "The drive may not support TRIM.\n");
        } else {
            fprintf(stderr, "ATA command failed (status=0x%02x)\n", status);
        }
        return 1;
    }

    return 0;
}

static int cmd_trim(int fd, uint32_t scrambled_cmd, int port, uint64_t lba, uint16_t count, uint8_t lba_sector)
{
    /*
     * TRIM uses DATA SET MANAGEMENT command (0x06)
     * Features = 0x01 (TRIM bit set)
     * Sector count = number of 512-byte data blocks (1 for up to 64 ranges)
     *
     * Data format: array of 8-byte range descriptors
     *   Bytes 0-5: LBA (48-bit little-endian)
     *   Bytes 6-7: Range length in sectors (16-bit little-endian)
     *
     * We can fit up to 59 ranges in 476 bytes (476/8 = 59)
     */
    uint8_t trim_data[512];
    memset(trim_data, 0, sizeof(trim_data));

    /* Build one TRIM range descriptor */
    trim_data[0] = lba & 0xff;
    trim_data[1] = (lba >> 8) & 0xff;
    trim_data[2] = (lba >> 16) & 0xff;
    trim_data[3] = (lba >> 24) & 0xff;
    trim_data[4] = (lba >> 32) & 0xff;
    trim_data[5] = (lba >> 40) & 0xff;
    trim_data[6] = count & 0xff;
    trim_data[7] = (count >> 8) & 0xff;

    printf("Sending TRIM: LBA=%llu, count=%u sectors to port %d (sector %d)\n",
           (unsigned long long)lba, count, port, lba_sector);

    /* Initialize controller at the ATA passthrough LBA */
    init_controller_at_lba(fd, lba_sector);
    reset_ata_cmd_id();

    /* Run identify disk command to verify connection */
    if (jmb_identify_disk(fd, scrambled_cmd, port, lba_sector) != 0) {
        return 1;
    }

    int ret = cmd_ata_passthru_out(fd, scrambled_cmd, port,
                                   ATA_DATA_SET_MANAGEMENT,
                                   0x01,  /* features: TRIM */
                                   0x01,  /* sector count: 1 block of range descriptors */
                                   trim_data, 512,
                                   lba_sector);

    if (ret == 0) {
        printf("TRIM command completed successfully\n");
    }
    return ret;
}

static int cmd_alarm_mute(int fd, uint32_t scrambled_cmd)
{
    /* Command format: {0x00, TYPE, SUBCMD, 0xff} */
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
    /* Command format: {0x00, TYPE, SUBCMD, 0xff, port, ...params} */
    uint8_t cmd[0x44];
    uint8_t resp[SECTORSIZE];

    memset(cmd, 0, sizeof(cmd));
    cmd[0] = 0x00;
    cmd[1] = JM_CMD_TYPE_SATA;
    cmd[2] = JM_SATA_IDENTIFY_DISK;
    cmd[3] = 0xff;
    cmd[4] = disk;  /* SATA port */

    if (!stop) {
        /* Set blink action and pattern */
        cmd[5] = 0x04;  /* Action: blink */
        cmd[5 + 0x20] = 0x0a;  /* Pattern */
    }
    /* When stop=1, all zeros after port = stop blinking */

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
    /* Get RAID port info to read current standby timer */
    uint8_t cmd[] = {0x00, 0x03, 0x02, 0xff, 0x00};
    uint8_t resp[SECTORSIZE];

    send_cmd(fd, scrambled_cmd, cmd, sizeof(cmd), resp);

    const uint8_t *info = resp + 0x0C;
    /* Standby timer at offset 0x62+4 = 0x66, stored as value/10 */
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
    /* ModifyRaidParam: {0x00, 0x03, 0x06, 0xff, raid_port, op_type, ...params} */
    uint8_t cmd[0x54];
    uint8_t resp[SECTORSIZE];

    memset(cmd, 0, sizeof(cmd));
    cmd[0] = 0x00;
    cmd[1] = JM_CMD_TYPE_RAID;
    cmd[2] = JM_RAID_MODIFY_PARAM;
    cmd[3] = 0xff;
    cmd[4] = 0;  /* raid port */
    cmd[5] = JM_PARAM_STANDBY_TIMER;

    /* Timer value at offset 0x52 (relative to cmd), stored as value/10 */
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
    /* ModifyRaidParam: {0x00, 0x03, 0x06, 0xff, raid_port, op_type, ...params} */
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
    cmd[4] = 0;  /* raid port */
    cmd[5] = JM_PARAM_REBUILD_PRIORITY;

    /* Priority at offset 0x50 */
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
    uint8_t state = info[0x46];  /* 0x42 + 4 */

    const char *states[] = {"Broken", "Degraded", "Rebuilding", "Normal", "Expansion", "Backup"};
    printf("RAID state: %s\n", state < 6 ? states[state] : "Unknown");

    if (state == 0x02) {
        uint32_t progress = info[0x60] | (info[0x61] << 8) |
                           (info[0x62] << 16) | (info[0x63] << 24);
        uint32_t capacity = info[0x40] | (info[0x41] << 8) |
                           (info[0x42] << 16) | (info[0x43] << 24);
        if (capacity > 0) {
            int pct = (int)((uint64_t)progress * 100 / capacity);
            printf("Rebuild progress: %d%%\n", pct);
        }
    }
    return 0;
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

    /* Get RAID port info */
    uint8_t cmd_raid[] = {0x00, 0x03, 0x02, 0xff, 0x00};
    send_cmd(fd, scrambled_cmd, cmd_raid, sizeof(cmd_raid), resp);

    /* Response offset: 0x0C (result_offset) + 0x04 (parse skip) = 0x10 */
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

    /* Parse members */
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
    printf("Usage: %s /dev/sdX <jmb39x|jms56x> [command] [args...]\n\n", prog);
    printf("If no command is given, displays RAID status.\n\n");
    printf("Commands:\n");
    printf("  status               Show RAID status (default)\n");
    printf("  alarm-mute (sa)      Mute alarm buzzer\n");
    printf("  identify (id) <disk> Blink LED on disk (0 or 1)\n");
    printf("  identify <disk> stop Stop LED blinking\n");
    printf("  standby (st) [secs]  Get/set standby timer (0=disable)\n");
    printf("  rebuild-priority (sp) <n>  Set rebuild priority (1=highest..5=lowest)\n");
    printf("  rebuild-status (gr)  Show rebuild status\n");
    printf("  trim <port> <lba> <count> [sector]  Send TRIM (JMB39x only, default sector: 250)\n");
    printf("  -h, --help           Show this help\n");
}

int main(int argc, char *argv[])
{
    /* Handle -h/--help anywhere */
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

    /* No command = show status (like JMraidcon.munin) */
    if (argc < 4) {
        ret = cmd_status(fd, scrambled_cmd, argv[2]);
        close(fd);
        return ret;
    }

    const char *cmd = argv[3];

    if (strcmp(cmd, "status") == 0) {
        ret = cmd_status(fd, scrambled_cmd, argv[2]);
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
            fprintf(stderr, "Usage: trim <port> <lba> <count> [sector]\n");
            fprintf(stderr, "  port:   SATA port (0 or 1)\n");
            fprintf(stderr, "  lba:    Starting LBA to TRIM\n");
            fprintf(stderr, "  count:  Number of sectors to TRIM (max 65535)\n");
            fprintf(stderr, "  sector: Communication sector (default: 250)\n");
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
