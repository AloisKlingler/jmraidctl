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
    else {
        fprintf(stderr, "Unknown command: %s\n", cmd);
        usage(argv[0]);
        ret = 1;
    }

    close(fd);
    return ret;
}
