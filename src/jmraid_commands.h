#ifndef _JMRAID_COMMANDS_H_
#define _JMRAID_COMMANDS_H_

#include <stdint.h>

/*
 * JMB39x/JMS56x RAID Controller Protocol
 * Command codes reverse-engineered from raidmgr_static
 */

/* Command type codes (byte 0) */
#define JM_CMD_TYPE_CHIP    0x01
#define JM_CMD_TYPE_SATA    0x02
#define JM_CMD_TYPE_RAID    0x03
#define JM_CMD_TYPE_SPECIAL 0x04

/* Sub-command codes for SATA/Disk operations (type 0x02) */
#define JM_SATA_GET_INFO       0x01  /* Get SATA summary info */
#define JM_SATA_GET_PORT_INFO  0x02  /* Get SATA port details */
#define JM_SATA_ATA_PASSTHRU   0x03  /* ATA passthrough (SMART) */
#define JM_SATA_IDENTIFY_DISK  0x06  /* Blink disk LED */

/* Sub-command codes for RAID operations (type 0x03) */
#define JM_RAID_GET_PORT_INFO  0x02  /* Get RAID port info */
#define JM_RAID_CREATE_DELETE  0x04  /* Create/delete RAID */
#define JM_RAID_MODIFY_PARAM   0x06  /* Modify RAID parameters */

/* ModifyRaidParam operation types (used with JM_RAID_MODIFY_PARAM) */
#define JM_PARAM_REBUILD_PRIORITY 0x04
#define JM_PARAM_ALARM_MUTE       0x06  
#define JM_PARAM_STANDBY_TIMER    0x08

/* Rebuild priority values */
#define JM_REBUILD_PRIORITY_HIGHEST 0x0400
#define JM_REBUILD_PRIORITY_HIGH    0x0800
#define JM_REBUILD_PRIORITY_MEDIUM  0x1000
#define JM_REBUILD_PRIORITY_LOW     0x2000
#define JM_REBUILD_PRIORITY_LOWEST  0x4000

/* Response offsets */
#define JM_RESP_OFFSET_STANDARD 0x0C  /* Standard response data offset */

#endif
