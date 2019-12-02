/*! ---------------------------------------------------------------------------
 * @file    tag_list.h
 * @brief
 *
 *
 * @author Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#ifndef __INC_TAG_LIST_H__
#define __INC_TAG_LIST_H__ 1

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

//Slot #0 is reserved, i.e. the maximum number of tags is limited by NUM_SLOTS-1; FCONFIG_SIZE and available memory size, see default_config.h
//Depending on a protocol of a discovery process, you need to ensure that the Node will have enough time in Superframe to discover of new tags.
//If there are a possibility of simultaneous ranging of a maximum number of tags, then the practical experience shows that with 
//Tag blink randomisation of 30%, the safe option to have 50% of time been allocated for the discovery of new tags.
//However, if the system has no possibility of a lot of simultaneously ranging tags, then MAX_KNOWN_TAG_LIST_SIZE can be increased up to NUM_SLOTS-1
#define MAX_KNOWN_TAG_LIST_SIZE                (2)
#define MAX_DISCOVERED_TAG_LIST_SIZE           (5)

/* Tag list */

typedef struct
{
    uint16_t    slot;
    union {
        uint8_t        addrShort[2];
        uint16_t    addr16;
    };
    union    {
        uint8_t        addrLong[8];
        uint64_t     addr64;
    };
    uint16_t    multFast;
    uint16_t    multSlow;
    uint16_t    mode;                           //IMU = bit 0

    union    {
        uint8_t        req;
        uint8_t        reqUpdatePending : 1;    //request to update Tag's configuration during range phase
    };
}__attribute__((packed))
tag_addr_slot_t;


tag_addr_slot_t *get_tag16_from_knownTagList(uint16_t addr16);
tag_addr_slot_t *get_tag64_from_knownTagList(uint64_t addr64);
tag_addr_slot_t *add_tag_to_knownTagList(uint64_t addr64, uint16_t addr16);
void del_tag64_from_knownTagList(uint64_t addr64);
void del_tag16_from_knownTagList(uint16_t addr16);

int      addTagToDList(uint64_t addr64);


uint16_t getDList_size(void);
uint64_t *getDList(void);

void init_knownTagList(void);
tag_addr_slot_t *get_knownTagList(void);
uint16_t get_knownTagList_size(void);


void initDList(void);

#ifdef __cplusplus
}
#endif

#endif /* __INC_TAG_LIST_H_ */
