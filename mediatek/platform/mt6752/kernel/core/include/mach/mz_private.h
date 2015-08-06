#ifndef __MZ_PRIVATE_H
#define __MZ_PRIVATE_H

/*mz private information data*/
#define ATAG_LK_INFO 0xCA02191A

struct tag_lk_info {
    unsigned int lk_version;
    unsigned int lk_mode;
    unsigned int hw_info;
    unsigned int sw_version;
    unsigned char  sn[64];
    unsigned char  psn[64];
    unsigned char  colortype[8];
    unsigned int rsv[4];
};

extern int lk_info_setup(const char * lk_info);
extern unsigned int mz_get_hw_version(void);
#endif
