#ifndef __FP_VENDOR_H__
#define __FP_VENDOR_H__

enum {
    FP_VENDOR_INVALID = 0,
    FPC_VENDOR,
    GOODIX_VENDOR,
    MICROARRAY_VENDOR,
};

#define FP_IOC_MAGIC 'F'
#define GET_FP_VENDOR_CMD   _IOWR(FP_IOC_MAGIC,1,int)

int get_fp_vendor(void);

#endif  /*__FP_VENDOR_H__*/
