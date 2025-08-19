#ifndef I2C_HUB_FILEX_H
#define I2C_HUB_FILEX_H

#include "i2c_hub.h"
#include "fx_api.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    char   *buf;
    size_t  cap;
    size_t  used;
    int     truncated;
} wr_t;

void i2c_hub_filex_init(void);

uint8_t *i2c_hub_filex_helper(uint8_t *tx_frame_ptr, const hub_cmd_t *cmd);

UINT i2c_hub_filex_mount(void);
UINT i2c_hub_filex_unmount(void);
UINT i2c_read_file_to_buf(FX_MEDIA *m, const char *abspath,
                         ULONG64 offset, UCHAR *dst, ULONG len, ULONG *out_read);
UINT i2c_write_file_from_buf(FX_MEDIA *m, const char *abspath,
                            ULONG64 offset, const UCHAR *src, ULONG len,
                            ULONG *out_written, UINT create_if_missing, UINT truncate);
UINT i2c_get_file_size64(FX_MEDIA *m, const char *abspath, ULONG64 *out_size);
UINT i2c_delete_file_path(FX_MEDIA *m, const char *abspath);
UINT i2c_mkdirs(FX_MEDIA *m, const char *absdir);

#ifdef __cplusplus
}
#endif
#endif /* I2C_HUB_FILEX_H */
