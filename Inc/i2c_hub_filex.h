#ifndef I2C_HUB_FILEX_H
#define I2C_HUB_FILEX_H

#include "fx_api.h"

#ifdef __cplusplus
extern "C" {
#endif

UINT is_filex_mount(void);
UINT is_hub_filex_unmount(void);

typedef enum {
    FX_WRITE_MODE_NEW = 0,
    FX_WRITE_MODE_APPEND = 1
} FX_WRITE_MODE;

UINT FX_ReadFile(const char *filename, char *buf, uint32_t size, uint32_t *bytesread);
UINT FX_WriteFile(const char *filename, const char *buf, uint32_t size, FX_WRITE_MODE mode);
UINT FX_DeleteFile(const char *filename);
UINT FX_ListDir(const char *path); /* show all entries with details */
UINT FX_ListDirSimple(const char *path); /* simple show single dir */
UINT FX_RemoveDir(const char *path, const char *except, UINT recursive);
UINT FX_Stat(const char *path, FX_DIR_ENTRY *entry);
UINT FX_Rename(const char *old_path, const char *new_path);

#ifdef __cplusplus
}
#endif
#endif /* I2C_HUB_FILEX_H */
