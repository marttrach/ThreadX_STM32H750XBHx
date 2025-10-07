#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <ctype.h>

#include "i2c_hub_filex.h"
#include "fx_stm32_sd_driver.h"
#include "app_filex.h"
#include "iot.h"

#define PATH_BUFFER_LEN   FX_MAXIMUM_PATH
#define ENTRY_NAME_LEN    FX_MAX_LONG_NAME_LEN
#define IO_CHUNK_SIZE     65536U

static volatile UINT g_sd_mounted = 0U;
static volatile UINT g_fx_initialized = 0U;

static inline int media_is_open(const FX_MEDIA *m)
{
    return (m && m->fx_media_id == FX_MEDIA_ID);
}

static UINT ensure_fx_initialized(void)
{
    if (!g_fx_initialized) {
        fx_system_initialize();
        g_fx_initialized = 1U;
    }
    return FX_SUCCESS;
}

static UINT ensure_sd_media_open(void)
{
    UINT st = ensure_fx_initialized();
    if (st != FX_SUCCESS) {
        return st;
    }

    if (g_sd_mounted && media_is_open(&sdio_disk)) {
        return FX_SUCCESS;
    }

    st = fx_media_open(&sdio_disk,
                       (CHAR *)FX_SD_VOLUME_NAME,
                       fx_stm32_sd_driver,
                       FX_NULL,
                       (VOID *)fx_sd_media_memory,
                       sizeof(fx_sd_media_memory));
    if (st == FX_SUCCESS) {
        g_sd_mounted = 1U;
    } else {
        DEBUG_DUMP(IOT_LOG_ERR, "FILEX media open failed: 0x%02X\r\n", st);
    }
    return st;
}

UINT is_filex_mount(void)
{
    return ensure_sd_media_open();
}

UINT is_hub_filex_unmount(void)
{
    if (!media_is_open(&sdio_disk)) {
        g_sd_mounted = 0U;
        return FX_SUCCESS;
    }
    UINT st = fx_media_close(&sdio_disk);
    if (st == FX_SUCCESS) {
        g_sd_mounted = 0U;
        DEBUG_DUMP(IOT_LOG_DEBUG, "FILEX media '%s' closed\r\n", FX_SD_VOLUME_NAME);
    }
    return st;
}

static UINT make_absolute_path(const char *input, CHAR *out, size_t outsz)
{
    if (!input || !out || outsz == 0U) {
        return FX_PTR_ERROR;
    }

    size_t len = strlen(input);
    if (len + 2U >= outsz) {
        return FX_INVALID_NAME;
    }

    size_t pos = 0U;
    if (input[0] != '/' && input[0] != '\\') {
        out[pos++] = '/';
    }

    for (size_t i = 0U; i < len && pos < outsz - 1U; ++i) {
        CHAR c = (input[i] == '\\') ? '/' : input[i];
        if (pos > 0U && c == '/' && out[pos - 1U] == '/') {
            continue; /* collapse duplicate slashes */
        }
        out[pos++] = c;
    }
    if (pos == 0U) {
        out[pos++] = '/';
    }
    if (pos > 1U && out[pos - 1U] == '/') {
        --pos;
    }
    out[pos] = '\0';
    return FX_SUCCESS;
}

static UINT path_split(const char *abspath, CHAR *dir, size_t dirsz, CHAR *base, size_t basesz)
{
    if (!abspath || abspath[0] != '/') {
        return FX_INVALID_NAME;
    }

    size_t len = strlen(abspath);
    if (len == 1U) {
        if (dir && dirsz >= 2U) {
            dir[0] = '/';
            dir[1] = '\0';
        }
        if (base && basesz > 0U) {
            base[0] = '\0';
        }
        return FX_SUCCESS;
    }

    while (len > 1U && abspath[len - 1U] == '/') {
        --len;
    }

    size_t sep = len;
    while (sep > 0U && abspath[sep - 1U] != '/') {
        --sep;
    }

    size_t dirlen = (sep <= 1U) ? 1U : (sep - 1U);
    size_t baselen = len - sep;

    if (dir && dirsz > 0U) {
        if (dirlen >= dirsz) {
            return FX_INVALID_NAME;
        }
        memcpy(dir, abspath, dirlen);
        dir[dirlen] = '\0';
    }

    if (base && basesz > 0U) {
        if (baselen == 0U || baselen >= basesz) {
            return FX_INVALID_NAME;
        }
        memcpy(base, abspath + sep, baselen);
        base[baselen] = '\0';
    }

    return FX_SUCCESS;
}

static UINT set_cwd(FX_MEDIA *m, const char *path)
{
    if (!m || !path) {
        return FX_PTR_ERROR;
    }

    UINT st = fx_directory_default_set(m, "/");
    if (st != FX_SUCCESS) {
        return st;
    }

    if (path[0] == '/' && path[1] == '\0') {
        return FX_SUCCESS;
    }

    const char *cursor = path;
    if (*cursor == '/') {
        ++cursor;
    }

    CHAR segment[ENTRY_NAME_LEN];
    while (*cursor != '\0') {
        size_t seglen = 0U;
        while (cursor[seglen] != '\0' && cursor[seglen] != '/') {
            ++seglen;
        }
        if (seglen > 0U) {
            if (seglen >= sizeof(segment)) {
                return FX_INVALID_NAME;
            }
            memcpy(segment, cursor, seglen);
            segment[seglen] = '\0';

            st = fx_directory_default_set(m, segment);
            if (st == FX_INVALID_PATH) {
                return st;
            }
            if (st != FX_SUCCESS) {
                return st;
            }
        }

        cursor += seglen;
        while (*cursor == '/') {
            ++cursor;
        }
    }

    return FX_SUCCESS;
}

static int is_dot_entry(const char *name)
{
    if (!name) {
        return 0;
    }
    if (strcmp(name, ".") == 0) {
        return 1;
    }
    if (strcmp(name, "..") == 0) {
        return 1;
    }
    return 0;
}

static int str_ieq(const char *a, const char *b)
{
    if (!a || !b) {
        return 0;
    }
    while (*a && *b) {
        int ca = tolower((unsigned char)*a++);
        int cb = tolower((unsigned char)*b++);
        if (ca != cb) {
            return 0;
        }
    }
    return (*a == '\0' && *b == '\0');
}

static int should_skip_folder(const char *name, UINT attr)
{
    if (!name) {
        return 1;
    }
    if (attr & FX_VOLUME) {
        return 1;
    }
    if (attr & FX_SYSTEM) {
        return 1;
    }
    if (attr & FX_HIDDEN) {
        return 1;
    }
    if (str_ieq(name, "System Volume Information")) {
        return 1;
    }
    if (str_ieq(name, "$RECYCLE.BIN")) {
        return 1;
    }
    return 0;
}

static UINT list_dir_recursive(FX_MEDIA *m, const char *path, UINT depth, FX_LOCAL_PATH *parent)
{
    FX_LOCAL_PATH current_path;
    UINT st = fx_directory_local_path_set(m, &current_path, (CHAR *)path);
    if (st != FX_SUCCESS) {
        if (parent) {
            fx_directory_local_path_restore(m, parent);
        }
        return st;
    }

    CHAR name[ENTRY_NAME_LEN];
    UINT attr;
    ULONG size;
    UINT year, month, day, hour, minute, second;

    st = fx_directory_first_full_entry_find(m, name, &attr, &size, &year, &month, &day, &hour, &minute, &second);
    while (st == FX_SUCCESS) {
        if (!is_dot_entry(name)) {
            if (attr & FX_DIRECTORY) {
                CHAR child_path[PATH_BUFFER_LEN];
                if (snprintf(child_path, sizeof(child_path), "%s%s%s",
                             path,
                             (strcmp(path, "/") == 0) ? "" : "/",
                             name) >= (int)sizeof(child_path)) {
                    st = FX_INVALID_NAME;
                    break;
                }
                DEBUG_DUMP(IOT_LOG_INFO,
                           "%04u-%02u-%02u %02u:%02u:%02u <DIR>      %s\r\n",
                           year, month, day, hour, minute, second,
                           child_path);
                if (!should_skip_folder(name, attr)) {
                    st = list_dir_recursive(m, child_path, depth + 1U, &current_path);
                    if (st != FX_SUCCESS) {
                        break;
                    }
                }
                st = fx_directory_local_path_restore(m, &current_path);
                if (st != FX_SUCCESS) {
                    break;
                }
            } else {
                DEBUG_DUMP(IOT_LOG_INFO,
                           "%04u-%02u-%02u %02u:%02u:%02u %-10lu %s%s%s\r\n",
                           year, month, day, hour, minute, second,
                           (unsigned long)size,
                           path,
                           (strcmp(path, "/") == 0) ? "" : "/",
                           name);
            }
        }
        st = fx_directory_next_full_entry_find(m, name, &attr, &size, &year, &month, &day, &hour, &minute, &second);
    }

    if (st == FX_NO_MORE_ENTRIES) {
        st = FX_SUCCESS;
    }

    if (parent) {
        fx_directory_local_path_restore(m, parent);
    } else {
        fx_directory_local_path_clear(m);
    }
    return st;
}

static int name_in_except_list(const char *name, const char *except_list)
{
    if (!name || !except_list) {
        return 0;
    }
    size_t len = strlen(except_list) + 1U;
    char *tmp = (char *)malloc(len);
    if (!tmp) {
        return 0;
    }
    memcpy(tmp, except_list, len);
    int matched = 0;
    char *ctx = NULL;
    for (char *token = strtok_r(tmp, ";", &ctx);
         token != NULL;
         token = strtok_r(NULL, ";", &ctx)) {
        if (str_ieq(name, token)) {
            matched = 1;
            break;
        }
    }
    free(tmp);
    return matched;
}

static UINT remove_dir_recursive(FX_MEDIA *m, const char *path, const char *except, UINT recursive)
{
    FX_LOCAL_PATH current;
    UINT st = fx_directory_local_path_set(m, &current, (CHAR *)path);
    if (st != FX_SUCCESS) {
        return st;
    }

    CHAR name[ENTRY_NAME_LEN];
    UINT attr;
    ULONG size;
    UINT year, month, day, hour, minute, second;

    st = fx_directory_first_full_entry_find(m, name, &attr, &size, &year, &month, &day, &hour, &minute, &second);
    while (st == FX_SUCCESS) {
        if (!is_dot_entry(name)) {
            if ((attr & FX_DIRECTORY) && recursive) {
                CHAR child_path[PATH_BUFFER_LEN];
                if (snprintf(child_path, sizeof(child_path), "%s%s%s",
                             path,
                             (strcmp(path, "/") == 0) ? "" : "/",
                             name) >= (int)sizeof(child_path)) {
                    st = FX_INVALID_NAME;
                    break;
                }
                st = remove_dir_recursive(m, child_path, NULL, recursive);
                if (st != FX_SUCCESS) {
                    break;
                }
            } else if (!(attr & FX_DIRECTORY)) {
                if (!name_in_except_list(name, except)) {
                    CHAR full_path[PATH_BUFFER_LEN];
                    if (snprintf(full_path, sizeof(full_path), "%s%s%s",
                                 path,
                                 (strcmp(path, "/") == 0) ? "" : "/",
                                 name) >= (int)sizeof(full_path)) {
                        st = FX_INVALID_NAME;
                        break;
                    }
                    UINT del_st = fx_file_delete(m, full_path);
                    if (del_st != FX_SUCCESS && del_st != FX_NOT_FOUND) {
                        st = del_st;
                        break;
                    }
                }
            }
        }
        st = fx_directory_next_full_entry_find(m, name, &attr, &size, &year, &month, &day, &hour, &minute, &second);
    }

    if (st == FX_NO_MORE_ENTRIES) {
        st = FX_SUCCESS;
    }

    fx_directory_local_path_restore(m, &current);

    if (st == FX_SUCCESS && strcmp(path, "/") != 0) {
        UINT del_dir = fx_directory_delete(m, (CHAR *)path);
        if (del_dir != FX_SUCCESS && del_dir != FX_DIR_NOT_EMPTY) {
            st = del_dir;
        }
    }
    return st;
}

UINT FX_ReadFile(const char *filename, char *buf, uint32_t size, uint32_t *bytesread)
{
    if (!buf || !bytesread) {
        return FX_PTR_ERROR;
    }
    *bytesread = 0U;

    UINT st = is_filex_mount();
    if (st != FX_SUCCESS) {
        return st;
    }

    CHAR abs_path[PATH_BUFFER_LEN];
    st = make_absolute_path(filename, abs_path, sizeof(abs_path));
    if (st != FX_SUCCESS) {
        return st;
    }

    CHAR dir[PATH_BUFFER_LEN];
    CHAR base[ENTRY_NAME_LEN];
    st = path_split(abs_path, dir, sizeof(dir), base, sizeof(base));
    if (st != FX_SUCCESS) {
        return st;
    }

    st = set_cwd(&sdio_disk, dir);
    if (st != FX_SUCCESS) {
        return st;
    }

    FX_FILE file;
    st = fx_file_open(&sdio_disk, &file, base, FX_OPEN_FOR_READ);
    if (st != FX_SUCCESS) {
        return st;
    }

    ULONG request = size;
    if (request == 0U) {
        ULONG64 file_size = file.fx_file_current_file_size;
        if (file_size > 0xFFFFFFFFULL) {
            request = 0xFFFFFFFFUL;
        } else {
            request = (ULONG)file_size;
        }
    }

    ULONG total = 0U;
    while (total < request) {
        ULONG chunk = request - total;
        if (chunk > IO_CHUNK_SIZE) {
            chunk = IO_CHUNK_SIZE;
        }

        ULONG got = 0U;
        st = fx_file_read(&file, (UCHAR *)buf + total, chunk, &got);
        if (st == FX_END_OF_FILE) {
            st = FX_SUCCESS;
        } else if (st != FX_SUCCESS) {
            fx_file_close(&file);
            return st;
        }

        total += got;
        if (got == 0U) {
            break;
        }
    }

    *bytesread = total;
    fx_file_close(&file);
    return st;
}

UINT FX_WriteFile(const char *filename, const char *buf, uint32_t size, FX_WRITE_MODE mode)
{
    if (!buf) {
        return FX_PTR_ERROR;
    }

    if ((mode != FX_WRITE_MODE_NEW) && (mode != FX_WRITE_MODE_APPEND)) {
        return FX_INVALID_OPTION;
    }

    UINT st = is_filex_mount();
    if (st != FX_SUCCESS) {
        return st;
    }

    CHAR abs_path[PATH_BUFFER_LEN];
    st = make_absolute_path(filename, abs_path, sizeof(abs_path));
    if (st != FX_SUCCESS) {
        return st;
    }

    CHAR dir[PATH_BUFFER_LEN];
    CHAR base[ENTRY_NAME_LEN];
    st = path_split(abs_path, dir, sizeof(dir), base, sizeof(base));
    if (st != FX_SUCCESS) {
        return st;
    }

    st = set_cwd(&sdio_disk, dir);
    if (st != FX_SUCCESS) {
        return st;
    }

    UINT existed = 0U;
    UINT create_st = fx_file_create(&sdio_disk, base);
    if (create_st == FX_ALREADY_CREATED) {
        existed = 1U;
    } else if (create_st != FX_SUCCESS) {
        return create_st;
    }

    FX_FILE file;
    st = fx_file_open(&sdio_disk, &file, base, FX_OPEN_FOR_WRITE);
    if (st != FX_SUCCESS) {
        return st;
    }

    if (mode == FX_WRITE_MODE_APPEND) {
        ULONG64 end_pos = file.fx_file_current_file_size;
        st = fx_file_extended_seek(&file, end_pos);
        if (st != FX_SUCCESS) {
            fx_file_close(&file);
            return st;
        }
    } else if (existed) {
        st = fx_file_truncate(&file, 0U);
        if (st != FX_SUCCESS) {
            fx_file_close(&file);
            return st;
        }
    }

    ULONG written_total = 0U;
    while (written_total < size) {
        ULONG chunk = size - written_total;
        if (chunk > IO_CHUNK_SIZE) {
            chunk = IO_CHUNK_SIZE;
        }

        st = fx_file_write(&file, (VOID *)(buf + written_total), chunk);
        if (st != FX_SUCCESS) {
            fx_file_close(&file);
            return st;
        }
        written_total += chunk;
    }

    fx_file_close(&file);
    (void)fx_media_flush(&sdio_disk);
    return FX_SUCCESS;
}

UINT FX_DeleteFile(const char *filename)
{
    if (!filename) {
        return FX_PTR_ERROR;
    }

    UINT st = is_filex_mount();
    if (st != FX_SUCCESS) {
        return st;
    }

    CHAR abs_path[PATH_BUFFER_LEN];
    st = make_absolute_path(filename, abs_path, sizeof(abs_path));
    if (st != FX_SUCCESS) {
        return st;
    }

    CHAR dir[PATH_BUFFER_LEN];
    CHAR base[ENTRY_NAME_LEN];
    st = path_split(abs_path, dir, sizeof(dir), base, sizeof(base));
    if (st != FX_SUCCESS) {
        return st;
    }

    st = set_cwd(&sdio_disk, dir);
    if (st != FX_SUCCESS) {
        return st;
    }

    st = fx_file_delete(&sdio_disk, base);
    if (st == FX_SUCCESS) {
        (void)fx_media_flush(&sdio_disk);
    }
    return st;
}

UINT FX_ListDir(const char *path)
{
    UINT st = is_filex_mount();
    if (st != FX_SUCCESS) {
        return st;
    }
    CHAR abs_path[PATH_BUFFER_LEN];
    st = make_absolute_path(path ? path : "/", abs_path, sizeof(abs_path));
    if (st != FX_SUCCESS) {
        return st;
    }
    DEBUG_DUMP(IOT_LOG_INFO, "Listing '%s'\r\n", abs_path);
    return list_dir_recursive(&sdio_disk, abs_path, 0U, NULL);
}

UINT FX_ListDirSimple(const char *path)
{
    UINT st = is_filex_mount();
    if (st != FX_SUCCESS) {
        return st;
    }

    CHAR abs_path[PATH_BUFFER_LEN];
    st = make_absolute_path(path ? path : "/", abs_path, sizeof(abs_path));
    if (st != FX_SUCCESS) {
        return st;
    }

    FX_LOCAL_PATH saved_path;
    UINT path_active = 0U;
    st = fx_directory_local_path_set(&sdio_disk, &saved_path, abs_path);
    if (st != FX_SUCCESS) {
        return st;
    }
    path_active = 1U;

    CHAR name[ENTRY_NAME_LEN];
    UINT attr;
    ULONG size;
    UINT year, month, day, hour, minute, second;
    UINT entry_count = 0U;

    st = fx_directory_first_full_entry_find(&sdio_disk, name, &attr, &size,
                                            &year, &month, &day, &hour, &minute, &second);
    while (st == FX_SUCCESS) {
        if (!is_dot_entry(name)) {
            ++entry_count;
            if (attr & FX_DIRECTORY) {
                DEBUG_DUMP(IOT_LOG_INFO, "[DIR ] %s%s%s\r\n",
                           abs_path,
                           (strcmp(abs_path, "/") == 0) ? "" : "/",
                           name);
            } else {
                DEBUG_DUMP(IOT_LOG_INFO, "[FILE] %s%s%s (%lu bytes)\r\n",
                           abs_path,
                           (strcmp(abs_path, "/") == 0) ? "" : "/",
                           name,
                           (unsigned long)size);
            }
        }
        st = fx_directory_next_full_entry_find(&sdio_disk, name, &attr, &size,
                                               &year, &month, &day, &hour, &minute, &second);
    }

    if (path_active) {
        fx_directory_local_path_restore(&sdio_disk, &saved_path);
        fx_directory_local_path_clear(&sdio_disk);
    }

    if (st == FX_NO_MORE_ENTRIES) {
        st = FX_SUCCESS;
    }

    if (st == FX_SUCCESS) {
        DEBUG_DUMP(IOT_LOG_INFO, "HUB_TARGET_SD: '%s' entry count=%u\r\n",
                   abs_path,
                   entry_count);
    }
    return st;
}

UINT FX_RemoveDir(const char *path, const char *except, UINT recursive)
{
    UINT st = is_filex_mount();
    if (st != FX_SUCCESS) {
        return st;
    }
    CHAR abs_path[PATH_BUFFER_LEN];
    st = make_absolute_path(path, abs_path, sizeof(abs_path));
    if (st != FX_SUCCESS) {
        return st;
    }
    return remove_dir_recursive(&sdio_disk, abs_path, except, recursive);
}

UINT FX_Stat(const char *path, FX_DIR_ENTRY *entry)
{
    if (!entry) {
        return FX_PTR_ERROR;
    }
    UINT st = is_filex_mount();
    if (st != FX_SUCCESS) {
        return st;
    }
    CHAR abs_path[PATH_BUFFER_LEN];
    st = make_absolute_path(path, abs_path, sizeof(abs_path));
    if (st != FX_SUCCESS) {
        return st;
    }
    UINT attr;
    ULONG size;
    UINT year, month, day, hour, minute, second;
    st = fx_directory_information_get(&sdio_disk, abs_path, &attr, &size, &year, &month, &day, &hour, &minute, &second);
    if (st == FX_SUCCESS) {
        memset(entry, 0, sizeof(*entry));
        entry->fx_dir_entry_attributes = (UCHAR)attr;
        entry->fx_dir_entry_file_size = (ULONG64)size;
        entry->fx_dir_entry_created_date = year;
        entry->fx_dir_entry_created_time = hour;
        entry->fx_dir_entry_time = hour;
        entry->fx_dir_entry_date = (month << 8) | day;
    }
    return st;
}

UINT FX_Rename(const char *old_path, const char *new_path)
{
    UINT st = is_filex_mount();
    if (st != FX_SUCCESS) {
        return st;
    }

    CHAR old_abs[PATH_BUFFER_LEN];
    CHAR new_abs[PATH_BUFFER_LEN];
    st = make_absolute_path(old_path, old_abs, sizeof(old_abs));
    if (st != FX_SUCCESS) {
        return st;
    }
    st = make_absolute_path(new_path, new_abs, sizeof(new_abs));
    if (st != FX_SUCCESS) {
        return st;
    }
    DEBUG_DUMP(IOT_LOG_INFO, "Renaming '%s' -> '%s'\r\n", old_abs, new_abs);

    if (strcmp(old_abs, new_abs) == 0) {
        DEBUG_DUMP(IOT_LOG_INFO, "  Source and destination are identical, skipping rename\r\n");
        return FX_SUCCESS;
    }

    CHAR old_dir[PATH_BUFFER_LEN];
    CHAR new_dir[PATH_BUFFER_LEN];
    CHAR old_base[ENTRY_NAME_LEN];
    CHAR new_base[ENTRY_NAME_LEN];
    st = path_split(old_abs, old_dir, sizeof(old_dir), old_base, sizeof(old_base));
    if (st != FX_SUCCESS) {
        return st;
    }
    st = path_split(new_abs, new_dir, sizeof(new_dir), new_base, sizeof(new_base));
    if (st != FX_SUCCESS) {
        return st;
    }

    if (strcmp(old_dir, new_dir) != 0) {
        DEBUG_DUMP(IOT_LOG_ERR, "Cross-directory rename not supported (%s -> %s)\r\n", old_dir, new_dir);
        return FX_NOT_IMPLEMENTED;
    }

    UINT attr;
    ULONG size;
    UINT year, month, day, hour, minute, second;

    st = fx_directory_information_get(&sdio_disk, old_abs, &attr, &size, &year, &month, &day, &hour, &minute, &second);
    if (st != FX_SUCCESS) {
        return st;
    }

    DEBUG_DUMP(IOT_LOG_INFO, "  Old entry: attr=0x%02X, size=%lu, date=%04u-%02u-%02u %02u:%02u:%02u\r\n",
               attr, (unsigned long)size, year, month, day, hour, minute, second);
    DEBUG_DUMP(IOT_LOG_INFO, "  Renaming %s\r\n", (attr & FX_DIRECTORY) ? "directory" : "file");

    st = set_cwd(&sdio_disk, old_dir);
    if (st != FX_SUCCESS) {
        return st;
    }

    if (attr & FX_DIRECTORY) {
        st = fx_directory_rename(&sdio_disk, old_base, new_base);
    } else {
        st = fx_file_rename(&sdio_disk, old_base, new_base);
    }

    DEBUG_DUMP(IOT_LOG_INFO, "  Rename status: 0x%02X\r\n", st);
    (void)fx_directory_default_set(&sdio_disk, "/");
    if (st == FX_SUCCESS) {
        (void)fx_media_flush(&sdio_disk);
    }
    return st;
}
