#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdint.h>
#include "i2c_hub_filex.h"
#include "i2c_hub_mem.h"
#include "stm32h7xx_hal.h"
#include "fx_stm32_sd_driver.h"
#include "app_filex.h"
#include "iot.h"

extern FX_MEDIA sdio_disk;
extern uint32_t fx_sd_media_memory;

static volatile UINT  g_sd_mounted = 0;
static volatile UINT  g_fx_inited  = 1;

static void u64_to_str10(char *out, size_t outsz, uint64_t v)
{
    if (!out || outsz == 0) return;
    char tmp[32];
    int i = 0;
    if (v == 0) { out[0] = '0'; out[1] = '\0'; return; }
    while (v && i < (int)sizeof(tmp)) {
        tmp[i++] = (char)('0' + (v % 10));
        v /= 10;
    }
    size_t n = (size_t)i;
    if (n >= outsz) n = outsz - 1;
    for (size_t j = 0; j < n; ++j) out[j] = tmp[n - 1 - j];
    out[n] = '\0';
}

static void human_size_u64(char *out, size_t outsz, uint64_t bytes)
{
    if (!out || outsz == 0) return;
    static const char *unit[] = {"B","KB","MB","GB","TB","PB"};
    int u = 0;
    while (u < 5 && bytes >= (1ULL << (10 * (u + 1)))) u++;
    uint64_t denom = 1ULL << (10 * u);
    uint64_t whole = bytes / denom;
    uint64_t frac  = ((bytes % denom) * 100) / denom;
    char wbuf[24]; u64_to_str10(wbuf, sizeof(wbuf), whole);
    int n = snprintf(out, outsz, "%s.%02lu %s", wbuf, (unsigned long)frac, unit[u]);
    if (n < 0) out[0] = '\0';
    else if ((size_t)n >= outsz) out[outsz - 1] = '\0';
}

static UINT set_cwd(FX_MEDIA *m, const char *path)
{
    UINT st = fx_directory_default_set(m, "/");
    if (st == FX_INVALID_PATH) {
        if (!path || path[0]=='\0' || (path[0]=='/' && path[1]=='\0')) return FX_SUCCESS;
        return FX_INVALID_PATH;
    }
    if (st != FX_SUCCESS) return st;

    st = fx_directory_default_set(m, "/");
    if (st != FX_SUCCESS) return st;

    if (!path || path[0] == '\0' || (path[0]=='/' && path[1]=='\0')) {
        return FX_SUCCESS;
    }

    const char *p = path;
    while (*p == '/') p++;

    CHAR seg[256];
    while (*p) {
        size_t len = 0;
        while (p[len] && p[len] != '/') len++;
        if (len == 0) { p++; continue; }
        if (len >= sizeof(seg)) return FX_INVALID_NAME;

        memcpy(seg, p, len);
        seg[len] = '\0';

        st = fx_directory_default_set(m, seg);
        if (st != FX_SUCCESS) return st;

        p += len;
        while (*p == '/') p++;
    }
    return FX_SUCCESS;
}

static UINT ensure_local_path(FX_MEDIA *m)
{
    static UINT ready = 0;
    static FX_LOCAL_PATH local;
    if (ready) return FX_SUCCESS;
    UINT st = fx_directory_local_path_set(m, &local, FX_NULL);
    if (st == FX_SUCCESS) ready = 1;
    return st;
}

static const char* fs_type_str(const FX_MEDIA *m)
{
    UINT t = m->fx_media_FAT_type;
#ifdef FX_exFAT
    if (t == FX_exFAT) return "exFAT";
#endif
#ifdef FX_FAT32
    if (t == FX_FAT32) return "FAT32";
#endif
#ifdef FX_FAT16
    if (t == FX_FAT16) return "FAT16";
#endif
#ifdef FX_FAT12
    if (t == FX_FAT12) return "FAT12";
#endif
    if (t == 32) return "FAT32";
    if (t == 16) return "FAT16";
    if (t == 12) return "FAT12";
    return "FAT";
}

static int ieq(const char *a, const char *b)
{
    unsigned char ca, cb;
    while (*a && *b) {
        ca = (unsigned char)*a++; cb = (unsigned char)*b++;
        if (ca >= 'A' && ca <= 'Z') ca = (unsigned char)(ca - 'A' + 'a');
        if (cb >= 'A' && cb <= 'Z') cb = (unsigned char)(cb - 'A' + 'a');
        if (ca != cb) return 0;
    }
    return *a == '\0' && *b == '\0';
}

static int should_skip_descend(const char *name, UINT attr)
{
    if (attr & FX_VOLUME)  return 1;
    if (attr & FX_SYSTEM)  return 1;
    if (attr & FX_HIDDEN)  return 1;
    if (ieq(name, "System Volume Information")) return 1;
    if (ieq(name, "$RECYCLE.BIN"))             return 1;
    return 0;
}

static UINT fs_get_space(FX_MEDIA *m, unsigned long long *total, unsigned long long *freeb)
{
    if (!m || !total || !freeb) return FX_PTR_ERROR;

    uint64_t total_sectors = (uint64_t)m->fx_media_total_sectors;
    uint64_t bps           = (uint64_t)m->fx_media_bytes_per_sector;

    *total = 0;
    *freeb = 0;

    {
        char ts[32], bpss[32];
        u64_to_str10(ts, sizeof(ts), total_sectors);
        u64_to_str10(bpss, sizeof(bpss), bps);
    }
    if (bps != 0) *total = (unsigned long long)(total_sectors * bps);
#ifdef FX_exFAT
    {
        ULONG64 free64 = 0;
        UINT st = fx_media_extended_space_available(m, &free64);
        if (st == FX_SUCCESS) {
            *freeb = (unsigned long long)free64;
            return FX_SUCCESS;
        }
    }
#endif
    {
        ULONG free32 = 0;
        UINT st = fx_media_space_available(m, &free32);
        if (st != FX_SUCCESS) return st;
        *freeb = (bps != 0) ? (unsigned long long)free32 * (unsigned long long)bps
                            : (unsigned long long)free32;
        return FX_SUCCESS;
    }
}

static void wr_init(wr_t *w, char *buf, size_t cap)
{
    w->buf = buf; w->cap = cap; w->used = 0; w->truncated = 0;
    if (cap) buf[0] = '\0';
}

static void wr_puts(wr_t *w, const char *s)
{
    if (w->used >= w->cap) { w->truncated = 1; return; }
    size_t sl = strlen(s);
    if (sl > w->cap - w->used) { sl = w->cap - w->used; w->truncated = 1; }
    memcpy(w->buf + w->used, s, sl);
    w->used += sl;
    if (w->used < w->cap) w->buf[w->used] = '\0';
}

static void wr_printf(wr_t *w, const char *fmt, ...)
{
    if (w->used >= w->cap) { w->truncated = 1; return; }
    va_list ap; va_start(ap, fmt);
    int n = vsnprintf(w->buf + w->used, w->cap - w->used, fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    if ((size_t)n > w->cap - w->used) { w->used = w->cap; w->truncated = 1; return; }
    w->used += (size_t)n;
}

static int is_dot_or_dotdot(const char *name)
{
    return ((name[0]=='.' && name[1]=='\0') ||
            (name[0]=='.' && name[1]=='.' && name[2]=='\0'));
}

static void print_indent(wr_t *w, int depth)
{
    for (int i = 0; i < depth; ++i) wr_puts(w, "  ");
}

static UINT list_tree(FX_MEDIA *m, const char *path, int depth, int max_depth,
                      unsigned long *cnt_dirs, unsigned long *cnt_files, wr_t *w)
{
    if (max_depth >= 0 && depth > max_depth) return FX_SUCCESS;

    UINT  st;
    CHAR  name[256];
    UINT  attr;
    ULONG fsize;
    UINT  y, mo, d, h, mi, s;

    (void)ensure_local_path(m);

    st = set_cwd(m, path);
    if (st != FX_SUCCESS) {
        print_indent(w, depth);
        wr_printf(w, "[ERR] cd %s (0x%X)\r\n", path, st);
        return st;
    }

    #define MAX_ENUM_GUARD 65535UL
    #define MAX_SUBDIRS    64
    typedef struct { char name[256]; UINT attr; } subdir_t;

    subdir_t *subdirs = (subdir_t *)hub_tmp_alloc(HUB_ALIGN_UP(MAX_SUBDIRS * sizeof(subdir_t)));
    if (!subdirs) {
        return FX_SUCCESS;
    }

    // subdir_t subdirs[MAX_SUBDIRS];
    unsigned nd = 0;
    unsigned long guard = MAX_ENUM_GUARD;   

    st = fx_directory_first_full_entry_find(m, name, &attr, &fsize, &y, &mo, &d, &h, &mi, &s);
    while (st == FX_SUCCESS && guard--) {

        if (!is_dot_or_dotdot(name)) {
            int is_dir = (attr & FX_DIRECTORY) ? 1 : 0;
            print_indent(w, depth);
            if (is_dir) {
                wr_printf(w, "- [%s]\r\n", name);
                (*cnt_dirs)++;

                if (!should_skip_descend(name, attr) && nd < MAX_SUBDIRS) {
                    size_t n = strnlen(name, sizeof(subdirs[nd].name)-1);
                    memcpy(subdirs[nd].name, name, n);
                    subdirs[nd].name[n] = '\0';
                    subdirs[nd].attr = attr;
                    nd++;
                }
            } else {
                char sz[32]; u64_to_str10(sz, sizeof(sz), (uint64_t)fsize);
                wr_printf(w, "- %s (%s B)\r\n", name, sz);
                (*cnt_files)++;
            }
        }

        st = fx_directory_next_full_entry_find(m, name, &attr, &fsize, &y, &mo, &d, &h, &mi, &s);
    }

    if (st != FX_NO_MORE_ENTRIES && st != FX_SUCCESS) {
        print_indent(w, depth);
        wr_printf(w, "[ERR] list %s (0x%X)\r\n", path, st);
        return st;
    }

    for (unsigned i = 0; i < nd; ++i) {
        char sub[512];
        if (path[0] == '\0' || (path[0]=='/' && path[1]=='\0'))
            (void)snprintf(sub, sizeof(sub), "/%s", subdirs[i].name);
        else
            (void)snprintf(sub, sizeof(sub), "%s/%s", path, subdirs[i].name);

        (void)list_tree(m, sub, depth + 1, max_depth, cnt_dirs, cnt_files, w);
        hub_tmp_free(subdirs);
        (void)set_cwd(m, path);
    }

    return FX_SUCCESS;
}


void i2c_hub_filex_init(void)
{
    if (!g_fx_inited) {
        fx_system_initialize();
        g_fx_inited = 1;
    }
}

UINT i2c_hub_filex_mount(void)
{
    if (!g_fx_inited) i2c_hub_filex_init();
    if (g_sd_mounted) return FX_SUCCESS;
    UINT st = FX_SUCCESS;
    if (st == FX_SUCCESS) g_sd_mounted = 1;
    return st;
}

UINT i2c_hub_filex_unmount(void)
{
    if (!g_sd_mounted) return FX_SUCCESS;
    UINT st = fx_media_close(&sdio_disk);
    if (st == FX_SUCCESS) g_sd_mounted = 0;
    return st;
}

uint8_t *i2c_hub_filex_helper(uint8_t *tx_frame_ptr, const hub_cmd_t *cmd)
{
    const uint32_t out_len   = (uint32_t)cmd->len;
    const uint32_t frame_len = RSP_HDR_SZ + out_len + 4;

    // tx_frame_ptr = (uint8_t *)hub_sdram_alloc_tx(ALIGN32(frame_len));
    tx_frame_ptr = (uint8_t *)hub_heap_alloc_aligned(ALIGN32(frame_len), HUB_DMA_ALIGN);
    if (!tx_frame_ptr) return NULL;

    hub_rsp_t *rsp = (hub_rsp_t *)tx_frame_ptr;
    rsp->status    = HUB_RSP_OK;
    rsp->reserved  = 0;
    rsp->len       = (uint16_t)out_len;
    rsp->data_addr = 0;

    char *payload = (char *)(tx_frame_ptr + RSP_HDR_SZ);
    wr_t w; wr_init(&w, payload, out_len);

    UINT st = i2c_hub_filex_mount();
    if (st != FX_SUCCESS) {
        wr_printf(&w, "SD mount failed: 0x%X\r\n", st);
        goto out;
    }

    unsigned long long total=0, freeb=0, used=0;
    st = fs_get_space(&sdio_disk, &total, &freeb);
    if (st != FX_SUCCESS) {
        wr_printf(&w, "Get space failed: 0x%X\r\n", st);
        goto out;
    }
    used = (total >= freeb) ? (total - freeb) : 0;

    char tbuf[32]={0}, ubuf[32]={0}, fbuf[32]={0};
    human_size_u64(tbuf, sizeof(tbuf), total);
    human_size_u64(ubuf, sizeof(ubuf), used);
    human_size_u64(fbuf, sizeof(fbuf), freeb);

    wr_printf(&w, "Format : %s\r\n", fs_type_str(&sdio_disk));
    wr_printf(&w, "Total  : %s\r\n", tbuf);
    wr_printf(&w, "Used   : %s\r\n", ubuf);
    wr_printf(&w, "Free   : %s\r\n\r\n", fbuf);
    wr_puts(&w, "/ (root)\r\n");
    wr_puts(&w, "Listing files and directories:\r\n");
    unsigned long cnt_dirs = 0, cnt_files = 0;
    (void)list_tree(&sdio_disk, "/", 0, -1, &cnt_dirs, &cnt_files, &w);
    DEBUG_DUMP(IOT_LOG_DEBUG, "list_tree: found %lu dirs, %lu files\r\n", cnt_dirs, cnt_files);
    wr_printf(&w, "\r\n[Summary] Dirs=%lu, Files=%lu", cnt_dirs, cnt_files);
out:
    if (w.used < out_len) memset(payload + w.used, 0, out_len - w.used);
    uint32_t crc = iot_hub_crc32_hard(tx_frame_ptr, RSP_HDR_SZ + out_len);
    memcpy(tx_frame_ptr + RSP_HDR_SZ + out_len, &crc, 4);
    dcache_clean32_range(tx_frame_ptr, frame_len);
    return tx_frame_ptr;
}

static UINT path_split(const char *abspath, CHAR *dir, size_t dirsz,
                       CHAR *base, size_t basesz)
{
    if (!abspath || abspath[0] != '/') return FX_INVALID_NAME;

    size_t L = strlen(abspath);
    if (L == 1) {
        if (dir && dirsz) { dir[0] = '/'; dir[1] = '\0'; }
        if (base && basesz) base[0] = '\0';
        return FX_SUCCESS;
    }
    while (L > 1 && abspath[L-1] == '/') L--;

    size_t i = L;
    while (i > 0 && abspath[i-1] != '/') i--;
    size_t dlen = (i == 1) ? 1 : (i - 1);
    size_t blen = L - i;

    if (dir && dirsz) {
        if (dlen >= dirsz) return FX_INVALID_NAME;
        memcpy(dir, abspath, dlen);
        dir[dlen] = '\0';
    }
    if (base && basesz) {
        if (blen == 0 || blen >= basesz) return FX_INVALID_NAME;
        memcpy(base, abspath + i, blen);
        base[blen] = '\0';
    }
    return FX_SUCCESS;
}

UINT fx_mkdirs(FX_MEDIA *m, const char *absdir)
{
    if (!absdir || absdir[0] != '/') return FX_INVALID_NAME;

    UINT st = set_cwd(m, "/");
    if (st != FX_SUCCESS) return st;

    if (absdir[1] == '\0') return FX_SUCCESS;

    const char *p = absdir;
    while (*p == '/') p++;

    CHAR seg[256];
    while (*p) {
        size_t len = 0;
        while (p[len] && p[len] != '/') len++;
        if (len == 0) { p++; continue; }
        if (len >= sizeof(seg)) return FX_INVALID_NAME;
        memcpy(seg, p, len); seg[len] = '\0';

        st = fx_directory_default_set(m, seg);
        if (st != FX_SUCCESS) {
            UINT st2 = fx_directory_create(m, seg);
            if (st2 != FX_SUCCESS && st2 != FX_ALREADY_CREATED) return st2;
            st = fx_directory_default_set(m, seg);
            if (st != FX_SUCCESS) return st;
        }
        p += len;
        while (*p == '/') p++;
    }
    return FX_SUCCESS;
}

UINT fx_get_file_size64(FX_MEDIA *m, const char *abspath, ULONG64 *out_size)
{
    if (!out_size) return FX_PTR_ERROR;
    *out_size = 0;

    CHAR dir[256], base[256];
    UINT st = path_split(abspath, dir, sizeof(dir), base, sizeof(base));
    if (st != FX_SUCCESS) return st;
    st = set_cwd(m, dir);
    if (st != FX_SUCCESS) return st;

    FX_FILE f;
    st = fx_file_open(m, &f, base, FX_OPEN_FOR_READ);
    if (st != FX_SUCCESS) return st;

#ifdef FX_ENABLE_EXFAT
    ULONG64 size64 = 0;
    st = fx_file_extended_best_effort_allocate(&f, 0, &size64);
    (void)st;
#endif
    st = fx_file_seek(&f, FX_SEEK_END);
    if (st == FX_SUCCESS) {
        *out_size = (ULONG64)f.fx_file_current_file_size;
    }
    fx_file_close(&f);
    return FX_SUCCESS;
}

UINT fx_read_file_to_buf(FX_MEDIA *m, const char *abspath,
                         ULONG64 offset, UCHAR *dst, ULONG len, ULONG *out_read)
{
    if (!dst) return FX_PTR_ERROR;
    if (out_read) *out_read = 0;

    CHAR dir[256], base[256];
    UINT st = path_split(abspath, dir, sizeof(dir), base, sizeof(base));
    if (st != FX_SUCCESS) return st;

    st = set_cwd(m, dir);
    if (st != FX_SUCCESS) return st;

    FX_FILE f;
    st = fx_file_open(m, &f, base, FX_OPEN_FOR_READ);
    if (st != FX_SUCCESS) return st;

    st = fx_file_seek(&f, (ULONG)offset);
    if (st != FX_SUCCESS) { fx_file_close(&f); return st; }

    ULONG total = 0;
    while (total < len) {
        ULONG chunk = len - total;
        if (chunk > 65536) chunk = 65536;

        ULONG got = 0;
        st = fx_file_read(&f, dst + total, chunk, &got);
        if (st != FX_SUCCESS && st != FX_END_OF_FILE) { fx_file_close(&f); return st; }
        total += got;
        if (got == 0 || st == FX_END_OF_FILE) break;
    }
    fx_file_close(&f);
    if (out_read) *out_read = total;
    return FX_SUCCESS;
}

UINT fx_write_file_from_buf(FX_MEDIA *m, const char *abspath,
                            ULONG64 offset, const UCHAR *src, ULONG len,
                            ULONG *out_written, UINT create_if_missing, UINT truncate)
{
    if (!src) return FX_PTR_ERROR;
    if (out_written) *out_written = 0;

    CHAR dir[256], base[256];
    UINT st = path_split(abspath, dir, sizeof(dir), base, sizeof(base));
    if (st != FX_SUCCESS) return st;

    st = fx_mkdirs(m, dir);
    if (st != FX_SUCCESS) return st;

    st = set_cwd(m, dir);
    if (st != FX_SUCCESS) return st;

    FX_FILE f;
    st = fx_file_open(m, &f, base, FX_OPEN_FOR_WRITE);
    if (st == FX_NOT_FOUND || st == FX_NOT_A_FILE) {
        if (!create_if_missing) return st;
        st = fx_file_create(m, base);
        if (st != FX_SUCCESS && st != FX_ALREADY_CREATED) return st;
        st = fx_file_open(m, &f, base, FX_OPEN_FOR_WRITE);
        if (st != FX_SUCCESS) return st;
    } else if (st != FX_SUCCESS) {
        return st;
    }

    if (truncate) {
        st = fx_file_truncate(&f, 0);
        if (st != FX_SUCCESS) { fx_file_close(&f); return st; }
        offset = 0;
    }

    st = fx_file_seek(&f, (ULONG)offset);
    if (st != FX_SUCCESS) { fx_file_close(&f); return st; }

    ULONG total = 0;
    while (total < len) {
        ULONG chunk = len - total;
        if (chunk > 65536) chunk = 65536;

        st = fx_file_write(&f, (void*)(src + total), chunk);
        if (st != FX_SUCCESS) { fx_file_close(&f); return st; }

        total += chunk;
    }

    (void)fx_file_flush(&f);
    (void)fx_media_flush(m);

    fx_file_close(&f);
    if (out_written) *out_written = total;
    return FX_SUCCESS;
}

UINT fx_delete_file_path(FX_MEDIA *m, const char *abspath)
{
    CHAR dir[256], base[256];
    UINT st = path_split(abspath, dir, sizeof(dir), base, sizeof(base));
    if (st != FX_SUCCESS) return st;
    st = set_cwd(m, dir);
    if (st != FX_SUCCESS) return st;
    return fx_file_delete(m, base);
}
