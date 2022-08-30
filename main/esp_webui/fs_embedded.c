/*
  fs_embedded.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Webserver backend - file system and embedded files for WebUI

  Part of grblHAL

  Copyright (c) 2022 Terje Io

  File data is extracted from files Copyright (c) 2021 Luc Lebosse
  https://github.com/luc-github/ESP3D-webui

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef ARDUINO
#include "../driver.h"
#else
#include "driver.h"
#endif

#include <stdlib.h>
#include <string.h>

#include "../grbl/vfs.h"

typedef struct {
    const char *name;
    size_t size;
    const unsigned char *data;
} esp_embedded_file_t;

typedef struct {
    const esp_embedded_file_t *file;
    size_t remaining;
} embedded_filehandle_t;

static esp_embedded_file_t favicon_ico = {
    .name = "favicon.ico"
};

static esp_embedded_file_t index_html_gz = {
    .name = "index.html.gz",
};

static esp_embedded_file_t ap_login_html = {
    .name = "ap_login.html",
};

// Array of pointers to files, NULL terminated
static const esp_embedded_file_t *ro_files[] = {
    &favicon_ico,
    &index_html_gz,
    &ap_login_html,
    NULL
};

static const esp_embedded_file_t *find_file (const char *filename)
{
    uint_fast16_t idx = 0;
    const esp_embedded_file_t *file = NULL;

    if(*filename == '/')
        filename++;

    do {
        if(!strcmp(ro_files[idx]->name, filename))
            file = ro_files[idx];
    } while(file == NULL && ro_files[++idx] != NULL);

    return file;
}

static vfs_file_t *fs_open (const char *filename, const char *mode)
{
    vfs_file_t *fileh = NULL;

    if(strchr(mode, 'r')) {

        const esp_embedded_file_t *file = NULL;

        if((file = find_file(filename)) && (fileh = malloc(sizeof(vfs_file_t) + sizeof(embedded_filehandle_t)))) {
            embedded_filehandle_t *f = (embedded_filehandle_t *)&fileh->handle;
            f->file = file;
            f->remaining = file->size;
        }
    }

    return fileh;
}

static void fs_close (vfs_file_t *file)
{
    free(file);
}

static size_t fs_read (void *buffer, size_t size, size_t count, vfs_file_t *file)
{
    size_t rcount = 0;
    embedded_filehandle_t *fileh = (embedded_filehandle_t *)&file->handle;

    if(fileh->remaining) {
        rcount = size * count > fileh->remaining ? fileh->remaining : size * count;
        memcpy(buffer, &fileh->file->data[fileh->file->size - fileh->remaining], rcount);
    }

    fileh->remaining -= rcount;

    return rcount;
}

static size_t fs_write (const void *buffer, size_t size, size_t count, vfs_file_t *file)
{
    return 0;
}

static size_t fs_tell (vfs_file_t *file)
{
    embedded_filehandle_t *fileh = (embedded_filehandle_t *)&file->handle;

    return fileh->file->size - fileh->remaining;
}

static bool fs_eof (vfs_file_t *file)
{
    embedded_filehandle_t *fileh = (embedded_filehandle_t *)&file->handle;

    return fileh->remaining == 0;
}

static int fs_unlink (const char *filename)
{
    return -1;
}

static int fs_dirop (const char *path)
{
    return -1;
}

static vfs_dir_t *fs_opendir (const char *path)
{
    return NULL;
}

static void fs_closedir (vfs_dir_t *dir)
{
}

static int fs_stat (const char *filename, vfs_stat_t *st)
{
    const esp_embedded_file_t *file;

    if((file = find_file(filename))) {
        memset(st, 0, sizeof(vfs_stat_t));
        st->st_size = file->size;
    }

    return file ? 0 : -1;
}

void fs_embedded_mount (void)
{
    extern const unsigned char favicon_ico_start[] asm("_binary_favicon_ico_start");
    extern const unsigned char favicon_ico_end[]   asm("_binary_favicon_ico_end");
    favicon_ico.size = favicon_ico_end - favicon_ico_start;
    favicon_ico.data = favicon_ico_start;

    extern const unsigned char ap_login_html_start[] asm("_binary_ap_login_html_start");
    extern const unsigned char ap_login_html_end[]   asm("_binary_ap_login_html_end");
    ap_login_html.size = ap_login_html_end - ap_login_html_start;
    ap_login_html.data = ap_login_html_start;

    extern const unsigned char index_html_gz_start[] asm("_binary_index_html_gz_start");
    extern const unsigned char index_html_gz_end[]   asm("_binary_index_html_gz_end");
    index_html_gz.size = index_html_gz_end - index_html_gz_start;
    index_html_gz.data = index_html_gz_start;

    static const vfs_t fs = {
        .fopen = fs_open,
        .fclose = fs_close,
        .fread = fs_read,
        .fwrite = fs_write,
        .ftell = fs_tell,
        .feof = fs_eof,
        .funlink = fs_unlink,
        .fmkdir = fs_dirop,
        .fchdir = fs_dirop,
        .frmdir = fs_dirop,
        .fopendir = fs_opendir,
        .fclosedir = fs_closedir,
        .fstat = fs_stat
    };

    vfs_mount("/embedded", &fs);
}
