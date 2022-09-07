/*
  fs_spiffs.c - VFS mount for spiffs

  Part of grblHAL

  Copyright (c) 2022 Terje Io

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

#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <dirent.h>
#include <sys/param.h>
#include <sys/unistd.h>
#include <sys/stat.h>

#include "driver.h"
#include "grbl/platform.h"
#include "grbl/vfs.h"

#include "esp_spiffs.h"

#define SPIFFS_PARTITION_LABEL "storage"

static vfs_file_t *fs_open (const char *filename, const char *mode)
{
    FILE *f;
    vfs_file_t *file = malloc(sizeof(vfs_file_t) + sizeof(FILE *));

    if(file) {

        if((f = fopen(filename, mode)) == NULL) {
            free(file);
            file = NULL;
        } else
            memcpy(&file->handle, f, sizeof(FILE));

        //else
          //  file->size = f_size((FIL *)&file->handle);
    }

    return file;
}

static void fs_close (vfs_file_t *file)
{
    fclose((FILE *)&file->handle);
    free(file);
}

static size_t fs_read (void *buffer, size_t size, size_t count, vfs_file_t *file)
{
    size_t bytesread = fread(buffer, size, count, (FILE *)&file->handle);

    return (size_t)bytesread;
}

static size_t fs_write (const void *buffer, size_t size, size_t count, vfs_file_t *file)
{
    size_t byteswritten = fwrite(buffer, size, count, (FILE *)&file->handle);

    return byteswritten;
}

static size_t fs_tell (vfs_file_t *file)
{
    return ftell((FILE *)&file->handle);
}

static bool fs_eof (vfs_file_t *file)
{
    return feof((FILE *)&file->handle) != 0;
}

static int fs_unlink (const char *filename)
{
    return unlink(filename);
}

static int fs_mkdir (const char *path)
{
    return -1; //mkdir(path);
}

static int fs_chdir (const char *path)
{
    return -1; //f_chdir(path);
}

char *fs_getcwd (char *buf, size_t size)
{
    static char cwd[255] = "";


    return cwd;
}

static vfs_dir_t *fs_opendir (const char *path)
{
    DIR *d;
    vfs_dir_t *dir = malloc(sizeof(vfs_dir_t) + sizeof(DIR));

    if(dir && (d = opendir(path)) == NULL)
    {
        free(dir);
        dir = NULL;
    } else
        memcpy(&dir->handle, d, sizeof(DIR));

    return dir;
}

static char *fs_readdir (vfs_dir_t *dir, vfs_dirent_t *dirent)
{
    struct dirent *fi;

    *dirent->name = '\0';

    if((fi = readdir((DIR *)&dir->handle)) == NULL || *fi->d_name == '\0')
        return NULL;

    if(fi->d_name && *fi->d_name != '\0')
        strcpy(dirent->name, fi->d_name);

//    dirent->size = fi->fsize;
    if(fi->d_type & DT_DIR)
        dirent->st_mode.directory = true;

    return dirent->name;
}

static void fs_closedir (vfs_dir_t *dir)
{
    if (dir) {
        vfs_errno = closedir((DIR *)&dir->handle);
        free(dir);
    }
}

static int fs_stat (const char *filename, vfs_stat_t *st)
{
    int ret;
    struct stat f;

    if((ret = stat(filename, &f)) == 0) {
        st->st_size = f.st_size;
        st->st_mode.mode = f.st_mode;
//        st->st_mtim = f.st_mtim;
    }

    return 0;
}

static bool fs_getfree (vfs_free_t *free)
{
    esp_err_t err = esp_spiffs_info(SPIFFS_PARTITION_LABEL, &free->size, &free->used);

    return err == ESP_OK;
}

static int fs_format (void)
{
    esp_err_t err = esp_spiffs_format(SPIFFS_PARTITION_LABEL);

    return err;
}

void fs_spiffs_mount (void)
{
    static const vfs_t fs = {
        .fs_name = "spiffs",
        .fopen = fs_open,
        .fclose = fs_close,
        .fread = fs_read,
        .fwrite = fs_write,
        .ftell = fs_tell,
        .feof = fs_eof,
        .funlink = fs_unlink,
        .fmkdir = fs_mkdir,
        .fchdir = fs_chdir,
        .frmdir = fs_unlink,
        .fopendir = fs_opendir,
        .readdir = fs_readdir,
        .fclosedir = fs_closedir,
        .fstat = fs_stat,
        .fgetcwd = fs_getcwd,
        .fgetfree = fs_getfree,
        .format = fs_format
    };

    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = SPIFFS_PARTITION_LABEL,
        .max_files = 4,
        .format_if_mount_failed = true
    };

    if(esp_vfs_spiffs_register(&conf) == ESP_OK)
        vfs_mount("/spiffs", &fs);
}
