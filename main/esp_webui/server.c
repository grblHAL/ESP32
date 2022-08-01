/*
  webui/server.c - An embedded CNC Controller with rs274/ngc (g-code) support

  WebUI backend for https://github.com/luc-github/ESP3D-webui

  Part of grblHAL

  Copyright (c) 2020-2022 Terje Io

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

#include "driver.h"

#if WEBUI_ENABLE

#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <sys/socket.h>

#include <cJSON.h>

#include "esp_spiffs.h"

#include "wifi.h"
#include "../esp_webui/webui.h"
#include "../webui/commands.h"
#include "networking/websocketd.h"
#include "networking/urldecode.h"
#include "networking/utils.h"
#include "networking/strutils.h"
#include "networking/http_upload.h"
#include "grbl/protocol.h"

#if SDCARD_ENABLE
#include "sdcard/sdcard.h"
#include "esp_vfs_fat.h"
#endif

#if WEBUI_AUTH_ENABLE
#include "../webui/login.h"
#endif

#ifndef WEBUI_AUTO_REPORT_INTERVAL
#define WEBUI_AUTO_REPORT_INTERVAL 0 // ms
#endif

extern struct fs_file *fs_create (void);
extern err_t fs_open(struct fs_file *file, const char *name);
extern void fs_close(struct fs_file *file);
extern int fs_read(struct fs_file *file, char *buffer, int count);
extern void fs_reset (void);

static const char *TAG = "webui";
static uint32_t auto_report_interval = WEBUI_AUTO_REPORT_INTERVAL;
static httpd_req_t *http_request = NULL;
static driver_reset_ptr driver_reset;
static on_report_options_ptr on_report_options;
static on_stream_changed_ptr on_stream_changed;
static on_execute_realtime_ptr on_execute_realtime;
static stream_write_ptr pre_stream;
static stream_write_ptr claim_stream;

void data_is_json (void)
{
    if(http_request)
        httpd_resp_set_type(http_request, HTTPD_TYPE_JSON);
}

static void stream_changed (stream_type_t type)
{
    if(on_stream_changed)
        on_stream_changed(type);

    if(!(type == StreamType_SDCard || type == StreamType_FlashFs) && hal.stream.write == claim_stream)
        hal.stream.write = pre_stream;
}

static void http_write (const char *s)
{
    httpd_resp_sendstr_chunk(http_request, s);
}

bool claim_output (struct fs_file **file)
{
    pre_stream = hal.stream.write;

    claim_stream = hal.stream.write = http_write; // (*file = fs_create()) ? hal.stream.write : NULL;

    return claim_stream != NULL;
}

void webui_set_http_request (httpd_req_t *req)
{
    http_request = req;
    httpd_resp_set_hdr(http_request, "Cache-Control", "no-cache");
}

void write_response (struct fs_file *file)
{
/*
    !! MCU crashes on free() in fs.c for some odd reason

    char buf[201];
    uint_fast16_t len = fs_bytes_left(file);

    pre_stream(uitoa(len));

    err_t t;
    fs_close(file);
    if(fs_open(file, file_is_json ? "cgi:qry.json" : "cgi:qry.txt") == 0) {

        pre_stream("open!");


        if(len < 200) {
            fs_read(file, buf, len);
            buf[len] = '\0';
            httpd_resp_sendstr(http_request, buf);
        } else {
            while((len = fs_bytes_left(file))) {
                fs_read(file, buf, len > 200 ? 200 : len);
                buf[len > 200 ? 200 : len] = '\0';
                httpd_resp_sendstr_chunk(http_request, buf);
            }
            httpd_resp_sendstr_chunk(http_request, NULL);
        }

        fs_close(file);
    } */
    hal.stream.write = pre_stream;
    httpd_resp_sendstr_chunk(http_request, NULL);
}

#if !WEBUI_AUTH_ENABLE

static webui_auth_level_t get_auth_level (http_request_t *req)
{
    return WebUIAuth_Admin;
}

#endif // WEBUI_AUTH_ENABLE

bool is_authorized (httpd_req_t *req, webui_auth_level_t min_level)
{
#if WEBUI_AUTH_ENABLE
    webui_auth_level_t auth_level;

    if((auth_level = get_auth_level(req)) < min_level) {
        httpd_resp_set_type(req, HTTPD_TYPE_TEXT);
        httpd_resp_set_status(req, auth_level < WebUIAuth_User ? "401 Unauthorized" : "403 Forbidden");
        httpd_resp_sendstr(req, auth_level < WebUIAuth_User ? "Login and try again\n" : "Not authorized\n");
        return false;
    }
#endif
    return true;
}

esp_err_t webui_http_command_handler (httpd_req_t *req)
{
    bool ok;
    char data[100], *cmd, *query = NULL;
    size_t qlen = httpd_req_get_url_query_len(req);

    if(qlen && (query = malloc(qlen + 1))) {

        httpd_req_get_url_query_str(req, query, qlen);

        if(http_get_key_value(query, "commandText", data, sizeof(data)) == NULL && http_get_key_value(query, "cmd", data, sizeof(data)) == NULL)
            http_get_key_value(query, "plain", data, sizeof(data));

        free(query);

    } else
        *data = '\0';

    if((ok = (*data != '\0'))) {

        hal.stream.state.webui_connected = On;

        if((cmd = strstr(data, "[ESP"))) {

            struct fs_file *file;
            status_code_t status;

            claim_output(&file);
            webui_set_http_request(req);

            cmd += 4;

            uint_fast16_t argc = 0;
            char c, cp = '\0', *args = NULL, **argv = NULL, *tmp, **tmp2, *tmp3;

            if((ok = (args = strchr(cmd, ']')))) {

                *args++ = '\0';

                if(*args) {

                    // Trim leading and trailing spaces
                    while(*args == ' ')
                        args++;

                    if((tmp = args + strlen(args)) != args) {
                        while(*(--tmp) == ' ')
                            *tmp = '\0';
                    }

                    // remove duplicate delimiters (spaces)
                    tmp = tmp3 = args;
                    while((c = *tmp++) != '\0') {
                        if(c != ' ' || cp != ' ')
                            *tmp3++ = c;
                        cp = c;
                    }
                    *tmp3 = '\0';
                }


                // tokenize arguments (if any)
                if(*args) {

                    argc = 1;
                    tmp = args;
                    while((c = *tmp++) != '\0') {
                        if(c == ' ')
                            argc++;
                    }

                    if(argc == 1)
                        argv = &args;
                    else if((ok = !!(argv = tmp2 = malloc(sizeof(char *) * argc)))) {

                        tmp = strtok(args, " ");
                        while(tmp) {
                            *tmp2++ = tmp;
                            tmp = strtok(NULL, " ");
                        }

                        tmp = args;
                        while((c = *tmp) != '\0') {
                            if(c == ' ')
                                *tmp = '\0';
                            tmp++;
                        }
                    } else {
                        httpd_resp_set_status(req, "500 Internal server error");
                        hal.stream.write("Failed to generate response");
                    }
                }

                ok &= (status = webui_command_handler(atol(cmd), argc, argv, get_auth_level(req))) == Status_OK;

    #if WEBUI_AUTH_ENABLE
                if(status == Status_AuthenticationRequired || status == Status_AccessDenied) {
                    httpd_resp_set_status(req, status == Status_AuthenticationRequired ? "401 Unauthorized" : "403 Forbidden");
//                    if(fs_bytes_left(file) == 0)
//                        hal.stream.write(status == Status_AuthenticationRequired ? "Login and try again\n" : "Not authorized\n"); // ??
                } else
    #endif
                if(status != Status_OK)
                    hal.stream.write("Bad command!\n");

                if(argc > 1 && argv)
                    free(argv);
            } else
                hal.stream.write("Bad command!\n");

            write_response(file);

        } else { // GCode - add to websocket input buffer

//          if(!is_authorized(req, WebUIAuth_User))
//              return ESP_OK;

            if(strlen(data) == 1)
                websocketd_RxPutC(*data);

            else {

                size_t len;
                char c, *block = strtok(data, "\n");

                while(block) {

                    if((len = strlen(block)) == 2 && *block == 0xC2) {
                        block++;
                        len--;
                    }

                    while((c = *block++))
                        websocketd_RxPutC(c);

                    if(len > 1)
                        websocketd_RxPutC(ASCII_LF);

                    block = strtok(NULL, "\n");
                }
            }

            httpd_resp_send(req, NULL, 0);
        }
    }

    return ok ? ESP_OK : ESP_FAIL;
}

#if SDCARD_ENABLE

static bool spiffs_isdirectory(char *filename)
{
    return strrchr(filename, '/') && filename[strlen(filename) - 1] == '.';
}

static bool spiffs_add_file (cJSON *files, char *filename, struct stat *file)
{
    bool ok;
    cJSON *fileinfo;

    if((ok = (fileinfo = cJSON_CreateObject()) != NULL))
    {
        ok = cJSON_AddStringToObject(fileinfo, "name", filename) != NULL;
        if(S_ISDIR(file->st_mode))
            ok &= cJSON_AddNumberToObject(fileinfo, "size", -1.0f) != NULL;
        else
            ok &= cJSON_AddStringToObject(fileinfo, "size", btoa(file->st_size)) != NULL;
        if(ok)
            cJSON_AddItemToArray(files, fileinfo);
    }

    return ok;
}

static bool spiffs_scan_dir (cJSON *files, char *path, uint_fast8_t depth)
{
    DIR *dir;
    struct dirent *entry;
    struct stat file;
    char *fname;

    if(!(dir = opendir(path)))
        return false;

    size_t pathlen = strlen(path);

    if(path[pathlen - 1] != '/')
        path[pathlen++] = '/';

    while((entry = readdir(dir))) {

        strcat(path, entry->d_name);

        if(stat(path, &file) == 0) {
            fname = strrchr(path, '/');
            if(spiffs_isdirectory(fname)) {
                file.st_mode = S_IFDIR;
                *fname = '\0';
                fname = strrchr(path, '/') + 1;
            } else
                fname++;

            if(fname - path == pathlen)
                spiffs_add_file(files, fname, &file);
            if(path[pathlen - 1] == '\0')
                path[pathlen - 1] = '/';
        }

        path[pathlen] = '\0';
    }

    closedir(dir);

    return true;
}

static bool spiffs_ls (httpd_req_t *req, char *path, char *status)
{
    bool ok;
    cJSON *root = cJSON_CreateObject(), *files = NULL;

    if((ok = (root && (files = cJSON_AddArrayToObject(root, "files"))))) {

        if(strlen(path) > 1)
            path[strlen(path) - 1] = '\0';

        spiffs_scan_dir(files, path, 1);

        cJSON_AddStringToObject(root, "path", path);

        size_t total = 0, used = 0;

        if(esp_spiffs_info(NULL, &total, &used) == ESP_OK) {

            uint32_t pct_used = (used * 100) / total;
            cJSON_AddStringToObject(root, "total", btoa(total));
            cJSON_AddStringToObject(root, "used", btoa(used));
            cJSON_AddStringToObject(root, "occupation", uitoa(pct_used == 0 ? 1 : pct_used));
        }
        cJSON_AddStringToObject(root, "mode", "direct");
        cJSON_AddStringToObject(root, "status", status);

        char *resp = cJSON_PrintUnformatted(root);

        httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
        httpd_resp_set_type(req, HTTPD_TYPE_JSON);
        httpd_resp_send(req, resp, strlen(resp));

        free(resp);
    }

    if(root)
        cJSON_Delete(root);

    return ok;
}

static bool spiffs_rmdir (char *path)
{
    bool ok = true;
    DIR *dir;
    struct dirent *entry;
    fs_path_t filepath;

    if(!(dir = opendir("/spiffs")))
        return false;

    path += 8; // skip "/spiffs/" part

    size_t pathlen = strlen(path);

    while(ok && (entry = readdir(dir))) {
        if(strlen(entry->d_name) >= pathlen && memcmp(entry->d_name, path, pathlen) == 0) {
            strcat(strcpy(filepath, "/spiffs/"), entry->d_name);
            ok = unlink(filepath) == 0;
        }
    }

    closedir(dir);

    return ok;
}

esp_err_t webui_spiffs_handler (httpd_req_t *req)
{
    bool ok;
    char *query = NULL;
    fs_path_t path;
    fs_filename_t filename;
    char action[20], status[sizeof(fs_filename_t) + 50];
    size_t qlen = httpd_req_get_url_query_len(req);

    *status = '\0';

    if(!is_authorized(req, WebUIAuth_User))
        return ESP_OK;

#if WEBUI_AUTH_ENABLE
    if(get_auth_level(req) == WebUIAuth_User)
        strcpy(path, "/user");
    else
#endif
    *path = '\0';

    if(qlen && (query = malloc(qlen + 1))) {

        httpd_req_get_url_query_str(req, query, qlen);

        http_get_key_value(query, "path", path + strlen(path), sizeof(path - strlen(path)));
        http_get_key_value(query, "filename", filename, sizeof(filename));
        http_get_key_value(query, "action", action, sizeof(action));

        if(*action && *filename) {

            struct stat file;
            char *fullname = ((file_server_data_t *)req->user_ctx)->scratch;

            strcat(strcpy(fullname, "/spiffs"), path);
            strcpy(path, fullname);
            strcat(fullname, filename);

            switch(strlookup(action, "delete,createdir,deletedir,list", ',')) {

                case 0: // delete
                    if(stat(fullname, &file) == 0) {
                        if(!spiffs_isdirectory(filename) && unlink(fullname) == 0)
                            sprintf(status, "%s deleted", filename);
                        else
                            sprintf(status, "Cannot delete %s!", filename);
                    } else
                        sprintf(status, "%s does not exist!", filename);
                    break;

                case 1: // createdir
                    strcat(fullname, "/.");
                    if(stat(fullname, &file) != 0) {
                        FILE *dir;
                        if((dir = fopen(fullname, "w"))) {
                            sprintf(status, "%s created", filename);
                            fclose(dir);
                        } else
                            sprintf(status, "Cannot create %s!", filename);
                    } else
                        sprintf(status, "%s already exists!", filename);
                    break;

                case 2: // deletedir
                    strcat(fullname, "/.");
                    if(strlen(fullname) == 10)
                        strcpy(status, "Cannot delete root directory!");
                    else if(stat(fullname, &file) == 0) {
                        if(spiffs_rmdir(fullname))
                            sprintf(status, "%s deleted", filename);
                        else
                            sprintf(status, "Error deleting %s!", filename);
                    } else
                        sprintf(status, "%s does not exist!", filename);
                    break;

                case 3: // list
                    {
                        strcpy(path, fullname);
                        char *file = strrchr(path, '/');
                        *++file = '\0'; // strip filename... WebUI sends "all" for some reason - which is not handled in original backend
                    }
                    break;

                default:
                    sprintf(status, "Invalid action \"%s\" for %s!", action, filename);
                    break;
            }
        }
    }

    if(*path == '\0')
        strcpy(path, "/spiffs/");

    if(!(ok = spiffs_ls(req, path, status)))
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to generate response");

    if(query)
        free(query);

    return ok ? ESP_OK : ESP_FAIL;
}

webui_auth_level_t auth_level;

static void spiffs_on_upload_name_parsed (char *name)
{
    static const char *prefix = "/spiffs/";

    size_t len = strlen(name), plen = strlen(prefix);
    if(*name == '/')
        plen--;

    if(len + plen <= HTTP_UPLOAD_MAX_PATHLENGTH) {
        memmove(name + plen, name, len + 1);
        memcpy(name, prefix, plen);
    }
}

esp_err_t webui_spiffs_upload_handler (httpd_req_t *req)
{
    bool ok = false;
    int ret;

    if(!is_authorized(req, WebUIAuth_User))
        return ESP_OK;

    char *rqhdr = NULL, *boundary;
    size_t len = httpd_req_get_hdr_value_len(req, "Content-Type");

    if(len) {
        rqhdr = malloc(len + 1);
        httpd_req_get_hdr_value_str(req, "Content-Type", rqhdr, len + 1);

        if((ok = (boundary = strstr(rqhdr, "boundary=")))) {
            boundary += strlen("boundary=");
            ok = http_upload_start(req, boundary, false);
            http_upload_on_filename_parsed(spiffs_on_upload_name_parsed);
        }
    }

    fs_path_t path;
    char *scratch = ((file_server_data_t *)req->user_ctx)->scratch;
    file_upload_t *upload = (file_upload_t *)req->sess_ctx;

    *path = '\0';

    if (ok) do { // Process received data

        if ((ret = httpd_req_recv(req, scratch, sizeof(fs_scratch_t))) <= 0) {
            ok = false;
            if (ret == HTTPD_SOCK_ERR_TIMEOUT)
                httpd_resp_send_408(req);
            break;
        }

        http_upload_chunk(req, scratch, (size_t)ret);

        if(*path == '\0' && *upload->path)
            strcat(strcpy(path, "/spiffs"), upload->path);

    } while(upload->state != Upload_Complete);

    if(*path == '\0') // in case something failed...
        strcpy(path, "/");

    if(!(ok = spiffs_ls(req, path, "ok")))
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to generate response");

    if(req->sess_ctx && req->free_ctx) {
        req->free_ctx(req->sess_ctx);
        req->sess_ctx = NULL;
    }

    if(rqhdr)
        free(rqhdr);

    return ok ? ESP_OK : ESP_FAIL;
}

#endif


esp_err_t webui_index_html_get_handler (httpd_req_t *req)
{
    extern const unsigned char index_html_gz_start[] asm("_binary_index_html_gz_start");
    extern const unsigned char index_html_gz_end[]   asm("_binary_index_html_gz_end");
    set_content_type_from_file(req, "index.html");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    return httpd_resp_send(req, (const char *)index_html_gz_start, index_html_gz_end - index_html_gz_start);
}

static void webui_reset (void)
{
    driver_reset();

//    fs_reset();

    if(hal.stream.write == claim_stream)
        hal.stream.write = pre_stream;
}

static void webui_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[PLUGIN:ESP32 WebUI v0.03]" ASCII_EOL);
}

static void webui_check_networking (uint_fast16_t state)
{
    network_settings_t *settings = get_network_settings();

    if(!(settings->services.http && settings->services.websocket))
        report_message("WebUI requires both http and websocket services enabled!", Message_Warning);
}

static void webui_auto_report (sys_state_t state)
{
    static uint32_t ms = 0;

    if(auto_report_interval > 0 && (hal.get_elapsed_ticks() - ms) >= auto_report_interval) {
        ms = hal.get_elapsed_ticks();
        if(hal.stream.state.webui_connected)
            protocol_enqueue_realtime_command(CMD_STATUS_REPORT);
    }

    on_execute_realtime(state);
}

void webui_init (void)
{
    driver_reset = hal.driver_reset;
    hal.driver_reset = webui_reset;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = webui_options;

    on_stream_changed = grbl.on_stream_changed;
    grbl.on_stream_changed = stream_changed;

    on_execute_realtime = grbl.on_execute_realtime;
    grbl.on_execute_realtime = webui_auto_report;

    //    grbl.on_user_command = webui_parse_command;

#if WEBUI_AUTH_ENABLE
    login_init();
#endif

    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 50,
      .format_if_mount_failed = true
    };

    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
    }

    protocol_enqueue_rt_command(webui_check_networking);
}

#endif
