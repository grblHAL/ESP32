/*
  webui/server.h - An embedded CNC Controller with rs274/ngc (g-code) support

  WebUI backend for https://github.com/luc-github/ESP3D-webui

  Part of grblHAL

  Copyright (c) 2019-2022 Terje Io

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

#ifndef __WEBUI_SERVER_H__
#define __WEBUI_SERVER_H__

#include <esp_http_server.h>
#include <sys/socket.h>

#include "freertos/task.h"
#include "web/backend.h"

#include "../webui/webui.h"

extern struct fs_file *fs_create (void);

esp_err_t webui_http_command_handler (httpd_req_t *req);
esp_err_t webui_sdcard_handler (httpd_req_t *req);
esp_err_t webui_sdcard_upload_handler (httpd_req_t *req);
esp_err_t webui_spiffs_handler (httpd_req_t *req);
esp_err_t webui_spiffs_upload_handler (httpd_req_t *req);
esp_err_t webui_index_html_get_handler (httpd_req_t *req);

void data_is_json (void);
bool claim_output (struct fs_file **file);
void webui_set_http_request (httpd_req_t *req);
void write_response (struct fs_file *file);

#endif

