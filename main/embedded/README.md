#### Embedded files for WebUI maintenance etc.

The embedded folder contains files that are linked into the binary and made available via a read-only filesystem mount to the code.

_index.html.gz_ is the maintenance page for the WebUI, when trying to open the WebUI for the first time this file is loaded in the browser.
It shows a page that allows you to upload the WebUI _index.html.gz_ that contains the WebUI proper to the local filesystem as well as other files you may want to have permanently available.

Later the mainetance page can be reopened by the URL _http://\<controller ip\>/?forcefallback=yes_. Replace _\<controller ip\>_ with the IP address of your controller.

_favicon.ico_ is the icon used for the WebUI tab in the browser.

_ap_login.html_ is used for connecting to a router when the controller is in WiFi AP mode. Normally you will be asked to open a web page when connecting to the controller, this is the page that will be served.

---

It is possible to access the local filesystem from a WebDAV client, if the WebDAV protocol is enabled in the controller, by specifying _/littlefs_ as the remote directory.

---

2022-09-24
