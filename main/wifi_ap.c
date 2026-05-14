#include "wifi_ap.h"

#include <string.h>
#include <stdio.h>

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "esp_app_format.h"
#include "esp_system.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"

#include "ble.h"
#include "version.h"

extern float ble_get_trip_km(void);

#define TAG                "WIFI_AP"
#define WIFI_AP_PASSWORD   "gbreceiver"
#define WIFI_AP_CHANNEL    1
#define WIFI_AP_MAX_CONN   1

static bool running = false;
static httpd_handle_t server = NULL;
static esp_netif_t *ap_netif = NULL;
static bool netif_event_inited = false;
static TimerHandle_t stop_timer = NULL;

static void stop_timer_cb(TimerHandle_t t)
{
    wifi_ap_stop();
}

// ---- Embedded web UI ----

static const char index_html[] =
"<!doctype html><html><head><meta charset=utf-8>"
"<meta name=viewport content='width=device-width,initial-scale=1'>"
"<title>GB Receiver</title>"
"<style>"
"body{font-family:system-ui,sans-serif;margin:0;background:#f5f5f7;color:#1d1d1f;}"
".wrap{max-width:560px;margin:0 auto;padding:16px;}"
".card{background:#fff;border:1px solid #d2d2d7;margin-bottom:12px;}"
".hdr{padding:10px 14px;border-bottom:1px solid #d2d2d7;font-size:11px;letter-spacing:.2em;text-transform:uppercase;color:#6e6e73;}"
".body{padding:12px 14px;}"
".row{display:flex;justify-content:space-between;padding:4px 0;font-size:13px;}"
".row span:first-child{color:#6e6e73;}"
"button{width:100%;padding:10px;margin:4px 0;font:inherit;font-size:12px;letter-spacing:.15em;text-transform:uppercase;border:1px solid #d2d2d7;background:#fff;cursor:pointer;}"
"button.pri{border-color:#f56300;color:#f56300;background:rgba(245,99,0,.06);}"
"button:disabled{opacity:.5;cursor:not-allowed;}"
".item{display:flex;justify-content:space-between;align-items:center;padding:8px;border:1px solid #d2d2d7;margin:4px 0;font-size:12px;}"
".item.live{border-color:#16a34a;background:#ecfdf5;}"
".mac{font-family:ui-monospace,monospace;color:#6e6e73;font-size:11px;}"
".badge{font-size:9px;color:#16a34a;font-weight:700;letter-spacing:.1em;}"
".bars{display:inline-flex;align-items:flex-end;gap:2px;height:14px;margin-left:6px;}"
".bars i{width:3px;background:#d2d2d7;}"
".bars i.on{background:#16a34a;}"
".empty{padding:16px;text-align:center;color:#6e6e73;font-size:12px;}"
"</style></head><body><div class=wrap>"
"<div class=card><div class=hdr>Device</div><div class=body id=status>...</div></div>"
"<div class=card><div class=hdr>Paired Remotes</div><div class=body><div id=paired class=empty>Loading...</div></div></div>"
"<div class=card><div class=body>"
"<button class=pri id=scanBtn onclick=scan()>Pair New Remote</button>"
"<button onclick=resetOdo()>Reset Odometer</button>"
"</div></div>"
"<div class=card id=scanCard style='display:none'><div class=hdr id=scanHdr>Scanning</div><div class=body><div id=scanList class=empty>Waiting...</div></div></div>"
"<div class=card><div class=hdr>Firmware Update</div><div class=body>"
"<input id=fwFile type=file accept='.bin' style='display:none' onchange='fwPick()'>"
"<button id=fwBtn onclick='fwPickClick()'>Upload Firmware (.bin)</button>"
"<div id=fwBar style='display:none;height:6px;background:#d2d2d7;margin:10px 0;overflow:hidden'><div id=fwFill style='width:0%;height:100%;background:#f56300;transition:width .2s'></div></div>"
"<div id=fwMsg style='display:none;font-size:12px;color:#6e6e73;margin-top:6px'></div>"
"<div style='font-size:11px;color:#6e6e73;margin-top:8px;line-height:1.4'>Download <code>gb_receiver.bin</code> from the <a href='https://github.com/georgebenett/gb_receiver/releases' target=_blank>releases page</a> before joining this Wi-Fi, then upload it here.</div>"
"</div></div>"
"</div><script>"
"let pollT,scanT;"
"async function api(p,o){const r=await fetch(p,o);return r.json();}"
"function bars(q){let s=[],n={Excellent:4,Good:3,Fair:2,Weak:1,Poor:0}[q]||0;for(let i=0;i<4;i++)s.push('<i class='+(i<n?'on':'')+' style=height:'+([4,7,10,13][i])+'px></i>');return '<span class=bars>'+s.join('')+'</span>';}"
"async function tick(){try{const s=await api('/api/status');"
"document.getElementById('status').innerHTML="
"'<div class=row><span>Firmware</span><span>v'+s.fw+'</span></div>'+"
"'<div class=row><span>Trip</span><span>'+s.trip_km.toFixed(2)+' km</span></div>'+"
"'<div class=row><span>BLE</span><span>'+(s.ble_connected?'Connected':'Idle')+'</span></div>'+"
"'<div class=row><span>Paired</span><span>'+s.paired_count+' / 4</span></div>';"
"const p=await api('/api/paired');"
"const ph=document.getElementById('paired');"
"if(!p.remotes.length)ph.outerHTML='<div id=paired class=empty>No remotes paired</div>';"
"else ph.outerHTML='<div id=paired>'+p.remotes.map((r,i)=>'<div class=\"item '+(r.connected?'live':'')+'\"><span><b>#'+(i+1)+'</b> '+(r.connected?'<span class=badge>● CONNECTED</span>':'')+'</span><span><span class=mac>'+r.mac+'</span> <button style=\"width:auto;padding:4px 8px;display:inline\" onclick=\"unpair(\\''+r.mac+'\\')\">×</button></span></div>').join('')+'</div>';"
"}catch(e){}}"
"async function scan(){document.getElementById('scanCard').style.display='block';document.getElementById('scanHdr').textContent='Scanning';document.getElementById('scanBtn').disabled=true;document.getElementById('scanList').outerHTML='<div id=scanList class=empty>Waiting for remotes...</div>';"
"await api('/api/scan',{method:'POST'});"
"clearInterval(scanT);scanT=setInterval(scanTick,800);scanTick();"
"setTimeout(()=>{clearInterval(scanT);scanTick();document.getElementById('scanHdr').textContent='Scan complete';document.getElementById('scanBtn').disabled=false;},9000);}"
"async function scanTick(){const r=await api('/api/scan_results');const list=document.getElementById('scanList');"
"if(!r.remotes.length){list.outerHTML='<div id=scanList class=empty>Waiting for remotes...</div>';return;}"
"r.remotes.sort((a,b)=>b.rssi-a.rssi);"
"list.outerHTML='<div id=scanList>'+r.remotes.map(x=>'<div class=item><span><b>'+x.name+'</b><br><span class=mac>'+x.mac+'</span></span><span>'+x.quality+bars(x.quality)+' <button style=\"width:auto;padding:4px 10px;display:inline\" onclick=\"pair(\\''+x.mac+'\\','+x.addr_type+')\">Pair</button></span></div>').join('')+'</div>';}"
"async function pair(mac,t){await api('/api/pair',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({mac,addr_type:t})});clearInterval(scanT);document.getElementById('scanCard').style.display='none';document.getElementById('scanBtn').disabled=false;alert('Paired. Wi-Fi will close in 3s so the remote can connect.');tick();}"
"async function unpair(mac){if(!confirm('Unpair '+mac+'?'))return;await api('/api/unpair',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({mac})});tick();}"
"async function resetOdo(){if(!confirm('Reset trip odometer?'))return;await api('/api/reset_odo',{method:'POST'});tick();}"
"function fwMsg(t){const m=document.getElementById('fwMsg');m.style.display='block';m.textContent=t;}"
"function fwPickClick(){document.getElementById('fwFile').click();}"
"function fwPick(){const f=document.getElementById('fwFile').files[0];if(!f)return;if(!confirm('Upload '+f.name+' ('+(f.size/1048576).toFixed(2)+' MB)? The device will reboot.')){document.getElementById('fwFile').value='';return;}fwUpload(f);}"
"function fwUpload(f){clearInterval(pollT);document.getElementById('fwBtn').disabled=true;document.getElementById('fwBar').style.display='block';document.getElementById('fwFill').style.width='0%';fwMsg('Uploading...');"
"const xhr=new XMLHttpRequest();xhr.open('POST','/api/ota',true);xhr.setRequestHeader('Content-Type','application/octet-stream');xhr.timeout=180000;"
"xhr.upload.onprogress=e=>{if(!e.lengthComputable)return;const p=e.loaded/e.total*100;document.getElementById('fwFill').style.width=p.toFixed(1)+'%';fwMsg('Uploading '+p.toFixed(0)+'%  ('+(e.loaded/1048576).toFixed(2)+' / '+(e.total/1048576).toFixed(2)+' MB)');};"
"xhr.onload=()=>{if(xhr.status===200){document.getElementById('fwFill').style.width='100%';fwMsg('Update complete. Device is rebooting — reconnect to Wi-Fi after reboot to verify.');}else{fwMsg('Failed: '+xhr.status+' '+(xhr.responseText||''));document.getElementById('fwBtn').disabled=false;pollT=setInterval(tick,2000);}};"
"xhr.onerror=()=>{fwMsg('Upload error / connection lost. If the device rebooted, this is expected.');};"
"xhr.ontimeout=()=>{fwMsg('Upload timed out.');document.getElementById('fwBtn').disabled=false;};"
"xhr.send(f);}"
"tick();pollT=setInterval(tick,2000);"
"</script></body></html>";

// ---- Helpers ----

static int parse_mac(const char *s, uint8_t out[6])
{
    if (!s) return -1;
    int v[6];
    if (sscanf(s, "%x:%x:%x:%x:%x:%x", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6) return -1;
    for (int i = 0; i < 6; i++) out[i] = (uint8_t)v[i];
    return 0;
}

static void mac_to_str(const uint8_t mac[6], char out[18])
{
    snprintf(out, 18, "%02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static const char *rssi_to_quality(int8_t rssi)
{
    if (rssi >= -55) return "Excellent";
    if (rssi >= -65) return "Good";
    if (rssi >= -75) return "Fair";
    if (rssi >= -85) return "Weak";
    return "Poor";
}

static esp_err_t send_json(httpd_req_t *req, cJSON *root)
{
    char *s = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    httpd_resp_set_type(req, "application/json");
    esp_err_t err = httpd_resp_send(req, s, HTTPD_RESP_USE_STRLEN);
    free(s);
    return err;
}

// ---- HTTP handlers ----

static esp_err_t h_index(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, index_html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t h_favicon(httpd_req_t *req)
{
    httpd_resp_set_status(req, "204 No Content");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t h_status(httpd_req_t *req)
{
    uint8_t cmac[6] = {0};
    bool linked = ble_get_connected_mac(cmac);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "fw", FW_VERSION);
    cJSON_AddNumberToObject(root, "trip_km", ble_get_trip_km());
    cJSON_AddBoolToObject(root, "ble_connected", linked);
    cJSON_AddNumberToObject(root, "paired_count", ble_get_paired_count());
    cJSON_AddBoolToObject(root, "scanning", ble_is_scanning());
    return send_json(req, root);
}

static esp_err_t h_paired(httpd_req_t *req)
{
    ble_paired_entry_t list[BLE_MAX_PAIRED_REMOTES];
    uint8_t n = ble_get_paired_list(list, BLE_MAX_PAIRED_REMOTES);
    uint8_t cmac[6] = {0};
    bool linked = ble_get_connected_mac(cmac);

    cJSON *root = cJSON_CreateObject();
    cJSON *arr = cJSON_AddArrayToObject(root, "remotes");
    for (int i = 0; i < n; i++) {
        char mac_s[18];
        mac_to_str(list[i].mac, mac_s);
        cJSON *e = cJSON_CreateObject();
        cJSON_AddStringToObject(e, "mac", mac_s);
        cJSON_AddNumberToObject(e, "addr_type", list[i].addr_type);
        cJSON_AddBoolToObject(e, "connected", linked && memcmp(list[i].mac, cmac, 6) == 0);
        cJSON_AddItemToArray(arr, e);
    }
    return send_json(req, root);
}

static esp_err_t h_scan(httpd_req_t *req)
{
    bool ok = ble_start_scan();
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "ok", ok);
    return send_json(req, root);
}

static esp_err_t h_scan_results(httpd_req_t *req)
{
    ble_scan_entry_t entries[BLE_SCAN_MAX_RESULTS];
    uint8_t n = ble_get_scan_results(entries, BLE_SCAN_MAX_RESULTS);

    cJSON *root = cJSON_CreateObject();
    cJSON *arr = cJSON_AddArrayToObject(root, "remotes");
    for (int i = 0; i < n; i++) {
        char mac_s[18];
        mac_to_str(entries[i].mac, mac_s);
        cJSON *e = cJSON_CreateObject();
        cJSON_AddStringToObject(e, "mac", mac_s);
        cJSON_AddStringToObject(e, "name", entries[i].name);
        cJSON_AddNumberToObject(e, "rssi", entries[i].rssi);
        cJSON_AddStringToObject(e, "quality", rssi_to_quality(entries[i].rssi));
        cJSON_AddNumberToObject(e, "addr_type", entries[i].addr_type);
        cJSON_AddItemToArray(arr, e);
    }
    return send_json(req, root);
}

static int read_body(httpd_req_t *req, char *buf, size_t cap)
{
    int total = 0, got;
    while (total < (int)cap - 1) {
        got = httpd_req_recv(req, buf + total, cap - 1 - total);
        if (got <= 0) break;
        total += got;
    }
    buf[total] = '\0';
    return total;
}

static esp_err_t h_pair(httpd_req_t *req)
{
    char body[128];
    if (read_body(req, body, sizeof(body)) <= 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no body");
    }
    cJSON *j = cJSON_Parse(body);
    bool ok = false;
    if (j) {
        cJSON *mac_j = cJSON_GetObjectItem(j, "mac");
        cJSON *at_j = cJSON_GetObjectItem(j, "addr_type");
        uint8_t mac[6];
        if (cJSON_IsString(mac_j) && parse_mac(mac_j->valuestring, mac) == 0) {
            uint8_t at = (cJSON_IsNumber(at_j)) ? (uint8_t)at_j->valueint : 0;
            ok = ble_pair_remote(mac, at);
        }
        cJSON_Delete(j);
    }
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "ok", ok);
    esp_err_t err = send_json(req, root);
    // Free the radio for the new BLE link a couple seconds after responding.
    if (ok) wifi_ap_stop_after(3000);
    return err;
}

static esp_err_t h_unpair(httpd_req_t *req)
{
    char body[64];
    if (read_body(req, body, sizeof(body)) <= 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no body");
    }
    cJSON *j = cJSON_Parse(body);
    bool ok = false;
    if (j) {
        cJSON *mac_j = cJSON_GetObjectItem(j, "mac");
        uint8_t mac[6];
        if (cJSON_IsString(mac_j) && parse_mac(mac_j->valuestring, mac) == 0) {
            ok = ble_unpair_remote_by_mac(mac);
        }
        cJSON_Delete(j);
    }
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "ok", ok);
    return send_json(req, root);
}

static esp_err_t h_reset_odo(httpd_req_t *req)
{
    ble_reset_trip_distance();
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "ok", true);
    return send_json(req, root);
}

// ---- OTA upload ----

#define OTA_CHUNK_BYTES 4096

static void ota_reboot_task(void *arg)
{
    // Give the HTTP response a moment to flush over the TCP socket before
    // we pull the rug out from under the Wi-Fi stack.
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
}

static esp_err_t ota_fail(httpd_req_t *req, esp_ota_handle_t handle,
                         char *buf, httpd_err_code_t code, const char *msg)
{
    if (handle) esp_ota_abort(handle);
    if (buf) free(buf);
    ESP_LOGE(TAG, "OTA failed: %s", msg);
    return httpd_resp_send_err(req, code, msg);
}

static esp_err_t h_ota(httpd_req_t *req)
{
    if (req->content_len <= 0) {
        return httpd_resp_send_err(req, HTTPD_411_LENGTH_REQUIRED,
                                   "Content-Length required");
    }

    const esp_partition_t *target = esp_ota_get_next_update_partition(NULL);
    if (!target) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "no OTA partition available");
    }
    if ((size_t)req->content_len > target->size) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "image larger than partition");
    }

    ESP_LOGW(TAG, "OTA start: %d bytes -> %s @ 0x%lx",
             req->content_len, target->label, (unsigned long)target->address);

    esp_ota_handle_t handle = 0;
    esp_err_t err = esp_ota_begin(target, OTA_WITH_SEQUENTIAL_WRITES, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin: %s", esp_err_to_name(err));
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "ota_begin failed");
    }

    char *buf = malloc(OTA_CHUNK_BYTES);
    if (!buf) {
        return ota_fail(req, handle, NULL, HTTPD_500_INTERNAL_SERVER_ERROR, "oom");
    }

    int remaining = req->content_len;
    bool header_checked = false;
    int last_logged_pct = -10;

    while (remaining > 0) {
        int want = remaining < OTA_CHUNK_BYTES ? remaining : OTA_CHUNK_BYTES;
        int got = httpd_req_recv(req, buf, want);
        if (got <= 0) {
            if (got == HTTPD_SOCK_ERR_TIMEOUT) continue;
            return ota_fail(req, handle, buf,
                            HTTPD_500_INTERNAL_SERVER_ERROR, "recv");
        }

        if (!header_checked) {
            // First chunk must cover the app descriptor so we can refuse
            // images that aren't gb_receiver builds.
            const size_t hdr_off = sizeof(esp_image_header_t) +
                                   sizeof(esp_image_segment_header_t);
            if (got < (int)(hdr_off + sizeof(esp_app_desc_t))) {
                return ota_fail(req, handle, buf,
                                HTTPD_400_BAD_REQUEST, "image header too small");
            }
            const esp_app_desc_t *new_desc =
                (const esp_app_desc_t *)(buf + hdr_off);
            const esp_app_desc_t *cur_desc = esp_app_get_description();
            if (strncmp(new_desc->project_name, cur_desc->project_name,
                        sizeof(new_desc->project_name)) != 0) {
                ESP_LOGE(TAG, "project mismatch: got '%.32s' want '%.32s'",
                         new_desc->project_name, cur_desc->project_name);
                return ota_fail(req, handle, buf,
                                HTTPD_400_BAD_REQUEST, "project name mismatch");
            }
            ESP_LOGW(TAG, "OTA image: project='%.32s' version='%.32s'",
                     new_desc->project_name, new_desc->version);
            header_checked = true;
        }

        err = esp_ota_write(handle, buf, got);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write: %s", esp_err_to_name(err));
            return ota_fail(req, handle, buf,
                            HTTPD_500_INTERNAL_SERVER_ERROR, "ota_write");
        }
        remaining -= got;

        int pct = 100 - (int)((int64_t)remaining * 100 / req->content_len);
        if (pct >= last_logged_pct + 10) {
            ESP_LOGI(TAG, "OTA progress: %d%%", pct);
            last_logged_pct = pct;
        }
    }
    free(buf);

    err = esp_ota_end(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end: %s", esp_err_to_name(err));
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "image validation failed");
    }
    err = esp_ota_set_boot_partition(target);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition: %s", esp_err_to_name(err));
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "set boot partition failed");
    }

    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "ok", true);
    cJSON_AddStringToObject(root, "next", target->label);
    esp_err_t send_err = send_json(req, root);

    ESP_LOGW(TAG, "OTA complete, rebooting...");
    xTaskCreate(ota_reboot_task, "ota_reboot", 2048, NULL, 1, NULL);
    return send_err;
}

static esp_err_t start_httpd(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_uri_handlers = 12;
    cfg.lru_purge_enable = true;
    // Phones fire 5–10 parallel captive-portal probes the moment they join.
    // Cap sockets so we don't spike memory and so old sockets get recycled
    // instead of piling up while we're handling a slow BLE op.
    cfg.max_open_sockets = 4;
    cfg.backlog_conn     = 2;
    // 30 s tolerates slow 11b uploads when the firmware blob (~1.4 MB) is
    // streamed in over the OTA endpoint. Small JSON handlers finish well
    // under this anyway, and lru_purge_enable recycles any stale sockets.
    cfg.recv_wait_timeout = 30;
    cfg.send_wait_timeout = 30;
    esp_err_t err = httpd_start(&server, &cfg);
    if (err != ESP_OK) return err;

    httpd_uri_t routes[] = {
        { .uri = "/",                  .method = HTTP_GET,  .handler = h_index },
        { .uri = "/favicon.ico",       .method = HTTP_GET,  .handler = h_favicon },
        { .uri = "/api/status",        .method = HTTP_GET,  .handler = h_status },
        { .uri = "/api/paired",        .method = HTTP_GET,  .handler = h_paired },
        { .uri = "/api/scan",          .method = HTTP_POST, .handler = h_scan },
        { .uri = "/api/scan_results",  .method = HTTP_GET,  .handler = h_scan_results },
        { .uri = "/api/pair",          .method = HTTP_POST, .handler = h_pair },
        { .uri = "/api/unpair",        .method = HTTP_POST, .handler = h_unpair },
        { .uri = "/api/reset_odo",     .method = HTTP_POST, .handler = h_reset_odo },
        { .uri = "/api/ota",           .method = HTTP_POST, .handler = h_ota },
    };
    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); i++) {
        httpd_register_uri_handler(server, &routes[i]);
    }
    return ESP_OK;
}

// ---- Public API ----

esp_err_t wifi_ap_init(void)
{
    if (netif_event_inited) return ESP_OK;
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ap_netif = esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    netif_event_inited = true;
    return ESP_OK;
}

esp_err_t wifi_ap_start(void)
{
    if (running) return ESP_OK;
    ESP_ERROR_CHECK(wifi_ap_init());

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);

    wifi_config_t apc = { 0 };
    apc.ap.channel       = WIFI_AP_CHANNEL;
    apc.ap.max_connection = WIFI_AP_MAX_CONN;
    apc.ap.authmode      = WIFI_AUTH_WPA2_PSK;
    snprintf((char *)apc.ap.ssid, sizeof(apc.ap.ssid),
             "GB-RX-%02X%02X%02X", mac[3], mac[4], mac[5]);
    apc.ap.ssid_len = strlen((char *)apc.ap.ssid);
    strlcpy((char *)apc.ap.password, WIFI_AP_PASSWORD, sizeof(apc.ap.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &apc));
    // 802.11b only: lowest peak TX current, least sensitive to BLE interference.
    // Skip 11g/n which spike current during OFDM bursts.
    esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11B);
    ESP_ERROR_CHECK(esp_wifi_start());
    // 34 = ~8.5 dBm. Default ~20 dBm causes brown-outs on bus-powered C3.
    esp_wifi_set_max_tx_power(34);
    esp_wifi_set_ps(WIFI_PS_NONE);

    esp_err_t err = start_httpd();
    if (err != ESP_OK) {
        esp_wifi_stop();
        return err;
    }
    running = true;
    // Pause BLE advertising — the AP owns the radio while it's up.
    ble_refresh_advertising();
    ESP_LOGI(TAG, "SoftAP started: SSID=%s pass=%s", apc.ap.ssid, WIFI_AP_PASSWORD);
    return ESP_OK;
}

esp_err_t wifi_ap_stop(void)
{
    if (!running) return ESP_OK;
    if (server) { httpd_stop(server); server = NULL; }
    esp_wifi_stop();
    running = false;
    ESP_LOGI(TAG, "SoftAP stopped");
    ble_refresh_advertising();
    return ESP_OK;
}

bool wifi_ap_is_running(void)
{
    return running;
}

void wifi_ap_stop_after(uint32_t delay_ms)
{
    if (!running) return;
    if (stop_timer == NULL) {
        stop_timer = xTimerCreate("ap_stop", pdMS_TO_TICKS(delay_ms),
                                  pdFALSE, NULL, stop_timer_cb);
    } else {
        xTimerChangePeriod(stop_timer, pdMS_TO_TICKS(delay_ms), 0);
    }
    if (stop_timer != NULL) xTimerStart(stop_timer, 0);
}
