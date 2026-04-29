#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "esp_psram.h"

static const char *TAG = "OV3660_STREAM";

// ── Wi-Fi Credentials ────────────────────────────────────────
#include "wifi_config.h"
// ── OV3660 Pin Map for ESP32-S3 DevKit N16R8 ─────────────────
#define CAM_PIN_PWDN    38
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK    15
#define CAM_PIN_SIOD     4
#define CAM_PIN_SIOC     5
#define CAM_PIN_D0      11
#define CAM_PIN_D1       9
#define CAM_PIN_D2       8
#define CAM_PIN_D3      10
#define CAM_PIN_D4      12
#define CAM_PIN_D5      18
#define CAM_PIN_D6      17
#define CAM_PIN_D7      16
#define CAM_PIN_VSYNC    6
#define CAM_PIN_HREF     7
#define CAM_PIN_PCLK    13

// ── Camera Init ───────────────────────────────────────────────
static esp_err_t camera_init(void)
{
    camera_config_t config = {
        .pin_pwdn     = CAM_PIN_PWDN,
        .pin_reset    = CAM_PIN_RESET,
        .pin_xclk     = CAM_PIN_XCLK,
        .pin_sccb_sda = CAM_PIN_SIOD,
        .pin_sccb_scl = CAM_PIN_SIOC,

        .pin_d7 = CAM_PIN_D7,  .pin_d6 = CAM_PIN_D6,
        .pin_d5 = CAM_PIN_D5,  .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3,  .pin_d2 = CAM_PIN_D2,
        .pin_d1 = CAM_PIN_D1,  .pin_d0 = CAM_PIN_D0,

        .pin_vsync = CAM_PIN_VSYNC,
        .pin_href  = CAM_PIN_HREF,
        .pin_pclk  = CAM_PIN_PCLK,

        // OV3660: 20MHz XCLK works well
        .xclk_freq_hz = 20000000,
        .ledc_timer   = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,

        // JPEG output for streaming
        .pixel_format = PIXFORMAT_JPEG,

        // OV3660 can do up to QXGA (2048x1536)
        // Start with SVGA for smooth streaming
        .frame_size   = FRAMESIZE_SVGA,   // 800x600

        // Quality: 10–15 is a good balance for OV3660
        .jpeg_quality = 12,

        // Use PSRAM for frame buffers (we have 8MB!)
        .fb_count     = 2,
        .fb_location  = CAMERA_FB_IN_PSRAM,
        .grab_mode    = CAMERA_GRAB_LATEST,  // always grab freshest frame
    };

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init FAILED (0x%x). Check wiring & pin map.", err);
        return err;
    }

    // OV3660 sensor tuning
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_brightness(s, 0);      // -2 to 2
        s->set_contrast(s, 0);        // -2 to 2
        s->set_saturation(s, 0);      // -2 to 2
        s->set_sharpness(s, 0);       // -2 to 2
        s->set_whitebal(s, 1);        // auto white balance ON
        s->set_awb_gain(s, 1);        // AWB gain ON
        s->set_wb_mode(s, 0);         // 0=auto
        s->set_exposure_ctrl(s, 1);   // auto exposure ON
        s->set_aec2(s, 1);            // AEC DSP ON
        s->set_gain_ctrl(s, 1);       // auto gain ON
        s->set_agc_gain(s, 0);        // gain ceiling
        s->set_gainceiling(s, (gainceiling_t)2);
        s->set_bpc(s, 1);             // black pixel correction
        s->set_wpc(s, 1);             // white pixel correction
        s->set_raw_gma(s, 1);         // gamma correction
        s->set_lenc(s, 1);            // lens correction
        s->set_hmirror(s, 0);
        s->set_vflip(s, 0);
        s->set_dcw(s, 1);
        ESP_LOGI(TAG, "OV3660 sensor tuning applied");
    }

    ESP_LOGI(TAG, "Camera ready — OV3660 on ESP32-S3 N16R8");
    return ESP_OK;
}

// ── MJPEG Streaming ───────────────────────────────────────────
#define BOUNDARY        "mjpeg_boundary"
#define CONTENT_TYPE    "multipart/x-mixed-replace;boundary=" BOUNDARY
#define FRAME_BOUNDARY  "\r\n--" BOUNDARY "\r\n"
#define FRAME_HEADER    "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n"

static esp_err_t stream_handler(httpd_req_t *req)
{
    camera_fb_t *fb  = NULL;
    esp_err_t    res = ESP_OK;
    char         hdr[64];

    httpd_resp_set_type(req, CONTENT_TYPE);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "X-Framerate", "30");

    ESP_LOGI(TAG, "Stream client connected");

    while (true) {
        fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGW(TAG, "Frame capture failed, retrying...");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        res = httpd_resp_send_chunk(req, FRAME_BOUNDARY, strlen(FRAME_BOUNDARY));
        if (res != ESP_OK) goto cleanup;

        size_t hlen = snprintf(hdr, sizeof(hdr), FRAME_HEADER, fb->len);
        res = httpd_resp_send_chunk(req, hdr, hlen);
        if (res != ESP_OK) goto cleanup;

        res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);

    cleanup:
        esp_camera_fb_return(fb);
        if (res != ESP_OK) {
            ESP_LOGI(TAG, "Stream client disconnected");
            break;
        }
    }
    return res;
}

// ── Snapshot Handler (/snap) ──────────────────────────────────
static esp_err_t snap_handler(httpd_req_t *req)
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition",
                       "inline; filename=snapshot.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    return res;
}

// ── Web UI (index page) ───────────────────────────────────────
static const char INDEX_HTML[] =
"<!DOCTYPE html><html><head>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>ESP32-S3 OV3660 Stream</title>"
"<style>"
"  body{margin:0;background:#111;color:#eee;font-family:sans-serif;text-align:center}"
"  h2{margin:12px 0;font-size:1rem;letter-spacing:2px;color:#0af}"
"  img{width:100%;max-width:800px;border:2px solid #0af;border-radius:4px}"
"  .btn{display:inline-block;margin:8px;padding:8px 20px;"
"       background:#0af;color:#000;border:none;border-radius:4px;"
"       font-weight:bold;cursor:pointer;text-decoration:none}"
"  .btn:hover{background:#08d}"
"</style></head><body>"
"<h2>📷 ESP32-S3 · OV3660 · LIVE STREAM</h2>"
"<img src='/stream' id='stream'>"
"<br>"
"<a class='btn' href='/snap' target='_blank'>📸 Snapshot</a>"
"</body></html>";

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, INDEX_HTML, sizeof(INDEX_HTML) - 1);
}

// ── HTTP Server ───────────────────────────────────────────────
static void start_webserver(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port        = 80;
    cfg.stack_size         = 8192;
    cfg.max_uri_handlers   = 8;
    cfg.lru_purge_enable   = true;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "HTTP server failed to start");
        return;
    }

    httpd_uri_t uris[] = {
        { .uri="/",      .method=HTTP_GET, .handler=index_handler },
        { .uri="/stream",.method=HTTP_GET, .handler=stream_handler },
        { .uri="/snap",  .method=HTTP_GET, .handler=snap_handler },
    };
    for (int i = 0; i < 3; i++)
        httpd_register_uri_handler(server, &uris[i]);

    ESP_LOGI(TAG, "Web server started — 3 endpoints registered");
}

// ── Wi-Fi ─────────────────────────────────────────────────────
static void on_wifi_event(void *arg, esp_event_base_t base,
                          int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Wi-Fi disconnected, reconnecting...");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        ESP_LOGI(TAG, "  ✅  Connected to Wi-Fi!");
        ESP_LOGI(TAG, "  🌐  http://" IPSTR, IP2STR(&e->ip_info.ip));
        ESP_LOGI(TAG, "  📸  http://" IPSTR "/snap", IP2STR(&e->ip_info.ip));
        ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        start_webserver();
    }
}

static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi_event, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_wifi_event, NULL);

    wifi_config_t wifi_cfg = {
        .sta = {
            .ssid     = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Wi-Fi init done, connecting to: %s", WIFI_SSID);
}

// ── Entry Point ───────────────────────────────────────────────
void app_main(void)
{
    // NVS (required for Wi-Fi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // Log PSRAM info
    if (esp_psram_is_initialized()) {
        ESP_LOGI(TAG, "PSRAM: %u KB available", esp_psram_get_size() / 1024);
    } else {
        ESP_LOGW(TAG, "PSRAM not detected — check sdkconfig");
    }

    // Init camera then Wi-Fi
    if (camera_init() != ESP_OK) {
        ESP_LOGE(TAG, "Halting — fix camera wiring first");
        return;
    }
    wifi_init();
}
