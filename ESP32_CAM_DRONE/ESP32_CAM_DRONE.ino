// ============================================================================
//  ESP32_CAM_DRONE.ino - Firmware Camera FPV truyền hình ảnh trực tiếp qua Wi-Fi
//  Dành cho board: AI-Thinker ESP32-CAM (Cảm biến OV2640)
//  2 Chế độ hoạt động thông minh:
//    1. Kết nối vào Wi-Fi Drone FC: "ESP32_DRONE_AP" (IP tĩnh: 192.168.4.50)
//    2. Tự phát Wi-Fi AP riêng nếu FC tắt: "DRONE_CAM_FPV" (IP: 192.168.4.1)
//  Kiến trúc 2 Server độc lập (Port 80 cho Web HUD, Port 81 cho Stream MJPEG)
// ============================================================================

#include "esp_camera.h"
#include <WiFi.h>
#include <ESPmDNS.h>

extern "C" {
  #include "esp_http_server.h"
}

// Khai báo trước nguyên mẫu hàm cho HTTP Server
static esp_err_t stream_handler(httpd_req_t *req);
static esp_err_t index_handler(httpd_req_t *req);
static esp_err_t flash_handler(httpd_req_t *req);
void startCameraServer();

// ================= CẤU HÌNH CHÂN CHO BOARD AI-THINKER ESP32-CAM =================
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// Đèn Flash LED trên board (GPIO 4)
#define LED_FLASH_PIN      4

// ================= THÔNG TIN WI-FI KHI KẾT NỐI VÀO FC =================
const char *fc_ssid     = "ESP32_DRONE_AP";       // Trùng tên với AP của FC
const char *fc_password = "dronepassword123";     // Trùng mật khẩu AP của FC

// Cấu hình IP tĩnh cho Camera trong mạng FC (192.168.4.50 để không xung đột với điện thoại)
IPAddress local_IP(192, 168, 4, 50);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

// Hai server độc lập (Chuẩn Espressif)
httpd_handle_t camera_httpd = NULL; // Port 80: Phục vụ Web HUD
httpd_handle_t stream_httpd = NULL; // Port 81: Chuyên luồng Video MJPEG

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

bool flashState = false;

// ================= GIAO DIỆN XEM TRỰC TIẾP TRÊN TRÌNH DUYỆT (HTML5 HUD) =================
static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
  <title>FPV Drone Camera</title>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body {
      background: #0d1117;
      color: #00ffcc;
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
      text-align: center;
      overflow: hidden;
      display: flex;
      flex-direction: column;
      height: 100vh;
    }
    .header {
      padding: 10px;
      background: rgba(13, 17, 23, 0.95);
      display: flex;
      justify-content: space-between;
      align-items: center;
      border-bottom: 1px solid #1f6feb;
    }
    .header h1 { font-size: 16px; letter-spacing: 2px; }
    .status { font-size: 13px; color: #39d353; font-weight: bold; }
    .stream-container {
      flex: 1;
      display: flex;
      justify-content: center;
      align-items: center;
      background: #000;
      position: relative;
    }
    #stream {
      max-width: 100%;
      max-height: 100%;
      object-fit: contain;
    }
    .hud-overlay {
      position: absolute;
      top: 15px;
      left: 15px;
      pointer-events: none;
      text-shadow: 1px 1px 2px #000;
      font-family: monospace;
      font-size: 14px;
      color: #00ffcc;
      text-align: left;
    }
    .crosshair {
      position: absolute;
      top: 50%;
      left: 50%;
      transform: translate(-50%, -50%);
      width: 44px;
      height: 44px;
      border: 1px dashed rgba(0, 255, 204, 0.6);
      border-radius: 50%;
      pointer-events: none;
    }
    .crosshair::before, .crosshair::after {
      content: '';
      position: absolute;
      background: rgba(0, 255, 204, 0.6);
    }
    .crosshair::before { top: 21px; left: -10px; width: 64px; height: 1px; }
    .crosshair::after  { top: -10px; left: 21px; width: 1px; height: 64px; }
    .controls {
      padding: 10px;
      background: rgba(13, 17, 23, 0.95);
      display: flex;
      justify-content: center;
      gap: 15px;
      border-top: 1px solid #21262d;
    }
    button {
      background: #21262d;
      color: #58a6ff;
      border: 1px solid #30363d;
      padding: 8px 18px;
      border-radius: 6px;
      font-size: 14px;
      cursor: pointer;
      font-weight: bold;
      transition: 0.2s;
    }
    button:active { background: #1f6feb; color: #fff; }
  </style>
</head>
<body>
  <div class="header">
    <h1>DRONE FPV LIVE</h1>
    <div class="status" id="fpsDisplay">● ĐANG TRUYỀN HÌNH</div>
  </div>
  <div class="stream-container">
    <img id="stream" src="" alt="Dang ket noi luong video...">
    <div class="hud-overlay">
      <div>IP: <span id="hudIp">...</span></div>
      <div>FPV MODE: VGA 640x480 LOW-LATENCY</div>
    </div>
    <div class="crosshair"></div>
  </div>
  <div class="controls">
    <button onclick="toggleFlash()">ĐÈN FLASH (BẬT/TẮT)</button>
    <button onclick="toggleFullscreen()">TOÀN MÀN HÌNH</button>
  </div>
  <script>
    window.addEventListener('DOMContentLoaded', () => {
      const host = window.location.hostname;
      document.getElementById('hudIp').innerText = host;
      // Nạp luồng video trực tiếp từ Server Port 81
      document.getElementById('stream').src = 'http://' + host + ':81/stream';
    });

    function toggleFlash() {
      fetch('/toggle_flash');
    }

    function toggleFullscreen() {
      if (!document.fullscreenElement) {
        document.documentElement.requestFullscreen().catch(() => {});
      } else {
        document.exitFullscreen().catch(() => {});
      }
    }
  </script>
</body>
</html>
)rawliteral";

// ================= HANDLER TRUYỀN HÌNH ẢNH STREAM (PORT 81) =================
static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      res = ESP_FAIL;
      break;
    }

    _jpg_buf_len = fb->len;
    _jpg_buf = fb->buf;

    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if (res == ESP_OK) {
      size_t hlen = snprintf(part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }

    esp_camera_fb_return(fb);
    fb = NULL;

    if (res != ESP_OK) {
      break;
    }
  }
  return res;
}

// ================= HANDLER TRANG CHỦ (PORT 80) =================
static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, strlen(INDEX_HTML));
}

// ================= HANDLER BẬT / TẮT ĐÈN FLASH (PORT 80) =================
static esp_err_t flash_handler(httpd_req_t *req) {
  flashState = !flashState;
  digitalWrite(LED_FLASH_PIN, flashState ? HIGH : LOW);
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, flashState ? "FLASH ON" : "FLASH OFF", HTTPD_RESP_USE_STRLEN);
}

// ================= KHỞI ĐỘNG CẢ 2 HTTP SERVER (PORT 80 & PORT 81) =================
void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.ctrl_port = 32768;

  httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = index_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t flash_uri = {
    .uri       = "/toggle_flash",
    .method    = HTTP_GET,
    .handler   = flash_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t stream_uri = {
    .uri       = "/stream",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };

  // 1. Khởi chạy Server Web HUD (Port 80)
  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &flash_uri);
    Serial.println("✓ [HTTP] Web HUD Server khoi dong tai Port 80 thanh cong!");
  }

  // 2. Khởi chạy Server Stream Video FPV (Port 81) - Task riêng không nghẽn Web
  config.server_port = 81;
  config.ctrl_port = 32769;
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    Serial.println("✓ [HTTP] Stream Video Server khoi dong tai Port 81 thanh cong!");
  }
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n========================================================");
  Serial.println("        ESP32-CAM DRONE FPV LIVE STREAM SERVER");
  Serial.println("========================================================");

  pinMode(LED_FLASH_PIN, OUTPUT);
  digitalWrite(LED_FLASH_PIN, LOW);

  // 1. Cấu hình phần cứng Camera OV2640 (Chuẩn Espressif cho AI-Thinker ESP32-CAM)
  camera_config_t config;
  memset(&config, 0, sizeof(camera_config_t)); // Xóa sạch bộ nhớ rác

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count     = 1;
  config.sccb_i2c_port = -1;
  config.jpeg_buffer_size = 0;

  // Cấu hình theo sự hiện diện của PSRAM
  if (psramFound()) {
    Serial.println("✓ Tim thay PSRAM (4MB). Khoi tao 2 Frame Buffer PSRAM...");
    config.frame_size   = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count     = 2;
    config.grab_mode    = CAMERA_GRAB_LATEST;
    config.fb_location  = CAMERA_FB_IN_PSRAM;
  } else {
    Serial.println("! Khong tim thay PSRAM. Khoi tao RAM noi SVGA...");
    config.frame_size   = FRAMESIZE_SVGA;
    config.fb_location  = CAMERA_FB_IN_DRAM;
  }

  // Khởi tạo camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("! Khoi tao mac dinh that bai (0x%x). Dang thu che do du phong VGA...\n", err);
    esp_camera_deinit();
    delay(100);
    config.frame_size   = FRAMESIZE_VGA;
    config.fb_count     = 1;
    config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location  = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
    err = esp_camera_init(&config);
  }

  if (err != ESP_OK) {
    Serial.printf("X LOI: Khoi tao Camera OV2640 that bai! Ma loi: 0x%x\n", err);
    Serial.println("-> HUONG DAN: Vui long kiem tra cap vang camera da cam chat vao khe chua!");
    while (true) { delay(1000); }
  }
  Serial.println("✓ Camera OV2640 khoi tao thanh cong!");

  // Đặt độ phân giải FPV chạy thực tế: VGA (640x480) mượt mà
  sensor_t *s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_framesize(s, FRAMESIZE_VGA);
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
  }

  // 2. Cấu hình mạng Wi-Fi
  WiFi.setSleep(false); // Vô hiệu hóa tiết kiệm điện để giảm tối đa độ trễ video
  
  Serial.printf("Dang tim va ket noi vao Wi-Fi Drone FC [%s]...\n", fc_ssid);
  WiFi.mode(WIFI_STA);
  // Gán IP tĩnh 192.168.4.50 (tránh xung đột với 192.168.4.2 của điện thoại)
  WiFi.config(local_IP, gateway, subnet);
  WiFi.begin(fc_ssid, fc_password);

  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 12) { // Chờ thử trong 6 giây
    delay(500);
    Serial.print(".");
    retry++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n========================================================");
    Serial.println("✓ KET NOI THANH CONG VAO WI-FI DRONE FC!");
    Serial.printf(">> DIA CHI TRUY CAP FPV: http://%s\n", WiFi.localIP().toString().c_str());
    Serial.println(">> HOAC TRUY CAP:        http://dronecam.local");
    Serial.println("========================================================");
  } else {
    // Nếu FC chưa bật (khi test bàn), tự động phát Wi-Fi AP riêng
    Serial.println("\n! Khong thay Wi-Fi Drone FC (hoac FC chua bat).");
    Serial.println("-> TU DONG CHUYEN SANG PHAT WI-FI AP DE BAN TEST DANG DUNG...");
    WiFi.disconnect();
    delay(200);
    WiFi.mode(WIFI_AP);
    WiFi.softAP("DRONE_CAM_FPV", "12345678");
    delay(500);
    Serial.println("========================================================");
    Serial.println("✓ DA PHAT WI-FI RIENG:  DRONE_CAM_FPV (Mat khau: 12345678)");
    Serial.printf(">> DIA CHI TRUY CAP FPV: http://%s\n", WiFi.softAPIP().toString().c_str());
    Serial.println(">> HOAC TRUY CAP:        http://dronecam.local");
    Serial.println("========================================================");
  }

  // Cấu hình mDNS để vào qua tên miền http://dronecam.local
  if (MDNS.begin("dronecam")) {
    MDNS.addService("http", "tcp", 80);
    Serial.println("✓ mDNS da san sang: http://dronecam.local");
  }

  // 3. Khởi chạy 2 Web Server FPV (Port 80 & 81)
  startCameraServer();
}

// ================= LOOP =================
void loop() {
  // Nếu đang ở chế độ kết nối FC, tự động kết nối lại nếu mất sóng
  if (WiFi.getMode() == WIFI_STA && WiFi.status() != WL_CONNECTED) {
    static unsigned long lastReconnect = 0;
    if (millis() - lastReconnect >= 4000) {
      lastReconnect = millis();
      Serial.println("[Wi-Fi] Mat song toi FC, dang thu ket noi lai...");
      WiFi.reconnect();
    }
  }
  delay(100);
}
