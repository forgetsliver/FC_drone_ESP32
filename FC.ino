// ============================================================================
//  KyThuatUAV FC - firmware bay ESP32
//  Điều khiển qua ứng dụng di động (Wi-Fi SoftAP + UDP)
//  Tương thích hoàn toàn với giao thức điều khiển Drone từ App
// ============================================================================

#include "types.h"
#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.295779513082320876798154814105
#endif

// ================= TÙY CHỌN TÍNH NĂNG =================
#define ENABLE_SIM_4G false // Đặt false: Tắt module SIM 4G, chỉ dùng Wi-Fi SoftAP để tối ưu tốc độ và giải phóng UART

// ================= CẤU HÌNH WI-FI & UDP =================
const char *ssid = "ESP32_DRONE_AP";       // Tên Wi-Fi phát từ ESP32
const char *password = "dronepassword123"; // Mật khẩu Wi-Fi

WiFiUDP udp;
const int localPort = 1234; // Port ESP32 lắng nghe lệnh
const int appPort = 8888;   // Port App lắng nghe Telemetry
IPAddress appIp;
bool hasAppIp = false;

// ================= PINOUT SPI 1: ICM-20948 =================
#define ICM_SCL 37
#define ICM_MISO 40
#define ICM_MOSI 38
#define ICM_CS 39

#if defined(ESP32)
SPIClass icmSPI(HSPI);
#else
SPIClass icmSPI;
#endif

// ================= CẤU HÌNH ESC FAST PWM (250Hz) =================
#define ESC1_PIN 4 // Front Right (Động cơ Trước - Phải)
#define ESC2_PIN 5 // Rear Right  (Động cơ Sau - Phải)
#define ESC3_PIN 6 // Rear Left   (Động cơ Sau - Trái)
#define ESC4_PIN 7 // Front Left  (Động cơ Trước - Trái)

#define PWM_FREQ 250       // 250Hz chuẩn (Chu kỳ 4000us) tương thích 100% mọi ESC
#define PWM_RES 14        // 14-bit (0 - 16383) cho độ phân giải siêu mịn
#define ESC_IDLE 1000      // 1000us: Mức ngắt động cơ hoàn toàn khi Disarm
#define ESC_MIN_ARMED 1060 // 1060us: Mức quay cầm chừng (Idle spin) khi Armed
#define ESC_MAX 2000       // 2000us: Mức ga tối đa (100% ga)
#define MIN_PULSE_US 1000
#define MAX_PULSE_US 2000

// ================= BIẾN TRẠNG THÁI VÀ ĐIỀU KHIỂN =================
FlightMode flightMode = MODE_MANUAL;

int currentThrottle = 1000;
int targetThrottle = 1000;
int landCounter = 0; // Bộ đếm nhịp giúp hạ ga khi hạ cánh siêu êm ái

float targetRoll =
    0.0f; // Góc Roll mục tiêu từ App (-20° đến +20°) -> Trái/Phải
float targetPitch =
    0.0f; // Góc Pitch mục tiêu từ App (-20° đến +20°) -> Tiến/Lùi

bool isArmed = false;

// ================= BIẾN ĐO GÓC & THỜI GIAN =================
float rollAngle = 0.0f, pitchAngle = 0.0f;
float gyroRollBias = 0.0f, gyroPitchBias = 0.0f;
float rollOffset = 0.0f, pitchOffset = 0.0f; // Độ lệch góc phẳng cơ khí (đưa về 0.0, 0.0)

unsigned long lastIMUTime = 0;
unsigned long lastLogTime = 0;
unsigned long lastTelemetryTime = 0;
unsigned long lastAppPacketTime =
    0; // Thời điểm nhận gói tin gần nhất từ Wi-Fi App
unsigned long last4GPacketTime = 0; // Thời điểm nhận gói tin gần nhất từ SIM 4G
bool has4GConnection = false;

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=========================================");
  Serial.println("  ESP32 DRONE FC - WI-FI UDP CONTROL");
  Serial.println("=========================================");

  // 1. Wi-Fi AP & UDP
  rc_wifi_init();

  // 2. SPI & Cảm biến ICM-20948
  pinMode(ICM_CS, OUTPUT);
  digitalWrite(ICM_CS, HIGH);
  icmSPI.begin(ICM_SCL, ICM_MISO, ICM_MOSI, ICM_CS);
  icmSPI.setFrequency(4000000);

  // 3. Khởi tạo PWM 4 ESC (250Hz Fast PWM)
  esc_init();

  // 4. Khởi tạo cảm biến ICM-20948
  if (!initICM20948_Direct()) {
    Serial.println("X LỖI: ICM-20948 không phản hồi!");
    while (1)
      delay(100);
  }

  // 5. Hiệu chuẩn Gyro
  imu_calibrate_gyro();

#if ENABLE_SIM_4G
  // 6. Khởi tạo Module SIM 4G A7680C (Chỉ chạy khi bật cờ ENABLE_SIM_4G)
  sim_4g_init();
#endif

  Serial.println("\n========================================================");
  Serial.println("  ✓ HE THONG DA SAN SANG! DANG KHOI DONG VONG LAP PID.");
#if ENABLE_SIM_4G
  Serial.println("  ✓ Che do: Ho tro ca Wi-Fi App va SIM 4G.");
#else
  Serial.println("  ✓ Che do: Dieu khien qua Wi-Fi SoftAP (ESP32_DRONE_AP).");
#endif
  Serial.println("========================================================\n");

  isArmed = false;
  flightMode = MODE_MANUAL;
  lastIMUTime = micros();
  lastLogTime = millis();
}

// ================= MAIN LOOP =================
void loop() {
  // 1a. Lắng nghe gói tin UDP từ Wi-Fi App
  handleUdpPackets();

#if ENABLE_SIM_4G
  // 1b. Lắng nghe gói tin từ Module SIM 4G A7680C
  sim_4g_update();
#endif

  // 3a. Định kỳ gửi Telemetry về App qua Wi-Fi (mỗi 100ms)
  if (millis() - lastTelemetryTime >= 100) {
    lastTelemetryTime = millis();
    sendTelemetry();
  }

#if ENABLE_SIM_4G
  // 3b. Định kỳ gửi Telemetry qua Module SIM 4G (mỗi 200ms)
  sim_4g_telemetry_step();
#endif

  // 4. Nút dừng khẩn cấp từ Serial Monitor
  checkSerialEmergencyStop();

  // 5. Vòng lặp cân bằng PID 250Hz (4ms)
  control_loop_step();
}