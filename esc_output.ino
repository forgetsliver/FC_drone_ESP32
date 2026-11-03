// ============================================================================
//  esc_output.ino - Xuất tín hiệu Fast PWM 250Hz (14-bit) cho 4 ESC qua LEDC
//  Tương thích hoàn toàn cho cả ESP32 Arduino Core 2.x và Core 3.x
//  Bao gồm hàm esc_calibrate() hiệu chuẩn đồng bộ hành trình ga cho cả 4 ESC
// ============================================================================

#include "types.h"
#include <Arduino.h>

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
#include <esp_arduino_version.h>
#endif

// ================= HÀM TƯƠNG THÍCH PWM LEDC CHO CẢ ESP32 CORE V2.X VÀ V3.X =================
void initESC(uint8_t pin, uint8_t channel) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, PWM_FREQ, PWM_RES);
#else
  ledcSetup(channel, PWM_FREQ, PWM_RES);
  ledcAttachPin(pin, channel);
#endif
}

void writeESCDuty(uint8_t pin, uint8_t channel, uint32_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(channel, duty);
#endif
}

// Chuyển đổi microseconds sang Duty Cycle 14-bit ở 250Hz (Chu kỳ 4000us)
// Duty = (us * 16383) / 4000
uint32_t usToDuty(uint32_t us) {
  return (uint32_t)(((uint64_t)us * ((1UL << PWM_RES) - 1UL)) / (1000000UL / PWM_FREQ));
}

void writeESC(uint8_t pin, uint8_t channel, int us) {
  us = constrain(us, ESC_IDLE, ESC_MAX);
  writeESCDuty(pin, channel, usToDuty((uint32_t)us));
}

// Xuất xung đồng loạt cho 4 ESC
void esc_write(int m1, int m2, int m3, int m4) {
  writeESC(ESC1_PIN, 0, m1);
  writeESC(ESC2_PIN, 1, m2);
  writeESC(ESC3_PIN, 2, m3);
  writeESC(ESC4_PIN, 3, m4);
}

// Khởi tạo và kích hoạt 4 kênh PWM cho ESC
void esc_init() {
  initESC(ESC1_PIN, 0);
  initESC(ESC2_PIN, 1);
  initESC(ESC3_PIN, 2);
  initESC(ESC4_PIN, 3);

  // Gửi xung 1000us khóa motor và đợi ESC nhận diện
  esc_write(ESC_IDLE, ESC_IDLE, ESC_IDLE, ESC_IDLE);
  delay(1500);
}

// ================= HÀM HIỆU CHUẨN ĐỒNG BỘ 4 ESC (ESC CALIBRATION) =================
void esc_calibrate() {
  Serial.println("\n========================================================");
  Serial.println("  [!] TIEN TRINH HIEU CHUAN DONG BO 4 ESC (CALIBRATION)");
  Serial.println("  CANH BAO: PHAI THAO HET CANH QUAT TRUOC KHI THUC HIEN!");
  Serial.println("========================================================");

  // Bước 1: Xuất xung Max 2000us
  Serial.println(">> Buoc 1: Dang xuat xung MAX (2000us) toi ca 4 ESC...");
  esc_write(ESC_MAX, ESC_MAX, ESC_MAX, ESC_MAX);
  Serial.println(">> HUONG DAN: CAM NGUON PIN LiPo vao ESC ngay bay gio!");
  Serial.println(">> ESC se phat am thanh khoi dong va 2 tieng 'bip bip' nhan MAX GA...");

  for (int i = 5; i > 0; i--) {
    Serial.printf(">> Dang giu Max ga... Chuan bi ha ga sau: %d giay...\n", i);
    delay(1000);
  }

  // Bước 2: Hạ xung về Min 1000us
  Serial.println(">> Buoc 2: Dang ha xung ve MIN (1000us)...");
  esc_write(ESC_IDLE, ESC_IDLE, ESC_IDLE, ESC_IDLE);
  Serial.println(">> ESC se phat tieng 'bip dai' xac nhan da luu nguong MIN GA!");
  delay(3000);

  Serial.println(">> [✓] HIEU CHUAN DONG BO 4 ESC HOAN TAT THANH CONG 100%!\n");
}