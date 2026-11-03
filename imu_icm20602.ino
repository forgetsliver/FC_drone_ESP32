// ============================================================================
//  imu_icm20602.ino - Bare-metal Driver SPI cho ICM-20948
//  Giao tiếp qua SPI phần cứng (HSPI: SCK 37, MISO 40, MOSI 38, CS 39)
// ============================================================================

#include <SPI.h>
#include "types.h"

// ================= CÁC HÀM GIAO TIẾP SPI ICM-20948 =================
uint8_t readICM8(uint8_t reg) {
  digitalWrite(ICM_CS, LOW);
  icmSPI.transfer(reg | 0x80);
  uint8_t val = icmSPI.transfer(0x00);
  digitalWrite(ICM_CS, HIGH);
  return val;
}

void writeICM8(uint8_t reg, uint8_t val) {
  digitalWrite(ICM_CS, LOW);
  icmSPI.transfer(reg & 0x7F);
  icmSPI.transfer(val);
  digitalWrite(ICM_CS, HIGH);
}

void readICMBytes(uint8_t reg, uint8_t *buf, uint8_t len) {
  digitalWrite(ICM_CS, LOW);
  icmSPI.transfer(reg | 0x80);
  for (uint8_t i = 0; i < len; i++) {
    buf[i] = icmSPI.transfer(0x00);
  }
  digitalWrite(ICM_CS, HIGH);
}

// ================= KHỞI TẠO CẢM BIẾN ICM-20948 TRỰC TIẾP =================
bool initICM20948_Direct() {
  writeICM8(0x7F, 0x00);
  delay(10);
  if (readICM8(0x00) != 0xEA)
    return false;

  // 1. Reset chip hoàn toàn (cần đợi ít nhất 100ms để chip ổn định)
  writeICM8(0x06, 0x80);
  delay(100);

  // 2. Tắt Sleep mode, bật Auto Clock
  writeICM8(0x06, 0x01);
  delay(30);

  // 3. Vô hiệu hóa I2C Slave interface (cố định SPI mode để không bị nhiễu bus)
  writeICM8(0x03, 0x10);
  delay(10);

  // 4. Chuyển sang User Bank 2 để cấu hình Gyro và Accel
  writeICM8(0x7F, 0x20);
  delay(10);

  // GYRO_CONFIG_1 (0x01): ±500 dps (FS_SEL=01), DLPF 51.2 Hz (DLPFCFG=3, FCHOICE=1) -> 0x1B
  writeICM8(0x01, 0x1B);

  // ACCEL_CONFIG (0x14): ±8g (FS_SEL=10), DLPF 23.9 Hz (DLPFCFG=4, FCHOICE=1) -> 0x25
  writeICM8(0x14, 0x25);
  delay(10);

  // 5. Quay về User Bank 0 để đọc dữ liệu vận hành
  writeICM8(0x7F, 0x00);
  delay(10);
  return true;
}

// ================= ĐỌC DỮ LIỆU THÔ TỪ THANH GHI 0x2D =================
void readIMURaw(float &ax, float &ay, float &az, float &gx, float &gy,
                float &gz) {
  uint8_t raw[12];
  readICMBytes(0x2D, raw, 12);
  int16_t raw_ax = (raw[0] << 8) | raw[1];
  int16_t raw_ay = (raw[2] << 8) | raw[3];
  int16_t raw_az = (raw[4] << 8) | raw[5];
  int16_t raw_gx = (raw[6] << 8) | raw[7];
  int16_t raw_gy = (raw[8] << 8) | raw[9];
  int16_t raw_gz = (raw[10] << 8) | raw[11];

  // Thang đo ±8g: 4096 LSB/g
  ax = (float)raw_ax / 4096.0f;
  ay = (float)raw_ay / 4096.0f;
  az = (float)raw_az / 4096.0f;

  // Thang đo ±500 dps: 65.5 LSB/(deg/s)
  gx = (float)raw_gx / 65.5f;
  gy = (float)raw_gy / 65.5f;
  gz = (float)raw_gz / 65.5f;
}

// ================= HIỆU CHUẨN GYRO & MẶT PHẲNG CÂN BẰNG (LEVEL ZERO) =================
void imu_calibrate_gyro() {
  Serial.println("Bat dau hieu chinh Gyro & Mat phang can bang (500 mau - giu drone bat dong)...");
  float sumGx = 0, sumGy = 0;
  float sumAccRoll = 0, sumAccPitch = 0;
  float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;

  for (int i = 0; i < 500; i++) {
    readIMURaw(ax, ay, az, gx, gy, gz);
    sumGx += gx;
    sumGy += gy;

    float denom = sqrt(ay * ay + az * az);
    if (denom < 0.001f)
      denom = 0.001f;
    sumAccRoll  += atan2(ay, az) * RAD_TO_DEG;
    sumAccPitch += atan2(-ax, denom) * RAD_TO_DEG;
    delay(4);
  }

  // 1. Độ lệch trôi con quay hồi chuyển (Gyro Bias)
  gyroRollBias  = sumGx / 500.0f;
  gyroPitchBias = sumGy / 500.0f;

  // 2. Độ lệch cơ khí khi đặt trên mặt phẳng bàn (Level Offset)
  rollOffset  = sumAccRoll  / 500.0f;
  pitchOffset = sumAccPitch / 500.0f;

  // 3. Đưa góc Roll và Pitch về [0.0, 0.0] hoàn toàn
  rollAngle  = 0.0f;
  pitchAngle = 0.0f;

  Serial.printf("✓ Gyro Bias: Roll=%.4f dps | Pitch=%.4f dps\n", gyroRollBias, gyroPitchBias);
  Serial.printf("✓ Acc Level Offset: Roll=%.2f deg | Pitch=%.2f deg -> Da can bang ve [R: 0.0, P: 0.0]!\n",
                rollOffset, pitchOffset);
}