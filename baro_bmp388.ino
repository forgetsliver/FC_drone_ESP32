// ============================================================================
//  BMP388 - Cảm biến áp suất khí quyển đo độ cao (Mô-đun mở rộng)
//  Mặc định tắt (#ifdef ENABLE_ALT_HOLD_BARO) để chạy đúng mã nguồn chuẩn ổn
//  định.
// ============================================================================

#ifdef ENABLE_ALT_HOLD_BARO

#include "Adafruit_BMP3XX.h"
#include <Adafruit_Sensor.h>
#include <SPI.h>

#define PIN_BARO_CS 36
#define PIN_BARO_MOSI 35
#define PIN_BARO_MISO 0
#define PIN_BARO_SCL 45

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp_fc;
SPIClass spi_baro(FSPI);

static float baro_alt_abs_m = 0.0f;
static float baro_alt_ref_m = 0.0f;
static bool baro_ready = false;

static void baro_measure_reference(uint16_t samples) {
  if (!baro_ready)
    return;
  double sum = 0.0;
  uint16_t got = 0;
  Serial.print("Dang lay mau cao do goc... ");

  for (uint8_t i = 0; i < 10; i++) {
    bmp_fc.performReading();
    delay(20);
  }

  for (uint16_t i = 0; i < samples; i++) {
    if (bmp_fc.performReading()) {
      sum += bmp_fc.readAltitude(SEALEVELPRESSURE_HPA);
      got++;
    }
    delay(20);
  }

  if (got == 0) {
    Serial.println("Loi: Khong lay duoc mau!");
    return;
  }

  baro_alt_ref_m = (float)(sum / got);
  Serial.print("Hoan tat. Cao do goc = ");
  Serial.print(baro_alt_ref_m, 3);
  Serial.println(" m");
}

bool baro_init() {
  baro_ready = false;
  spi_baro.begin(PIN_BARO_SCL, PIN_BARO_MISO, PIN_BARO_MOSI, -1);

  if (!bmp_fc.begin_SPI(PIN_BARO_CS, &spi_baro)) {
    Serial.println("BMP388 khong tim thay - kiem tra day SPI 2");
    return false;
  }

  bmp_fc.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp_fc.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp_fc.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp_fc.setOutputDataRate(BMP3_ODR_50_HZ);

  baro_ready = true;
  baro_measure_reference(50);
  return true;
}

bool baro_update() {
  if (!baro_ready)
    return false;
  if (bmp_fc.performReading()) {
    baro_alt_abs_m = bmp_fc.readAltitude(SEALEVELPRESSURE_HPA);
    return true;
  }
  return false;
}

float baro_get_altitude_m() { return baro_alt_abs_m - baro_alt_ref_m; }

bool baro_is_present() { return baro_ready; }

#endif // ENABLE_ALT_HOLD_BARO