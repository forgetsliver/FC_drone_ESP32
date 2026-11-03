// ============================================================================
//  rc_wifi_udp.ino - Giao tiếp Wi-Fi SoftAP & UDP điều khiển từ App di động
//  Bao gồm: Xử lý gói Joystick, gói Lệnh, Failsafe và gửi Telemetry
// ============================================================================

#include "types.h"
#include <WiFi.h>
#include <WiFiUdp.h>

// ================= KHỞI TẠO WI-FI VÀ UDP =================
void rc_wifi_init() {
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("Wi-Fi AP IP: ");
  Serial.println(IP); // 192.168.4.1

  udp.begin(localPort);
  Serial.printf("UDP Listening on port %d, App Telemetry port %d\n", localPort,
                appPort);
}

// ================= XỬ LÝ LỆNH TỪ APP QUA UDP =================
void handleUdpPackets() {
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    char packetBuffer[255];
    int len = udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = '\0';
    }

    appIp = udp.remoteIP();
    hasAppIp = true;
    lastAppPacketTime = millis(); // Cập nhật thời điểm nhận gói tin từ App

    String msg = String(packetBuffer);

    // 1. Gói Joystick: "JOY:leftX,leftY,rightX,rightY"
    if (msg.startsWith("JOY:")) {
      int firstComma = msg.indexOf(',');
      int secondComma = msg.indexOf(',', firstComma + 1);
      int thirdComma = msg.indexOf(',', secondComma + 1);

      if (firstComma != -1 && secondComma != -1 && thirdComma != -1) {
        float leftX = msg.substring(4, firstComma).toFloat();
        float leftY = msg.substring(firstComma + 1, secondComma).toFloat();
        float rightX = msg.substring(secondComma + 1, thirdComma).toFloat();
        float rightY = msg.substring(thirdComma + 1).toFloat();

        // Tự động mở khóa động cơ khi phi công thao tác cần ga / joystick
        if (!isArmed) {
          isArmed = true;
          Serial.println(
              "\n[WIFI] TỰ ĐỘNG MỞ KHÓA ĐỘNG CƠ (ARMED) QUA JOYSTICK!");
        }

        // Tự động thoát khỏi HOVER hoặc LAND -> Chuyển về điều khiển thủ công
        if (flightMode != MODE_MANUAL) {
          flightMode = MODE_MANUAL;
          Serial.println("\n[WIFI] TỰ ĐỘNG THOÁT HOVER/LAND -> CHUYỂN VỀ THỦ "
                         "CÔNG (MANUAL)");
        }

        // Chuyển leftY (-1.0 -> 1.0) thành ga mục tiêu (1000us -> 2000us)
        float throttleVal = 1000.0f + ((leftY + 1.0f) / 2.0f) * 1000.0f;
        targetThrottle = constrain((int)throttleVal, 1000, 2000);

        // Chuyển rightX, rightY thành góc nghiêng mục tiêu Roll / Pitch
        targetRoll = rightX * 20.0f;
        targetPitch = rightY * 20.0f;
      }
    }
    // 2. Gói Lệnh: "CMD:..."
    else if (msg.startsWith("CMD:")) {
      String cmd = msg.substring(4);
      if (cmd == "ARM") {
        isArmed = true;
        flightMode = MODE_MANUAL;
        Serial.println("\n[WIFI] LỆNH TỪ APP: MỞ KHÓA ĐỘNG CƠ (ARMED)");
      } else if (cmd == "DISARM") {
        isArmed = false;
        flightMode = MODE_MANUAL;
        targetThrottle = 1000;
        currentThrottle = 1000;
        pid_reset_all();
        Serial.println("\n[WIFI] LỆNH TỪ APP: TẮT ĐỘNG CƠ (DISARMED)");
      } else if (cmd == "HOVER") {
        if (isArmed) {
          flightMode = MODE_HOVER;
          targetThrottle = 1500; // Mức ga 50% lơ lửng
          targetRoll = 0.0f;
          targetPitch = 0.0f;
          Serial.println(
              "\n[WIFI] LỆNH TỪ APP: CHUYỂN SANG GIỮ ĐỘ CAO LƠ LỬNG (HOVER)");
        }
      } else if (cmd == "LAND") {
        if (isArmed) {
          flightMode = MODE_LAND;
          targetThrottle = 1000; // Mức ga hạ cánh dừng động cơ an toàn
          targetRoll = 0.0f;
          targetPitch = 0.0f;
          landCounter = 0;
          Serial.println(
              "\n[WIFI] LỆNH TỪ APP: BẮT ĐẦU HẠ CÁNH TỰ ĐỘNG (AUTO LAND)");
        }
      } else if (cmd == "CALIB") {
        if (!isArmed) {
          esc_calibrate();
        } else {
          Serial.println("\n[WIFI] Khong the Calibrate ESC khi dang ARMED!");
        }
      }
    }
  }
}


// ================= GỬI DỮ LIỆU TELEMETRY VỀ APP =================
void sendTelemetry() {
  if (!hasAppIp)
    return;

  char telBuffer[128];
  snprintf(telBuffer, sizeof(telBuffer),
           "TEL:%.1f,%.1f,%d,%.1f,%.1f,%.1f,%.1f,%.2f", pitchAngle, rollAngle,
           98, 0.0f, 0.0f, 0.0f, 0.0f, 11.8f);

  udp.beginPacket(appIp, appPort);
  udp.write((const uint8_t *)telBuffer, strlen(telBuffer));
  udp.endPacket();
}

// ================= PHÍM NGẮT KHẨN CẤP TỪ SERIAL MONITOR =================
void checkSerialEmergencyStop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == 'x' || c == 'X') {
      if (isArmed) {
        isArmed = false;
        flightMode = MODE_MANUAL;
        targetThrottle = 1000;
        currentThrottle = 1000;
        pid_reset_all();
        Serial.println("\n[!] KHẨN CẤP: ĐÃ NGẮT ĐỘNG CƠ TỪ SERIAL MONITOR!");
      }
    }
  }
}
