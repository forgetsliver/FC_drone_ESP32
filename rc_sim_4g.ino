// ============================================================================
//  rc_sim_4g.ino - Điều khiển Drone qua Module SIM 4G LTE A7680C (Giao thức MQTT)
//  Tương thích hoàn toàn với giao thức điều khiển Drone (JOY, CMD, TEL)
//
//  SƠ ĐỒ CHÂN KẾT NỐI:
//    - TX (A7680C)  -->  RX (ESP32)  [GPIO 44 trên ESP32-S3]
//    - RX (A7680C)  -->  TX (ESP32)  [GPIO 43 trên ESP32-S3]
//    - RST (A7680C) -->  Chân 1 (GPIO 1 trên ESP32-S3, nếu có)
//    - GND (A7680C) -->  GND (ESP32)
//    - VCC (A7680C) -->  Nguồn 3.8V - 4.2V (tối thiểu 2A)
// ============================================================================

#include "types.h"
#include <Arduino.h>

// ================= CẤU HÌNH CHÂN UART & RESET CHO ESP32-S3 =================
#define PIN_SIM_RX 44 // Chân RX ESP32 nối tới TX module SIM (GPIO 44)
#define PIN_SIM_TX 43 // Chân TX ESP32 nối tới RX module SIM (GPIO 43)
#define PIN_SIM_RST 1  // Chân RST trên board ESP32-S3 (GPIO 1, nếu có đấu dây)

#define SIM_BAUDRATE 115200 // Tốc độ baud mặc định của A7680C

// ================= CẤU HÌNH NHÀ MẠNG & MQTT CLOUD BROKER =================
// APN Nhà mạng tại VN (Viettel: "v-internet", VinaPhone: "m3-world", MobiFone: "m-wap")
#define SIM_APN "m3-world" // Đang để mặc định VinaPhone, đổi sang "v-internet" nếu dùng Viettel

// Dùng broker EMQX (Cụm máy chủ Singapore, ping từ VN chỉ ~35ms, siêu nhanh cho Drone)
#define MQTT_BROKER "broker.emqx.io"
#define MQTT_PORT 1883
#define MQTT_TOPIC_SUB  "ktyav/drone/control"    // App publish lệnh vào đây (JOY, CMD)
#define MQTT_TOPIC_PUB  "ktyav/drone/telemetry"  // Drone publish dữ liệu bay lên đây

// Khởi tạo cổng HardwareSerial 1 cho module 4G
HardwareSerial Serial4G(1);

// Biến trạng thái kênh truyền 4G
unsigned long last4GTelemetryTime = 0;
unsigned long last4GCsqTime = 0;
int sim4G_rssi = 0; // Cường độ sóng CSQ (0 - 31)
bool mqttConnected = false;
char mqttClientId[32];

static String simRxBuffer = "";

// ================= HÀM GỬI LỆNH VÀ IN PHẢN HỒI THỰC TẾ LÊN SERIAL =================
static String sim_exec(const char *cmd, unsigned long timeoutMs = 3000) {
  // Xóa sạch bộ đệm rác cũ trước khi gửi lệnh mới
  while (Serial4G.available() > 0) {
    Serial4G.read();
  }

  Serial4G.println(cmd);
  unsigned long start = millis();
  String resp = "";
  while (millis() - start < timeoutMs) {
    while (Serial4G.available() > 0) {
      char c = (char)Serial4G.read();
      resp += c;
    }
    // Nếu đã nhận được phản hồi kết thúc (OK hoặc ERROR hoặc mã trạng thái mạng)
    if (resp.indexOf("OK") != -1 || resp.indexOf("ERROR") != -1 ||
        resp.indexOf("+NETOPEN: 0") != -1 ||
        resp.indexOf("+CMQTTSTART:") != -1 ||
        resp.indexOf("+CMQTTCONNECT:") != -1 ||
        resp.indexOf("+CMQTTSUB:") != -1) {
      delay(80);
      while (Serial4G.available() > 0) {
        resp += (char)Serial4G.read();
      }
      break;
    }
    delay(10);
  }
  resp.trim();
  String singleLine = resp;
  singleLine.replace("\r", " ");
  singleLine.replace("\n", " ");
  while (singleLine.indexOf("  ") != -1) {
    singleLine.replace("  ", " ");
  }
  Serial.print("// -> SIM tra ve: [ ");
  Serial.print(singleLine);
  Serial.println(" ]");
  return resp;
}

static bool waitForPrompt(unsigned long timeoutMs = 2000) {
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    if (Serial4G.available()) {
      char c = (char)Serial4G.read();
      if (c == '>')
        return true;
    }
    delay(5);
  }
  return false;
}

// ================= KHỞI TẠO MODULE SIM 4G & KẾT NỐI MQTT =================
void sim_4g_init() {
  Serial.println("\n[4G] Dang khoi tao module SIM 4G A7680C (VinaPhone)...");

  // Tạo Client ID duy nhất theo địa chỉ MAC của chip ESP32 (tránh trùng lặp trên Broker)
  uint32_t chipId = (uint32_t)(ESP.getEfuseMac() >> 32);
  snprintf(mqttClientId, sizeof(mqttClientId), "ktyav_uav_%04X",
           (uint16_t)chipId);

  // 1. Reset cứng module A7680C qua chân RST (GPIO 1)
  if (PIN_SIM_RST >= 0) {
    pinMode(PIN_SIM_RST, OUTPUT);
    digitalWrite(PIN_SIM_RST, LOW);
    delay(300);
    digitalWrite(PIN_SIM_RST, HIGH);
    delay(1500);
  }

  // 2. Khởi tạo UART kết nối với A7680C
  Serial4G.begin(SIM_BAUDRATE, SERIAL_8N1, PIN_SIM_RX, PIN_SIM_TX);
  delay(300);

  // 3. Kiểm tra AT cơ bản & SIM & đo sóng
  sim_exec("AT", 1000);
  sim_exec("ATE0", 1000);
  sim_exec("AT+CMEE=2", 1000);
  sim_exec("AT+CPIN?", 1500);
  sim_exec("AT+CSQ", 1000);
  sim_exec("AT+CREG?", 1000);

  // 4. Dọn dẹp sạch phiên kết nối cũ nếu có
  sim_exec("AT+CMQTTDISC=0,60", 1000);
  sim_exec("AT+CMQTTREL=0", 1000);
  sim_exec("AT+CMQTTSTOP", 1000);
  sim_exec("AT+NETCLOSE", 1500);
  delay(500);

  // 5. Cấu hình Socket APN cho VinaPhone (Lệnh chuẩn A7680C là CGSOCKCONT)
  char sockApnCmd[64];
  snprintf(sockApnCmd, sizeof(sockApnCmd), "AT+CGSOCKCONT=1,\"IP\",\"%s\"",
           SIM_APN);
  sim_exec(sockApnCmd, 2000);
  delay(300);

  // 6. Mở kết nối mạng di động (NETOPEN)
  String netResp = sim_exec("AT+NETOPEN", 6000);
  // Nếu chưa thấy +NETOPEN: 0, đợi thêm một nhịp ngắn
  if (netResp.indexOf("+NETOPEN: 0") == -1 &&
      netResp.indexOf("already opened") == -1) {
    delay(1000);
    sim_exec("AT+IPADDR", 2000);
  } else {
    sim_exec("AT+IPADDR", 2000);
  }
  delay(300);

  // 7. Khởi động dịch vụ MQTT
  String startResp = sim_exec("AT+CMQTTSTART", 3000);
  delay(300);

  // 8. Cấp phát Client MQTT
  char accqCmd[80];
  snprintf(accqCmd, sizeof(accqCmd), "AT+CMQTTACCQ=0,\"%s\",0", mqttClientId);
  sim_exec(accqCmd, 2000);
  delay(300);

  // 9. Kết nối tới Cloud MQTT Broker (broker.emqx.io:1883)
  char connCmd[128];
  snprintf(connCmd, sizeof(connCmd), "AT+CMQTTCONNECT=0,\"tcp://%s:%d\",60,1",
           MQTT_BROKER, MQTT_PORT);
  String connResp = sim_exec(connCmd, 9000);

  if (connResp.indexOf("+CMQTTCONNECT: 0,0") != -1 ||
      connResp.indexOf("OK") != -1) {
    mqttConnected = true;
    Serial.println("// >>> [4G] KET NOI CLOUD MQTT BROKER THANH CONG! <<<");
  } else {
    mqttConnected = false;
    Serial.println(
        "// [!] [4G] CHUA THE KET NOI MQTT (Xem ma loi tra ve o tren)!");
  }

  // 10. Đăng ký nhận lệnh từ App (Subscribe topic: ktyav/drone/control)
  if (mqttConnected) {
    char subTopCmd[48];
    snprintf(subTopCmd, sizeof(subTopCmd), "AT+CMQTTSUBTOPIC=0,%d,1",
             (int)strlen(MQTT_TOPIC_SUB));
    Serial4G.println(subTopCmd);
    if (waitForPrompt(1500)) {
      Serial4G.print(MQTT_TOPIC_SUB);
      delay(150);
      String subResp = sim_exec("AT+CMQTTSUB=0", 3000);
      if (subResp.indexOf("+CMQTTSUB: 0,0") != -1 ||
          subResp.indexOf("OK") != -1) {
        Serial.println("// [4G] DA SUBSCRIBE KENH LENH: ktyav/drone/control");
      }
    }
  }

  simRxBuffer.reserve(256);
}

// ================= XỬ LÝ DÒNG LỆNH NHẬN ĐƯỢC TỪ 4G / MQTT =================
void sim_4g_process_command(String msg) {
  msg.trim();
  if (msg.length() == 0)
    return;

  // 1. Xử lý gói tin Joystick: "JOY:leftX,leftY,rightX,rightY"
  int joyIndex = msg.indexOf("JOY:");
  if (joyIndex != -1) {
    String joyData = msg.substring(joyIndex + 4);
    int firstComma = joyData.indexOf(',');
    int secondComma = joyData.indexOf(',', firstComma + 1);
    int thirdComma = joyData.indexOf(',', secondComma + 1);

    if (firstComma != -1 && secondComma != -1 && thirdComma != -1) {
      float leftX = joyData.substring(0, firstComma).toFloat();
      float leftY = joyData.substring(firstComma + 1, secondComma).toFloat();
      float rightX = joyData.substring(secondComma + 1, thirdComma).toFloat();
      float rightY = joyData.substring(thirdComma + 1).toFloat();

      has4GConnection = true;
      last4GPacketTime = millis(); // Cập nhật thời điểm nhận gói 4G gần nhất

      // Tự động mở khóa động cơ (ARMED) khi có tín hiệu cần điều khiển từ 4G
      if (!isArmed) {
        isArmed = true;
        Serial.println("\n[4G] TỰ ĐỘNG MỞ KHÓA ĐỘNG CƠ (ARMED) QUA KÊNH 4G!");
      }

      // Tự động thoát khỏi HOVER hoặc LAND khi người điều khiển can thiệp
      if (flightMode != MODE_MANUAL) {
        flightMode = MODE_MANUAL;
        Serial.println(
            "\n[4G] THOÁT HOVER/LAND -> CHUYỂN VỀ THỦ CÔNG (MANUAL) QUA 4G!");
      }

      // Quy đổi leftY (-1.0 -> 1.0) thành ga mục tiêu (1000us -> 2000us)
      float throttleVal = 1000.0f + ((leftY + 1.0f) / 2.0f) * 1000.0f;
      targetThrottle = constrain((int)throttleVal, 1000, 2000);

      // Quy đổi rightX, rightY thành góc nghiêng Roll / Pitch (-20° đến +20°)
      targetRoll = rightX * 20.0f;
      targetPitch = rightY * 20.0f;
    }
    return;
  }

  // 2. Xử lý gói tin Lệnh: "CMD:..."
  int cmdIndex = msg.indexOf("CMD:");
  if (cmdIndex != -1) {
    String cmd = msg.substring(cmdIndex + 4);
    cmd.trim();
    has4GConnection = true;
    last4GPacketTime = millis();

    if (cmd == "ARM") {
      isArmed = true;
      flightMode = MODE_MANUAL;
      Serial.println("\n[4G] LỆNH TỪ 4G: MỞ KHÓA ĐỘNG CƠ (ARMED)");
    } else if (cmd == "DISARM") {
      isArmed = false;
      flightMode = MODE_MANUAL;
      targetThrottle = 1000;
      currentThrottle = 1000;
      pid_reset_all();
      Serial.println("\n[4G] LỆNH TỪ 4G: TẮT ĐỘNG CƠ (DISARMED)");
    } else if (cmd == "HOVER") {
      if (isArmed) {
        flightMode = MODE_HOVER;
        targetThrottle = 1500; // Ga 50% giữ lơ lửng
        targetRoll = 0.0f;
        targetPitch = 0.0f;
        Serial.println("\n[4G] LỆNH TỪ 4G: GIỮ ĐỘ CAO LƠ LỬNG (HOVER)");
      }
    } else if (cmd == "LAND") {
      if (isArmed) {
        flightMode = MODE_LAND;
        targetThrottle = 1000;
        targetRoll = 0.0f;
        targetPitch = 0.0f;
        landCounter = 0;
        Serial.println(
            "\n[4G] LỆNH TỪ 4G: BẮT ĐẦU HẠ CÁNH TỰ ĐỘNG (AUTO LAND)");
      }
    } else if (cmd == "CALIB") {
      if (!isArmed) {
        esc_calibrate();
      } else {
        Serial.println("\n[4G] Khong the Calibrate ESC khi dang ARMED!");
      }
    }
    return;
  }

  // 3. Phân tích phản hồi sóng +CSQ từ A7680C: "+CSQ: <rssi>,<ber>"
  int csqIndex = msg.indexOf("+CSQ:");
  if (csqIndex != -1) {
    int comma = msg.indexOf(',', csqIndex);
    if (comma != -1) {
      sim4G_rssi = msg.substring(csqIndex + 5, comma).toInt();
      Serial.printf("[4G] CAP NHAT SONG CSQ: %d/31\n", sim4G_rssi);
    }
  }
}

// ================= ĐỌC VÀ XỬ LÝ UART KHÔNG CHẶN (NON-BLOCKING) =================
void sim_4g_update() {
  while (Serial4G.available() > 0) {
    char c = (char)Serial4G.read();
    if (c == '\n' || c == '\r') {
      if (simRxBuffer.length() > 0) {
        sim_4g_process_command(simRxBuffer);
        simRxBuffer = "";
      }
    } else {
      if (simRxBuffer.length() < 250) {
        simRxBuffer += c;
      } else {
        simRxBuffer = ""; // Tránh tràn bộ đệm
      }
    }
  }

  // Định kỳ mỗi 15 giây hỏi cường độ sóng 4G một lần
  if (millis() - last4GCsqTime >= 15000) {
    last4GCsqTime = millis();
    Serial4G.println("AT+CSQ");
  }
}

// ================= GỬI DỮ LIỆU TELEMETRY LÊN CLOUD MQTT =================
void sim_4g_send_telemetry() {
  if (!mqttConnected)
    return;

  char telBuffer[128];
  // Định dạng Telemetry: TEL:pitch,roll,csq_rssi,0,0,0,0,11.8
  snprintf(telBuffer, sizeof(telBuffer),
           "TEL:%.1f,%.1f,%d,%.1f,%.1f,%.1f,%.1f,%.2f", pitchAngle, rollAngle,
           sim4G_rssi, 0.0f, 0.0f, 0.0f, 0.0f, 11.8f);

  int topLen = strlen(MQTT_TOPIC_PUB);
  int payLen = strlen(telBuffer);

  // Gửi lệnh Publish tới topic ktyav/drone/telemetry (non-blocking nhanh)
  Serial4G.printf("AT+CMQTTTOPIC=0,%d\r\n", topLen);
  if (waitForPrompt(15)) {
    Serial4G.print(MQTT_TOPIC_PUB);
    Serial4G.printf("\r\nAT+CMQTTPAYLOAD=0,%d\r\n", payLen);
    if (waitForPrompt(15)) {
      Serial4G.print(telBuffer);
      Serial4G.println("\r\nAT+CMQTTPUB=0,0,60");
    }
  }
}

// Hàm gọi định kỳ gửi Telemetry qua 4G (mỗi 500ms để tiết kiệm data & tối ưu ping)
void sim_4g_telemetry_step() {
  if (millis() - last4GTelemetryTime >= 500) {
    last4GTelemetryTime = millis();
    sim_4g_send_telemetry();
  }
}