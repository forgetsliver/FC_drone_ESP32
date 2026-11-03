// ============================================================================
//  control_task.ino - Vòng lặp điều khiển chính (250Hz - 4ms)
//  Thực hiện: Timing micros -> Smooth Throttle Ramping -> Đọc IMU -> Ước lượng
//  góc
//             -> Bộ điều khiển PID -> Quad-X Mixer -> Xuất ESC
// ============================================================================

#include "types.h"

void control_loop_step() {
  unsigned long currentMicros = micros();
  if (currentMicros - lastIMUTime >= 4000) {
    float dt = (currentMicros - lastIMUTime) / 1000000.0f;
    lastIMUTime = currentMicros;

    // ---------- 1. Tự động tăng / giảm ga mượt mà (Smooth Throttle Ramping)
    // ----------
    if (flightMode == MODE_LAND) {
      // Khi đang hạ cánh tự động: Giảm ga siêu chậm (cứ 4 vòng PID = 16ms giảm
      // 1us ga)
      landCounter++;
      if (landCounter >= 4) {
        landCounter = 0;
        if (currentThrottle > targetThrottle) {
          currentThrottle--;
        }
      }

      // Khi đã tiếp đất an toàn sát mặt đất (ga chạm 1000us) -> Tắt hẳn động cơ
      if (currentThrottle <= 1000) {
        isArmed = false;
        flightMode = MODE_MANUAL;
        targetThrottle = 1000;
        currentThrottle = 1000;
        landCounter = 0;
        pid_reset_all();
        Serial.println(
            "\n[!] ĐÃ HẠ CÁNH THÀNH CÔNG AN TOÀN! TẮT ĐỘNG CƠ DỪNG HOÀN TOÀN.");
      }
    } else {
      landCounter = 0;
      // Chế độ Manual hoặc Hover: Tăng/giảm ga mượt mà tiệm cận targetThrottle
      // (mỗi vòng +/- 3us)
      if (currentThrottle < targetThrottle) {
        currentThrottle = min(currentThrottle + 3, targetThrottle);
      } else if (currentThrottle > targetThrottle) {
        currentThrottle = max(currentThrottle - 3, targetThrottle);
      }
    }

    // ---------- 2. Cảm biến và ước lượng góc nghiêng (Lọc bù chống trôi tĩnh & chống rung bay) ----------
    float ax, ay, az, gx, gy, gz;
    readIMURaw(ax, ay, az, gx, gy, gz);

    float denominator = sqrt(ay * ay + az * az);
    if (denominator < 0.001f)
      denominator = 0.001f;

    // Trừ độ lệch mặt phẳng cơ khí (rollOffset, pitchOffset) để đưa góc về chuẩn 0.0 độ tuyệt đối
    float accRoll  = (atan2(ay, az) * RAD_TO_DEG) - rollOffset;
    float accPitch = (atan2(-ax, denominator) * RAD_TO_DEG) - pitchOffset;

    float gyroRollRate = gx - gyroRollBias;
    float gyroPitchRate = gy - gyroPitchBias;

    // Tích phân con quay hồi chuyển Gyro
    float gyroPredictRoll  = rollAngle  + gyroRollRate  * dt;
    float gyroPredictPitch = pitchAngle + gyroPitchRate * dt;

    // Trọng số bù thích ứng (Adaptive Complementary Filter):
    // 1) Khi chưa ARM hoặc để yên trên bàn (Ga = 0%): alpha = 0.04f (khoá chặt 100% vào trọng lực, tuyệt đối KHÔNG trôi góc)
    // 2) Khi ARM và đẩy ga (đang bay/thử cánh): tự động hạ alpha khi rung lắc để cánh quạt không làm nhảy góc
    float alpha = 0.04f;

    if (isArmed && currentThrottle > 1050) {
      float a_total = sqrt(ax * ax + ay * ay + az * az);
      float a_err = fabs(a_total - 1.0f);

      if (a_err < 0.15f) {
        alpha = 0.015f; // Bay êm: bù vừa phải
      } else if (a_err < 0.40f) {
        alpha = 0.003f; // Rung nhẹ: giảm bớt tác động của Accel
      } else {
        alpha = 0.0005f; // Rung mạnh (30-40% ga): giữ góc bằng Gyro lọc tần số cao
      }
    }

    rollAngle  = (1.0f - alpha) * gyroPredictRoll  + alpha * accRoll;
    pitchAngle = (1.0f - alpha) * gyroPredictPitch + alpha * accPitch;

    int esc1 = ESC_IDLE;
    int esc2 = ESC_IDLE;
    int esc3 = ESC_IDLE;
    int esc4 = ESC_IDLE;

    // ---------- 3. Bộ điều khiển PID và Quad-X Mixer ----------
    // Chỉ kích hoạt PID khi ĐÃ MỞ KHÓA (isArmed) VÀ GA VƯỢT MỨC KHỞI ĐỘNG (> 1050us)
    if (isArmed && currentThrottle > 1050) {
      // Khi ga thấp còn nằm sát mặt đất (dưới 1120us): Xả dần tích phân để tránh dồn lệch ga động cơ
      if (currentThrottle < 1120) {
        pid_bleed_integrators();
      }

      float pid_roll = 0.0f, pid_pitch = 0.0f;
      pid_compute(targetRoll, targetPitch, rollAngle, pitchAngle, gyroRollRate,
                  gyroPitchRate, dt, pid_roll, pid_pitch);

      // Quad-X Motor Mixing CHUẨN ĐỊNH HƯỚNG
      // Front motors (+pitch): Tăng ga khi chúc mũi (pitch < 0 -> pid_pitch > 0) để nâng đầu lên
      // Rear motors  (-pitch): Giảm ga khi chúc mũi
      // Right motors (-roll) : Tăng ga khi nghiêng phải (roll > 0 -> pid_roll < 0) để nâng cánh phải
      // Left motors  (+roll) : Giảm ga khi nghiêng phải
      esc1 = currentThrottle - pid_roll - pid_pitch; // Front Right (chân 4)
      esc2 = currentThrottle + pid_roll - pid_pitch; // Rear Right  (chân 5)
      esc3 = currentThrottle - pid_roll + pid_pitch; // Rear Left   (chân 6)
      esc4 = currentThrottle + pid_roll + pid_pitch; // Front Left  (chân 7)

      // Chặn cận an toàn: Khi đã ARM và đẩy ga, không động cơ nào bị giảm dưới ngưỡng cầm chừng ESC_MIN_ARMED
      esc1 = constrain(esc1, ESC_MIN_ARMED, ESC_MAX);
      esc2 = constrain(esc2, ESC_MIN_ARMED, ESC_MAX);
      esc3 = constrain(esc3, ESC_MIN_ARMED, ESC_MAX);
      esc4 = constrain(esc4, ESC_MIN_ARMED, ESC_MAX);

      esc_write(esc1, esc2, esc3, esc4);
    } else {
      // Khi chưa Arm hoặc ga sát đáy: Reset toàn bộ tích phân và xuất xung tắt ga ESC_IDLE (1000us)
      pid_reset_all();
      esc1 = esc2 = esc3 = esc4 = ESC_IDLE;
      esc_write(ESC_IDLE, ESC_IDLE, ESC_IDLE, ESC_IDLE);
    }

    // ---------- 4. In thông số giám sát chi tiết từng động cơ lên Serial Monitor (mỗi 200ms) ----------
    if (millis() - lastLogTime >= 200) {
      lastLogTime = millis();
      int p1 = (esc1 - 1000) / 10;
      int p2 = (esc2 - 1000) / 10;
      int p3 = (esc3 - 1000) / 10;
      int p4 = (esc4 - 1000) / 10;
      int pThr = (currentThrottle - 1000) / 10;

      Serial.printf("[%s] Ga:%3d%% (%4dus) | M1(Truoc-Phai):%4dus (%2d%%) | M2(Sau-Phai):%4dus (%2d%%) | M3(Sau-Trai):%4dus (%2d%%) | M4(Truoc-Trai):%4dus (%2d%%) | Goc(R:%4.1f, P:%4.1f)\n",
                    isArmed ? "ARMED" : "DISARM",
                    pThr, currentThrottle,
                    esc1, p1,
                    esc2, p2,
                    esc3, p3,
                    esc4, p4,
                    rollAngle, pitchAngle);
    }
  }
}