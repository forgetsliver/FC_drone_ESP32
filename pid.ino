// ============================================================================
//  pid.ino - Bộ điều khiển PID cân bằng góc Roll / Pitch
//  Kèm bộ lọc thông thấp (Low-Pass Filter) cho D-term dựa trên tốc độ con quay
//  (D-on-rate)
// ============================================================================

#include "types.h"

// ================= HỆ SỐ PID TINH CHỈNH =================
// ================= HỆ SỐ PID TINH CHỈNH =================
// Kp: Lực kéo góc về 0 (tăng từ 0.85 lên 2.5 để đủ lực chống nghiêng khi nhấc đất)
// Ki: Lực bù trọng tâm lệch
// Kd: Lực hãm chống dao động lắc (tăng từ 0.04 lên 0.35)
float Kp_roll = 2.5f, Ki_roll = 0.02f, Kd_roll = 0.35f;
float Kp_pitch = 2.5f, Ki_pitch = 0.02f, Kd_pitch = 0.35f;

float errorRoll = 0.0f, errorPitch = 0.0f;
float integralRoll = 0.0f, integralPitch = 0.0f;

// Bộ lọc Low-Pass Filter cho D-term
float dRoll_filtered = 0.0f, dPitch_filtered = 0.0f;
const float D_FILTER_ALPHA = 0.2f;

void pid_reset_all() {
  integralRoll = 0.0f;
  integralPitch = 0.0f;
  dRoll_filtered = 0.0f;
  dPitch_filtered = 0.0f;
  errorRoll = 0.0f;
  errorPitch = 0.0f;
}

// Xả dần tích phân để chống dồn lệch ga khi máy bay còn nằm trên mặt đất
void pid_bleed_integrators() {
  integralRoll *= 0.8f;
  integralPitch *= 0.8f;
}

void pid_compute(float target_r, float target_p, float current_r,
                 float current_p, float gyro_r_rate, float gyro_p_rate,
                 float dt, float &pid_r_out, float &pid_p_out) {
  errorRoll = target_r - current_r;
  errorPitch = target_p - current_p;

  // Tích phân giới hạn Anti-windup [-50, 50]
  integralRoll = constrain(integralRoll + errorRoll * dt, -50.0f, 50.0f);
  integralPitch = constrain(integralPitch + errorPitch * dt, -50.0f, 50.0f);

  // Vi phân D-term tính trên vận tốc góc con quay hồi chuyển
  float raw_dRoll = -gyro_r_rate;
  float raw_dPitch = -gyro_p_rate;

  dRoll_filtered += D_FILTER_ALPHA * (raw_dRoll - dRoll_filtered);
  dPitch_filtered += D_FILTER_ALPHA * (raw_dPitch - dPitch_filtered);

  pid_r_out = (Kp_roll * errorRoll) + (Ki_roll * integralRoll) +
              (Kd_roll * dRoll_filtered);
  pid_p_out = (Kp_pitch * errorPitch) + (Ki_pitch * integralPitch) +
              (Kd_pitch * dPitch_filtered);

  // Giới hạn đầu ra PID [-250, 250]
  pid_r_out = constrain(pid_r_out, -250.0f, 250.0f);
  pid_p_out = constrain(pid_p_out, -250.0f, 250.0f);
}