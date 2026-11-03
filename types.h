// ============================================================================
//  KyThuatUAV FC - firmware bay ESP32
//  types.h: Định nghĩa các kiểu dữ liệu dùng chung trong hệ thống
// ============================================================================
#ifndef KYTHUATUAV_TYPES_H
#define KYTHUATUAV_TYPES_H

#include <Arduino.h>

// Chế độ bay (Flight Modes)
enum FlightMode { MODE_MANUAL = 0, MODE_HOVER = 1, MODE_LAND = 2 };

// ================= NGUYÊN MẪU HÀM (FUNCTION PROTOTYPES) =================
// 1. Wi-Fi & UDP (rc_wifi_udp.ino)
void rc_wifi_init();
void handleUdpPackets();
void sendTelemetry();
void checkSerialEmergencyStop();

// 2. ESC Output (esc_output.ino)
void esc_init();
void esc_write(int m1, int m2, int m3, int m4);
void esc_calibrate();

// 3. Cảm biến ICM-20948 (imu_icm20602.ino)
bool initICM20948_Direct();
void readIMURaw(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);
void imu_calibrate_gyro();

// 4. PID Controller (pid.ino)
void pid_reset_all();
void pid_bleed_integrators();
void pid_compute(float target_r, float target_p, float current_r,
                 float current_p, float gyro_r_rate, float gyro_p_rate,
                 float dt, float &pid_r_out, float &pid_p_out);

// 5. Vòng lặp điều khiển chính (control_task.ino)
void control_loop_step();

// 6. Module SIM 4G LTE A7680C (rc_sim_4g.ino)
void sim_4g_init();
void sim_4g_update();
void sim_4g_telemetry_step();

// ================= BIẾN TOÀN CỤC CHIA SẺ (EXTERN) =================
extern FlightMode flightMode;
extern bool isArmed;
extern int currentThrottle;
extern int targetThrottle;
extern int landCounter;
extern float targetRoll;
extern float targetPitch;
extern float rollAngle;
extern float pitchAngle;
extern float gyroRollBias;
extern float gyroPitchBias;
extern float rollOffset;
extern float pitchOffset;
extern unsigned long lastIMUTime;
extern unsigned long lastLogTime;
extern unsigned long lastTelemetryTime;
extern unsigned long lastAppPacketTime;
extern unsigned long last4GPacketTime;
extern bool has4GConnection;

#endif // KYTHUATUAV_TYPES_H