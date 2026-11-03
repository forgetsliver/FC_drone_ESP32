// ============================================================================
//  altitude_kf.ino - Bộ lọc Kalman 2 trạng thái ước lượng độ cao & tốc độ leo
//  Mô-đun mở rộng: Mặc định tắt (#ifdef ENABLE_ALT_HOLD_BARO) để chạy đúng
//  chuẩn.
// ============================================================================

#ifdef ENABLE_ALT_HOLD_BARO

#include <BasicLinearAlgebra.h>
using namespace BLA;

#define KF_DT_CTRL 0.004f // 250Hz = 4ms
#define KF_SIGMA_ACC_CMS2 4.0f
#define KF_SIGMA_BARO_CM 30.0f

BLA::Matrix<2, 2> Fz;
BLA::Matrix<2, 1> Gz;
BLA::Matrix<2, 2> Pz;
BLA::Matrix<2, 2> Qz;
BLA::Matrix<2, 1> Sz;
BLA::Matrix<1, 2> Hz;
BLA::Matrix<2, 2> Iz;
BLA::Matrix<1, 1> Acczz;
BLA::Matrix<2, 1> Kz;
BLA::Matrix<1, 1> Rz;
BLA::Matrix<1, 1> Lz;
BLA::Matrix<1, 1> Mz;

static float kf_alt_cm = 0.0f;
static float kf_climb_cms = 0.0f;

void alt_kf_init() {
  Fz = {1, KF_DT_CTRL, 0, 1};
  Gz = {0.5f * KF_DT_CTRL * KF_DT_CTRL, KF_DT_CTRL};
  Hz = {1, 0};
  Iz = {1, 0, 0, 1};
  Qz = Gz * ~Gz * KF_SIGMA_ACC_CMS2 * KF_SIGMA_ACC_CMS2;
  Rz = {KF_SIGMA_BARO_CM * KF_SIGMA_BARO_CM};
  Pz = {0, 0, 0, 0};
  Sz = {0, 0};

  kf_alt_cm = 0.0f;
  kf_climb_cms = 0.0f;
}

void alt_kf_update(float alt_meas_m, float acc_earth_z_g) {
  Acczz = {acc_earth_z_g * 981.0f};

  Sz = Fz * Sz + Gz * Acczz;
  Pz = Fz * Pz * ~Fz + Qz;

  Lz = Hz * Pz * ~Hz + Rz;

  float Lz_scalar = Lz(0, 0);
  if (Lz_scalar < 1e-6f)
    return;
  Kz = Pz * ~Hz * (1.0f / Lz_scalar);

  Mz = {alt_meas_m * 100.0f};
  Sz = Sz + Kz * (Mz - Hz * Sz);
  Pz = (Iz - Kz * Hz) * Pz;

  kf_alt_cm = Sz(0, 0);
  kf_climb_cms = Sz(1, 0);
}

float alt_kf_get_altitude_cm() { return kf_alt_cm; }
float alt_kf_get_climb_cms() { return kf_climb_cms; }

#endif // ENABLE_ALT_HOLD_BARO
