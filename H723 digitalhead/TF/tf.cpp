#include "tf.h"
void TF_LazyMat4::updateR(Real yaw, Real pitch, Real roll) {
    Y = yaw;
    P = pitch;
    R = roll;
    mat_ok = false;
  }
  const TF_Mat4 &TF_LazyMat4::getMat() {
    if (!mat_ok)//检查IMU是否更新
      calc();
    return T;
  }
  const TF_Mat4 &TF_LazyMat4::getInvMat() {
    if (!mat_ok)//检查IMU是否更新
      calc();
    return Tinv;
  }
void TF_LazyMat4::calc() {
    // 顺序: ZYX (yaw-pitch-roll)
    Real cy = cos(Y), sy = sin(Y);
    Real cp = cos(P), sp = sin(P);
    Real cr = cos(R), sr = sin(R);

    T.m[0][0] = Tinv.m[0][0] = cy * cp;
    T.m[0][1] = Tinv.m[1][0] = cy * sp * sr - sy * cr;
    T.m[0][2] = Tinv.m[2][0] = cy * sp * cr + sy * sr;

    T.m[1][0] = Tinv.m[0][1] = sy * cp;
    T.m[1][1] = Tinv.m[1][1] = sy * sp * sr + cy * cr;
    T.m[1][2] = Tinv.m[2][1] = sy * sp * cr - cy * sr;

    T.m[2][0] = Tinv.m[0][2] = -sp;
    T.m[2][1] = Tinv.m[1][2] = cp * sr;
    T.m[2][2] = Tinv.m[2][2] = cp * cr;

    Tinv.m[0][3] = -(T.m[0][0] * T.m[0][3] + T.m[1][0] * T.m[1][3] +
                     T.m[2][0] * T.m[2][3]);
    Tinv.m[1][3] = -(T.m[0][1] * T.m[0][3] + T.m[1][1] * T.m[1][3] +
                     T.m[2][1] * T.m[2][3]);
    Tinv.m[2][3] = -(T.m[0][2] * T.m[0][3] + T.m[1][2] * T.m[1][3] +
                     T.m[2][2] * T.m[2][3]);
		mat_ok=true;
  }

	
  const TF_Mat4 &TF_Tree::getMat(uint8_t type) {
    switch (type) {
    case 0:
      return rel_dyn[0].getMat();
    case 1:
      return rel_fixed[0];
    case 2:
      return rel_fixed[1];
    case 3:
      return rel_fixed[2];
    case 4:
      if (!rel_combie_ok[0]) {
        TF_Mat4::mul(rel_fixed[2], rel_dyn[0].getInvMat(), rel_combie[0]);
        rel_combie_ok[0] = true;
      }
      return rel_combie[0];
    case 5:
      if (!rel_combie_ok[1]) {
        TF_Mat4::mul(rel_dyn[0].getMat(), rel_fixed[1], rel_combie[1]);
        rel_combie_ok[1] = true;
      }
      return rel_combie[1];
    }
    static TF_Mat4 error;
    return error;
  }
