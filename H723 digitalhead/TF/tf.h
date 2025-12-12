#ifndef TF_TREE_H_
#define TF_TREE_H_
#include "math.h"
#include "stdint.h"
typedef float Real;
#define TF_NODISCARD 

struct TF_Vec3 {
  Real x, y, z;
  TF_Vec3(Real x, Real y, Real z) : x(x), y(y), z(z) {}
		
	TF_Vec3 ypd2xyz() const{
		Real cp = cos(y);
		Real sp = sin(y);
		Real cy = cos(x);
		Real sy = sin(x);
		return TF_Vec3(z*cp*cy, z*cp*sy, z*sp);
	}
	TF_Vec3 radian2angle() const {
		#define TF_R2A 180.f / 3.14159265358979323846f
		return TF_Vec3(x*TF_R2A,y*TF_R2A,z*TF_R2A);
		#undef TF_R2A
	}
	TF_Vec3 operator*(Real m)const{
		return TF_Vec3(x*m,y*m,z*m);
	}
	TF_Vec3 operator+(TF_Vec3 o)const{
		return TF_Vec3(x+o.x,y+o.y,z+o.z);
	}
};//三维向量
struct TF_Mat4 {
  Real m[4][4];

  TF_Mat4(bool fill0 = false) 
	{
    if (fill0)
      for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
          m[i][j] = 0;
  }//根据构造函数传参初始化变换矩阵m

  /// @brief 矩阵乘法
  /// @brief a 左矩阵
  /// @brief b 右矩阵
  /// @brief o 输出矩阵
  static void mul(const TF_Mat4 &a, const TF_Mat4 &b, TF_Mat4 &o) 
	{
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j) 
			{
        o.m[i][j] = 0;
        for (int k = 0; k < 4; ++k)
          o.m[i][j] += a.m[i][k] * b.m[k][j];
      }
  }

  /// @brief 坐标变换
  TF_NODISCARD TF_Vec3 transformPoint(const TF_Vec3 &v) const {
    return TF_Vec3(m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z + m[0][3],
                   m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z + m[1][3],
                   m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z + m[2][3]);
  }
  /// @brief 速度变换
  TF_NODISCARD TF_Vec3 transformVelocity(const TF_Vec3 &v) const {
    return TF_Vec3(m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                   m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                   m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z);
  }
  /// @brief 齐次变换矩阵的逆
  /// @details 旋转矩阵R的逆: R是正交矩阵, 故R的逆为R的转置矩阵, 即 R^-1 = R^T
  /// @details 平移向量t的逆: t^-1 = -R^T * t
  TF_NODISCARD TF_Mat4 inv() const {
    TF_Mat4 o = *this;
    // 旋转矩阵的逆
    o.m[0][1] = m[1][0];
    o.m[0][2] = m[2][0];
    o.m[1][2] = m[2][1];
    o.m[1][0] = m[0][1];
    o.m[2][0] = m[0][2];
    o.m[2][1] = m[1][2];
    // 平移向量的逆
    o.m[0][3] = -(m[0][0] * m[0][3] + m[1][0] * m[1][3] + m[2][0] * m[2][3]);
    o.m[1][3] = -(m[0][1] * m[0][3] + m[1][1] * m[1][3] + m[2][1] * m[2][3]);
    o.m[2][3] = -(m[0][2] * m[0][3] + m[1][2] * m[1][3] + m[2][2] * m[2][3]);
    return o;
  }
};//坐标系类，其中包含对三维坐标的一系列方法
struct TF_LazyMat4 {
  Real x, y, z, Y, P, R;
  TF_Mat4 T, Tinv;
  bool mat_ok;

  TF_LazyMat4(Real x = 0, Real y = 0, Real z = 0, Real Y = 0, Real P = 0,
              Real R = 0)
      : x(x), y(y), z(z), Y(Y), P(P), R(R), mat_ok(false) {
    T.m[0][3] = x;
    T.m[1][3] = y;
    T.m[2][3] = z;

    T.m[3][0] = Tinv.m[3][0] = 0;
    T.m[3][1] = Tinv.m[3][1] = 0;
    T.m[3][2] = Tinv.m[3][2] = 0;
    T.m[3][3] = Tinv.m[3][3] = 1;
  }

  void updateR(Real yaw, Real pitch, Real roll);
  const TF_Mat4 &getMat();
  const TF_Mat4 &getInvMat();
	
private:
  void calc();
};
struct TF_Tree {
public:
  static const uint8_t TF_BASE_ODOM = 0; // dynamic车体到云台

  static const uint8_t TF_ODOM_CAMERA = 1; // fixed云台到相机
  static const uint8_t TF_ODOM_GUN = 2;    // fixed云台到枪管

  static const uint8_t TF_CAMERA_ODOM = 3; // fixed相机到云台
  static const uint8_t TF_CAMERA_BASE = 4; // combie相机到车体
  static const uint8_t TF_BASE_GUN = 5;    // combie车体到枪管

  TF_Tree(TF_LazyMat4 tf_bo, TF_LazyMat4 tf_oc, TF_LazyMat4 tf_og) {
    rel_dyn[0] = tf_bo;
    rel_fixed[0] = tf_oc.getMat();
    rel_fixed[1] = tf_og.getMat();
    rel_fixed[2] = tf_oc.getInvMat();
  }
	Real check(Real a)
	{
		if(a == 0)
		{
			return rel_dyn[0].Y;
		}
		else if(a == 1)
		{
			return rel_dyn[0].P;
		}
		else if(a == 2)
		{
			return rel_dyn[0].R;
		}
		return 0;
	}

private:
  TF_LazyMat4 rel_dyn[1];
  TF_Mat4 rel_fixed[2 + 1];
  TF_Mat4 rel_combie[2];
  bool rel_combie_ok[2];

  const TF_Mat4 &getMat(uint8_t type);

public:
  TF_Vec3 transformPoint(TF_Vec3 p, uint8_t type) {
    return getMat(type).transformPoint(p);
  }
  TF_Vec3 transformVelocity(TF_Vec3 v, uint8_t type) {
    return getMat(type).transformVelocity(v);
  }
	void updateIMU(Real Y,Real P,Real R){
		rel_dyn[0].updateR(Y,P,R);
		rel_combie_ok[0]=0;
		rel_combie_ok[1]=0;
	}
	
	static TF_Tree makeSentry(){
		/*TODO*/
		return TF_Tree(
			TF_LazyMat4(0,0,0,0,0,0),
			TF_LazyMat4(0,0,0,0,0,0),
			TF_LazyMat4(0,0,0,0,0,0)
			);
	}
};
#endif
