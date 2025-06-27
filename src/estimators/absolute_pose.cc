// Copyright (c) 2018, ETH Zurich and UNC Chapel Hill.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of ETH Zurich and UNC Chapel Hill nor the names of
//       its contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: Johannes L. Schoenberger (jsch-at-demuc-dot-de)

#include "estimators/absolute_pose.h"

#include "base/polynomial.h"
#include "estimators/utils.h"
#include "util/logging.h"
#include "estimators/generalized_absolute_pose.h"
namespace colmap {
namespace {

Eigen::Vector3d LiftImagePoint(const Eigen::Vector2d& point) {
  return point.homogeneous() / std::sqrt(point.squaredNorm() + 1);
}

}  // namespace

std::vector<P3PEstimator::M_t> P3PEstimator::Estimate(
    const std::vector<X_t>& points2D, const std::vector<Y_t>& points3D) {
  CHECK_EQ(points2D.size(), 3);
  CHECK_EQ(points3D.size(), 3);

  Eigen::Matrix3d points3D_world;
  points3D_world.col(0) = points3D[0];
  points3D_world.col(1) = points3D[1];
  points3D_world.col(2) = points3D[2];
 // 归一化像素坐标 ,这里输入的points2D是 ImageToWorld
  // ImageToWorld(准确的说是图像到相机坐标系)
    //   *u = (x - c1) / f1;*v = (y - c2) / f2;
  const Eigen::Vector3d u = LiftImagePoint(points2D[0]);
  const Eigen::Vector3d v = LiftImagePoint(points2D[1]);
  const Eigen::Vector3d w = LiftImagePoint(points2D[2]);
 // 计算角度
  //P=(0,0,0),因此向量Pu, Pv, Pw的坐标就是u, v, w三点坐标
  // a.b=||a||*||b||*cos<a,b>
  // Angles between 2D points.
  const double cos_uv = u.transpose() * v;
  const double cos_uw = u.transpose() * w;
  const double cos_vw = v.transpose() * w;

  // Distances between 2D points.
  //计算世界坐标系下的三角形三个边的长度
  const double dist_AB_2 = (points3D[0] - points3D[1]).squaredNorm();
  const double dist_AC_2 = (points3D[0] - points3D[2]).squaredNorm();
  const double dist_BC_2 = (points3D[1] - points3D[2]).squaredNorm();

  const double dist_AB = std::sqrt(dist_AB_2);

  const double a = dist_BC_2 / dist_AB_2;
  const double b = dist_AC_2 / dist_AB_2;

  // Helper variables for calculation of coefficients.
  const double a2 = a * a;
  const double b2 = b * b;
  const double p = 2 * cos_vw;
  const double q = 2 * cos_uw;
  const double r = 2 * cos_uv;
  const double p2 = p * p;
  const double p3 = p2 * p;
  const double q2 = q * q;
  const double r2 = r * r;
  const double r3 = r2 * r;
  const double r4 = r3 * r;
  const double r5 = r4 * r;

  // Build polynomial coefficients: a4*x^4 + a3*x^3 + a2*x^2 + a1*x + a0 = 0.
  // b0y-b1=0
  Eigen::Matrix<double, 5, 1> coeffs;
  coeffs(0) = -2 * b + b2 + a2 + 1 + a * b * (2 - r2) - 2 * a;
  coeffs(1) = -2 * q * a2 - r * p * b2 + 4 * q * a + (2 * q + p * r) * b +
              (r2 * q - 2 * q + r * p) * a * b - 2 * q;
  coeffs(2) = (2 + q2) * a2 + (p2 + r2 - 2) * b2 - (4 + 2 * q2) * a -
              (p * q * r + p2) * b - (p * q * r + r2) * a * b + q2 + 2;
  coeffs(3) = -2 * q * a2 - r * p * b2 + 4 * q * a +
              (p * r + q * p2 - 2 * q) * b + (r * p + 2 * q) * a * b - 2 * q;
  coeffs(4) = a2 + b2 - 2 * a + (2 - p2) * b - 2 * a * b + 1;

  Eigen::VectorXd roots_real;
  Eigen::VectorXd roots_imag;
  if (!FindPolynomialRootsCompanionMatrix(coeffs, &roots_real, &roots_imag)) {
    return std::vector<P3PEstimator::M_t>({});
  }

  std::vector<M_t> models;
  models.reserve(roots_real.size());

  for (Eigen::VectorXd::Index i = 0; i < roots_real.size(); ++i) {
    const double kMaxRootImag = 1e-10;
    if (std::abs(roots_imag(i)) > kMaxRootImag) {
      continue;
    }

    const double x = roots_real(i);
    if (x < 0) {
      continue;
    }

    const double x2 = x * x;
    const double x3 = x2 * x;

    // Build polynomial coefficients: b1*y + b0 = 0.
    //b0和b1 的系数
    const double bb1 =
        (p2 - p * q * r + r2) * a + (p2 - r2) * b - p2 + p * q * r - r2;
    const double b1 = b * bb1 * bb1;
    const double b0 =
        ((1 - a - b) * x2 + (a - 1) * q * x - a + b + 1) *
        (r3 * (a2 + b2 - 2 * a - 2 * b + (2 - r2) * a * b + 1) * x3 +
         r2 *
             (p + p * a2 - 2 * r * q * a * b + 2 * r * q * b - 2 * r * q -
              2 * p * a - 2 * p * b + p * r2 * b + 4 * r * q * a +
              q * r3 * a * b - 2 * r * q * a2 + 2 * p * a * b + p * b2 -
              r2 * p * b2) *
             x2 +
         (r5 * (b2 - a * b) - r4 * p * q * b +
          r3 * (q2 - 4 * a - 2 * q2 * a + q2 * a2 + 2 * a2 - 2 * b2 + 2) +
          r2 * (4 * p * q * a - 2 * p * q * a * b + 2 * p * q * b - 2 * p * q -
                2 * p * q * a2) +
          r * (p2 * b2 - 2 * p2 * b + 2 * p2 * a * b - 2 * p2 * a + p2 +
               p2 * a2)) *
             x +
         (2 * p * r2 - 2 * r3 * q + p3 - 2 * p2 * q * r + p * q2 * r2) * a2 +
         (p3 - 2 * p * r2) * b2 +
         (4 * q * r3 - 4 * p * r2 - 2 * p3 + 4 * p2 * q * r - 2 * p * q2 * r2) *
             a +
         (-2 * q * r3 + p * r4 + 2 * p2 * q * r - 2 * p3) * b +
         (2 * p3 + 2 * q * r3 - 2 * p2 * q * r) * a * b + p * q2 * r2 -
         2 * p2 * q * r + 2 * p * r2 + p3 - 2 * r3 * q);

    // Solve for y.
    const double y = b0 / b1;
    const double y2 = y * y;

    //x=PA/PC ;Y=PA/PC    AB^2/PC^2=(PA^2+PB^2)/PC^2-2xy*cos_uv ;x,y,cos_uv都是已知值,这里求解得到
    // AB/PC=?
    const double nu = x2 + y2 - 2 * x * y * cos_uv;

    const double dist_PC = dist_AB / std::sqrt(nu);
    const double dist_PB = y * dist_PC;
    const double dist_PA = x * dist_PC;
   //距离转换到相机坐标系下的坐标
    Eigen::Matrix3d points3D_camera;
    // u,v,w是单位向量,单位向量都是一样的
    // A/||PA|| =u
    points3D_camera.col(0) = u * dist_PA;  // A'
    points3D_camera.col(1) = v * dist_PB;  // B'
    points3D_camera.col(2) = w * dist_PC;  // C'

    // Find transformation from the world to the camera system.
    // 因为3d点的世界坐标知道,相机坐标下的坐标也知道,所以利用相似变换得到R,T
    // p3p的核心集就是求解A,B,C在相机坐标系下的坐标
    // umeyama 和icp的损失函数是一样的；
    const Eigen::Matrix4d transform =
        Eigen::umeyama(points3D_world, points3D_camera, false);
    models.push_back(transform.topLeftCorner<3, 4>());
  }

//  std::cout<<" pnp use p3p  gao "<< models.empty()<<"\n";
//  if (models.empty())
//  {
//      std::cout<<" warning p3p gao error!!!!!!!!!!!\n";
//  }

  return models;
}


void P3PEstimator::Residuals(const std::vector<X_t>& points2D,
                             const std::vector<Y_t>& points3D,
                             const M_t& proj_matrix,
                             std::vector<double>* residuals) {
  ComputeSquaredReprojectionError(points2D, points3D, proj_matrix, residuals);
}

// ECCV18 Nordberg by licheng
void  P3PNordberg::gauss_newton_refineL(Eigen::Vector3d &L,
                                        const double & a12, const double & a13, const double & a23,
                                        const double & b12, const double & b13, const double & b23)
{
    for (int i = 0; i < 5; ++i)
     {
       double l1 = L(0);
       double l2 = L(1);
       double l3 = L(2);
       double r1 = l1 * l1 + l2 * l2 + b12 * l1 * l2 - a12;
       double r2 = l1 * l1 + l3 * l3 + b13 * l1 * l3 - a13;
       double r3 = l2 * l2 + l3 * l3 + b23 * l2 * l3 - a23;

       if (std::abs(r1) + std::abs(r2) + std::abs(r3) < 1e-10)
         break;

       double dr1dl1 = 2.0 * l1 + b12 * l2;
       double dr1dl2 = 2.0 * l2 + b12 * l1;

       double dr2dl1 = 2.0 * l1 + b13 * l3;
       double dr2dl3 = 2.0 * l3 + b13 * l1;

       double dr3dl2 = 2.0 * l2 + b23 * l3;
       double dr3dl3 = 2.0 * l3 + b23 * l2;

       Eigen::Vector3d r(r1, r2, r3);
       {
         double v0 = dr1dl1;
         double v1 = dr1dl2;
         double v3 = dr2dl1;
         double v5 = dr2dl3;
         double v7 = dr3dl2;
         double v8 = dr3dl3;
         double det = 1.0 / (-v0 * v5 * v7 - v1 * v3 * v8);

         Eigen::Matrix3d Ji;
         Ji << -v5 * v7, -v1 * v8, v1 * v5,
               -v3 * v8, v0 * v8, -v0 * v5,
                v3 * v7, -v0 * v7, -v1 * v3;
         Eigen::Vector3d L1 = Eigen::Vector3d(L) - det * (Ji * r);
         {
           double l1 = L1(0);
           double l2 = L1(1);
           double l3 = L1(2);
           double r11 = l1 * l1 + l2 * l2 + b12 * l1 * l2 - a12;
           double r12 = l1 * l1 + l3 * l3 + b13 * l1 * l3 - a13;
           double r13 = l2 * l2 + l3 * l3 + b23 * l2 * l3 - a23;
           if (std::abs(r11) + std::abs(r12) + std::abs(r13) > std::abs(r1) + std::abs(r2) + std::abs(r3))
           {
             break;
           }
           else
             L = L1;
         }
       }
     }
}

inline bool P3PNordberg::root2real(const double & b, const double & c, double & r1, double & r2)
{
    double v = b * b -4.0 * c;
    if (v < 0.0) {
        r1 = r2 = 0.5 * b;
        return false;
    }
    double y = std::sqrt(v);
    if (b < 0.0) {
        r1 = 0.5 * (-b + y);
        r2 = 0.5 * (-b - y);
    } else {
        r1 = 2.0 * c / (-b + y);
        r2 = 2.0 * c / (-b - y);
    }
    return true;

}

double P3PNordberg::cubick(const double &b, const double &c, const double &d)
{

    double r0;

    if (b * b >= 3.0 * c)
    {

      double v = std::sqrt(b * b - 3.0 * c);
      double t1 = (-b - v) / (3.0);


      double k = ((t1 + b) * t1 + c) * t1 + d;

      if (k > 0.0)
      {

        r0 = t1 - std::sqrt(-k / (3.0 * t1 + b));

      }
      else
      {
        double t2 = (-b + v) / 3.0;
        k = ((t2 + b) * t2 + c) * t2 + d;
        // Find rightmost root of 0.5 * (r0 - t2)^2 * (6 * t2 +2 * b) + k1 = 0
        r0 = t2 + std::sqrt(-k / (3.0 * t2 + b));
      }
    }
    else
    {
      // r0=1.0/(cubick_inv(c/d,b/d,1.0/d));
      // about half work...
      // if(std::abs((((r0+b)*r0+c)*r0+d))>1e-10)
      r0 = -b / 3.0;
      if (std::abs(((3.0 * r0 + 2.0 * b) * r0 + c)) < 1e-4)
        r0 += 1;
      //else r0-=1;
      //double fx=(((r0+b)*r0+c)*r0+d); r0-=10; if(fx<0) r0+=20;
    }

    // Do ITER Newton-Raphson iterations
    // Break if position of root changes less than 1e-13
    // double starterr=std::abs(r0*(r0*(r0 + b) + c) + d);
    // TODO(RJ:) I have hardcoded the number of iteration here, it's a hardcoded in a define in the orginal implementation
    // according to the author, increasing it could lead to a better solution (more robust)
    for (unsigned int cnt = 0; cnt < 50; ++cnt)
    {
      double fx = (((r0 + b) * r0 + c) * r0 + d);

      if ((cnt < 7 || std::abs(fx) > 1e-13))
      {
        double fpx = ((3.0 * r0 + 2.0 * b) * r0 + c);
        r0 -= fx / fpx;
      }
      else
        break;
    }
    return r0;
}

void P3PNordberg::eigwithknown0(const Eigen::Matrix3d &x, Eigen::Matrix3d &E, Eigen::Vector3d &L)
{
      L(2) = 0.0;

      Eigen::Vector3d v3(x(3) * x(7) - x(6) * x(4),
              x(6) * x(1) - x(7) * x(0),
              x(4) * x(0) - x(3) * x(1));

      v3.normalize();

      double x01_squared = x(0, 1) * x(0, 1);
      // get the two other...
      double b = -x(0, 0) - x(1, 1) - x(2, 2);
      double c = -x01_squared - x(0, 2) * x(0, 2) - x(1, 2) * x(1, 2) +
                 x(0, 0) * (x(1, 1) + x(2, 2)) + x(1, 1) * x(2, 2);
      double e1, e2;
      // roots(poly(x))
      root2real(b, c, e1, e2);

      if (std::abs(e1) < std::abs(e2))
        std::swap(e1, e2);
      L(0) = e1;
      L(1) = e2;

      double mx0011 = -x(0, 0) * x(1, 1);
      double prec_0 = x(0, 1) * x(1, 2) - x(0, 2) * x(1, 1);
      double prec_1 = x(0, 1) * x(0, 2) - x(0, 0) * x(1, 2);

      double e = e1;
      double tmp = 1.0 / (e * (x(0, 0) + x(1, 1)) + mx0011 - e * e + x01_squared);
      double a1 = -(e * x(0, 2) + prec_0) * tmp;
      double a2 = -(e * x(1, 2) + prec_1) * tmp;
      double rnorm = 1.0 / std::sqrt(a1 * a1 + a2 * a2 + 1.0);
      a1 *= rnorm;
      a2 *= rnorm;
      Eigen::Vector3d v1(a1, a2, rnorm);

      // e = e2;
      double tmp2 = 1.0 / (e2 * (x(0, 0) + x(1, 1)) + mx0011 - e2 * e2 + x01_squared);
      double a21 = -(e2 * x(0, 2) + prec_0) * tmp2;
      double a22 = -(e2 * x(1, 2) + prec_1) * tmp2;
      double rnorm2 = 1.0 / std::sqrt(a21 * a21 + a22 * a22 + 1.0);
      a21 *= rnorm2;
      a22 *= rnorm2;
      Eigen::Vector3d v2(a21, a22, rnorm2);

      // optionally remove axb from v1,v2
      // costly and makes a very small difference!
      // v1=(v1-v1.dot(v3)*v3);v1.normalize();
      // v2=(v2-v2.dot(v3)*v3);v2.normalize();
      // v2=(v2-v1.dot(v2)*v2);v2.normalize();
      E << v1(0), v2(0), v3(0),
          v1(1), v2(1), v3(1),
          v1(2), v2(2), v3(2);
}



std::vector<P3PEstimator::M_t> P3PNordberg::Estimate(const std::vector<X_t>& points2D,
                                                   const std::vector<Y_t>& points3D)
{
    CHECK_EQ(points2D.size(), 3);
    CHECK_EQ(points3D.size(), 3);
    P3PNordberg p3p_nordberg;
    Eigen::Vector3d P1 = points3D[0];
    Eigen::Vector3d P2 = points3D[1];
    Eigen::Vector3d P3 = points3D[2];

    // feature point
    Eigen::Vector3d f1 = points2D[0].homogeneous();
    Eigen::Vector3d f2 = points2D[1].homogeneous();
    Eigen::Vector3d f3 = points2D[2].homogeneous();

    f1.normalize();
    f2.normalize();
    f3.normalize();

    double b12 = -2.0 * (f1.dot(f2));
    double b13 = -2.0 * (f1.dot(f3));
    double b23 = -2.0 * (f2.dot(f3));

    // implicit creation of Vec3, can be removed
    Eigen::Vector3d d12 = P1 - P2;
    Eigen::Vector3d d13 = P1 - P3;
    Eigen::Vector3d d23 = P2 - P3;
    Eigen::Vector3d d12xd13(d12.cross(d13));

    double a12 = d12.squaredNorm();
    double a13 = d13.squaredNorm();
    double a23 = d23.squaredNorm();


    //a*g^3 + b*g^2 + c*g + d = 0
    double c31 = -0.5 * b13;
    double c23 = -0.5 * b23;
    double c12 = -0.5 * b12;
    double blob = (c12 * c23 * c31 - 1.0);

    double s31_squared = 1.0 - c31 * c31;
    double s23_squared = 1.0 - c23 * c23;
    double s12_squared = 1.0 - c12 * c12;

    double p3 = a13 * (a23 * s31_squared - a13 * s23_squared);
    double p2 = 2.0 * blob * a23 * a13 + a13 * (2.0 * a12 + a13) * s23_squared + a23 * (a23 - a12) * s31_squared;
    double p1 = a23 * (a13 - a23) * s12_squared - a12 * a12 * s23_squared - 2.0 * a12 * (blob * a23 + a13 * s23_squared);
    double p0 = a12 * (a12 * s23_squared - a23 * s12_squared);


    p3 = 1.0 / p3;
    p2 *= p3;
    p1 *= p3;
    p0 *= p3;

    // get sharpest real root of above...
    double g =p3p_nordberg.cubick(p2, p1, p0);

    double A00 = a23 * (1.0 - g);
    double A01 = (a23 * b12) * 0.5;
    double A02 = (a23 * b13 * g) * (-0.5);
    double A11 = a23 - a12 + a13 * g;
    double A12 = b23 * (a13 * g - a12) * 0.5;
    double A22 = g * (a13 - a23) - a12;

    Eigen::Matrix3d A;
    A << A00, A01, A02,
         A01, A11, A12,
         A02, A12, A22;

    Eigen::Matrix3d V;
    Eigen::Vector3d L;

    p3p_nordberg.eigwithknown0(A, V, L);

    double v = std::sqrt(std::max(0.0, -L(1) / L(0)));

    int valid = 0;
    std::array<Eigen::Vector3d, 4> Ls;
    // use the t=Vl with t2,st2,t3 and solve for t3 in t2
    { //+v
      double s = v;

      double w2 = 1.0 / (s * V(0,1) - V(0,0));
      double w0 = (V(1,0) - s * V(1,1)) * w2;
      double w1 = (V(2,0) - s * V(2,1)) * w2;

      double a = 1.0 / ((a13 - a12) * w1 * w1 - a12 * b13 * w1 - a12);
      double b = (a13 * b12 * w1 - a12 * b13 * w0 - 2.0 * w0 * w1 * (a12 - a13)) * a;
      double c = ((a13 - a12) * w0 * w0 + a13 * b12 * w0 + a13) * a;

      if (b * b - 4.0 * c >= 0.0)
      {
        double tau1, tau2;
        p3p_nordberg.root2real(b, c, tau1, tau2);
        if (tau1 > 0.0)
        {
          double tau = tau1;
          double d = a23 / (tau * (b23 + tau) + 1.0);
          if(d > 0.0) {
            double l2 = std::sqrt(d);
            double l3 = tau * l2;

            double l1 = w0 * l2 + w1 * l3;
            if (l1 >= 0.0)
            {
              Ls[valid] = Eigen::Vector3d(l1, l2, l3);
              ++valid;
            }
          }
        }
        if (tau2 > 0.0)
        {
          double tau = tau2;
          double d = a23 / (tau * (b23 + tau) + 1.0);
          if(d > 0.0) {
            double l2 = std::sqrt(d);
            double l3 = tau * l2;
            double l1 = w0 * l2 + w1 * l3;
            if (l1 >= 0.0)
            {
              Ls[valid] =Eigen::Vector3d(l1, l2, l3);
              ++valid;
            }
          }
        }
      }
    }

    { //-v
      double s = -v;
      double w2 = 1.0 / (s * V(0, 1) - V(0, 0));
      double w0 = (V(1, 0) - s * V(1, 1)) * w2;
      double w1 = (V(2, 0) - s * V(2, 1)) * w2;

      double a = 1.0 / ((a13 - a12) * w1 * w1 - a12 * b13 * w1 - a12);
      double b = (a13 * b12 * w1 - a12 * b13 * w0 - 2.0 * w0 * w1 * (a12 - a13)) * a;
      double c = ((a13 - a12) * w0 * w0 + a13 * b12 * w0 + a13) * a;

      if (b * b - 4.0 * c >= 0)
      {
        double tau1, tau2;

        p3p_nordberg.root2real(b, c, tau1, tau2);
        if (tau1 > 0)
        {
          double tau = tau1;
          double d = a23 / (tau * (b23 + tau) + 1.0);
          if(d > 0.0) {
            double l2 = std::sqrt(d);
            double l3 = tau * l2;

            double l1 = w0 * l2 + w1 * l3;
            if (l1 >= 0)
            {
              Ls[valid] = Eigen::Vector3d(l1, l2, l3);
              ++valid;
            }
          }
        }
        if (tau2 > 0)
        {
          double tau = tau2;
          double d = a23 / (tau * (b23 + tau) + 1.0);
          if(d > 0.0) {
            double l2 = std::sqrt(d);
            double l3 = tau * l2;

            double l1 = w0 * l2 + w1 * l3;
            if (l1 >= 0)
            {
              Ls[valid] = Eigen::Vector3d(l1, l2, l3);
              ++valid;
            }
          }
        }
      }
    }
    if ( valid<=0 )
    {
        return std::vector<P3PNordberg::M_t>({});
    }


    // if constexpr (refinement_iterations>0)
    for (int i = 0; i < valid; ++i)
    {
      p3p_nordberg.gauss_newton_refineL(Ls[i], a12, a13, a23, b12, b13, b23);
    }

    Eigen::Vector3d ry1, ry2, ry3;
    Eigen::Vector3d yd1;
    Eigen::Vector3d yd2;
    Eigen::Vector3d yd1xd2;
    Eigen::Matrix3d Xmat;
    Xmat << d12(0), d13(0), d12xd13(0),
            d12(1), d13(1), d12xd13(1),
            d12(2), d13(2), d12xd13(2);

    Xmat = Xmat.inverse().eval();

    std::vector<M_t> models;
    models.reserve(valid);
    for (int i = 0; i < valid; ++i)
    {
      // compute the rotation:
      ry1 = f1 * Ls[i](0);
      ry2 = f2 * Ls[i](1);
      ry3 = f3 * Ls[i](2);

      yd1 = ry1 - ry2;
      yd2 = ry1 - ry3;
      yd1xd2 = yd1.cross(yd2);

      Eigen::Matrix3d Ymat;
      Ymat << yd1(0), yd2(0), yd1xd2(0),
              yd1(1), yd2(1), yd1xd2(1),
              yd1(2), yd2(2), yd1xd2(2);

      Eigen::Matrix3d Rs = Ymat * Xmat;
      M_t proj_matrix;
      proj_matrix.leftCols<3>() = Rs;
      proj_matrix.rightCols<1>() = ry1 - Rs * P1 ;
      models.push_back(proj_matrix);
    }
//    std::cout<<" pnp use p3p_nordberg \n";
    return models;
}

void P3PNordberg::Residuals(const std::vector<X_t> &points2D, const std::vector<Y_t> &points3D, const M_t &proj_matrix, std::vector<double> *residuals)
{
      ComputeSquaredReprojectionError(points2D, points3D, proj_matrix, residuals);
}
// p3p -ke
void P3Pke::solveQuarticPolynomial(const std::array<double, 5> &coeffs, std::array<double, 4> &real_roots)
{
//    P3Pke k3;
    const double a = coeffs[0];
      const double b = coeffs[1] / a;
      const double c = coeffs[2] / a;
      const double d = coeffs[3] / a;
      const double e = coeffs[4] / a;

      const std::complex<double> Q1 = c * c - 3. * b * d + 12. * e;
      const std::complex<double> Q2 = 2. * c * c * c - 9. * b * c * d
                                      + 27. * d * d + 27. * b * b * e - 72. * c * e;
      const std::complex<double> Q3 = 8. * b * c - 16. * d - 2. * b * b * b;
      const std::complex<double> Q4 = 3. * b * b - 8. * c;

      const std::complex<double> Q5 = complex_cbrt(Q2 / 2.
                                      + sqrt(Q2 * Q2 / 4. - Q1 * Q1 * Q1));
      const std::complex<double> Q6 = (Q1 / Q5 + Q5) / 3.;
      const std::complex<double> Q7 = 2. * sqrt(Q4 / 12. + Q6);

      real_roots = {
        {(-b - Q7 - sqrt(4. * Q4 / 6. - 4. * Q6 - Q3 / Q7)).real() / 4.,
          (-b - Q7 + sqrt(4. * Q4 / 6. - 4. * Q6 - Q3 / Q7)).real() / 4.,
          (-b + Q7 - sqrt(4. * Q4 / 6. - 4. * Q6 + Q3 / Q7)).real() / 4.,
          (-b + Q7 + sqrt(4. * Q4 / 6. - 4. * Q6 + Q3 / Q7)).real() / 4.}};
}

void P3Pke::polishQuarticPolynomialRoots(const std::array<double, 5> &coeffs, std::array<double, 4> &roots, const int iterations)
{
    for (int i = 0; i < iterations; ++i)
    {
      for (auto & root : roots)
      {
        const double error =
          coeffs[4] + root * (coeffs[3] +
                              root * (coeffs[2] +
                                      root * (coeffs[1] +
                                              root * coeffs[0])));

        const double derivative =
          coeffs[3] + root * (2 * coeffs[2] +
                              root * ((4 * coeffs[0] * root + 3 * coeffs[1])));

        root -= error / derivative;
      }
    }
}

std::vector<P3Pke::M_t> P3Pke::Estimate( const std::vector<X_t>& points2D, const std::vector<Y_t>& points3D) {
    CHECK_EQ(points2D.size(), 3);
    CHECK_EQ(points3D.size(), 3);
    const Eigen::Vector3d w1 = points3D[0];
    const Eigen::Vector3d w2 = points3D[1];
    const Eigen::Vector3d w3 = points3D[2];
    //k1
    const Eigen::Vector3d u0 = w1 - w2;
    const double nu0 = u0.norm();
    const Eigen::Vector3d k1 = u0.normalized();
    // feature point
    const Eigen::Vector3d b1 = points2D[0].homogeneous();
    const Eigen::Vector3d b2 = points2D[1].homogeneous();
    const Eigen::Vector3d b3 = points2D[2].homogeneous();
    //k3 tz
    Eigen::Vector3d k3 = b1.cross(b2);
    const double nk3 = k3.norm();
    k3 = k3.normalized();

    const Eigen::Vector3d tz = b1.cross(k3);

    const Eigen::Vector3d v1 = b1.cross(b3);
    const Eigen::Vector3d v2 = b2.cross(b3);


    const Eigen::Vector3d u1 = w1 - w3;
    // coefficients related terms
    const double u1k1 = u1.dot(k1);
    const double k3b3 = k3.dot(b3);
    // f1i
    double f11 = k3b3;
    double f13 = k3.dot(v1);
    const double f15 = -u1k1 * f11;
    //delta
    const Eigen::Vector3d nl = u1.cross(k1).normalized();
    const double delta = u1.cross(k1).norm();
    f11 *= delta;
    f13 *= delta;
    // f2i
     const double u2k1 = u1k1 - nu0;
     double f21 = tz.dot(v2);
     double f22 = nk3 * k3b3;
     double f23 = k3.dot(v2);
     const double f24 = u2k1 * f22;
     const double f25 = -u2k1 * f21;
     f21 *= delta;
     f22 *= delta;
     f23 *= delta;
     const double g1 = f13 * f22;
     const double g2 = f13 * f25 - f15 * f23;
     const double g3 = f11 * f23 - f13 * f21;
     const double g4 = -f13 * f24;
     const double g5 = f11 * f22;
     const double g6 = f11 * f25 - f15 * f21;
     const double g7 = -f15 * f24;
     const std::array<double, 5> coeffs = {
       {g5 * g5 + g1 * g1 + g3 * g3,
         2 * (g5 * g6 + g1 * g2 + g3 * g4),
         g6 * g6 + 2 * g5 * g7 + g2 * g2 + g4 * g4 - g1 * g1 - g3 * g3,
         2 * (g6 * g7 - g1 * g2 - g3 * g4),
         g7 * g7 - g2 * g2 - g4 * g4}
     };
     P3Pke ke;
     std::array<double, 4> s;
     ke.solveQuarticPolynomial(coeffs, s);
     ke.polishQuarticPolynomialRoots(coeffs, s);

     const Eigen::Vector3d temp = k1.cross(nl);

       Eigen::Matrix3d Ck1nl;
       Ck1nl << k1, nl, temp;

       Eigen::Matrix3d Cb1k3tzT;
       Cb1k3tzT << b1.transpose(), k3.transpose(), tz.transpose();

       const Eigen::Vector3d b3p = b3 * (delta / k3b3);
       std::vector<M_t> models;
       models.reserve(s.size());
       for (const auto ctheta1p : s) {
         if (std::abs(ctheta1p) > 1)
         {
           continue;
         }
         const double stheta1p = ((k3b3 > 0) ? 1 : -1) * sqrt(1 - ctheta1p * ctheta1p);
         const double ntheta3 = stheta1p / ((g5 * ctheta1p + g6) * ctheta1p + g7);
         const double ctheta3 = (g1 * ctheta1p + g2) * ntheta3;
         const double stheta3 = (g3 * ctheta1p + g4) * ntheta3;

         Eigen::Matrix3d C13;
         C13 <<
           ctheta3,            0,         -stheta3,
           stheta1p * stheta3, ctheta1p,  stheta1p * ctheta3,
           ctheta1p * stheta3, -stheta1p, ctheta1p * ctheta3;

         const Eigen::Matrix3d R = (Ck1nl * C13) * Cb1k3tzT;
         const Eigen::Vector3d rp3 = R.transpose() * w3; // R' * p3
         M_t proj_matrix;
         proj_matrix.leftCols<3>() = R.transpose();
         proj_matrix.rightCols<1>() =  (b3p * stheta1p) - rp3 ;
         models.push_back(proj_matrix);
       }
       if ( models.empty() )
       {
//           std::cout<<"warning p3p_ke compute error!!!!!!!!!!!\n";
           return std::vector<P3Pke::M_t>({});
       }
//       std::cout << "***********p3p_ke******************"<<models.empty()<<"\n";
       return models;

}

void P3Pke::Residuals(const std::vector<X_t>& points2D,
                             const std::vector<Y_t>& points3D,
                             const M_t& proj_matrix,
                             std::vector<double>* residuals) {
  ComputeSquaredReprojectionError(points2D, points3D, proj_matrix, residuals);
}

// EPNP
std::vector<EPNPEstimator::M_t> EPNPEstimator::Estimate(
    const std::vector<X_t>& points2D, const std::vector<Y_t>& points3D) {
  CHECK_GE(points2D.size(), 4);
  CHECK_EQ(points2D.size(), points3D.size());

  EPNPEstimator epnp;
  M_t proj_matrix;
  if (!epnp.ComputePose(points2D, points3D, &proj_matrix)) {
    return std::vector<EPNPEstimator::M_t>({});
  }

  return std::vector<EPNPEstimator::M_t>({proj_matrix});
}

void EPNPEstimator::Residuals(const std::vector<X_t>& points2D,
                              const std::vector<Y_t>& points3D,
                              const M_t& proj_matrix,
                              std::vector<double>* residuals) {
  ComputeSquaredReprojectionError(points2D, points3D, proj_matrix, residuals);
}

// // 与opencv 中epnp 源码一模一样
bool EPNPEstimator::ComputePose(const std::vector<Eigen::Vector2d>& points2D,
                                const std::vector<Eigen::Vector3d>& points3D,
                                Eigen::Matrix3x4d* proj_matrix) {
  points2D_ = &points2D;
  points3D_ = &points3D;
 // 选择虚拟控制点 ==>巧妙之处在于引进了虚拟控制点
  ChooseControlPoints();

  if (!ComputeBarycentricCoordinates()) {
    return false;
  }
 // 控制点在相机坐标系下的坐标
  // 12 的原因是不知道4个重心坐标系下的坐标在相机坐标系的坐标 即cj_c;4*3
  const Eigen::Matrix<double, Eigen::Dynamic, 12> M = ComputeM();
  const Eigen::Matrix<double, 12, 12> MtM = M.transpose() * M;
  // 求解Mx=0,SVD 分解
  Eigen::JacobiSVD<Eigen::Matrix<double, 12, 12>> svd(
      MtM, Eigen::ComputeFullV | Eigen::ComputeFullU);
  const Eigen::Matrix<double, 12, 12> Ut = svd.matrixU().transpose();

  // 列出 <L*belta=Rho>,6个方程,10个未知数?   原文Reinearization:Ri_j=Ri*Rj
  // {C4_2=6}*10个未知数
  const Eigen::Matrix<double, 6, 10> L6x10 = ComputeL6x10(Ut);
  // 计算控制点在重心坐标系(新世界坐标系)下的两两距离
  const Eigen::Matrix<double, 6, 1> rho = ComputeRho();
 // 求解belta
  Eigen::Vector4d betas[4];
  std::array<double, 4> reproj_errors;
  std::array<Eigen::Matrix3d, 4> Rs;
  std::array<Eigen::Vector3d, 4> ts;
  ////////////////////////////
 // 第一次求近似解和高斯牛顿迭代优化
  FindBetasApprox1(L6x10, rho, &betas[1]);
  RunGaussNewton(L6x10, rho, &betas[1]);
  reproj_errors[1] = ComputeRT(Ut, betas[1], &Rs[1], &ts[1]);
  // 第二次求解近似解和高斯牛顿优化
  FindBetasApprox2(L6x10, rho, &betas[2]);
  RunGaussNewton(L6x10, rho, &betas[2]);
  reproj_errors[2] = ComputeRT(Ut, betas[2], &Rs[2], &ts[2]);
  // 第三次求解近似解和高斯牛顿优化
  FindBetasApprox3(L6x10, rho, &betas[3]);
  RunGaussNewton(L6x10, rho, &betas[3]);
  reproj_errors[3] = ComputeRT(Ut, betas[3], &Rs[3], &ts[3]);  // 得到了相机坐标系的坐标,那么就可根据ICP方法求解R,T

  // 选择重投影误差最小的作为最终解
  int best_idx = 1;
  if (reproj_errors[2] < reproj_errors[1]) {
    best_idx = 2;
  }
  if (reproj_errors[3] < reproj_errors[best_idx]) {
    best_idx = 3;
  }

  proj_matrix->leftCols<3>() = Rs[best_idx];
  proj_matrix->rightCols<1>() = ts[best_idx];
//  std::cout<<" pnp use epnp \n";
  return true;
}

// 第一步选择4个非共面的虚拟点作为控制点
void EPNPEstimator::ChooseControlPoints() {
  // Take C0 as the reference points centroid:
  cws_[0].setZero();
  for (size_t i = 0; i < points3D_->size(); ++i) {
    cws_[0] += (*points3D_)[i];
  }
  cws_[0] /= points3D_->size(); // 第一个控制点选择重心位置
  //其余控制点点选择在数据的主方向上
  Eigen::Matrix<double, Eigen::Dynamic, 3> PW0(points3D_->size(), 3);
  for (size_t i = 0; i < points3D_->size(); ++i) {
    PW0.row(i) = (*points3D_)[i] - cws_[0]; // 每个点减去重心坐标,建立A 矩阵
  }
 // 计算A.T*A的特征值,lamda1,lamda2,lamda3,对应特征向量:v1,v2,v3
// 任何一个3D点都可以表示为四个控制点的线性组合
  const Eigen::Matrix3d PW0tPW0 = PW0.transpose() * PW0;
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      PW0tPW0, Eigen::ComputeFullV | Eigen::ComputeFullU);
  const Eigen::Vector3d D = svd.singularValues();
  const Eigen::Matrix3d Ut = svd.matrixU().transpose();//
// 其余三个控制点坐标
  for (int i = 1; i < 4; ++i) {
    const double k = std::sqrt(D(i - 1) / points3D_->size());
    cws_[i] = cws_[0] + k * Ut.row(i - 1).transpose();
  }
}

bool EPNPEstimator::ComputeBarycentricCoordinates() {
  Eigen::Matrix3d CC;
  for (int i = 0; i < 3; ++i) {
    for (int j = 1; j < 4; ++j) {
      CC(i, j - 1) = cws_[j][i] - cws_[0][i];
    }
  }

  if (CC.colPivHouseholderQr().rank() < 3) {
    return false;
  }

  const Eigen::Matrix3d CC_inv = CC.inverse();

  alphas_.resize(points2D_->size());
  for (size_t i = 0; i < points3D_->size(); ++i) {
    for (int j = 0; j < 3; ++j) {
      alphas_[i][1 + j] = CC_inv(j, 0) * ((*points3D_)[i][0] - cws_[0][0]) +
                          CC_inv(j, 1) * ((*points3D_)[i][1] - cws_[0][1]) +
                          CC_inv(j, 2) * ((*points3D_)[i][2] - cws_[0][2]);
    }
    alphas_[i][0] = 1.0 - alphas_[i][1] - alphas_[i][2] - alphas_[i][3];
  }

  return true;
} // 计算重心坐标系的alpha
// 相机模型
// 一个点列出2n*12个参数
Eigen::Matrix<double, Eigen::Dynamic, 12> EPNPEstimator::ComputeM() {
  Eigen::Matrix<double, Eigen::Dynamic, 12> M(2 * points2D_->size(), 12);
  for (size_t i = 0; i < points3D_->size(); ++i) {
    for (size_t j = 0; j < 4; ++j) {
      M(2 * i, 3 * j) = alphas_[i][j];
      M(2 * i, 3 * j + 1) = 0.0;
      M(2 * i, 3 * j + 2) = -alphas_[i][j] * (*points2D_)[i].x();

      M(2 * i + 1, 3 * j) = 0.0;
      M(2 * i + 1, 3 * j + 1) = alphas_[i][j];
      M(2 * i + 1, 3 * j + 2) = -alphas_[i][j] * (*points2D_)[i].y();
    }
  }
  return M;
}

Eigen::Matrix<double, 6, 10> EPNPEstimator::ComputeL6x10(
    const Eigen::Matrix<double, 12, 12>& Ut) {
  Eigen::Matrix<double, 6, 10> L6x10;

  std::array<std::array<Eigen::Vector3d, 6>, 4> dv;
  for (int i = 0; i < 4; ++i) {
    int a = 0, b = 1;
    for (int j = 0; j < 6; ++j) {
      dv[i][j][0] = Ut(11 - i, 3 * a) - Ut(11 - i, 3 * b);
      dv[i][j][1] = Ut(11 - i, 3 * a + 1) - Ut(11 - i, 3 * b + 1);
      dv[i][j][2] = Ut(11 - i, 3 * a + 2) - Ut(11 - i, 3 * b + 2);

      b += 1;
      if (b > 3) {
        a += 1;
        b = a + 1;
      }
    }
  }

  for (int i = 0; i < 6; ++i) {
    L6x10(i, 0) = dv[0][i].transpose() * dv[0][i];
    L6x10(i, 1) = 2.0 * dv[0][i].transpose() * dv[1][i];
    L6x10(i, 2) = dv[1][i].transpose() * dv[1][i];
    L6x10(i, 3) = 2.0 * dv[0][i].transpose() * dv[2][i];
    L6x10(i, 4) = 2.0 * dv[1][i].transpose() * dv[2][i];
    L6x10(i, 5) = dv[2][i].transpose() * dv[2][i];
    L6x10(i, 6) = 2.0 * dv[0][i].transpose() * dv[3][i];
    L6x10(i, 7) = 2.0 * dv[1][i].transpose() * dv[3][i];
    L6x10(i, 8) = 2.0 * dv[2][i].transpose() * dv[3][i];
    L6x10(i, 9) = dv[3][i].transpose() * dv[3][i];
  }

  return L6x10;
}

Eigen::Matrix<double, 6, 1> EPNPEstimator::ComputeRho() {
  Eigen::Matrix<double, 6, 1> rho;
  rho[0] = (cws_[0] - cws_[1]).squaredNorm(); // 0-1
  rho[1] = (cws_[0] - cws_[2]).squaredNorm(); // 0-2
  rho[2] = (cws_[0] - cws_[3]).squaredNorm(); // 0-3
  rho[3] = (cws_[1] - cws_[2]).squaredNorm(); // 1-2
  rho[4] = (cws_[1] - cws_[3]).squaredNorm(); // 1-3
  rho[5] = (cws_[2] - cws_[3]).squaredNorm(); // 2-3
  return rho;
}

// 非齐次线性方程组的通解=齐次线性方程组的通解+非齐次线性方程组的一个特解
// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approx_1 = [B11 B12     B13         B14]

void EPNPEstimator::FindBetasApprox1(const Eigen::Matrix<double, 6, 10>& L6x10,
                                     const Eigen::Matrix<double, 6, 1>& rho,
                                     Eigen::Vector4d* betas) {
  Eigen::Matrix<double, 6, 4> L_6x4;
  for (int i = 0; i < 6; ++i) {
    L_6x4(i, 0) = L6x10(i, 0);
    L_6x4(i, 1) = L6x10(i, 1);
    L_6x4(i, 2) = L6x10(i, 3);
    L_6x4(i, 3) = L6x10(i, 6);
  }

  Eigen::JacobiSVD<Eigen::Matrix<double, 6, 4>> svd(
      L_6x4, Eigen::ComputeFullV | Eigen::ComputeFullU);
  Eigen::Matrix<double, 6, 1> Rho_temp = rho;
  const Eigen::Matrix<double, 4, 1> b4 = svd.solve(Rho_temp);

  if (b4[0] < 0) {
    (*betas)[0] = std::sqrt(-b4[0]);
    (*betas)[1] = -b4[1] / (*betas)[0];
    (*betas)[2] = -b4[2] / (*betas)[0];
    (*betas)[3] = -b4[3] / (*betas)[0];
  } else {
    (*betas)[0] = std::sqrt(b4[0]);
    (*betas)[1] = b4[1] / (*betas)[0];
    (*betas)[2] = b4[2] / (*betas)[0];
    (*betas)[3] = b4[3] / (*betas)[0];
  }
}

// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approx_2 = [B11 B12 B22                            ]

void EPNPEstimator::FindBetasApprox2(const Eigen::Matrix<double, 6, 10>& L6x10,
                                     const Eigen::Matrix<double, 6, 1>& rho,
                                     Eigen::Vector4d* betas) {
  Eigen::Matrix<double, 6, 3> L_6x3(6, 3);

  for (int i = 0; i < 6; ++i) {
    L_6x3(i, 0) = L6x10(i, 0);
    L_6x3(i, 1) = L6x10(i, 1);
    L_6x3(i, 2) = L6x10(i, 2);
  }

  Eigen::JacobiSVD<Eigen::Matrix<double, 6, 3>> svd(
      L_6x3, Eigen::ComputeFullV | Eigen::ComputeFullU);
  Eigen::Matrix<double, 6, 1> Rho_temp = rho;
  const Eigen::Matrix<double, 3, 1> b3 = svd.solve(Rho_temp);

  if (b3[0] < 0) {
    (*betas)[0] = std::sqrt(-b3[0]);
    (*betas)[1] = (b3[2] < 0) ? std::sqrt(-b3[2]) : 0.0;
  } else {
    (*betas)[0] = std::sqrt(b3[0]);
    (*betas)[1] = (b3[2] > 0) ? std::sqrt(b3[2]) : 0.0;
  }

  if (b3[1] < 0) {
    (*betas)[0] = -(*betas)[0];
  }

  (*betas)[2] = 0.0;
  (*betas)[3] = 0.0;
}

// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approx_3 = [B11 B12 B22 B13 B23                    ]

void EPNPEstimator::FindBetasApprox3(const Eigen::Matrix<double, 6, 10>& L6x10,
                                     const Eigen::Matrix<double, 6, 1>& rho,
                                     Eigen::Vector4d* betas) {
  Eigen::JacobiSVD<Eigen::Matrix<double, 6, 5>> svd(
      L6x10.leftCols<5>(), Eigen::ComputeFullV | Eigen::ComputeFullU);
  Eigen::Matrix<double, 6, 1> Rho_temp = rho;
  const Eigen::Matrix<double, 5, 1> b5 = svd.solve(Rho_temp);

  if (b5[0] < 0) {
    (*betas)[0] = std::sqrt(-b5[0]);
    (*betas)[1] = (b5[2] < 0) ? std::sqrt(-b5[2]) : 0.0;
  } else {
    (*betas)[0] = std::sqrt(b5[0]);
    (*betas)[1] = (b5[2] > 0) ? std::sqrt(b5[2]) : 0.0;
  }
  if (b5[1] < 0) {
    (*betas)[0] = -(*betas)[0];
  }
  (*betas)[2] = b5[3] / (*betas)[0];
  (*betas)[3] = 0.0;
}

void EPNPEstimator::RunGaussNewton(const Eigen::Matrix<double, 6, 10>& L6x10,
                                   const Eigen::Matrix<double, 6, 1>& rho,
                                   Eigen::Vector4d* betas) {
  Eigen::Matrix<double, 6, 4> A;
  Eigen::Matrix<double, 6, 1> b;

  const int kNumIterations = 5;
  for (int k = 0; k < kNumIterations; ++k) {
    for (int i = 0; i < 6; ++i) {
      A(i, 0) = 2 * L6x10(i, 0) * (*betas)[0] + L6x10(i, 1) * (*betas)[1] +
                L6x10(i, 3) * (*betas)[2] + L6x10(i, 6) * (*betas)[3];
      A(i, 1) = L6x10(i, 1) * (*betas)[0] + 2 * L6x10(i, 2) * (*betas)[1] +
                L6x10(i, 4) * (*betas)[2] + L6x10(i, 7) * (*betas)[3];
      A(i, 2) = L6x10(i, 3) * (*betas)[0] + L6x10(i, 4) * (*betas)[1] +
                2 * L6x10(i, 5) * (*betas)[2] + L6x10(i, 8) * (*betas)[3];
      A(i, 3) = L6x10(i, 6) * (*betas)[0] + L6x10(i, 7) * (*betas)[1] +
                L6x10(i, 8) * (*betas)[2] + 2 * L6x10(i, 9) * (*betas)[3];

      b(i) = rho[i] - (L6x10(i, 0) * (*betas)[0] * (*betas)[0] +
                       L6x10(i, 1) * (*betas)[0] * (*betas)[1] +
                       L6x10(i, 2) * (*betas)[1] * (*betas)[1] +
                       L6x10(i, 3) * (*betas)[0] * (*betas)[2] +
                       L6x10(i, 4) * (*betas)[1] * (*betas)[2] +
                       L6x10(i, 5) * (*betas)[2] * (*betas)[2] +
                       L6x10(i, 6) * (*betas)[0] * (*betas)[3] +
                       L6x10(i, 7) * (*betas)[1] * (*betas)[3] +
                       L6x10(i, 8) * (*betas)[2] * (*betas)[3] +
                       L6x10(i, 9) * (*betas)[3] * (*betas)[3]);
    }

    const Eigen::Vector4d x = A.colPivHouseholderQr().solve(b);

    (*betas) += x;
  }
}

double EPNPEstimator::ComputeRT(const Eigen::Matrix<double, 12, 12>& Ut,
                                const Eigen::Vector4d& betas,
                                Eigen::Matrix3d* R, Eigen::Vector3d* t) {
  ComputeCcs(betas, Ut);
  ComputePcs();

  SolveForSign();

  EstimateRT(R, t);

  return ComputeTotalReprojectionError(*R, *t);
}

void EPNPEstimator::ComputeCcs(const Eigen::Vector4d& betas,
                               const Eigen::Matrix<double, 12, 12>& Ut) {
  for (int i = 0; i < 4; ++i) {
    ccs_[i][0] = ccs_[i][1] = ccs_[i][2] = 0.0;
  }

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      for (int k = 0; k < 3; ++k) {
        ccs_[j][k] += betas[i] * Ut(11 - i, 3 * j + k);
      }
    }
  }
}

void EPNPEstimator::ComputePcs() {
  pcs_.resize(points2D_->size());
  for (size_t i = 0; i < points3D_->size(); ++i) {
    for (int j = 0; j < 3; ++j) {
      pcs_[i][j] = alphas_[i][0] * ccs_[0][j] + alphas_[i][1] * ccs_[1][j] +
                   alphas_[i][2] * ccs_[2][j] + alphas_[i][3] * ccs_[3][j];
    }
  }
}

void EPNPEstimator::SolveForSign() {
  if (pcs_[0][2] < 0.0) {
    for (int i = 0; i < 4; ++i) {
      ccs_[i] = -ccs_[i];
    }
    for (size_t i = 0; i < points3D_->size(); ++i) {
      pcs_[i] = -pcs_[i];
    }
  }
}

void EPNPEstimator::EstimateRT(Eigen::Matrix3d* R, Eigen::Vector3d* t) {
  Eigen::Vector3d pc0 = Eigen::Vector3d::Zero();
  Eigen::Vector3d pw0 = Eigen::Vector3d::Zero();

  for (size_t i = 0; i < points3D_->size(); ++i) {
    pc0 += pcs_[i];
    pw0 += (*points3D_)[i];
  }
  pc0 /= points3D_->size();
  pw0 /= points3D_->size();

  Eigen::Matrix3d abt = Eigen::Matrix3d::Zero();
  for (size_t i = 0; i < points3D_->size(); ++i) {
    for (int j = 0; j < 3; ++j) {
      abt(j, 0) += (pcs_[i][j] - pc0[j]) * ((*points3D_)[i][0] - pw0[0]);
      abt(j, 1) += (pcs_[i][j] - pc0[j]) * ((*points3D_)[i][1] - pw0[1]);
      abt(j, 2) += (pcs_[i][j] - pc0[j]) * ((*points3D_)[i][2] - pw0[2]);
    }
  }

  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      abt, Eigen::ComputeFullV | Eigen::ComputeFullU);
  const Eigen::Matrix3d abt_U = svd.matrixU();
  const Eigen::Matrix3d abt_V = svd.matrixV();

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      (*R)(i, j) = abt_U.row(i) * abt_V.row(j).transpose();
    }
  }

  if (R->determinant() < 0) {
    Eigen::Matrix3d Abt_v_prime = abt_V;
    Abt_v_prime.col(2) = -abt_V.col(2);
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        (*R)(i, j) = abt_U.row(i) * Abt_v_prime.row(j).transpose();
      }
    }
  }

  *t = pc0 - *R * pw0;
}

double EPNPEstimator::ComputeTotalReprojectionError(const Eigen::Matrix3d& R,
                                                    const Eigen::Vector3d& t) {
  Eigen::Matrix3x4d proj_matrix;
  proj_matrix.leftCols<3>() = R;
  proj_matrix.rightCols<1>() = t;

  std::vector<double> residuals;
  ComputeSquaredReprojectionError(*points2D_, *points3D_, proj_matrix,
                                  &residuals);

  double reproj_error = 0.0;
  for (const double residual : residuals) {
    reproj_error += std::sqrt(residual);
  }

  return reproj_error;
}

}  // namespace colmap
