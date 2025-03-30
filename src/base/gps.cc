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

#include "base/gps.h"

#include "util/math.h"

namespace colmap {
typedef Eigen::Vector3d Vec3;
// WGS84 Ellipsoid
static const double WGS84_A = 6378137.0;      // major axis
static const double WGS84_B = 6356752.314245; // minor axis
/*
GPSTransform::GPSTransform(const int ellipsoid) {
  switch (ellipsoid) {
    case GRS80:
      a_ = 6378137;
      b_ = 6.356752314140356e+06;
      f_ = 0.003352810681182;
      break;
    case WGS84:
      a_ = 6378137;
      b_ = 6.356752314245179e+06;
      f_ = 0.003352810664747;
      break;
    default:
      a_ = std::numeric_limits<double>::quiet_NaN();
      b_ = std::numeric_limits<double>::quiet_NaN();
      f_ = std::numeric_limits<double>::quiet_NaN();
      throw std::invalid_argument("Ellipsoid not defined");
  }

  e2_ = (a_ * a_ - b_ * b_) / (a_ * a_);
}

std::vector<Eigen::Vector3d> GPSTransform::EllToXYZ(
    const std::vector<Eigen::Vector3d>& ell) const {
  std::vector<Eigen::Vector3d> xyz(ell.size());

  for (size_t i = 0; i < ell.size(); ++i) {
    const double lat = DegToRad(ell[i](0));
    const double lon = DegToRad(ell[i](1));
    const double alt = ell[i](2);

    const double sin_lat = sin(lat);
    const double sin_lon = sin(lon);
    const double cos_lat = cos(lat);
    const double cos_lon = cos(lon);

    // Normalized radius
    const double N = a_ / sqrt(1 - e2_ * sin_lat * sin_lat);

    xyz[i](0) = (N + alt) * cos_lat * cos_lon;
    xyz[i](1) = (N + alt) * cos_lat * sin_lon;
    xyz[i](2) = (N * (1 - e2_) + alt) * sin_lat;
  }

  return xyz;
}

std::vector<Eigen::Vector3d> GPSTransform::XYZToEll(
    const std::vector<Eigen::Vector3d>& xyz) const {
  std::vector<Eigen::Vector3d> ell(xyz.size());

  for (size_t i = 0; i < ell.size(); ++i) {
    const double x = xyz[i](0);
    const double y = xyz[i](1);
    const double z = xyz[i](2);

    const double xx = x * x;
    const double yy = y * y;

    const double kEps = 1e-12;

    // Latitude
    double lat = atan2(z, sqrt(xx + yy));
    double alt;

    for (size_t j = 0; j < 100; ++j) {
      const double sin_lat0 = sin(lat);
      const double N = a_ / sqrt(1 - e2_ * sin_lat0 * sin_lat0);
      alt = sqrt(xx + yy) / cos(lat) - N;
      const double prev_lat = lat;
      lat = atan((z / sqrt(xx + yy)) * 1 / (1 - e2_ * N / (N + alt)));

      if (std::abs(prev_lat - lat) < kEps) {
        break;
      }
    }

    ell[i](0) = RadToDeg(lat);

    // Longitude
    ell[i](1) = RadToDeg(atan2(y, x));
    // Alt
    ell[i](2) = alt;
  }

  return ell;
}
*/
 Eigen::Vector3d GPSTransform::lla_to_utm(double lat, double lon, double alt)
 {
       double a=WGS84_A;
       double b=WGS84_B;
       a /= 1000; // meters to kilometers
       b /= 1000; // meters to kilometers

       /// CONSTANTS
       static const double N0_n = 0;
       static const double N0_s = 1e4;
       static const double E0 = 5e2;
       static const double k0 = 0.9996;
       const double f = (a - b) / a;

       const double n    = f / (2 - f);
       const double n_2  = n   * n;
       const double n_3  = n_2 * n;
       const double n_4  = n_3 * n;
       const double n_5  = n_4 * n;
       const double n_6  = n_5 * n;
       const double n_7  = n_6 * n;
       const double n_8  = n_7 * n;
       const double n_9  = n_8 * n;
       const double n_10 = n_9 * n;

       const int lon_zone = 1 + floor((lon + 180) / 6);

       double lon_0 = 3 + 6 * (lon_zone - 1) - 180;

       lon_0=DegToRad(lon_0);
       lat = DegToRad(lat);
       lon = DegToRad(lon);

       const double A = a / (1 + n) * (1 + n_2/4 + n_4/64 + n_6/256 + n_8*25.0/16384.0 + n_10*49.0/65536.0);

       const double a1 = (1.0/2.0)*n - (2.0/3.0)*n_2 + (5.0/16.0)*n_3 + (41.0/180.0)*n_4 - (127.0/288.0)*n_5 + (7891.0/37800.0)*n_6 + (72161.0/387072.0)*n_7 - (18975107.0/50803200.0)*n_8 + (60193001.0/290304000.0)*n_9 + (134592031.0/1026432000.0)*n_10;
       const double a2 = (13.0/48.0)*n_2 - (3.0/5.0)*n_3 + (557.0/1440.0)*n_4 + (281.0/630.0)*n_5 - (1983433.0/1935360.0)*n_6 + (13769.0/28800.0)*n_7 + (148003883.0/174182400.0)*n_8 - (705286231.0/465696000.0)*n_9 + (1703267974087.0/3218890752000.0)*n_10;
       const double a3 = (61.0/240.0)*n_3 - (103.0/140.0)*n_4 + (15061.0/26880.0)*n_5 + (167603.0/181440.0)*n_6 - (67102379.0/29030400.0)*n_7 + (79682431.0/79833600.0)*n_8 + (6304945039.0/2128896000.0)*n_9 - (6601904925257.0/1307674368000.0)*n_10;
       const double a4 = (49561.0/161280.0)*n_4 - (179.0/168.0)*n_5 + (6601661.0/7257600.0)*n_6 + (97445.0/49896.0)*n_7 - (40176129013.0/7664025600.0)*n_8 + (138471097.0/66528000.0)*n_9 + (48087451385201.0/5230697472000.0)*n_10;
       const double a5 = (34729.0/80640.0)*n_5 - (3418889.0/1995840.0)*n_6 + (14644087.0/9123840.0)*n_7 + (2605413599.0/622702080.0)*n_8 - (31015475399.0/2583060480.0)*n_9 + (5820486440369.0/1307674368000.0)*n_10;
       const double a6 = (212378941.0/319334400.0)*n_6 - (30705481.0/10378368.0)*n_7 + (175214326799.0/58118860800.0)*n_8 + (870492877.0/96096000.0)*n_9 - (1328004581729009.0/47823519744000.0)*n_10;
       const double a7 = (1522256789.0/1383782400.0)*n_7 - (16759934899.0/3113510400.0)*n_8 + (1315149374443.0/221405184000.0)*n_9 + (71809987837451.0/3629463552000.0)*n_10;
       const double a8 = (1424729850961.0/743921418240.0)*n_8 - (256783708069.0/25204608000.0)*n_9 + (2468749292989891.0/203249958912000.0)*n_10;
       const double a9 = (21091646195357.0/6080126976000.0)*n_9 - (67196182138355857.0/3379030566912000.0)*n_10;
       const double a10 = (77911515623232821.0/12014330904576000.0)*n_10;

       const double t = sinh(atanh(sin(lat)) - 2*sqrt(n)/(1+n) * atanh(2*sqrt(n)/(1+n)*sin(lat)));
       const double xi = atan(t/cos(lon-lon_0));
       const double eta = atanh(sin(lon-lon_0) / sqrt(1+t*t));

       const double N0 = (lat > 0 ? N0_n : N0_s);

       const double E = E0 + k0 * A * (eta + a1*cos(2*1*xi)*sinh(2*1*eta) + a2*cos(2*2*xi)*sinh(2*2*eta) + a3*cos(2*3*xi)*sinh(2*3*eta) + a4*cos(2*4*xi)*sinh(2*4*eta) + a5*cos(2*5*xi)*sinh(2*5*eta) + a6*cos(2*6*xi)*sinh(2*6*eta) + a7*cos(2*7*xi)*sinh(2*7*eta) + a8*cos(2*8*xi)*sinh(2*8*eta) + a9*cos(2*9*xi)*sinh(2*9*eta) + a10*cos(2*10*xi)*sinh(2*10*eta));
       const double N = N0 + k0 * A * (xi + a1*sin(2*1*xi)*cosh(2*1*eta) + a2*sin(2*2*xi)*cosh(2*2*eta) + a3*sin(2*3*xi)*cosh(2*3*eta) + a4*sin(2*4*xi)*cosh(2*4*eta) + a5*sin(2*5*xi)*cosh(2*5*eta) + a6*sin(2*6*xi)*cosh(2*6*eta) + a7*sin(2*7*xi)*cosh(2*7*eta) + a8*sin(2*8*xi)*cosh(2*8*eta) + a9*sin(2*9*xi)*cosh(2*9*eta) + a10*sin(2*10*xi)*cosh(2*10*eta));

       // Scale E,N from kilometers to meters
       return Vec3(E * 1000, N * 1000, alt);
 }


}  // namespace colmap
