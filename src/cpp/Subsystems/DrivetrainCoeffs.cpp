#include "Subsystems/DrivetrainCoeffs.h"

#include <Eigen/Core>

frc::StateSpacePlantCoeffs<4, 2, 2> MakeDrivetrainPlantCoeffs() {
  Eigen::Matrix<double, 4, 4> A;
  A(0, 0) = 1.0;
  A(0, 1) = 0.004998362738969779;
  A(0, 2) = 0.0;
  A(0, 3) = 2.5577150383259955e-05;
  A(1, 0) = 0.0;
  A(1, 1) = 0.9796368194189216;
  A(1, 2) = 0.0;
  A(1, 3) = 0.010060128568752168;
  A(2, 0) = 0.0;
  A(2, 1) = 2.5577150383259948e-05;
  A(2, 2) = 1.0;
  A(2, 3) = 0.004998362738969779;
  A(3, 0) = 0.0;
  A(3, 1) = 0.010060128568752167;
  A(3, 2) = 0.0;
  A(3, 3) = 0.9796368194189216;
  Eigen::Matrix<double, 4, 4> Ainv;
  Ainv(0, 0) = 1.0;
  Ainv(0, 1) = -0.005102530982811772;
  Ainv(0, 2) = 0.0;
  Ainv(0, 3) = 2.6290321902299037e-05;
  Ainv(1, 0) = 0.0;
  Ainv(1, 1) = 1.0208941197900505;
  Ainv(1, 2) = 0.0;
  Ainv(1, 3) = -0.010483809812561865;
  Ainv(2, 0) = 0.0;
  Ainv(2, 1) = 2.6290321902299044e-05;
  Ainv(2, 2) = 1.0;
  Ainv(2, 3) = -0.005102530982811774;
  Ainv(3, 0) = 0.0;
  Ainv(3, 1) = -0.010483809812561865;
  Ainv(3, 2) = 0.0;
  Ainv(3, 3) = 1.0208941197900507;
  Eigen::Matrix<double, 4, 2> B;
  B(0, 0) = 2.784095299156685e-05;
  B(0, 1) = -1.379027909830132e-05;
  B(1, 0) = 0.010979094204566662;
  B(1, 1) = -0.005424059312670187;
  B(2, 0) = -1.3790279098301316e-05;
  B(2, 1) = 2.784095299156685e-05;
  B(3, 0) = -0.005424059312670186;
  B(3, 1) = 0.010979094204566662;
  Eigen::Matrix<double, 2, 4> C;
  C(0, 0) = 1;
  C(0, 1) = 0;
  C(0, 2) = 0;
  C(0, 3) = 0;
  C(1, 0) = 0;
  C(1, 1) = 0;
  C(1, 2) = 1;
  C(1, 3) = 0;
  Eigen::Matrix<double, 2, 2> D;
  D(0, 0) = 0;
  D(0, 1) = 0;
  D(1, 0) = 0;
  D(1, 1) = 0;
  return frc::StateSpacePlantCoeffs<4, 2, 2>(A, Ainv, B, C, D);
}

frc::StateSpaceControllerCoeffs<4, 2, 2> MakeDrivetrainControllerCoeffs() {
  Eigen::Matrix<double, 2, 4> K;
  K(0, 0) = 79.5163618412469;
  K(0, 1) = 13.470787766058692;
  K(0, 2) = 2.458843244748564;
  K(0, 3) = 1.9406067584031343;
  K(1, 0) = 2.4588432447491027;
  K(1, 1) = 1.9406067584031457;
  K(1, 2) = 79.51636184124824;
  K(1, 3) = 13.470787766058724;
  Eigen::Matrix<double, 2, 4> Kff;
  Kff(0, 0) = 154.56837009827595;
  Kff(0, 1) = 1.523894583174555;
  Kff(0, 2) = -74.0856431675135;
  Kff(0, 3) = -0.7284089721433848;
  Kff(1, 0) = -74.08564316751352;
  Kff(1, 1) = -0.7284089721433852;
  Kff(1, 2) = 154.56837009827598;
  Kff(1, 3) = 1.5238945831745552;
  Eigen::Matrix<double, 2, 1> Umin;
  Umin(0, 0) = -12.0;
  Umin(1, 0) = -12.0;
  Eigen::Matrix<double, 2, 1> Umax;
  Umax(0, 0) = 12.0;
  Umax(1, 0) = 12.0;
  return frc::StateSpaceControllerCoeffs<4, 2, 2>(K, Kff, Umin, Umax);
}

frc::StateSpaceObserverCoeffs<4, 2, 2> MakeDrivetrainObserverCoeffs() {
  Eigen::Matrix<double, 4, 2> L;
  L(0, 0) = 1.077982621173968;
  L(0, 1) = 0.0082169364459126;
  L(1, 0) = 15.29255023574382;
  L(1, 1) = 1.6891523558585273;
  L(2, 0) = 0.00821693644591232;
  L(2, 1) = 1.0779826211739687;
  L(3, 0) = 1.6891523558584716;
  L(3, 1) = 15.292550235744002;
  return frc::StateSpaceObserverCoeffs<4, 2, 2>(L);
}

frc::StateSpaceLoop<4, 2, 2> MakeDrivetrainLoop() {
  return frc::StateSpaceLoop<4, 2, 2>(MakeDrivetrainPlantCoeffs(),
                                      MakeDrivetrainControllerCoeffs(),
                                      MakeDrivetrainObserverCoeffs());
}
