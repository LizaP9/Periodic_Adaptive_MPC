/*! @file MiniCheetah.h
 *  @brief Utility function to build a Mini Cheetah Quadruped object
 *
 * This file is based on MiniCheetahFullRotorModel_mex.m and builds a model
 * of the Mini Cheetah robot.  The inertia parameters of all bodies are
 * determined from CAD.
 *
 */

#pragma once

#include "FloatingBaseModel.h"
#include "Quadruped.h"
#include "cppTypes.h"

/*!
 * Generate a Quadruped model of Mini Cheetah
 */
template<typename T>
Quadruped<T> buildMiniCheetah(RobotType robot_type)
{
  Quadruped<T> cheetah;
  cheetah._robotType = robot_type;

  switch (cheetah._robotType)
  {
    case RobotType::A1:
    {
      cheetah._bodyMass = 6.0;
      cheetah._bodyLength = 0.1805 * 2;
      cheetah._bodyWidth = 0.047 * 2;
      cheetah._bodyHeight = 0.05 * 2;
      cheetah._abadGearRatio = 1;
      cheetah._hipGearRatio = 1;
      cheetah._kneeGearRatio = 1;
      cheetah._abadLinkLength = 0.0838;
      cheetah._hipLinkLength = 0.2;
      cheetah._kneeLinkY_offset = 0.0;
      cheetah._kneeLinkLength = 0.2;
      cheetah._maxLegLength = 0.4;

      cheetah._motorTauMax = 3.f;
      cheetah._batteryV = 24;
      cheetah._motorKT = .05; // this is flux linkage * pole pairs
      cheetah._motorR = 0.173;
      cheetah._jointDamping = .01;
      cheetah._jointDryFriction = .2;

      // rotor inertia if the rotor is oriented so it spins around the z-axis
      Mat3<T> rotorRotationalInertiaZ;
      rotorRotationalInertiaZ << 33, 0, 0, 0, 33, 0, 0, 0, 63;
      rotorRotationalInertiaZ = 1e-6 * rotorRotationalInertiaZ;

      Mat3<T> RY = coordinateRotation<T>(CoordinateAxis::Y, M_PI / 2);
      Mat3<T> RX = coordinateRotation<T>(CoordinateAxis::X, M_PI / 2);
      Mat3<T> rotorRotationalInertiaX = RY * rotorRotationalInertiaZ * RY.transpose();
      Mat3<T> rotorRotationalInertiaY = RX * rotorRotationalInertiaZ * RX.transpose();

      // spatial inertias
      Mat3<T> abadRotationalInertia;
      abadRotationalInertia << 469, -9.4, -0.34, -9.4, 807, -0.47, -0.34, -0.47, 553;
      abadRotationalInertia = abadRotationalInertia * 1e-6;
      Vec3<T> abadCOM(-0.003311, 0.000635, 0.000031); // LEFT
      SpatialInertia<T> abadInertia(0.696, abadCOM, abadRotationalInertia);

      Mat3<T> hipRotationalInertia;
      hipRotationalInertia << 5529, 4.825, 343, 4.825, 5139, 22, 343, 22, 1367;
      hipRotationalInertia = hipRotationalInertia * 1e-6;
      Vec3<T> hipCOM(-0.003237, -0.022327, -0.027326);
      SpatialInertia<T> hipInertia(1.013, hipCOM, hipRotationalInertia);

      Mat3<T> kneeRotationalInertia, kneeRotationalInertiaRotated;
      kneeRotationalInertiaRotated << 2997, 0, -141, 0, 3014, 0, -141, 0, 32;
      kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6;
      kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();
      Vec3<T> kneeCOM(0.006435, 0, -0.107388);
      SpatialInertia<T> kneeInertia(0.166, kneeCOM, kneeRotationalInertia);

      Vec3<T> rotorCOM(0, 0, 0);
      SpatialInertia<T> rotorInertiaX(0.605, rotorCOM, rotorRotationalInertiaX);
      SpatialInertia<T> rotorInertiaY(0.605, rotorCOM, rotorRotationalInertiaY);

      Mat3<T> bodyRotationalInertia;
      bodyRotationalInertia << 15853, 0, 0, 0, 37799, 0, 0, 0, 45654;
      bodyRotationalInertia = bodyRotationalInertia * 1e-6;
      Vec3<T> bodyCOM(0, 0.0041, -0.0005);
      SpatialInertia<T> bodyInertia(cheetah._bodyMass, bodyCOM, bodyRotationalInertia);

      cheetah._abadInertia = abadInertia;
      cheetah._hipInertia = hipInertia;
      cheetah._kneeInertia = kneeInertia;
      cheetah._abadRotorInertia = rotorInertiaX;
      cheetah._hipRotorInertia = rotorInertiaY;
      cheetah._kneeRotorInertia = rotorInertiaY;
      cheetah._bodyInertia = bodyInertia;

      // locations
      cheetah._abadRotorLocation = Vec3<T>(0, 0, 0);
      cheetah._abadLocation = Vec3<T>(cheetah._bodyLength, cheetah._bodyWidth, 0) * 0.5;
      cheetah._hipLocation = Vec3<T>(0, cheetah._abadLinkLength, 0);
      cheetah._hipRotorLocation = Vec3<T>(0, 0.0, 0);
      cheetah._kneeLocation = Vec3<T>(0, 0, -cheetah._hipLinkLength);
      cheetah._kneeRotorLocation = Vec3<T>(0, 0, 0);

      break;
    }

    case RobotType::GO1:
    {
      cheetah._bodyMass = 5.204;
      cheetah._bodyLength = 0.3762;
      cheetah._bodyWidth = 0.0935;
      cheetah._bodyHeight = 0.114;
      cheetah._abadGearRatio = 1;
      cheetah._hipGearRatio = 1;
      cheetah._kneeGearRatio = 1;
      cheetah._abadLinkLength = 0.08;
      cheetah._hipLinkLength = 0.213;
      cheetah._kneeLinkY_offset = 0.0;
      cheetah._kneeLinkLength = 0.213;
      cheetah._maxLegLength = 0.4;

      cheetah._motorTauMax = 23.f;
      cheetah._batteryV = 24;
      cheetah._motorKT = .05; // this is flux linkage * pole pairs
      cheetah._motorR = 0.173;
      cheetah._jointDamping = .01;
      cheetah._jointDryFriction = .2;

      // rotor inertia if the rotor is oriented so it spins around the z-axis
      Mat3<T> rotorRotationalInertiaZ;
      rotorRotationalInertiaZ << 33, 0, 0, 0, 33, 0, 0, 0, 63;
      rotorRotationalInertiaZ = 1e-6 * rotorRotationalInertiaZ;

      Mat3<T> RY = coordinateRotation<T>(CoordinateAxis::Y, M_PI / 2);
      Mat3<T> RX = coordinateRotation<T>(CoordinateAxis::X, M_PI / 2);
      Mat3<T> rotorRotationalInertiaX = RY * rotorRotationalInertiaZ * RY.transpose();
      Mat3<T> rotorRotationalInertiaY = RX * rotorRotationalInertiaZ * RX.transpose();

      // spatial inertias
      Mat3<T> abadRotationalInertia;
      //xx xy xz  yx yy yz  zx zy zz
      float xx = 0.000374268192;
      float xy = 0.000036844422;
      float xz = -0.000000986754;
      float yx = xy;
      float yy = 0.000635923669;
      float yz = -0.000001172894;
      float zx = xz;
      float zy = yz;
      float zz = 0.000457647394;
      abadRotationalInertia << xx, xy, xz, yx, yy, yz, zx, zy, zz;
      Vec3<T> abadCOM(-0.00541, -0.00074, 0.000006); // LEFT
      SpatialInertia<T> abadInertia(0.591, abadCOM, abadRotationalInertia);

      Mat3<T> hipRotationalInertia;
      xx = 0.005851561134;
      xy = 0.000001783284;
      xz = 0.000328291374;
      yx = xy;
      yy = 0.005596155105;
      yz = 0.000021430713;
      zx = xz;
      zy = yz;
      zz = 0.00107157026;
      hipRotationalInertia << xx, xy, xz, yx, yy, yz, zx, zy, zz;
      Vec3<T> hipCOM(-0.003468, -0.018947, -0.032736);
      SpatialInertia<T> hipInertia(0.92, hipCOM, hipRotationalInertia);

      Mat3<T> kneeRotationalInertia, kneeRotationalInertiaRotated;
      xx = 0.002939186297;
      xy = 0.000001440899;
      xz = -0.000105359550;
      yx = xy;
      yy = 0.00295576935;
      yz = -0.000024397752;
      zx = xz;
      zy = yz;
      zz = 0.000030273372;
      kneeRotationalInertiaRotated << xx, xy, xz, yx, yy, yz, zx, zy, zz;
      kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();
      Vec3<T> kneeCOM(0.006286, 0.001307, -0.122269);
      SpatialInertia<T> kneeInertia(0.131, kneeCOM, kneeRotationalInertia);

      Vec3<T> rotorCOM(0, 0, 0);
      SpatialInertia<T> rotorInertiaX(0.055, rotorCOM, rotorRotationalInertiaX);
      SpatialInertia<T> rotorInertiaY(0.055, rotorCOM, rotorRotationalInertiaY);

      Mat3<T> bodyRotationalInertia;
      xx = 0.0168352186;
      xy = 0.0004636141;
      xz = 0.0002367952;
      yx = xy;
      yy = 0.0656071082;
      yz = 0.000036671;
      zx = xz;
      zy = yz;
      zz = 0.0742720659;
      bodyRotationalInertia << xx, xy, xz, yx, yy, yz, zx, zy, zz;
      Vec3<T> bodyCOM(0.0223, 0.002, -0.0005);
      SpatialInertia<T> bodyInertia(cheetah._bodyMass, bodyCOM, bodyRotationalInertia);

      cheetah._abadInertia = abadInertia;
      cheetah._hipInertia = hipInertia;
      cheetah._kneeInertia = kneeInertia;
      cheetah._abadRotorInertia = rotorInertiaX;
      cheetah._hipRotorInertia = rotorInertiaY;
      cheetah._kneeRotorInertia = rotorInertiaY;
      cheetah._bodyInertia = bodyInertia;

      // locations
      cheetah._abadRotorLocation = Vec3<T>(0, 0, 0);
      cheetah._abadLocation = Vec3<T>(cheetah._bodyLength, cheetah._bodyWidth, 0) * 0.5;
      cheetah._hipLocation = Vec3<T>(0, cheetah._abadLinkLength, 0);
      cheetah._hipRotorLocation = Vec3<T>(0, 0.0, 0);
      cheetah._kneeLocation = Vec3<T>(0, 0, -cheetah._hipLinkLength);
      cheetah._kneeRotorLocation = Vec3<T>(0, 0, 0);
      break;
    }
  }

  return cheetah;
}
