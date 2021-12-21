package com._604robotics.quixsam.odometry;

import com._604robotics.robotnik.swerve.QuixSwerveModuleState;

import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class SwerveDriveOdometryMeasurement {
  private final Rotation2d gyroAngle;
  private final QuixSwerveModuleState[] moduleStates;

  private final double sigmaX;
  private final double sigmaY;
  private final Rotation2d sigmaTheta;

  public SwerveDriveOdometryMeasurement(
      Rotation2d gyroAngle,
      double sigmaX,
      double sigmaY,
      Rotation2d sigmaTheta,
      QuixSwerveModuleState... moduleStates) {
    this.gyroAngle = gyroAngle;
    this.moduleStates = moduleStates;
    this.sigmaX = sigmaX;
    this.sigmaY = sigmaY;
    this.sigmaTheta = sigmaTheta;
  }

  public Rotation2d getGyroAngle() {
    return gyroAngle;
  }

  public QuixSwerveModuleState[] getModuleStates() {
    return moduleStates;
  }

  public double getSigmaX() {
    return sigmaX;
  }

  public double getSigmaY() {
    return sigmaY;
  }

  public Rotation2d getSigmaTheta() {
    return sigmaTheta;
  }
}
