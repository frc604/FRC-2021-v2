package com._604robotics.robot2021.auto.macros;

import com._604robotics.robot2021.modules.AntiJamRoller;
import com._604robotics.robot2021.modules.Revolver;
import com._604robotics.robot2021.modules.Swerve;
import com._604robotics.robot2021.modules.Tower;
import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.prefabs.controller.ProfiledPIDController;
import com._604robotics.robotnik.prefabs.vision.Limelight;

import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

public class AutoAimMacro extends Coordinator {
  private Swerve drive;
  private Limelight limelight;

  private Swerve.AutoAngle autoAngle;

  private ProfiledPIDController snappingController;


  public AutoAimMacro(Swerve drive, Limelight limelight) {
    this.drive = drive;
    this.limelight = limelight;

    this.autoAngle = drive.new AutoAngle();

    this.snappingController = new ProfiledPIDController(
      0.075, 0.0, 0.0,
      new TrapezoidProfile.Constraints(720.0, 360.0),
      drive::getRawHeadingDegrees,
      (value) -> autoAngle.desiredAngularVel.set(value)
    );

    snappingController.setOutputRange(-1000, 1000);
  }

  @Override
  public void begin() {
    snappingController.setInitialState(new TrapezoidProfile.State(drive.getRawHeadingDegrees(), drive.getAngularVelDegrees()));
    snappingController.enable();
  }

  @Override
  public boolean run() {
    double setpoint = -limelight.getLatestMeasurement().getBestTarget().getYaw() + drive.getRawHeadingDegrees();

    // System.out.println("Setpoint:" + -robot.limelight.getLatestMeasurement().getBestTarget().getYaw());
    // System.out.println("Current:" + robot.drive.getRawHeadingDegrees());

    snappingController.setGoal(setpoint);
    autoAngle.activate();
    return true;
  }
}
