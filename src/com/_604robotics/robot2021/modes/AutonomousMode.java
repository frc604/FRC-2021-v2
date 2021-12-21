package com._604robotics.robot2021.modes;

import static com._604robotics.robot2021.constants.Calibration.Auto.KA_VOLT_SECONDS_SQUARED_PER_METER;
import static com._604robotics.robot2021.constants.Calibration.Auto.KS_VOLTS;
import static com._604robotics.robot2021.constants.Calibration.Auto.KV_VOLT_SECONDS_PER_METER;
import static com._604robotics.robot2021.constants.Calibration.Auto.MAX_CENTRIPETAL_ACCELERATION_RADIANS_PER_SECOND_SQUARED;
import static com._604robotics.robot2021.constants.Calibration.Auto.MAX_SPEED_METERS_PER_SECOND;

import com._604robotics.robot2021.robot2021;
import com._604robotics.robot2021.auto.paths.QuikPlanPreGen;
import com._604robotics.robot2021.constants.Calibration;
import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.DashboardManager;
import com._604robotics.robotnik.Logger;
import com._604robotics.robotnik.Output;
import com._604robotics.robotnik.prefabs.auto.QuikPlanReader;
import com._604robotics.robotnik.prefabs.auto.QuikPlanSwerveReader;
import com._604robotics.robotnik.prefabs.auto.TrajectoryCreator;
import com._604robotics.robotnik.prefabs.auto.angular.AngularMotionConstraint;
import com._604robotics.robotnik.prefabs.auto.angular.TurnInPlaceTrajectory;
import com._604robotics.robotnik.prefabs.coordinators.StatefulCoordinator;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import java.util.List;

public class AutonomousMode extends Coordinator {
  private static final Logger logger = new Logger(AutonomousMode.class);

  private final com._604robotics.robot2021.robot2021 robot;

  private Coordinator selectedModeMacro;

  public QuikPlanSwerveReader reader;

  public final Output<AutonMode> autonMode;

  public AutonomousMode(robot2021 robot) {
    this.robot = robot;
    System.out.println("ONCE");

    autonMode =
        DashboardManager.getInstance()
            .registerEnumOutput(
                "Auton Mode Chooser", AutonMode.QUIX_PLAN_PREGEN, AutonMode.class, robot.drive);

    // trajectoryCreator =
    //     new TrajectoryCreator(
    //         robot.drive.driveKinematics,
    //         Calibration.Auto.TRACKER_CONSTANTS,
    //         new DifferentialDriveKinematicsConstraint(
    //             robot.drive.driveKinematics, MAX_SPEED_METERS_PER_SECOND),
    //         new DifferentialDriveVoltageConstraint(
    //             new SimpleMotorFeedforward(
    //                 KS_VOLTS, KV_VOLT_SECONDS_PER_METER, KA_VOLT_SECONDS_SQUARED_PER_METER),
    //             robot.drive.driveKinematics,
    //             10),
    //         new CentripetalAccelerationConstraint(
    //             MAX_CENTRIPETAL_ACCELERATION_RADIANS_PER_SECOND_SQUARED));

    reader = new QuikPlanSwerveReader(robot.drive);
  }

  @Override
  public void begin() {
    System.out.println(autonMode.get());

    reader.loadChosenFile();

    robot.drive.zeroGyroOffset();

    switch (autonMode.get()) {
      case MANUAL:
        selectedModeMacro = robot.teleopMode;
      case QUIX_PLAN_PREGEN:
        selectedModeMacro = new QuikPlanPreGen(robot, reader);
        break;
      case OFF:
      default:
        selectedModeMacro = null;
        break;
    }

    System.out.println(selectedModeMacro);
    // robot.drive.disableMotorSafety();

    if (selectedModeMacro != null) {
      selectedModeMacro.start();
    }
  }

  @Override
  public boolean run() {
    robot.drive.updateOdometry();

    if (selectedModeMacro == null) {
      return false;
    }
    return selectedModeMacro.execute();
  }

  @Override
  public void end() {
    // robot.drive.enableMotorSafety();
    // robot.drive.setIdleMode(IdleMode.kCoast);
    if (selectedModeMacro != null) {
      selectedModeMacro.stop();
    }
  }

  // public QuikPlanReader getReader() {
  //   return reader;
  // }

  // private class FallBackMacro extends StatefulCoordinator {
  //   public FallBackMacro() {
  //     super(FallBackMacro.class);

  //     addState("Pathfind back 144in", new PathReverse(Units.inchesToMeters(36)));
  //   }
  // }

  // private class FallForwardMacro extends StatefulCoordinator {
  //   public FallForwardMacro() {
  //     super(FallForwardMacro.class);

  //     addState("Pathfind forward 144in", new PathStraight(Units.inchesToMeters(36)));
  //   }
  // }

  // private class PathStraight extends SparkTrajectoryTracker {

  //   public PathStraight(double meters) {
  //     super(
  //         trajectoryCreator.getTrajectory(
  //             List.of(new Pose2d(), new Pose2d(meters, 0, new Rotation2d(0))), false),
  //         robot.drive,
  //         Calibration.Auto.TRACKER_CONSTANTS);
  //   }
  // }

  // private class PathReverse extends SparkTrajectoryTracker {

  //   public PathReverse(double meters) {
  //     super(
  //         trajectoryCreator.getTrajectory(
  //             List.of(new Pose2d(), new Pose2d(-meters, 0, new Rotation2d(0))), true),
  //         robot.drive,
  //         Calibration.Auto.TRACKER_CONSTANTS);
  //   }
  // }

  // private class TurnInPlaceLeft extends SparkAngularTrajectoryTracker {

  //   public TurnInPlaceLeft() {
  //     super(
  //         new TurnInPlaceTrajectory(
  //             new Pose2d(), Rotation2d.fromDegrees(-360.0), new AngularMotionConstraint(2, 1)),
  //         robot.drive,
  //         Calibration.Auto.TRACKER_CONSTANTS);
  //   }
  // }

  public enum AutonMode {
    OFF,
    // Following are actual strategy selections
    MANUAL,
    TURN_IN_PLACE_LEFT,

    // Calibration autons to verify angles and distances
    FAILSAFE_FORWARD_12,
    FAILSAFE_BACKWARD_12,

    // 10 ball autonomous
    // FULL_FIELD_10_BALL,

    // IFR @ Home
    QUIX_PLAN_PREGEN,
    GALATIC_SEARCH,
  }
}
