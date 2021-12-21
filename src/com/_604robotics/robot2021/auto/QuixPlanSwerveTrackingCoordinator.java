package com._604robotics.robot2021.auto;

import com._604robotics.robot2021.constants.Calibration;
import com._604robotics.robot2021.modules.Swerve;
import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.prefabs.auto.QuikPlanSwerveReader;
import com._604robotics.robotnik.prefabs.auto.QuikPlanSwerveReader.TrajectoryState;
import com._604robotics.robotnik.prefabs.auto.SwerveTrackerConstants;
import com._604robotics.robotnik.prefabs.auto.FalconDashboard;
import com._604robotics.robotnik.prefabs.auto.TrackerConstants;
import com._604robotics.robotnik.prefabs.swerve.QuixHolonomicDriveController;
import com._604robotics.robotnik.prefabs.swerve.QuixSwerveModuleState;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpiutil.math.numbers.N2;
import java.util.List;

public class QuixPlanSwerveTrackingCoordinator extends Coordinator {
  private final Timer timer = new Timer();
  private QuikPlanSwerveReader reader;
  private Swerve swerve;
  private final QuixHolonomicDriveController controller;
  private double totalTime;
  private Field2d field = new Field2d();

  private double globalCurrentTime = 0.0;

  public QuixPlanSwerveTrackingCoordinator(
      QuikPlanSwerveReader reader, Swerve swerve, SwerveTrackerConstants constants) {
    this.reader = reader;
    this.swerve = swerve;

    controller = new QuixHolonomicDriveController(constants.xController, constants.yController, constants.thetaController);
  }

  @Override
  public void begin() {
    SmartDashboard.putData("field", field);

    List<Double> initialState = reader.getState(0.0);
    swerve.zeroOdometry(
        new Pose2d(initialState.get(0), initialState.get(1), new Rotation2d(initialState.get(2))));

    FalconDashboard.getInstance().publishRobotPose(swerve.getPose());
    FalconDashboard.getInstance()
        .publishPathPose(
            new Pose2d(
                initialState.get(0), initialState.get(1), new Rotation2d(initialState.get(2))));

    totalTime = reader.getTotalTime();
    globalCurrentTime = 0.0;
    timer.reset();
    timer.start();
  }

  @Override
  public boolean run() {
    double curTime = timer.get();
    globalCurrentTime = curTime;

    if (timer.hasElapsed(totalTime)) {
      return false;
    } else {
      List<Double> state = reader.getState(curTime);

      var targetChassisSpeeds = controller.calculate(
        swerve.getPose(),
        new Pose2d(
          state.get(TrajectoryState.x.index),
          state.get(TrajectoryState.y.index),
          new Rotation2d(state.get(TrajectoryState.Theta.index))
        ),
        state.get(TrajectoryState.dx.index),
        state.get(TrajectoryState.dy.index),
        state.get(TrajectoryState.dTheta.index)
      );

      var targetModuleStates = swerve.kinematics.toSwerveModuleStates(targetChassisSpeeds);
      useOutput(targetModuleStates);

      FalconDashboard.getInstance().publishRobotPose(swerve.getPose());
      FalconDashboard.getInstance().publishPathPose(new Pose2d(state.get(0), state.get(1), new Rotation2d(state.get(2))));
      return true;
    }
  }

  @Override
  public void end() {
    stop();
  }

  public boolean doShoot() {
    return reader.doShoot(globalCurrentTime);
  }

  public void useOutput(QuixSwerveModuleState[] moduleStates) {}

  public void stop() {}
}