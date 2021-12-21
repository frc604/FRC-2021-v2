package com._604robotics.robot2021.modes;

import com._604robotics.quixsam.mathematics.Interpolatable;
import com._604robotics.robot2021.robot2021;
import com._604robotics.robot2021.constants.Calibration;
import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.Logger;
import com._604robotics.robotnik.prefabs.auto.FalconDashboard;
import com._604robotics.robotnik.prefabs.auto.QuikPlanLive;
import com._604robotics.robotnik.prefabs.vision.VisionCamera.GamePiece;
import com._604robotics.robotnik.prefabs.vision.VisionManager;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.Pair;
import java.util.List;

public class DisabledMode extends Coordinator {
  private static final Logger logger = new Logger(DisabledMode.class);

  private final com._604robotics.robot2021.robot2021 robot;

  public QuikPlanLive livePlan;

  public DisabledMode(robot2021 robot) {
    this.robot = robot;
    this.livePlan = new QuikPlanLive();
  }

  @Override
  public void begin() {
    SmartDashboard.putBoolean("Load File", true);
  }

  @Override
  public boolean run() {
    robot.drive.updateOdometry();
    FalconDashboard.getInstance().publishRobotPose(robot.drive.getPose());
    

    // // Search Gamepieces
    // if (SmartDashboard.getBoolean("Enable Searching", false)) {
    //   VisionManager.getInstance().update();
    //   List<Translation2d> balls =
    //       robot.ballCam.searchGamePiece(robot.drive.getPose(), GamePiece.POWERCELL);
    //   String ballString = "";
    //   for (Translation2d ball : balls) {
    //     ballString = ballString + ball.toString() + " ";
    //   }
    //   SmartDashboard.putString("Balls", ballString);

    if (SmartDashboard.getBoolean("Load File", false)) {
      this.robot.autonomousMode.reader.loadChosenFile();
      SmartDashboard.putBoolean("Load File", false);
    }

    // if (SmartDashboard.getBoolean("Load Selected Path", false)) {
    //   robot.autonomousMode.getReader().loadChosenFile();
    //   SmartDashboard.putBoolean("Load Selected Path", false);
    // }

    return true;
  }

  @Override
  public void end() {}
}
