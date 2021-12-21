package com._604robotics.robot2021.auto.macros;

import com._604robotics.robot2021.robot2021;
import com._604robotics.robotnik.prefabs.coordinators.ParallelCoordinator;
import com._604robotics.robotnik.prefabs.coordinators.StatefulCoordinator;

public class CombinedFeedMacro extends ParallelCoordinator {

  public CombinedFeedMacro(robot2021 robot) {
    super(CombinedFeedMacro.class);

    addCoordinators(
        new AutoAimMacro(robot.drive, robot.limelight),
        new FeedMacro(robot.revolver, robot.tower, robot.antiJamRoller),
        new ShooterControlMacro(robot.shooter)
    );
  }
}
