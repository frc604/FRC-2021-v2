package com._604robotics.robot2021.auto.macros;

import com._604robotics.robot2021.robot2021;
import com._604robotics.robotnik.prefabs.coordinators.ParallelRaceCoordinator;

public class ShootSequence extends ParallelRaceCoordinator {
  public ShootSequence(robot2021 robot) {
    super(ShootSequence.class);

    addCoordinators(new ShooterControlMacro(robot.shooter), new CombinedFeedMacro(robot));
  }
}
