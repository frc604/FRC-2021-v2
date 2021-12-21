package com._604robotics.robot2021.auto.macros;

import com._604robotics.robot2021.modules.AntiJamRoller;
import com._604robotics.robot2021.modules.Intake;
import com._604robotics.robot2021.modules.IntakeDeploy;
import com._604robotics.robot2021.modules.Revolver;
import com._604robotics.robot2021.modules.Tower;
import com._604robotics.robotnik.Coordinator;

public class IntakeMacro extends Coordinator {
  private Intake.Suck intakeSuck;
  private IntakeDeploy.Deploy deploy;
  private Revolver.Intake revolverIntake;
  private Tower.AntiJam towerAntiJam;
  private AntiJamRoller.AntiJam roller;

  public IntakeMacro(
      Intake intake,
      IntakeDeploy intakeDeploy,
      Revolver revolver,
      Tower tower,
      AntiJamRoller antiJam) {
    this.intakeSuck = intake.suck;
    this.deploy = intakeDeploy.deploy;
    this.revolverIntake = revolver.intake;
    this.towerAntiJam = tower.antiJam;
    this.roller = antiJam.antiJam;
  }

  @Override
  public boolean run() {
    intakeSuck.activate();
    deploy.activate();
    revolverIntake.activate();
    towerAntiJam.activate();
    roller.activate();
    return true;
  }
}
