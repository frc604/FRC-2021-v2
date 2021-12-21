package com._604robotics.robot2021.auto.macros;

import java.util.function.BooleanSupplier;

import com._604robotics.robot2021.modules.AntiJamRoller;
import com._604robotics.robot2021.modules.Intake;
import com._604robotics.robot2021.modules.IntakeDeploy;
import com._604robotics.robot2021.modules.Revolver;
import com._604robotics.robot2021.modules.Tower;
import com._604robotics.robotnik.Coordinator;

import edu.wpi.first.wpilibj.Timer;


public class FeedOTFMacro extends Coordinator {
  private Revolver.Empty revolverEmpty;
  private Tower.Empty towerEmpty;
  private AntiJamRoller.AntiJam roller;

  private Intake.Suck intakeSuck;
  private IntakeDeploy.Deploy deploy;

  private BooleanSupplier doShoot;

  private final Timer timer = new Timer();

  public FeedOTFMacro(
    Intake intake,
    IntakeDeploy intakeDeploy,
    Revolver revolver,
    Tower tower,
    AntiJamRoller antiJam,
    BooleanSupplier doShoot) {
    this.revolverEmpty = revolver.empty;
    this.towerEmpty = tower.empty;
    this.roller = antiJam.antiJam;
    this.intakeSuck = intake.suck;
    this.deploy = intakeDeploy.deploy;
    this.doShoot = doShoot;
  }

  @Override
  public void begin() {
    timer.reset();
    timer.start();
  }

  @Override
  public boolean run() {
    if (doShoot.getAsBoolean() && timer.hasElapsed(2)) {
      revolverEmpty.activate();
      towerEmpty.activate();
      roller.activate();
      deploy.activate();
      intakeSuck.activate();
    } else {
      intakeSuck.activate();
      deploy.activate();
      revolverEmpty.activate();
      roller.activate();
    }
    return true;
  }
}
