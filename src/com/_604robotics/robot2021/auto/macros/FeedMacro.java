package com._604robotics.robot2021.auto.macros;

import com._604robotics.robot2021.modules.AntiJamRoller;
import com._604robotics.robot2021.modules.Revolver;
import com._604robotics.robot2021.modules.Tower;
import com._604robotics.robotnik.Coordinator;

public class FeedMacro extends Coordinator {
  private Revolver.Empty revolverEmpty;
  private Tower.Empty towerEmpty;
  private AntiJamRoller.AntiJam roller;

  public FeedMacro(Revolver revolver, Tower tower, AntiJamRoller antiJam) {
    this.revolverEmpty = revolver.empty;
    this.towerEmpty = tower.empty;
    this.roller = antiJam.antiJam;
  }

  @Override
  public boolean run() {
    revolverEmpty.activate();
    towerEmpty.activate();
    roller.activate();
    return true;
  }
}
