package com._604robotics.robot2021.auto.macros;

import com._604robotics.robot2021.modules.Shooter;
import com._604robotics.robotnik.Coordinator;

public class ShooterControlMacro extends Coordinator {
  private Shooter.Setpoint setpoint;

  public ShooterControlMacro(Shooter shooter) {
    this.setpoint = shooter.setpoint;
  }

  @Override
  public boolean run() {
    setpoint.setpoint.set(600.0);
    setpoint.activate();
    return true;
  }
}
