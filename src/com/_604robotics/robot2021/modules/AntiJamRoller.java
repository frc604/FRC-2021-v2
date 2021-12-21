package com._604robotics.robot2021.modules;

import com._604robotics.robot2021.constants.Calibration;
import com._604robotics.robot2021.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.prefabs.motorcontrol.QuixTalonSRX;

public class AntiJamRoller extends Module {
  public final QuixTalonSRX antiJamMotor =
      new QuixTalonSRX(Ports.ROLLER_MOTOR, "Anti Jam Roller Motor", this);

  public AntiJamRoller() {
    super(AntiJamRoller.class);

    setDefaultAction(idle);
  }

  public class Idle extends Action {
    public Idle() {
      super(AntiJamRoller.this, Idle.class);
    }

    @Override
    public void run() {
      antiJamMotor.stopMotor();
    }
  }

  public class AntiJam extends Action {
    public AntiJam() {
      super(AntiJamRoller.this, AntiJam.class);
    }

    @Override
    public void run() {
      antiJamMotor.set(Calibration.AntiJamRoller.ANTI_JAM_SPEED);
    }
  }

  public class Reverse extends Action {
    public Reverse() {
      super(AntiJamRoller.this, Reverse.class);
    }

    @Override
    public void run() {
      antiJamMotor.set(Calibration.AntiJamRoller.REVERSE_SPEED);
    }
  }

  public final Idle idle = new Idle();
  public final AntiJam antiJam = new AntiJam();
  public final Reverse reverse = new Reverse();
}
