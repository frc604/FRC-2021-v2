package com._604robotics.robot2021.modules;

import com._604robotics.robot2021.constants.Calibration;
import com._604robotics.robot2021.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Input;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.prefabs.motorcontrol.QuixTalonSRX;

public class Tower extends Module {
  public final QuixTalonSRX towerMotor = new QuixTalonSRX(Ports.TOWER_MOTOR, "Tower Motor", this);

  public Tower() {
    super(Tower.class);

    setDefaultAction(idle);
  }

  public class Idle extends Action {
    public Idle() {
      super(Tower.this, Idle.class);
    }

    @Override
    public void run() {
      towerMotor.stopMotor();
    }
  }

  public class Empty extends Action {
    public Empty() {
      super(Tower.this, Empty.class);
    }

    @Override
    public void run() {
      towerMotor.set(Calibration.Tower.EMPTY_SPEED);
    }
  }

  public class AntiJam extends Action {
    public AntiJam() {
      super(Tower.this, AntiJam.class);
    }

    @Override
    public void run() {
      towerMotor.set(Calibration.Tower.ANTI_JAM_SPEED);
    }
  }

  public class Speed extends Action {
    public Input<Double> power;

    private Speed() {
      super(Tower.this, Speed.class);
      power = addInput("Power", 0.0, true);
    }

    @Override
    public void run() {
      towerMotor.set(power.get());
    }
  }

  public final Idle idle = new Idle();
  public final Empty empty = new Empty();
  public final Intake intake = new Intake();
  public final AntiJam antiJam = new AntiJam();
  public final Speed speed = new Speed();
}
