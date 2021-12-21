package com._604robotics.robot2021.modules;

import com._604robotics.robot2021.constants.Calibration;
import com._604robotics.robot2021.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Input;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.prefabs.motorcontrol.QuixTalonSRX;

public class Revolver extends Module {
  public final QuixTalonSRX revolverMotor =
      new QuixTalonSRX(Ports.REVOLVER_MOTOR, "Revolver Motor", this);

  public Revolver() {
    super(Revolver.class);

    setDefaultAction(idle);
  }

  public class Idle extends Action {
    public Idle() {
      super(Revolver.this, Idle.class);
    }

    @Override
    public void run() {
      revolverMotor.stopMotor();
    }
  }

  public class Empty extends Action {
    public Empty() {
      super(Revolver.this, Empty.class);
    }

    @Override
    public void run() {
      revolverMotor.set(Calibration.Revolver.EMPTY_SPEED);
    }
  }

  public class Intake extends Action {
    public Intake() {
      super(Revolver.this, Intake.class);
    }

    @Override
    public void run() {
      revolverMotor.set(Calibration.Revolver.INTAKE_SPEED);
    }
  }

  public class Reverse extends Action {
    public Reverse() {
      super(Revolver.this, Reverse.class);
    }

    @Override
    public void run() {
      revolverMotor.set(Calibration.Intake.REVERSE_SPEED);
    }
  }

  public class Speed extends Action {
    public Input<Double> power;

    private Speed() {
      super(Revolver.this, Speed.class);
      power = addInput("Power", 0.0, true);
    }

    @Override
    public void run() {
      revolverMotor.set(power.get());
    }
  }

  public final Idle idle = new Idle();
  public final Empty empty = new Empty();
  public final Intake intake = new Intake();
  public final Reverse reverse = new Reverse();
  public final Speed speed = new Speed();
}
