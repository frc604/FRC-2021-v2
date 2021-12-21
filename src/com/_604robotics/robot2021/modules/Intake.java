package com._604robotics.robot2021.modules;

import com._604robotics.robot2021.constants.Calibration;
import com._604robotics.robot2021.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Input;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.prefabs.motorcontrol.QuixTalonSRX;

public class Intake extends Module {
  public final QuixTalonSRX intakeMotor =
      new QuixTalonSRX(Ports.INTAKE_MOTOR, "Intake Motor", this);

  public Intake() {
    super(Intake.class);

    setDefaultAction(idle);
  }

  public class Idle extends Action {
    public Idle() {
      super(Intake.this, Idle.class);
    }

    @Override
    public void run() {
      intakeMotor.stopMotor();
    }
  }

  public class Suck extends Action {
    public Suck() {
      super(Intake.this, Suck.class);
    }

    @Override
    public void run() {
      intakeMotor.set(Calibration.Intake.INTAKE_SPEED);
    }
  }

  public class Reverse extends Action {
    public Reverse() {
      super(Intake.this, Reverse.class);
    }

    @Override
    public void run() {
      intakeMotor.set(Calibration.Intake.REVERSE_SPEED);
    }
  }

  public class Speed extends Action {
    public Input<Double> power;

    private Speed() {
      super(Intake.this, Speed.class);
      power = addInput("Power", 0.0, true);
    }

    @Override
    public void run() {
      intakeMotor.set(power.get());
    }
  }

  public final Idle idle = new Idle();
  public final Suck suck = new Suck();
  public final Reverse reverse = new Reverse();
  public final Speed speed = new Speed();
}
