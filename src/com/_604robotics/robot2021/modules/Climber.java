package com._604robotics.robot2021.modules;

import com._604robotics.robot2021.constants.Calibration;
import com._604robotics.robot2021.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;
import com._604robotics.robotnik.prefabs.devices.NEOEncoder;
import com._604robotics.robotnik.prefabs.motorcontrol.Motor;
import com._604robotics.robotnik.prefabs.motorcontrol.QuixSparkMAX;
import com._604robotics.robotnik.prefabs.motorcontrol.controllers.MotorControllerPIDConfig;
import com._604robotics.robotnik.prefabs.motorcontrol.controllers.SparkPID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

public class Climber extends Module {
  private QuixSparkMAX climbMotor = new QuixSparkMAX(Ports.CLIMBER_MOTOR, "Climber Motor", Motor.kNEO, this);

  private NEOEncoder climbEncoder = new NEOEncoder(climbMotor);
  private SparkPID climbPID = new SparkPID(climbMotor, climbEncoder, new MotorControllerPIDConfig(0.03, 0, 0));

  public final Output<Double> climbPosition = addOutput("Climb Position", () -> climbEncoder.getPosition());

  public Climber() {
    super(Climber.class);
    climbMotor.controller.getEncoder().setPosition(0.0);

    climbMotor.controller.setIdleMode(IdleMode.kBrake);
    climbMotor.controller.setSoftLimit(SoftLimitDirection.kForward, 365);

    climbMotor.controller.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);
    climbMotor.controller.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);

    climbMotor.setCurrentLimit(60);

    setDefaultAction(idle);
  }

  public class Idle extends Action {

    public Idle() {
      super(Climber.this, Idle.class);
    }

    @Override
    public void run() {
      climbMotor.set(0.0);
    }
  }

  public class Extend extends Action {

    public Extend() {
      super(Climber.this, Extend.class);
    }

    @Override
    public void begin() {
        climbPID.setSetpointPosition(climbEncoder.getPosition() + 160);
    }
  }

  public class Retract extends Action {

    public Retract() {
      super(Climber.this, Retract.class);
    }

    @Override
    public void run() {
      climbMotor.set(-0.75);
    }
  }

  public class Climb extends Action {

    public Climb() {
      super(Climber.this, Climb.class);
    }

    @Override
    public void run() {
      
      climbMotor.set(Calibration.Climber.CLIMB_SPEED);
    }
  }

  public Action idle = new Idle();
  public Action extend = new Extend();
  public Action retract = new Retract();
  public Action climb = new Climb();
}