package com._604robotics.robot2021.modules;

import com._604robotics.robot2021.constants.Calibration;
import com._604robotics.robot2021.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Input;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;
import com._604robotics.robotnik.prefabs.devices.FalconEncoder;
import com._604robotics.robotnik.prefabs.motorcontrol.Motor;
import com._604robotics.robotnik.prefabs.motorcontrol.QuixTalonFX;
import com._604robotics.robotnik.prefabs.motorcontrol.controllers.MotorControllerPIDConfig;
import com._604robotics.robotnik.prefabs.motorcontrol.controllers.TalonPID;
import com._604robotics.robotnik.prefabs.motorcontrol.gearing.GearRatio;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

public class Shooter extends Module {
  public QuixTalonFX leftMotor;
  private QuixTalonFX rightMotor;

  private QuixTalonFX feederMotor;

  private TalonPID shooterPID;
  private TalonPID feederPID;

  private FalconEncoder leftEncoder;
  private FalconEncoder feederEncoder;

  private SimpleMotorFeedforward shooterFeedforward;
  private SimpleMotorFeedforward feederFeedforward;

  private GearRatio pulley;

  public Output<Double> pidError;
  public Output<Double> pidSetpoint;
  public Output<Double> encoderClicks;
  public Output<Double> encoderRate;
  public Output<Double> feederClicks;
  public Output<Double> feederRate;

  public Shooter() {
    super(Shooter.class);

    leftMotor = new QuixTalonFX(Ports.SHOOTER_LEFT_MOTOR, "Left Motor", Motor.kFalcon500, this);
    rightMotor = new QuixTalonFX(Ports.SHOOTER_RIGHT_MOTOR, "Right Motor", Motor.kFalcon500, this);

    feederMotor = new QuixTalonFX(Ports.SHOOTER_FEEDER_MOTOR, "Feeder Motor", Motor.kFalcon500, this);

    rightMotor.getController().follow(leftMotor.getController());
    rightMotor.getController().setInverted(InvertType.OpposeMaster);

    leftMotor.controller.setNeutralMode(NeutralMode.Coast);
    rightMotor.controller.setNeutralMode(NeutralMode.Coast);
    feederMotor.controller.setNeutralMode(NeutralMode.Coast);

    feederMotor.setInverted(true);
    leftMotor.setInverted(false);

    leftMotor.setVoltageCompSaturation(12, true);
    rightMotor.setVoltageCompSaturation(12, true);
    feederMotor.setVoltageCompSaturation(12, true);

    pulley = new GearRatio(Calibration.Shooter.DRIVING_TEETH, Calibration.Shooter.DRIVEN_TEETH);

    leftEncoder = new FalconEncoder(leftMotor, pulley);
    leftEncoder.setInverted(true);

    feederEncoder = new FalconEncoder(feederMotor, new GearRatio(1, 1));

    leftEncoder.setdistancePerRotation(-2 * Math.PI);
    feederEncoder.setdistancePerRotation(-2 * Math.PI);

    leftEncoder.zero();

    leftMotor.setCurrentLimit(40);
    leftMotor.enableCurrentLimit(true);

    rightMotor.setCurrentLimit(40);
    rightMotor.enableCurrentLimit(true);

    feederMotor.setCurrentLimit(40);
    feederMotor.enableCurrentLimit(true);

    shooterPID = new TalonPID(leftMotor, leftEncoder, new MotorControllerPIDConfig(Calibration.Shooter.kP, 0.0, Calibration.Shooter.kD));

    feederPID = new TalonPID(feederMotor, feederEncoder, new MotorControllerPIDConfig(Calibration.Feeder.kP, 0.0, Calibration.Feeder.kD));

    shooterFeedforward = Calibration.Shooter.feedforward;
    feederFeedforward = Calibration.Feeder.feedforward;

    encoderClicks = addOutput("Encoder Clicks", () -> leftEncoder.getPosition());
    encoderRate = addOutput("Encoder Rate", () -> leftEncoder.getVelocity());

    feederClicks = addOutput("Feeder Clicks", () -> feederEncoder.getPosition());
    feederRate = addOutput("Feeder Rate", () -> feederEncoder.getVelocity());

    setDefaultAction(stop);
  }

  public class Move extends Action {
    public Input<Double> power;

    private Move() {
      super(Shooter.this, Move.class);
      power = addInput("Power", 0.0, true);
    }

    @Override
    public void begin() {}

    @Override
    public void run() {
      leftMotor.set(power.get());
      // feederMotor.set(1.0);
    }

    @Override
    public void end() {}
  }

  public class Stop extends Action {

    private Stop() {
      super(Shooter.this, Stop.class);
    }

    @Override
    public void begin() {}

    @Override
    public void run() {
      leftMotor.stopMotor();
      feederMotor.stopMotor();

      leftMotor.set(0.0);
      feederMotor.set(0.0);
    }

    @Override
    public void end() {}
  }

  public class Setpoint extends Action {
    public Input<Double> setpoint;

    private Setpoint() {
      super(Shooter.this, Setpoint.class);
      setpoint = addInput("Setpoint", 0.0, true);
    }

    @Override
    public void begin() {}

    @Override
    public void run() {
      shooterPID.setSetpointVelocity(-(setpoint.get()+50), shooterFeedforward.calculate(setpoint.get()));
      feederPID.setSetpointVelocity(-setpoint.get(), feederFeedforward.calculate(setpoint.get()));
    }

    @Override
    public void end() {}
  }

  public Stop stop = new Stop();
  public Move move = new Move();
  public Setpoint setpoint = new Setpoint();
}