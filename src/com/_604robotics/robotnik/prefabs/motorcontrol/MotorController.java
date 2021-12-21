package com._604robotics.robotnik.prefabs.motorcontrol;

import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.prefabs.motorcontrol.wpilibj.SpeedController;

public abstract class MotorController implements SpeedController {

  protected final Motor motor;

  protected final Module module;

  protected int currentLimit = 0;

  protected boolean isLimiting = false;

  public MotorController(Motor motor, Module module) {
    this.motor = motor;
    this.module = module;
  }

  public MotorController(Module module) {
    this.motor = null;
    this.module = module;
  }

  public abstract void set(double power);

  public abstract double get();

  public abstract double getOutputVoltage();

  public abstract double getInputVoltage();

  public abstract double getOutputCurrent();

  public abstract void setInverted(boolean inverted);

  public abstract boolean getInverted();

  public abstract void disable();

  public abstract void stopMotor();

  public Motor getMotor() {
    return motor;
  }

  public void setCurrentLimit(int limit) {
    currentLimit = limit;
  }

  public void enableCurrentLimit(boolean enable) {
    isLimiting = enable;
  }

  public int getCurrentLimit() {
    return currentLimit;
  }

  public boolean isCurrentLimiting() {
    return isLimiting;
  }

  public Module getModule() {
    return module;
  }
}
