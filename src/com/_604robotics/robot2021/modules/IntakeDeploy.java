package com._604robotics.robot2021.modules;

import com._604robotics.robot2021.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class IntakeDeploy extends Module {
  private DoubleSolenoid solenoid;

  public Output<Boolean> isDeployed;

  public IntakeDeploy() {
    super(IntakeDeploy.class);

    solenoid = new DoubleSolenoid(Ports.INTAKE_DEPLOY_A, Ports.INTAKE_DEPLOY_B);

    isDeployed = addOutput("Deployed", () -> deploy.isRunning());

    setDefaultAction(retract);
  }

  public class Retract extends Action {
    public Retract() {
      super(IntakeDeploy.this, Retract.class);
    }

    @Override
    public void run() {
      solenoid.set(Value.kForward);
    }
  }

  public class Deploy extends Action {
    public Deploy() {
      super(IntakeDeploy.this, Deploy.class);
    }

    @Override
    public void run() {
      solenoid.set(Value.kReverse);
    }
  }

  public Retract retract = new Retract();
  public Deploy deploy = new Deploy();
}
