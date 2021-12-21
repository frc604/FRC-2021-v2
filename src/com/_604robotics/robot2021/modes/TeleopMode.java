package com._604robotics.robot2021.modes;

import com._604robotics.marionette.InputPlayer;
import com._604robotics.marionette.InputRecorder;
import com._604robotics.marionette.InputRecording;
import com._604robotics.marionette.MarionetteJoystick;
import com._604robotics.robot2021.constants.Calibration;
import com._604robotics.robot2021.modules.AntiJamRoller;
import com._604robotics.robot2021.modules.Intake;
import com._604robotics.robot2021.modules.IntakeDeploy;
import com._604robotics.robot2021.modules.Revolver;
import com._604robotics.robot2021.modules.Shooter;
import com._604robotics.robot2021.modules.Swerve;
import com._604robotics.robot2021.modules.Tower;
import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.Logger;
import com._604robotics.robotnik.prefabs.auto.FalconDashboard;
import com._604robotics.robotnik.prefabs.controller.ProfiledPIDController;
import com._604robotics.robotnik.prefabs.flow.SmartTimer;
import com._604robotics.robotnik.prefabs.flow.Toggle;
import com._604robotics.robotnik.prefabs.inputcontroller.xbox.XboxController;
import com._604robotics.robotnik.prefabs.motion.MotionConstraints;
import com._604robotics.robotnik.prefabs.motion.MotionState;
import com._604robotics.robotnik.prefabs.motion.TrapezoidalMotionProfile;
import com._604robotics.robotnik.prefabs.swerve.QuixSwerveModuleState;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpiutil.math.MathUtil;

public class TeleopMode extends Coordinator {

  private static final Logger logger = new Logger(TeleopMode.class);

  private final InputPlayer inputPlayer = new InputPlayer();
  private InputRecorder inputRecorder;

  private final MarionetteJoystick driverJoystick = new MarionetteJoystick(0, inputPlayer, 0);
  private final MarionetteJoystick manipJoystick = new MarionetteJoystick(1, inputPlayer, 1);

  private final XboxController driver = new XboxController(driverJoystick);
  private final XboxController manip = new XboxController(manipJoystick);

  private final com._604robotics.robot2021.robot2021 robot;

  private final DriveManager driveManager;
  private final IntakeManager intakeManager;
  private final ShooterManager shooterManager;
  // private final AutoCenterManager autoCenterManager;

  private boolean autoCentering = false;
  private boolean aligned = false;

  private boolean climbMode = false;

  private final Logger test = new Logger("Teleop");

  public TeleopMode(com._604robotics.robot2021.robot2021 robot) {
    driver.leftStick.x.setDeadband(Calibration.TELEOP_DRIVE_DEADBAND);
    driver.leftStick.y.setDeadband(Calibration.TELEOP_DRIVE_DEADBAND);

    driver.leftStick.x.setFactor(Calibration.TELEOP_FACTOR);
    driver.leftStick.y.setFactor(Calibration.TELEOP_FACTOR);

    driver.rightStick.x.setDeadband(Calibration.TELEOP_DRIVE_DEADBAND);
    driver.rightStick.y.setDeadband(Calibration.TELEOP_DRIVE_DEADBAND);

    // driver.rightStick.x.setFactor(Calibration.TELEOP_FACTOR);
    driver.rightStick.x.setFactor(1); // WEIRD_WHY_?FES:RLJTH *ROHT guirg
    driver.rightStick.y.setFactor(Calibration.TELEOP_FACTOR);

    manip.leftStick.x.setFactor(Calibration.TELEOP_FACTOR);
    manip.leftStick.y.setFactor(Calibration.TELEOP_FACTOR);

    manip.rightStick.x.setFactor(Calibration.TELEOP_FACTOR);
    manip.rightStick.y.setFactor(Calibration.TELEOP_FACTOR);

    this.robot = robot;

    driveManager = new DriveManager();
    intakeManager = new IntakeManager();
    shooterManager = new ShooterManager();
    // autoCenterManager = new AutoCenterManager();
  }

  // <editor-fold desc="Getting Controller Values"
  private double driverLeftJoystickY = 0.0;
  private double driverLeftJoystickX = 0.0;
  private double driverLeftTrigger = 0.0;

  private boolean driverLeftJoystickButton = false;
  private boolean driverLeftTriggerButton = false;
  private boolean driverLeftBumper = false;

  private double driverRightJoystickY = 0.0;
  private double driverRightJoystickX = 0.0;
  private double driverRightTrigger = 0.0;

  private boolean driverRightJoystickButton = false;
  private boolean driverRightTriggerButton = false;
  private boolean driverRightBumper = false;

  private boolean driverBack = false;
  private boolean driverStart = false;
  private boolean driverA = false;
  private boolean driverB = false;
  private boolean driverX = false;
  private boolean driverY = false;

  private boolean driverDPad = false;

  private double manipLeftJoystickY = 0.0;
  private double manipLeftJoystickX = 0.0;
  private double manipLeftTrigger = 0.0;

  private boolean manipLeftJoystickButton = false;
  private boolean manipLeftTriggerButton = false;
  private boolean manipLeftBumper = false;

  private double manipRightJoystickY = 0.0;
  private double manipRightJoystickX = 0.0;
  private double manipRightTrigger = 0.0;

  private boolean manipRightJoystickButton = false;
  private boolean manipRightTriggerButton = false;
  private boolean manipRightBumper = false;

  private boolean manipBack = false;
  private boolean manipStart = false;
  private boolean manipA = false;
  private boolean manipB = false;
  private boolean manipX = false;
  private boolean manipY = false;
  private boolean manipDPad = false;

  private boolean hatchCollisionChecker;
  private boolean armCollisionChecker;
  // </editor-fold>

  public void startPlayback(InputRecording recording) {
    inputPlayer.startPlayback(recording);
  }

  public void stopPlayback() {
    inputPlayer.stopPlayback();
  }

  @Override
  protected void begin() {
    if (inputPlayer.isPlaying()) {
      logger.info("Playing back Marionette recording");
    }
    hatchCollisionChecker = false;
    armCollisionChecker = false;
  }

  @Override
  protected boolean run() {
    updateControls();
    process();
    return true;
  }

  @Override
  protected void end() {
    if (inputRecorder != null) {
      final InputRecorder oldInputRecorder = inputRecorder;
      inputRecorder = null;
    }
  }

  private void updateControls() {
    driverLeftJoystickY = driver.leftStick.y.get();
    driverLeftJoystickX = driver.leftStick.x.get();
    driverLeftTrigger = driver.triggers.left.get();

    driverLeftJoystickButton = driver.buttons.leftStick.get();
    driverLeftTriggerButton = driver.buttons.lt.get();
    driverLeftBumper = driver.buttons.lb.get();

    driverRightJoystickY = driver.rightStick.y.get();
    driverRightJoystickX = driver.rightStick.x.get();
    driverRightTrigger = driver.triggers.right.get();

    driverRightJoystickButton = driver.buttons.rightStick.get();
    driverRightTriggerButton = driver.buttons.rt.get();
    driverRightBumper = driver.buttons.rb.get();

    driverBack = driver.buttons.back.get();
    driverStart = driver.buttons.start.get();
    driverA = driver.buttons.a.get();
    driverB = driver.buttons.b.get();
    driverX = driver.buttons.x.get();
    driverY = driver.buttons.y.get();

    driverDPad = driver.dpad.pressed.get();

    manipLeftJoystickY = manip.leftStick.y.get();
    manipLeftJoystickX = manip.leftStick.x.get();
    manipLeftTrigger = manip.triggers.left.get();

    manipLeftJoystickButton = manip.buttons.leftStick.get();
    manipLeftTriggerButton = manip.buttons.lt.get();
    manipLeftBumper = manip.buttons.lb.get();

    manipRightJoystickY = manip.rightStick.y.get();
    manipRightJoystickX = manip.rightStick.x.get();
    manipRightTrigger = manip.triggers.right.get();

    manipRightJoystickButton = manip.buttons.rightStick.get();
    manipRightTriggerButton = manip.buttons.rt.get();
    manipRightBumper = manip.buttons.rb.get();

    manipBack = manip.buttons.back.get();
    manipStart = manip.buttons.start.get();
    manipA = manip.buttons.a.get();
    manipB = manip.buttons.b.get();
    manipX = manip.buttons.x.get();
    manipY = manip.buttons.y.get();

    manipDPad = manip.dpad.pressed.get();
  }

  private void process() {
    driveManager.run();
    intakeManager.run();
    shooterManager.run();
  }

  private class DriveManager {
    private final Swerve.OpenLoop openLoop;
      private final Swerve.AutoAngle autoAngle;
    private final Swerve.Idle idle;

    private CurrentDrive currentDrive;
    // private CurrentDrive selectedDrive;

    private ProfiledPIDController snappingController;

    private Toggle inverted;
    
    private boolean isSnapping = false;


    public DriveManager() {
      idle = robot.drive.new Idle();
      openLoop = robot.drive.new OpenLoop();
      autoAngle = robot.drive.new AutoAngle();

      snappingController = new ProfiledPIDController(
        0.075, 0.0, 0.0,
        new TrapezoidProfile.Constraints(720.0, 360.0),
        robot.drive::getRawHeadingDegrees,
        (value) -> autoAngle.desiredAngularVel.set(value)
      );

      snappingController.setOutputRange(-1000, 1000);

      currentDrive = CurrentDrive.OPENLOOP;
      inverted = new Toggle(false);
    }

    public void run() {
      double leftX = driver.leftStick.x.get();
      double leftY = driver.leftStick.y.get();
      double rightX = -driver.rightStick.x.get();

      if (driverLeftJoystickButton) {
        leftX *= 0.8;
        leftY *= 0.8;
        rightX *= 0.8;
      }

      if (driverStart) {
        climbMode = true;

        if (driverBack) {
          robot.climber.extend.activate();
        } else if (driverRightBumper) {
          robot.climber.climb.activate();
        } else if (driverLeftBumper) {
          robot.climber.retract.activate();
        } else {
          robot.climber.idle.activate();
        }
        
      } else {
        climbMode = false;
        robot.climber.idle.activate();
      }

      if (driverY || driverA || driverB) {
        currentDrive = CurrentDrive.SNAPPING;
      }

      if (Math.abs(rightX) > 0.1) {
        currentDrive = CurrentDrive.OPENLOOP;
      }

      if (driverX) {
        robot.drive.zeroGyroOffset();
      }

      // // Get Dashboard option for drive
      // switch (robot.drive.driveMode.get()) {
      //   case OFF:
      //     currentDrive = CurrentDrive.IDLE;
      //     selectedDrive = CurrentDrive.IDLE;
      //     break;
      //   case ARCADE:
      //     currentDrive = CurrentDrive.ARCADE;
      //     selectedDrive = CurrentDrive.ARCADE;
      //     break;
      //   case TANK:
      //     currentDrive = CurrentDrive.TANK;
      //     selectedDrive = CurrentDrive.TANK;
      //     break;
      //   case DYNAMIC:
      //     // Dynamic Drive mode detection logic
      //     if (currentDrive == CurrentDrive.TANK) {
      //       if (Math.abs(rightY) <= 0.2 && Math.abs(rightX) > 0.3) {
      //         currentDrive = CurrentDrive.ARCADE;
      //         selectedDrive = CurrentDrive.ARCADE;
      //       }
      //     } else { // currentDrive == CurrentDrive.ARCADE
      //       if (Math.abs(rightX) <= 0.2 && Math.abs(rightY) > 0.3) {
      //         currentDrive = CurrentDrive.TANK;
      //       }
      //     }
      //     break;
      //   default:
      //     System.out.println("This should never happen!");
      //     System.out.println("Current value is:" + robot.drive.driveMode.get());
      // }

      switch (currentDrive) {
        case IDLE:
          idle.activate();
          break;
        case OPENLOOP:
          isSnapping = false;
          openLoop.xPower.set(leftX * 0.5);
          openLoop.yPower.set(leftY * 0.5);
          openLoop.rotPower.set(rightX * 0.5);

          openLoop.activate();
          break;
        case SNAPPING:
          autoAngle.xPower.set(leftX);
          autoAngle.yPower.set(leftY);

          if (driverY) {
            snapTo(CardinalDirections.NORTH);
          } else if (driverA) {
            snapTo(CardinalDirections.SOUTH);
          } else if (driverB) {
            snapTo(CardinalDirections.EAST);
          }

          autoAngle.activate();
          break;
      }

      robot.drive.updateOdometryWithVision(robot.limelight.getLatestMeasurement());
      FalconDashboard.getInstance().publishRobotPose(robot.drive.getPose());
    }

    private void snapTo(CardinalDirections direction) {
      Rotation2d snapAngle = new Rotation2d();
      switch (direction) {
        case NORTH:
          snapAngle = Rotation2d.fromDegrees(0);
          break;
        case EAST:
          snapAngle = Rotation2d.fromDegrees(-90);
          break;
        case SOUTH:
          snapAngle = Rotation2d.fromDegrees(180);
          break;
        case WEST:
          snapAngle = Rotation2d.fromDegrees(90);
          break;
      }

      SmartDashboard.putNumber("Error", snappingController.getPositionError());
      SmartDashboard.putNumber("Output", snappingController.getGoal().position);

      double setpoint = -robot.limelight.getLatestMeasurement().getBestTarget().getYaw() + robot.drive.getRawHeadingDegrees();

      // System.out.println("Setpoint:" + -robot.limelight.getLatestMeasurement().getBestTarget().getYaw());
      // System.out.println("Current:" + robot.drive.getRawHeadingDegrees());

      snappingController.setGoal(setpoint);

      if (!isSnapping) {
        snappingController.setInitialState(new TrapezoidProfile.State(robot.drive.getRawHeadingDegrees(), robot.drive.getAngularVelDegrees()));
        snappingController.enable();

        isSnapping = true;
      }

      // if (snappingController.atSetpoint()) {
      //   snappingController.disable();
      //   isSnapping = false;
      // }
    }
  }

  private class IntakeManager {
    private final Intake.Suck suck;
    private final Intake.Idle idle;
    private final Intake.Reverse reverseIntake;
    private final IntakeDeploy.Deploy deploy;
    private final IntakeDeploy.Retract retract;
    private final Tower.AntiJam antiJam;
    private final Revolver.Intake revolve;
    private final Revolver.Reverse reverseRevolver;
    private final AntiJamRoller.AntiJam roller;
    private final AntiJamRoller.Reverse reverseRoller;

    public IntakeManager() {
      suck = robot.intake.suck;
      idle = robot.intake.idle;
      reverseIntake = robot.intake.reverse;
      deploy = robot.intakeDeploy.deploy;
      retract = robot.intakeDeploy.retract;
      antiJam = robot.tower.antiJam;
      revolve = robot.revolver.intake;
      reverseRevolver = robot.revolver.reverse;
      roller = robot.antiJamRoller.antiJam;
      reverseRoller = robot.antiJamRoller.reverse;
    }

    public void run() {
      if (driverLeftTrigger >= 0.2) {
        deploy.activate();
        suck.activate();
        revolve.activate();
        roller.activate();
        antiJam.activate();
      } else if (driverLeftBumper && !climbMode) {
        reverseIntake.activate();
        reverseRevolver.activate();
        reverseRoller.activate();
      } else {
        retract.activate();
      }
    }
  }

  private class ShooterManager {
    private final Shooter.Setpoint setpoint;
    private final Shooter.Move move;
    private final Shooter.Stop stop;
    private final Tower.Empty emptyTower;
    private final Tower.Idle idleTower;
    private final Revolver.Empty emptyRevolver;
    private final Revolver.Idle idleRevolver;
    private final AntiJamRoller.AntiJam roller;
    private final AntiJamRoller.Idle rollerIdle;

    private final SmartTimer shootTimer;

    private Toggle shooterToggle;

    private double shooterSetpoint = 600.0;

    public ShooterManager() {
      setpoint = robot.shooter.setpoint;
      move = robot.shooter.move;
      emptyTower = robot.tower.empty;
      emptyRevolver = robot.revolver.empty;

      stop = robot.shooter.stop;
      idleTower = robot.tower.idle;
      idleRevolver = robot.revolver.idle;

      roller = robot.antiJamRoller.antiJam;
      rollerIdle = robot.antiJamRoller.idle;

      shootTimer = new SmartTimer();

      shooterToggle = new Toggle(false);
      SmartDashboard.putNumber("Shooter Speed", 10.0);
    }

    public void run() {
      double speed = SmartDashboard.getNumber("Shooter Speed", 300.0);
      shooterSetpoint = MathUtil.clamp(speed, 0, 600);

      shooterToggle.update(driverRightTrigger > 0.5);

      if (driverRightTrigger > 0.5) {
        setpoint.setpoint.set(shooterSetpoint);
        setpoint.activate();
      } else {
        stop.activate();
      }

      if (driverRightBumper && !climbMode) {
        emptyTower.activate();
        emptyRevolver.activate();
        roller.activate();
      }

    }
  }

  private static double placeInScope(double currentAngle, double desiredAngle) {
    // Place the desired angle in the scope [360(n-1), 360(n)] of the current angle.
    double angle = Math.floor(currentAngle / 360.0) * 360.0 + mod(desiredAngle, 360);

    // Constraint the desired angle to be < 180 degrees from the current angle.
    if ((angle - currentAngle) > 180) {
        angle -= 360;
    } else if ((angle - currentAngle) < -180) {
        angle += 360;
    }
    
    return angle;
  }

  private static double mod(double a, double n) {
    return (a % n + n) % n;
  }

  public enum CurrentDrive {
    IDLE,
    OPENLOOP,
    SNAPPING,
  }

  private enum CardinalDirections {
    NORTH,
    SOUTH,
    EAST,
    WEST,
  }
}
