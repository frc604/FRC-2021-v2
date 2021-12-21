package com._604robotics.robotnik.vision;

import com._604robotics.robot2021.constants.Calibration;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.Pair;

import java.util.ArrayList;
import java.util.List;
import org.apache.commons.math3.geometry.euclidean.threed.Line;
import org.apache.commons.math3.geometry.euclidean.threed.Plane;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

/** A base class that represents a vision Camera. */
public abstract class VisionCamera extends Module {
  private final Vector3D pose;
  private final double tilt;

  private PipelineVisionPacket latestPacket =
      new PipelineVisionPacket(false, new Target(), new ArrayList<Target>(), 0.0);

  public final Output<Double> bestTargetYaw =
      addOutput("Best Target Yaw", () -> latestPacket.bestTarget.yaw);
  public final Output<Double> bestTargetPitch =
      addOutput("Best Target Pitch", () -> latestPacket.bestTarget.pitch);

  public final Output<Double> latency = addOutput("latency", () -> latestPacket.latency);

  /** Instantiates a new Vision camera. */
  public VisionCamera(String name, Vector3D pose, double tilt) {
    super(name);
    this.pose = pose;
    this.tilt = tilt;
  }

  /**
   * Gets latest measurement.
   *
   * @return the latest measurement
   */
  public abstract PipelineVisionPacket getLatestMeasurement();

  public void pull() {
    latestPacket = getLatestMeasurement();
  }

  public double getTilt() {
    return tilt;
  }

  public List<Translation2d> searchGamePiece(Pose2d fieldToRobot, GamePiece gamePiece) {
    Transform2d robotToCamera =
        new Transform2d(new Translation2d(pose.getX(), pose.getY()), new Rotation2d());
    Pose2d fieldToCamera = fieldToRobot.transformBy(robotToCamera);

    Vector3D cameraPose = new Vector3D(fieldToCamera.getX(), fieldToCamera.getY(), pose.getZ());
    Plane gamePiecePlane =
        new Plane(new Vector3D(0, 0, gamePiece.radius), new Vector3D(0, 0, gamePiece.radius), 1e-9);

    List<Translation2d> gamePieceTranslations = new ArrayList<>();

    if (latestPacket.hasTargets()) {
      for (Target t : latestPacket.getTargets()) {
        Vector3D tiltedTargetVector =
            new Vector3D(Math.toRadians(t.getYaw()), Math.toRadians(t.getPitch() + tilt));
        Line line = new Line(cameraPose, tiltedTargetVector.add(cameraPose), 1e-9);
        Vector3D gamePiecePoint = gamePiecePlane.intersection(line);
        System.out.println(gamePiecePoint);
        gamePieceTranslations.add(
            new Translation2d(
                Calibration.Auto.XDISTACEBALLCAM.get(t.getPitch()), -gamePiecePoint.getY()));
      }
    }

    return gamePieceTranslations;
  }

  /** A data class for a pipeline packet. */
  public static class PipelineVisionPacket {
    private final boolean hasTargets;
    private final Target bestTarget;
    private final List<Target> targets;
    private final double latency;

    public PipelineVisionPacket(
        boolean hasTargets, Target bestTarget, List<Target> targets, double latency) {
      this.hasTargets = hasTargets;
      this.bestTarget = bestTarget;
      this.targets = targets;
      this.latency = latency;
    }

    /**
     * If the vision packet has valid targets.
     *
     * @return if targets are found.
     */
    public boolean hasTargets() {
      return hasTargets;
    }

    /**
     * Gets best target.
     *
     * @return the best target.
     */
    public Target getBestTarget() {
      return bestTarget;
    }

    /**
     * Gets targets.
     *
     * @return the targets.
     */
    public List<Target> getTargets() {
      return targets;
    }

    /**
     * Gets latency.
     *
     * @return the latency.
     */
    public double getLatency() {
      return latency;
    }
  }

  /** A data class for a target measurement. */
  public static class Target {
    private final double yaw;
    private final double pitch;
    private final double area;
    private final double skew;
    private final ArrayList<Pair<Double, Double>> corners;

    public Target(double yaw, double pitch, double area, double skew) {
      this.yaw = yaw;
      this.pitch = pitch;
      this.area = area;
      this.skew = skew;
      this.corners = new ArrayList<>();
    }

    public Target(double yaw, double pitch, double area, double skew, ArrayList<Pair<Double, Double>> corners) {
      this.yaw = yaw;
      this.pitch = pitch;
      this.area = area;
      this.skew = skew;
      this.corners = corners;
    }

    public Target() {
      this(0.0, 0.0, 0.0, 0.0, new ArrayList<>());
    }

    /**
     * Returns a Vector3D from the yaw and pitch from the vision measurement.
     *
     * @return the vector 3D.
     */
    public Vector3D getVector3D() {
      return new Vector3D(Math.toRadians(yaw), Math.toRadians(pitch));
    }

    /**
     * Gets yaw.
     *
     * @return the yaw
     */
    public double getYaw() {
      return yaw;
    }

    /**
     * Gets pitch.
     *
     * @return the pitch
     */
    public double getPitch() {
      return pitch;
    }

    /**
     * Gets area.
     *
     * @return the area
     */
    public double getArea() {
      return area;
    }

    /**
     * Gets skew.
     *
     * @return the skew
     */
    public double getSkew() {
      return skew;
    }

    public ArrayList<Pair<Double, Double>> getCorners() {
      return corners;
    }
  }

  public static enum GamePiece {
    POWERCELL(0.005),
    THREEPOINTFIVE(Units.inchesToMeters(3.5)),
    ONEM(1);

    public final double radius;

    private GamePiece(double radius) {
      this.radius = radius;
    }
  }
}
