package com._604robotics.quixsam;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.NavigableSet;
import java.util.Set;
import java.util.TreeMap;
import java.util.concurrent.ConcurrentSkipListMap;

import com._604robotics.quixsam.mathematics.DoubleInterpolatableTreeMap;
import com._604robotics.quixsam.mathematics.Interpolatable;
import com._604robotics.quixsam.odometry.QuixsamSwerveDriveOdometry;
import com._604robotics.quixsam.odometry.SendableOdometryMeasurment;
import com._604robotics.quixsam.vision.SendableVisionMeasurment;
import com._604robotics.quixsam.odometry.SwerveDriveOdometryMeasurement;
import com._604robotics.robotnik.prefabs.swerve.QuixSwerveDriveKinematics;
import com._604robotics.robotnik.prefabs.vision.VisionCamera;
import com._604robotics.robotnik.prefabs.vision.VisionCamera.Target;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpiutil.math.Pair;

public class QuixsamSwerveLocalizer {
    private int currentID = 0;
    private HashMap<Integer, Double> idMap = new HashMap<>();

    private TreeMap<Double, SwerveDriveOdometryMeasurement> odometryMap = new TreeMap<>();
    private DoubleInterpolatableTreeMap<Pose2d> poseMap = new DoubleInterpolatableTreeMap<>();

    private TreeMap<Double, Pair<SendableOdometryMeasurment, SendableVisionMeasurment>> buffer = new TreeMap<>();

    private QuixsamSwerveDriveOdometry rawOdometry;
    private QuixsamSwerveDriveOdometry playbackOdometry;

    private QuixsamNetworkTable networkTable;

    private double timeTreshold = 0.2; // seconds

    public QuixsamSwerveLocalizer(String name, QuixSwerveDriveKinematics kinematics, Pose2d priori, Pose2d prioriSigma, Rotation2d initialGyroAngle) {
        rawOdometry = new QuixsamSwerveDriveOdometry(kinematics, initialGyroAngle, priori);
        playbackOdometry = new QuixsamSwerveDriveOdometry(kinematics, initialGyroAngle, priori);

        networkTable = new QuixsamNetworkTable(name, priori, prioriSigma, this::computeEstimate);
    }

    public void update(SwerveDriveOdometryMeasurement odometry, VisionCamera.PipelineVisionPacket vision) {
        double currentTime = Timer.getFPGATimestamp();

        rawOdometry.updateWithTime(currentTime, odometry.getGyroAngle(), odometry.getModuleStates());
        playbackOdometry.updateWithTime(currentTime, odometry.getGyroAngle(), odometry.getModuleStates());
        odometryMap.put(currentTime, odometry);

        poseMap.set(currentTime, Interpolatable.interPose2d(rawOdometry.getPoseMeters()));

        double visionTime = currentTime - (vision.getLatency() / 1000);
        Pose2d interpolatedPose = poseMap.get(visionTime);

        System.out.println("Interp Pose: " + interpolatedPose);
        System.out.println("REAL Pose: " + rawOdometry.getPoseMeters());

        SendableOdometryMeasurment sendableOdometryMeasurment;
        SendableVisionMeasurment sendableVisionMeasurment = null;

        Target bestTarget = vision.getBestTarget();
        if (bestTarget.getCorners().size() == 4) {
            sendableVisionMeasurment = new SendableVisionMeasurment(0);

            for (Pair<Double, Double> corner : bestTarget.getCorners()) {
                sendableVisionMeasurment.addMeasurment(new Pair<>(corner.getFirst(), corner.getSecond()), new Pair<>(0.1, 0.1));
            }

            sendableOdometryMeasurment = new SendableOdometryMeasurment(0, rawOdometry.getPoseMeters(), new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(0.1)));
            buffer.put(visionTime, new Pair<>(sendableOdometryMeasurment, sendableVisionMeasurment));
        } else {
            sendableOdometryMeasurment = new SendableOdometryMeasurment(0, rawOdometry.getPoseMeters(), new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(0.1)));
            buffer.put(currentTime, new Pair<>(sendableOdometryMeasurment, sendableVisionMeasurment));
        }
    }

    public void update(SwerveDriveOdometryMeasurement odometry) {
        double currentTime = Timer.getFPGATimestamp();

        rawOdometry.updateWithTime(currentTime, odometry.getGyroAngle(), odometry.getModuleStates());
        playbackOdometry.updateWithTime(currentTime, odometry.getGyroAngle(), odometry.getModuleStates());
        odometryMap.put(currentTime, odometry);

        poseMap.set(currentTime, Interpolatable.interPose2d(rawOdometry.getPoseMeters()));

        buffer.put(currentTime, new Pair<>(new SendableOdometryMeasurment(0, rawOdometry.getPoseMeters(), new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(0.1))), null));
    }

    public void computeEstimate(QuixsamEsimate estimate) {
        int estimateID = estimate.getID();
        double estimateTime = idMap.get(estimateID);

        playbackOdometry.resetPosition(estimate.getPose(), odometryMap.get(estimateTime).getGyroAngle());

        Double lastKey = odometryMap.ceilingKey(estimateTime); //least key >= time

        double prevTime = odometryMap.lowerKey(lastKey);
        while (lastKey != null) {
            SwerveDriveOdometryMeasurement lastMeasurment = odometryMap.get(lastKey);
            playbackOdometry.updateWithTime(prevTime, lastKey, lastMeasurment.getGyroAngle(), lastMeasurment.getModuleStates());
            prevTime = lastKey;
            odometryMap.remove(lastKey);
            lastKey = odometryMap.ceilingKey(lastKey);
        }
    }

    public Pose2d getPose() {
        return playbackOdometry.getPoseMeters();
    }

    public void periodic() {
        double currentTime = Timer.getFPGATimestamp();

        ArrayList<Double> keys = new ArrayList<>(buffer.keySet());
        for (double key : keys) {
            if (currentTime - key > timeTreshold) {
                currentID += 1;
                idMap.put(currentID, key);

                buffer.get(key).getFirst().setId(currentID);
                networkTable.publishOdometry(buffer.get(key).getFirst());

                if (buffer.get(key).getSecond() != null) {
                    buffer.get(key).getSecond().setId(currentID);
                    networkTable.publishVision(buffer.get(key).getSecond());
                }

                buffer.remove(key);
            }
        }
    }
}
