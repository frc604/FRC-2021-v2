package com._604robotics.robot2021.auto;

import java.util.ArrayList;
import java.util.List;

import com._604robotics.robot2021.modules.Swerve;
import com._604robotics.robotnik.prefabs.auto.QuikPlanSwerveReader;
import com._604robotics.robotnik.prefabs.auto.SwerveTrackerConstants;
import com._604robotics.robotnik.prefabs.swerve.QuixSwerveModuleState;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

public class QuixPlanSwerveTrajectoryTracker extends QuixPlanSwerveTrackingCoordinator {
    private final Swerve.Auto auto;
    private final Swerve swerve;
    

    public QuixPlanSwerveTrajectoryTracker(QuikPlanSwerveReader reader, Swerve swerve, SwerveTrackerConstants constants) {
        super(reader, swerve, constants);
        this.swerve = swerve;

        this.auto = swerve.new Auto();
    }

    @Override
    public void useOutput(QuixSwerveModuleState[] moduleStates) {
        // auto.moduleStates.set(moduleStates);

        swerve.driveClosedLoop(moduleStates);

        // System.out.println("Velocity: " + moduleStates[0].speedMetersPerSecond);
        // System.out.println("Angle: " + moduleStates[0].angle);

        auto.activate();
    }

    @Override
    public void stop() {
        QuixSwerveModuleState[] moduleStates = {
            new QuixSwerveModuleState(),
            new QuixSwerveModuleState(),
            new QuixSwerveModuleState(),
            new QuixSwerveModuleState()
        };

       swerve.driveClosedLoop(moduleStates);

        // auto.moduleStates.set(moduleStates);

        auto.activate();
    }
}
