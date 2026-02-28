// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;

public class LaunchCalc{

    private static final InterpolatingDoubleTreeMap FlywheelLUT = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap HoodAngleLUT = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap TimeLUT = new InterpolatingDoubleTreeMap();

    static {
        // Populate the LUTs with empirical data
        // Example entries (distance in meters, flywheel speed in RPM, hood angle in degrees)
        FlywheelLUT.put(1.0, 1450.0);
        FlywheelLUT.put(2.0, 1550.0);
        FlywheelLUT.put(3.0, 1720.0);
        FlywheelLUT.put(4.0, 1860.0);
        FlywheelLUT.put(5.0, 2050.0);

        HoodAngleLUT.put(1.0, 82.5);
        HoodAngleLUT.put(2.0, 60.0);
        HoodAngleLUT.put(3.0, 50.0);
        HoodAngleLUT.put(4.0, 48.0);
        HoodAngleLUT.put(5.0, 45.0);

        TimeLUT.put(1.0, 0.81);
        TimeLUT.put(2.0, 0.72);
        TimeLUT.put(3.0, 0.68);
        TimeLUT.put(4.0, 0.77);
        TimeLUT.put(5.0, 0.86);
    }

    public static double findFlywheelSpeed(Pose2d currentPose, Pose2d targetPose) {
        double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        SmartDashboard.putNumber("distance to target", distance);
        return FlywheelLUT.get(distance);
    }

    public static double findHoodAngle(Pose2d currentPose, Pose2d targetPose) {
        double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        return HoodAngleLUT.get(distance);
    }

    /** Calculates the target to aim for when shooting on the move. */
    public static Pose2d findTargetOnTheMove(Pose2d currentPose, Pose2d targetPose, Translation2d velocity) {
        double distance = 0;
        double timeOfFlight = 0;
        Pose2d newTarget = targetPose;
        for (int i = 0; i < ShooterConstants.kShootOnTheMoveIterations; i++) {
            distance = currentPose.getTranslation().getDistance(newTarget.getTranslation());
            timeOfFlight = TimeLUT.get(distance);
            newTarget = targetPose.plus(new Transform2d(velocity.times(-timeOfFlight), Rotation2d.kZero));
        }
        return newTarget;
    }
   
}