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
    private static final InterpolatingDoubleTreeMap LaunchAngleLUT = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap TimeLUT = new InterpolatingDoubleTreeMap();

    static {
        // Populate the LUTs with empirical data
        // Example entries (distance in meters, flywheel speed in RPM, hood angle in degrees)
        FlywheelLUT.put(1.0, 1700.0);
        FlywheelLUT.put(2.0, 1800.0);
        FlywheelLUT.put(3.0, 1950.0);
        FlywheelLUT.put(4.0, 2050.0);
        FlywheelLUT.put(5.0, 2250.0);
        FlywheelLUT.put(6.0, 2350.0);
        FlywheelLUT.put(8.0, 2650.0);

        LaunchAngleLUT.put(1.0, 82.5);
        LaunchAngleLUT.put(2.0, 77.5);
        LaunchAngleLUT.put(3.0, 72.5);
        LaunchAngleLUT.put(4.0, 67.5);
        LaunchAngleLUT.put(5.0, 67.5);
        LaunchAngleLUT.put(6.0, 65.0);
        LaunchAngleLUT.put(8.0, 65.0);

        TimeLUT.put(1.0, 0.97);
        TimeLUT.put(2.0, 0.95);
        TimeLUT.put(3.0, 1.09);
        TimeLUT.put(4.0, 1.0);
        TimeLUT.put(5.0, 1.17);
    }

    public static double findFlywheelSpeed(Pose2d currentPose, Pose2d targetPose) {
        double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        SmartDashboard.putNumber("distance to target", distance);
        return FlywheelLUT.get(distance);
    }

    public static double findHoodAngle(Pose2d currentPose, Pose2d targetPose) {
        double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        return LaunchAngleLUT.get(distance);
    }

    /** Calculates the target to aim for when shooting on the move. */
    public static Pose2d findTargetOnTheMove(Pose2d currentPose, Pose2d targetPose, Translation2d velocity, int iterations) {
        double distance = 0;
        double timeOfFlight = 0;
        Pose2d newTarget = targetPose;
        for (int i = 0; i < iterations; i++) {
            distance = currentPose.getTranslation().getDistance(newTarget.getTranslation());
            timeOfFlight = TimeLUT.get(distance);
            newTarget = targetPose.plus(new Transform2d(velocity.times(-timeOfFlight*ShooterConstants.kShootOnTheMoveMultiplier), Rotation2d.kZero));
            // move newTarget slightly away from currentPose
            if (velocity.getDistance(new Translation2d()) > 0.05) {
                Transform2d offset = currentPose.minus(newTarget).times(-ShooterConstants.kSOTMOffsetMultiplier);
                // offset = offset.div(offset.getTranslation().getDistance(new Translation2d())); // unit vector
                newTarget = newTarget.plus(offset);
            }
        }
        return newTarget;
    }

    public static Pose2d findTargetOnTheMove(Pose2d currentPose, Pose2d targetPose, Translation2d velocity) {
        return findTargetOnTheMove(currentPose, targetPose, velocity, ShooterConstants.kShootOnTheMoveIterations);
    }
   
}