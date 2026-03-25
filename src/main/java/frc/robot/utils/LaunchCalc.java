// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;

public class LaunchCalc{

    private static final InterpolatingDoubleTreeMap FlywheelLUT = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap LaunchAngleLUT = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap TimeLUT = new InterpolatingDoubleTreeMap();

    static {
        // Populate the LUTs with empirical data
        // Example entries (distance in meters, flywheel speed in RPM, hood angle in degrees)
        FlywheelLUT.put(Units.inchesToMeters(37.625), 2700.0); //updated 3/16
        FlywheelLUT.put(Units.inchesToMeters(37.625 + 36), 2800.0); //updated 3/16
        FlywheelLUT.put(Units.inchesToMeters(37.625 + 72), 3200.0); //updated 3/16
        FlywheelLUT.put(Units.inchesToMeters(37.625 + 108), 3500.0); //updated 3/16 //3.973m
        FlywheelLUT.put(Units.inchesToMeters(37.625 + 144), 3800.0); //4.902m
        FlywheelLUT.put(Units.inchesToMeters(37.625 + 180), 3800.0);
        FlywheelLUT.put(Units.inchesToMeters(180 + 56), 4400.0);

        LaunchAngleLUT.put(Units.inchesToMeters(37.625), 0.0);
        LaunchAngleLUT.put(Units.inchesToMeters(37.625 + 36), 0.15);
        LaunchAngleLUT.put(Units.inchesToMeters(37.625 + 72), 0.25);
        LaunchAngleLUT.put(Units.inchesToMeters(37.625 + 108), 0.3);
        LaunchAngleLUT.put(Units.inchesToMeters(37.625 + 144), 0.32);
        LaunchAngleLUT.put(Units.inchesToMeters(37.625 + 180), 0.43);
        LaunchAngleLUT.put(Units.inchesToMeters(180 + 56), 0.43);

        TimeLUT.put(1.43, 0.93);
        TimeLUT.put(2.38, 1.01);
        TimeLUT.put(3.27, 1.31);
        TimeLUT.put(4.05, 1.38);
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
            // timeOfFlight = TimeLUT.get(distance);
            timeOfFlight = 0.188614 * distance + 0.632681; // linear regression of the time lookup table as of 3/18
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