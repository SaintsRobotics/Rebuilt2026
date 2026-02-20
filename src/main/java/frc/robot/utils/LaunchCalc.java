// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class LaunchCalc{

    private static final InterpolatingDoubleTreeMap FlywheelLUT = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap HoodAngleLUT = new InterpolatingDoubleTreeMap();

    static {
        // Populate the LUTs with empirical data
        // Example entries (distance in meters, flywheel speed in RPM, hood angle in degrees)
        FlywheelLUT.put(1.0, 3000.0);
        FlywheelLUT.put(2.0, 4000.0);
        FlywheelLUT.put(3.0, 5000.0);
        FlywheelLUT.put(4.0, 6000.0);
        FlywheelLUT.put(5.0, 7000.0);

        HoodAngleLUT.put(1.0, 10.0);
        HoodAngleLUT.put(2.0, 20.0);
        HoodAngleLUT.put(3.0, 30.0);
        HoodAngleLUT.put(4.0, 40.0);
        HoodAngleLUT.put(5.0, 50.0);
    }

    public static double findFlywheelSpeed(Pose2d currentPose, Pose2d targetPose) {
        double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        return FlywheelLUT.get(distance);
    }

    public static double findHoodAngle(Pose2d currentPose, Pose2d targetPose) {
        double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        return HoodAngleLUT.get(distance);
    }
   
}