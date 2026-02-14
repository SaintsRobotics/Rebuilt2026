// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.Map;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

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

    public static double returnFlywheelSpeed(Pose2d currentPose, Pose2d targetPose) {
        double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        return FlywheelLUT.get(distance);
    }
   
}