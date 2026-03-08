// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;

/** Picks whether to score or ferry based on the robot's location */
public class FindTarget {

  public static Pose2d getTarget(Pose2d robotPose) {

    // if (DriverStation.getAlliance().isEmpty()) {
    //   return AllianceFlipUtil.apply(FieldConstants.kHubPose);
    // }

    // switch (DriverStation.getAlliance().get()) {
    //   case Blue:
    //     return FieldConstants.kBlueAllianceRegion.isInRegion(robotPose) ? FieldConstants.kHubPose : targetAllianceWall(robotPose);
    //   case Red:
    //     return FieldConstants.kRedAllianceRegion.isInRegion(robotPose) ? AllianceFlipUtil.apply(FieldConstants.kHubPose) : targetAllianceWall(robotPose);
    //   default:
    //     return AllianceFlipUtil.apply(FieldConstants.kHubPose);
    // }

    return shouldScoreHub(robotPose) ? AllianceFlipUtil.apply(FieldConstants.kHubPose) : targetAllianceWall(robotPose);
  }

  public static boolean shouldScoreHub(Pose2d robotPose) {
    if (DriverStation.getAlliance().isEmpty()) {
      return false;
    }

    switch (DriverStation.getAlliance().get()) {
      case Blue:
        return FieldConstants.kBlueAllianceRegion.isInRegion(robotPose);
      case Red:
        return FieldConstants.kRedAllianceRegion.isInRegion(robotPose);
      default:
        return AllianceFlipUtil.shouldFlip();
    }
  }

  private static Pose2d targetAllianceWall(Pose2d robotPose) {
    return new Pose2d(
      DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Blue) ? Meters.zero() : Meters.of(FieldConstants.kFieldLength),
      robotPose.getMeasureY(),
      robotPose.getRotation()
    );
    
  }
}
