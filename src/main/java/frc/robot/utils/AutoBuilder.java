// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.AutonConstants;

/** Builds autons from BLine paths and robot commands. */
public class AutoBuilder {

    private final FollowPath.Builder m_pathBuilder;

    public AutoBuilder(DriveSubsystem robotDrive) {
        m_pathBuilder = new FollowPath.Builder(
            robotDrive,
            robotDrive::getPose,
            robotDrive::getRobotRelativeSpeeds,
            robotDrive::autonDrive,
            new PIDController(AutonConstants.kTranslationP, AutonConstants.kTranslationI, AutonConstants.kTranslationD),
            new PIDController(AutonConstants.kRotationP, AutonConstants.kRotationI, AutonConstants.kRotationD),
            new PIDController(AutonConstants.kCrossTrackP, AutonConstants.kCrossTrackI, AutonConstants.kCrossTrackD)
        )//.withDefaultShouldFlip()
         .withPoseReset(robotDrive::resetOdometry);
    }

    public SendableChooser<Command> buildAutoChooser() {
        SendableChooser<Command> autoChooser = new SendableChooser<>();

        // Add autons below
        autoChooser.addOption("RightTrench_AllCenterFuel_Tower", new SequentialCommandGroup(
            // intake command goes here
            new ParallelCommandGroup(
                m_pathBuilder.build(new Path("RTrench_CenterFuel"))
                // shooter command goes here
            ),
            m_pathBuilder.build(new Path("CenterLeft_Tower"))
        ));        

        return autoChooser;
    }
}
