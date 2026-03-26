// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.IntakeSubsystem.ArmPosition;
import frc.robot.Constants.AutonConstants;
import frc.robot.commands.RunIntake;
import frc.robot.commands.ShooterCommand;

/** Builds autons from BLine paths and robot commands. */
public class AutoBuilder {

    private final FollowPath.Builder m_pathBuilder;

    private final DriveSubsystem m_drive;
    private final ShooterSubsystem m_shooter;
    private final TurretSubsystem m_turret;
    private final IntakeSubsystem m_intake;
    private final Supplier<Pose2d> m_targetSupplier;

    public AutoBuilder(
        DriveSubsystem robotDrive, 
        ShooterSubsystem shooter, 
        TurretSubsystem turret, 
        IntakeSubsystem intake,
        Supplier<Pose2d> target) 
    {
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

        m_drive = robotDrive;
        m_shooter = shooter;
        m_turret = turret;
        m_intake = intake;
        m_targetSupplier = target;
    }

    public SendableChooser<Command> buildAutoChooser() {
        SendableChooser<Command> autoChooser = new SendableChooser<>();

        // Add autons below
        autoChooser.addOption("RightTrench_AllCenterFuel_Tower", rightSideAuto());

        return autoChooser;
    }

    public Command rightSideAuto() {
        return Commands.sequence(
            Commands.runOnce(() -> m_intake.setArmPosition(ArmPosition.Extended), m_intake),
            Commands.deadline(
                m_pathBuilder.build(new Path("RTrench_CenterFuel")),
                new RunIntake(m_intake)
            ),
            shootFor(5),
            Commands.deadline(
                m_pathBuilder.build(new Path("RTrench_HubFuel")),
                new RunIntake(m_intake)
            ),
            shootFor(5)
        );
    }

    private Command shootFor(double seconds) {
        return new ShooterCommand(m_shooter, m_turret, m_drive::getPose, m_targetSupplier).withTimeout(seconds).asProxy();
    }
}
