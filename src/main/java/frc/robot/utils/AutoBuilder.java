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
        autoChooser.addOption("Right Double Sweep", rightSideAuto());
        autoChooser.addOption("Left Double Sweep", leftSideAuto());
        autoChooser.addOption("Middle Depot", middleAuto());
        autoChooser.addOption("Right Outpost", rightOutpostAuto());

        return autoChooser;
    }

    public Command rightSideAuto() {
        return Commands.sequence(
            deployIntake(),
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

    public Command leftSideAuto() {
        return Commands.sequence(
            deployIntake(),
            Commands.deadline(
                m_pathBuilder.build(new Path("Left_CenterFuel")), 
                new RunIntake(m_intake)
            ),
            shootFor(5),
            Commands.deadline(
                m_pathBuilder.build(new Path("Left_HubFuel")), 
                new RunIntake(m_intake)
            ),
            shootFor(5)
        );
    }

    public Command middleAuto() {
        return Commands.parallel(
            Commands.deadline(
                m_pathBuilder.build(new Path("Middle_Depot")),
                Commands.waitSeconds(0.5)
                    .andThen(deployIntake())
                    .andThen(new RunIntake(m_intake))
            ),
            shootFor(15)
        );
    }

    public Command rightOutpostAuto() {
        return Commands.sequence(
            deployIntake(),
            Commands.deadline(
                m_pathBuilder.build(new Path("RTrench_CenterFuel")), 
                new RunIntake(m_intake)
            ),
            Commands.parallel(
                m_pathBuilder.build(new Path("Right_Outpost")),
                shootFor(15)
            )
        );
    }

    private Command shootFor(double seconds) {
        return new ShooterCommand(m_shooter, m_turret, m_drive::getPose, m_targetSupplier).withTimeout(seconds).asProxy();
    }

    private Command deployIntake() {
        return Commands.runOnce(() -> m_intake.setArmPosition(ArmPosition.Extended), m_intake);
    }
}
