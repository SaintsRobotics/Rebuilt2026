// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.LaunchCalc;

public class ShooterCommand extends Command {

    private final ShooterSubsystem m_shooterSubsystem;
    private final Supplier<Pose2d> m_robotPoseSupplier;
    private final Supplier<Pose2d> m_targetSupplier;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, Supplier<Pose2d> robotPose, Supplier<Pose2d> currentTarget) {
        m_shooterSubsystem = shooterSubsystem;
        m_targetSupplier = currentTarget;
        m_robotPoseSupplier = robotPose;
        addRequirements(m_shooterSubsystem);
    }

    public void initialize() {
        
        m_shooterSubsystem.reset();

    }

    public void execute() {
        
        m_shooterSubsystem.setShooterParameters(
            // LaunchCalc.findFlywheelSpeed(m_robotPoseSupplier.get(), m_targetSupplier.get()), 
            // 90 - LaunchCalc.findHoodAngle(m_robotPoseSupplier.get(), m_targetSupplier.get())
            SmartDashboard.getNumber("Shooter/Input Shooter Speed", 0),
            SmartDashboard.getNumber("Shooter/Hood Angle Input", 0)
        );
        
        if (m_shooterSubsystem.isShooterReady()) {
            m_shooterSubsystem.setSpindexer(true);
            m_shooterSubsystem.setTransfer(true);
        }
    }

    public void end(boolean interrupted) {
        
        m_shooterSubsystem.shooterStop();
        m_shooterSubsystem.setSpindexer(false);
        m_shooterSubsystem.setTransfer(false);

    }

    public boolean isFinished() {
        
        return false;

    }

}

