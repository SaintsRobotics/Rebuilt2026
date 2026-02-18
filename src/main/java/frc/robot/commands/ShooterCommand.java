// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {

    private final ShooterSubsystem m_shooterSubsystem;

    public ShooterCommand(ShooterSubsystem shooterSubsystem) {
        m_shooterSubsystem = shooterSubsystem;
        addRequirements(m_shooterSubsystem);
    }

    public void initialize() {
        
        m_shooterSubsystem.reset();

    }

    public void execute() {
        
        m_shooterSubsystem.setShooterParameters(1, 45);
        
    }

    public void end(boolean interrupted) {
        
        m_shooterSubsystem.shooterStop();

    }

    public boolean isFinished() {
        
        return false;

    }

}

