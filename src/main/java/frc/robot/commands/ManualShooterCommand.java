// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualShooterCommand extends Command {

  private final ShooterSubsystem m_shooterSubsystem;
  private final TurretSubsystem m_turretSubsystem;
  private final double m_shooterSpeed;
  private final double m_hoodAngle;

  /** Creates a new ManualShooterCommand. */
  public ManualShooterCommand(ShooterSubsystem shooterSubsystem, TurretSubsystem turretSubsystem, double shooterSpeed, double hoodAngle) {
    m_shooterSubsystem = shooterSubsystem;
    m_turretSubsystem = turretSubsystem;
    m_shooterSpeed = shooterSpeed;
    m_hoodAngle = hoodAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterSubsystem.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_shooterSubsystem.setShooterParameters(
            m_shooterSpeed,
            m_hoodAngle
            // SmartDashboard.getNumber("Shooter/Input Shooter Speed", 0),
            // SmartDashboard.getNumber("Shooter/Hood Angle Input", 0)
        );
        
        if (m_shooterSubsystem.isShooterReady() && m_shooterSubsystem.isHoodReady() && m_turretSubsystem.atSetpoint()) {
            m_shooterSubsystem.setSpindexer(true);
            m_shooterSubsystem.setTransfer(true);
        }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.shooterStop();
    m_shooterSubsystem.setSpindexer(false);
    m_shooterSubsystem.setTransfer(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
