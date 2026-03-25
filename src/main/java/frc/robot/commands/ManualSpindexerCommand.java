// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualSpindexerCommand extends Command {
  /** Creates a new ManualSpindexerCommand. */

  private final ShooterSubsystem m_shooter;
  private final double m_spindexerSpeed;
  private final double m_transferSpeed;

  public ManualSpindexerCommand(ShooterSubsystem shooter, double spindexerSpd, double transferSpd) {
    m_shooter = shooter;
    m_spindexerSpeed = spindexerSpd;
    m_transferSpeed = transferSpd;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setTransfer(m_transferSpeed);
    m_shooter.setSpindexer(m_spindexerSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setTransfer(0);
    m_shooter.setSpindexer(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
