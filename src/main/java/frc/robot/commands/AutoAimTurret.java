// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AutoAimTurret extends Command {
  private final TurretSubsystem m_turretSubsystem;
  private final DriveSubsystem m_driveSubsystem;
  private final Pose2d m_targetPose;
  private final boolean m_finishWhenAtTarget;

  /* Creates a new AutoAimTurret */
  public AutoAimTurret(
      TurretSubsystem turretSubsystem,
      DriveSubsystem driveSubsystem,
      Pose2d targetPose,
      boolean finishWhenAtTarget) {
    m_turretSubsystem = turretSubsystem;
    m_driveSubsystem = driveSubsystem;
    m_targetPose = targetPose;
    m_finishWhenAtTarget = finishWhenAtTarget;

    addRequirements(turretSubsystem);
  }

  public AutoAimTurret(TurretSubsystem turretSubsystem, DriveSubsystem driveSubsystem) {
    this(turretSubsystem, driveSubsystem, TurretConstants.kHubPose, false);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // Calculate and update the turret setpoint
    m_turretSubsystem.calculateSetpoint(
        m_driveSubsystem.getPose(),
        m_targetPose,
        m_driveSubsystem.getRotationSpeed());
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return m_finishWhenAtTarget && m_turretSubsystem.atSetpoint();
  }
}
