// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.FieldConstants;
<<<<<<< HEAD
import frc.robot.subsystems.TurretSubsystem;
=======
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.IntakeCommand;
>>>>>>> main

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
<<<<<<< HEAD
//   private final TurretSubsystem m_turret = new TurretSubsystem();
=======
  // private final IntakeSubsystem m_intake = new IntakeSubsystem();
  // private final TurretSubsystem m_turret = new TurretSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
>>>>>>> main

  private final XboxController m_driverController = new XboxController(IOConstants.kDriverControllerPort);
  private final XboxController m_operatorController = new XboxController(IOConstants.kOperatorControllerPort);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the trigger bindings
    configureBindings();

    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(
                    -m_driverController.getLeftY(),
                    IOConstants.kControllerDeadband)
                    * DriveConstants.kMaxSpeedMetersPerSecond
                    * (1 - (m_driverController
                        .getRightBumperButton() ? IOConstants.kSlowModeScalar : 0))
                    * 0.8,
                MathUtil.applyDeadband(
                    -m_driverController.getLeftX(),
                    IOConstants.kControllerDeadband)
                    * DriveConstants.kMaxSpeedMetersPerSecond
                    * (1 - (m_driverController
                        .getRightBumperButton() ? 1 : 0)
                        * IOConstants.kSlowModeScalar)
                    * 0.8,
                MathUtil.applyDeadband(
                    m_driverController.getRightX(),
                    IOConstants.kControllerDeadband)
                    * DriveConstants.kMaxAngularSpeedRadiansPerSecond
                    * (1 - (m_driverController
                        .getRightBumperButton() ? 1 : 0)
                        * IOConstants.kSlowModeScalar)
                    * -1,
                true),
                    m_robotDrive));
    
    // m_turret.setDefaultCommand(new RunCommand(() -> {
    //     m_turret.calculateSetpoint(m_robotDrive.getPose(), TurretConstants.kHubPose, m_robotDrive.getRotationSpeed());
    // },
    // m_turret));
}


  /**
   *  Driver Controls:
   * 
   *    Driving:
   *      left axis X/Y                         axis  Translation
   *      right axis X                          axis  Rotation
   *      start                                 press Reset heading
   *      back                                  press Reset position

   */
  private void configureBindings() {
      // -------- driving bindings -------- //

    // driver reset heading
    new JoystickButton(m_driverController, Button.kStart.value)
        .onTrue(new InstantCommand(m_robotDrive::zeroHeading, m_robotDrive));

    // driver reset odometry
    new JoystickButton(m_driverController, Button.kBack.value)
        .onTrue(new InstantCommand(() -> m_robotDrive.resetOdometry(new Pose2d()), m_robotDrive));
    
    // run intake
    new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .whileTrue(new IntakeCommand(m_intake));
  }

  /**
   * This periodic loop runs every 10ms (100Hz)
   * 
   * <p>
   * Should be used for any code that needs to be run more frequently than the
   * default 20ms loop (50Hz) such as PID Controllers.
   * </p>
   */
  public void fastPeriodic() {
    m_robotDrive.fastPeriodic();
  }

  public void periodic() {
    
    SmartDashboard.putBoolean("In Blue", FieldConstants.kBlueAllianceRegion.isInRegion(m_robotDrive.getPose()));
    SmartDashboard.putBoolean("In Red", FieldConstants.kRedAllianceRegion.isInRegion(m_robotDrive.getPose()));
    SmartDashboard.putBoolean("In Trench", FieldConstants.kTrenchesRegion.isInRegion(m_robotDrive.getPose()));

  }

}
