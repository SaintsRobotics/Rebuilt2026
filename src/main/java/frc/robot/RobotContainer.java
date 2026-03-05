// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.FuelSim;
import frc.robot.utils.LaunchCalc;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final TurretSubsystem m_turret = new TurretSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();

  private final XboxController m_driverController = new XboxController(IOConstants.kDriverControllerPort);
  private final XboxController m_operatorController = new XboxController(IOConstants.kOperatorControllerPort);

  public final FuelSim fuelSim;
  private final StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().getStructTopic("SotM Target/5 iterations", Pose2d.struct).publish();
  private final StructPublisher<Pose2d> publisher2 = NetworkTableInstance.getDefault().getStructTopic("SotM Target/10 iterations", Pose2d.struct).publish();
  private Pose2d currentTarget = TurretConstants.kHubPose;


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the trigger bindings
    configureBindings();

    fuelSim = new FuelSim();
    configureFuelSim();

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
    
    m_turret.setDefaultCommand(new RunCommand(() -> {
        m_turret.calculateSetpoint(m_robotDrive.getPose(), currentTarget, m_robotDrive.getRotationSpeed());
    },
    m_turret));
    
    m_shooter.setDefaultCommand(new ShooterCommand(m_shooter, m_robotDrive::getPose, () -> {return currentTarget;}));
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


    new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .onTrue(new InstantCommand(() -> {fuelSim.launchFuel(
            LinearVelocity.ofBaseUnits(
                Units.rotationsPerMinuteToRadiansPerSecond(m_shooter.getAvgShooterSpeed()) * ShooterConstants.kFlywheelRadius, 
                LinearVelocityUnit.combine(Meters, Seconds)),
                Angle.ofBaseUnits(Units.degreesToRadians(m_shooter.getHoodAngle()), Radians),
                Angle.ofBaseUnits(Units.degreesToRadians(m_turret.getTurretPosition()) + m_robotDrive.getPose().getRotation().getRadians(), Radians),
                Distance.ofBaseUnits(0.5, Meters));}));
    
    // Simulation launch fuel
    SmartDashboard.putData(new InstantCommand(() -> {
        fuelSim.launchFuel(
            LinearVelocity.ofBaseUnits(
                Units.rotationsPerMinuteToRadiansPerSecond(m_shooter.getAvgShooterSpeed()) * ShooterConstants.kFlywheelRadius, 
                LinearVelocityUnit.combine(Meters, Seconds)),
                Angle.ofBaseUnits(Units.degreesToRadians(m_shooter.getHoodAngle()), Radians),
                Angle.ofBaseUnits(Units.degreesToRadians(m_turret.getTurretPosition()) - m_robotDrive.getPose().getRotation().getRadians(), Radians),
                Distance.ofBaseUnits(0.5, Meters));
    })
    .withName("Launch Fuel"));
  }

  private void configureFuelSim() {
    //fuelSim.spawnStartingFuel();
    fuelSim.start();
    SmartDashboard.putData(new InstantCommand(() -> {
        fuelSim.clearFuel();
        //fuelSim.spawnStartingFuel();
    })
    .withName("Reset Fuel")
    .ignoringDisable(true));

    fuelSim.registerRobot(
        Units.inchesToMeters(32), 
        Units.inchesToMeters(32), 
        Units.inchesToMeters(5), 
        m_robotDrive::getPose, 
        m_robotDrive::getRobotRelativeSpeeds);
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
    m_shooter.fastPeriodic();

    ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(m_robotDrive.getRobotRelativeSpeeds(), m_robotDrive.getPose().getRotation());
    currentTarget = LaunchCalc.findTargetOnTheMove(
        m_robotDrive.getPose(), 
        TurretConstants.kHubPose, 
        new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
    publisher.set(currentTarget);
    // publisher2.set(LaunchCalc.findTargetOnTheMove(m_robotDrive.getPose(), TurretConstants.kHubPose, new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), 10));
  }
}
