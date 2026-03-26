// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.commands.AutoAimTurret;
import frc.robot.commands.ManualShooterCommand;
import frc.robot.commands.ManualSpindexerCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.IntakeSubsystem.ArmPosition;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.AutoBuilder;
import frc.robot.utils.FindTarget;
import frc.robot.utils.FuelSim;
import frc.robot.utils.LaunchCalc;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.RunIntake;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem(m_robotDrive::getPose);
  private final TurretSubsystem m_turret = new TurretSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();

  private final XboxController m_driverController = new XboxController(IOConstants.kDriverControllerPort);
  private final XboxController m_operatorController = new XboxController(IOConstants.kOperatorControllerPort);

  private final SendableChooser<Command> m_autoChooser;
  private boolean m_autoAimTurret = false; // swtich back to false later

  public final FuelSim fuelSim;
  private final StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().getStructTopic("SotM Target/5 iterations", Pose2d.struct).publish();
  private Pose2d m_currentTarget = FieldConstants.kHubPose;

  // private final PowerDistribution m_powerDistribution = new PowerDistribution(0, ModuleType.kRev);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // PowerDistribution pd = new PowerDistribution(1, ModuleType.kRev);
    // Configure the trigger bindings
    configureBindings();

    fuelSim = new FuelSim();
    configureFuelSim();

    // Pathplanner auton initialization
    // AutoBuilder.configure(
    //     m_robotDrive::getPose, 
    //     (pose) -> m_robotDrive.resetOdometry(pose), 
    //     () -> m_robotDrive.getRobotRelativeSpeeds(), 
    //     (speeds) -> m_robotDrive.autonDrive(speeds),
    //     new PPHolonomicDriveController(AutonConstants.kTranslationConstants, AutonConstants.kRotationConstants),
    //     AutonConstants.kBotConfig, 
    //     () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
    //     m_robotDrive);
    
    // configureAuton();

    // m_autoChooser = AutoBuilder.buildAutoChooser();

    // BLine Auton
    AutoBuilder autoBuilder = new AutoBuilder(m_robotDrive, m_shooter, m_turret, m_intake, () -> m_currentTarget);
    m_autoChooser = autoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

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
    //     m_turret.calculateSetpoint(m_robotDrive.getPose(), currentTarget, m_robotDrive.getRotationSpeed());
    // },
    // m_turret));
    
    //m_shooter.setDefaultCommand(new ShooterCommand(m_shooter, m_robotDrive::getPose, () -> {return currentTarget;}));

    // Set turret to continuously aim at the hub
    // m_turret.setDefaultCommand(new AutoAimTurret(m_turret, m_robotDrive));
    // m_turret.setDefaultCommand(new RunCommand(() -> {m_turret.setSetpoint(m_turret.getSetpoint() + MathUtil.applyDeadband(m_operatorController.getLeftX(), IOConstants.kControllerDeadband) * 0.5);}, m_turret));
    SmartDashboard.putData(CommandScheduler.getInstance());
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

    // // driver reset odometry
    new JoystickButton(m_driverController, Button.kBack.value)
        .onTrue(new InstantCommand(() -> m_robotDrive.resetOdometry(new Pose2d()), m_robotDrive));


    // new JoystickButton(m_driverController, Button.kLeftBumper.value)
    //     .onTrue(new InstantCommand(() -> {fuelSim.launchFuel(
    //         LinearVelocity.ofBaseUnits(
    //             Units.rotationsPerMinuteToRadiansPerSecond(m_shooter.getAvgShooterSpeed()) * ShooterConstants.kFlywheelRadius, 
    //             LinearVelocityUnit.combine(Meters, Seconds)),
    //             Angle.ofBaseUnits(Units.degreesToRadians(m_shooter.getHoodAngle()), Radians),
    //             Angle.ofBaseUnits(Units.degreesToRadians(m_turret.getTurretPosition()) + m_robotDrive.getPose().getRotation().getRadians(), Radians),
    //             Distance.ofBaseUnits(0.5, Meters));}));
    
    // // Simulation launch fuel
    // SmartDashboard.putData(new InstantCommand(() -> {
    //     fuelSim.launchFuel(
    //         LinearVelocity.ofBaseUnits(
    //             Units.rotationsPerMinuteToRadiansPerSecond(m_shooter.getAvgShooterSpeed()) * ShooterConstants.kFlywheelRadius, 
    //             LinearVelocityUnit.combine(Meters, Seconds)),
    //             Angle.ofBaseUnits(Units.degreesToRadians(m_shooter.getHoodAngle()), Radians),
    //             Angle.ofBaseUnits(Units.degreesToRadians(m_turret.getTurretPosition()) - m_robotDrive.getPose().getRotation().getRadians(), Radians),
    //             Distance.ofBaseUnits(0.5, Meters));
    // })
    // .withName("Launch Fuel"));

    // driver shoot
    new Trigger(() -> {return m_driverController.getRightTriggerAxis() > 0.5;})
        .whileTrue(new ShooterCommand(m_shooter, m_turret, m_robotDrive::getPose, () -> {return m_currentTarget;}));

    
    // intake toggle pivot and run intake
    new Trigger(() -> {return m_driverController.getLeftTriggerAxis() > 0.5;})
        .whileTrue(new RunIntake(m_intake));
    new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .onTrue(new InstantCommand(m_intake::togglePivot));
    
    // operator manual turret
    new JoystickButton(m_operatorController, Button.kB.value)
        .onTrue(new InstantCommand(() -> m_autoAimTurret = false));
    new JoystickButton(m_operatorController, Button.kA.value)
        .onTrue(new InstantCommand(() -> m_autoAimTurret = true));
    new Trigger(() -> {return m_autoAimTurret;})
        .whileTrue(new AutoAimTurret(m_turret, m_robotDrive, () -> m_currentTarget, false))
        .onFalse(new InstantCommand(() -> m_turret.setTarget(100), m_turret));
    
    // operator turret cardinal directions
    new JoystickButton(m_driverController, Button.kY.value)
        .onTrue(new InstantCommand(() -> m_turret.setTarget(TurretConstants.kTurretFrontAngle), m_turret));
    new JoystickButton(m_driverController, Button.kB.value)
        .onTrue(new InstantCommand(() -> m_turret.setTarget(TurretConstants.kTurretRightAngle), m_turret));
    new JoystickButton(m_driverController, Button.kA.value)
        .onTrue(new InstantCommand(() -> m_turret.setTarget(TurretConstants.kTurretBackAngle), m_turret));
    new JoystickButton(m_driverController, Button.kX.value)
        .onTrue(new InstantCommand(() -> m_turret.setTarget(TurretConstants.kTurretLeftAngle), m_turret));

    // new JoystickButton(m_operatorController, Button.kB.value)
    //     .onTrue(new InstantCommand(() -> m_turret.setSetpoint(40)));
    // new JoystickButton(m_operatorController, Button.kA.value)
    //     .onTrue(new InstantCommand(() -> m_turret.setSetpoint(130)));
    // new JoystickButton(m_operatorController, Button.kX.value)
    //     .whileTrue(new RunCommand(() -> m_turret.setSetpoint(140), m_turret));

    // operator manual shoot
    new Trigger(() -> {return m_operatorController.getRightTriggerAxis() > 0.5;})
        .whileTrue(new ManualShooterCommand(m_shooter, m_turret, 3300, 0.3));

    // operator reverse spindexer
    new JoystickButton(m_operatorController, Button.kRightBumper.value)
        .whileTrue(new ManualSpindexerCommand(m_shooter, -1.0, -0.5));
    
    // operator turret joystick (for testing purposes)
    new JoystickButton(m_operatorController, Button.kLeftStick.value)
        .whileTrue(new RunCommand(() -> m_turret.setTarget(m_turret.getTurretPosition()+MathUtil.applyDeadband(m_operatorController.getRightX(), IOConstants.kControllerDeadband) * 1.5), m_turret));

    // operator turret manual fine tuning
    new JoystickButton(m_operatorController, Button.kBack.value)
        .whileTrue(new RunCommand(() -> m_turret.setManualOffset(m_turret.getManualOffset() + MathUtil.applyDeadband(m_operatorController.getRightX(), IOConstants.kControllerDeadband) * 1.5)));

    SmartDashboard.putData("Rotate Turret +90", new InstantCommand(() -> m_turret.setTarget(m_turret.getTurretPosition() + 90)));
    SmartDashboard.putData("Rotate Turret -90", new InstantCommand(() -> m_turret.setTarget(m_turret.getTurretPosition() - 90)));
    
  }

  private void configureAuton() {

    NamedCommands.registerCommand("Score", new ShooterCommand(m_shooter, m_turret, m_robotDrive::getPose, () -> {return m_currentTarget;}).asProxy().withTimeout(20.0));
    NamedCommands.registerCommand("Climb", Commands.none());
    NamedCommands.registerCommand("Ferry", new ShooterCommand(m_shooter, m_turret, m_robotDrive::getPose, () -> {return m_currentTarget;}).asProxy());
    NamedCommands.registerCommand("Intake", new RunIntake(m_intake));
    NamedCommands.registerCommand("Deploy Intake", new InstantCommand(() -> m_intake.setArmPosition(ArmPosition.Extended)));
    NamedCommands.registerCommand("Toggle Auto Aim", new InstantCommand(() -> m_autoAimTurret = true));

    new EventTrigger("Intake")
        .whileTrue(new RunIntake(m_intake));

    new EventTrigger("Score")
        .whileTrue(new ShooterCommand(m_shooter, m_turret, m_robotDrive::getPose, () -> {return m_currentTarget;}).asProxy().withTimeout(20.0));
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

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
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
    m_turret.fastPeriodic();
    m_shooter.fastPeriodic();

    ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(m_robotDrive.getRobotRelativeSpeeds(), m_robotDrive.getPose().getRotation());
    m_currentTarget = LaunchCalc.findTargetOnTheMove(
        m_robotDrive.getPose(), 
        FindTarget.getTarget(m_robotDrive.getPose()), 
        new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
        
    publisher.set(m_currentTarget);
    // publisher2.set(LaunchCalc.findTargetOnTheMove(m_robotDrive.getPose(), TurretConstants.kHubPose, new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), 10));
  }

  public void periodic() {
    
    SmartDashboard.putBoolean("In Blue", FieldConstants.kBlueAllianceRegion.isInRegion(m_robotDrive.getPose()));
    SmartDashboard.putBoolean("In Red", FieldConstants.kRedAllianceRegion.isInRegion(m_robotDrive.getPose()));
    SmartDashboard.putBoolean("In Trench", FieldConstants.kTrenchesRegion.isInRegion(m_robotDrive.getPose()));

    SmartDashboard.putBoolean("Should Score Hub", FindTarget.shouldScoreHub(m_robotDrive.getPose()));
    SmartDashboard.putBoolean("Auto Aim Enabled", m_autoAimTurret);
  }

  public void reset() {
    m_turret.reset();
    m_shooter.reset();
  }
}

