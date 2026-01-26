// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {

  private final SparkFlex m_turretMotor = new SparkFlex(0, MotorType.kBrushless);
  private final PIDController m_turretPID = new PIDController(TurretConstants.kTurretP, 0, TurretConstants.kTurretD);
  private final SimpleMotorFeedforward m_turretFeedforward = new SimpleMotorFeedforward(TurretConstants.kTurretS, TurretConstants.kTurretV);

  private double relativeAngularVelocity;
  private double previousTargetAngle;

  // simulation classes
  private final DCMotor m_motorSim = DCMotor.getNeoVortex(1);
  private final SingleJointedArmSim m_turretSim = new SingleJointedArmSim(
    m_motorSim, 
    1, 
    1, 
    5, 
    -Units.degreesToRadians(TurretConstants.kTurretMaxRotation), 
    Units.degreesToRadians(TurretConstants.kTurretMaxRotation), 
    false, 
    0);
  private final SparkFlexSim m_turretMotorSim = new SparkFlexSim(m_turretMotor, m_motorSim);
  private final MechanismLigament2d m_turretMech;
  private final MechanismLigament2d m_turretMechGoal;

  private final StructArrayPublisher<Pose3d> publisher = NetworkTableInstance.getDefault().getStructArrayTopic("Turret Poses", Pose3d.struct).publish();


  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {
    // configure motor
    SparkFlexConfig motorConfig = new SparkFlexConfig();
    motorConfig.encoder.positionConversionFactor(360);
    motorConfig.idleMode(IdleMode.kBrake);
    m_turretMotor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    // simulation
    Mechanism2d mech2d = new Mechanism2d(1, 1);
    MechanismRoot2d mechRoot = mech2d.getRoot("turretRoot", 0.5, 0.5);
    m_turretMech = mechRoot.append(new MechanismLigament2d("turret", 0.4, 0));
    m_turretMechGoal = mechRoot.append(new MechanismLigament2d("turret goal", 0.4, 0, 1, new Color8Bit(Color.kRed)));
    SmartDashboard.putData("Turret Mechanism", mech2d);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double output = MathUtil.clamp(
      m_turretPID.calculate(getTurretPosition()) + m_turretFeedforward.calculate(relativeAngularVelocity), 
      -TurretConstants.kTurretMaxSpeed, 
      TurretConstants.kTurretMaxSpeed);
    m_turretMotor.set(output);

    SmartDashboard.putNumber("Turret Angle", getTurretPosition());
    SmartDashboard.putNumber("Turret Setpoint", m_turretPID.getSetpoint());
  }

  @Override
  public void simulationPeriodic() {
    // set inputs and update simulation
    m_turretSim.setInput(m_turretMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage());
    m_turretSim.update(0.02);

    // update the SparkFlexSim
    m_turretMotorSim.iterate(
      Units.radiansPerSecondToRotationsPerMinute(m_turretSim.getVelocityRadPerSec()), 
      RobotController.getBatteryVoltage(), 
      0.02);
    m_turretMotorSim.getRelativeEncoderSim().setPosition(Units.radiansToDegrees(m_turretSim.getAngleRads()));

    // update battery
    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(m_turretSim.getCurrentDrawAmps()));
    
    // update mechanism2d
    m_turretMech.setAngle(Units.radiansToDegrees(m_turretSim.getAngleRads()));
    m_turretMechGoal.setAngle(m_turretPID.getSetpoint());
  }

  /** Calculates and sets the turret setpoint to track the location of the target. **/
  public void calculateSetpoint(Pose2d robotPose, Pose2d targetPose, double newAngularVelocity) {
    Pose2d rotatedOffset = TurretConstants.kTurretOffset.rotateBy(robotPose.getRotation());
    Pose2d turretLocation = robotPose.plus(new Transform2d(rotatedOffset.getTranslation(), rotatedOffset.getRotation()));
    Pose2d targetDistance = targetPose.relativeTo(turretLocation);
    double targetAngle = targetDistance.getTranslation().getAngle().getDegrees();
    double shortestPath = Units.radiansToDegrees(MathUtil.angleModulus(Units.degreesToRadians(targetAngle) - Units.degreesToRadians(getTurretPosition())));
    if (Math.abs(getTurretPosition() + shortestPath) < TurretConstants.kTurretMaxRotation) {
      targetAngle = getTurretPosition() + shortestPath;
    }
    m_turretPID.setSetpoint(targetAngle);

    relativeAngularVelocity = (targetAngle - previousTargetAngle) / 0.02;
    previousTargetAngle = targetAngle;

    SmartDashboard.putNumber("Turret Shortest Path", shortestPath);
    // publishing Pose3ds of the target angle and the current angle to NetworkTables; mainly for AdvantageScope
    Pose3d turretTarget3d = 
      new Pose3d(turretLocation)
      .plus(new Transform3d(
        new Translation3d(0, 0, 1), 
        new Rotation3d(new Rotation2d(Units.degreesToRadians(targetAngle)))));
    Pose3d turret3d = 
      new Pose3d(turretLocation)
      .plus(new Transform3d(
        new Translation3d(0, 0, 0.8), 
        new Rotation3d(new Rotation2d(Units.degreesToRadians(getTurretPosition())))));
    publisher.set(new Pose3d[] {turretTarget3d, turret3d});
  }

  public double getTurretPosition() {
    return m_turretMotor.getEncoder().getPosition();
  }
}
