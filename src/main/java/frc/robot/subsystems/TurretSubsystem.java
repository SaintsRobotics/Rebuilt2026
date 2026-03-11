// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {

  private final SparkMax m_turretMotor = new SparkMax(TurretConstants.kTurretMotorPort, MotorType.kBrushless);
  private final PIDController m_turretPID = new PIDController(TurretConstants.kTurretP, TurretConstants.kTurretI, TurretConstants.kTurretD);
  private final SimpleMotorFeedforward m_turretFeedforward = new SimpleMotorFeedforward(TurretConstants.kTurretS, TurretConstants.kTurretV);

  // CRT Encoders
  private final CANcoder m_encoder1;
  private final CANcoder m_encoder2;


  private double relativeAngularVelocity;
  private double previousTargetAngle;  
  private double lastCalculatedPosition;

  // simulation classes
  private final DCMotor m_motorSim = DCMotor.getNeo550(1);
  private final SingleJointedArmSim m_turretSim = new SingleJointedArmSim(
    m_motorSim,
    TurretConstants.kTurretSimGearRatio, 
    TurretConstants.kTurretSimMOI,    
    TurretConstants.kTurretSimLength,   
    -Units.degreesToRadians(TurretConstants.kTurretMaxRotation / 2),  
    Units.degreesToRadians(TurretConstants.kTurretMaxRotation / 2),  
    false,  
    Units.degreesToRadians(0));  
  private final SparkMaxSim m_turretMotorSim = new SparkMaxSim(m_turretMotor, m_motorSim);

  private final StructPublisher<Pose3d> m_turretCurrentPublisher =
    NetworkTableInstance.getDefault().getStructTopic("Turret/Current", Pose3d.struct).publish();
  private final StructPublisher<Pose3d> m_turretTargetPublisher =
    NetworkTableInstance.getDefault().getStructTopic("Turret/Target", Pose3d.struct).publish();


  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {
    m_encoder1 = new CANcoder(TurretConstants.kEncoder1CANId);
    m_encoder2 = new CANcoder(TurretConstants.kEncoder2CANId);
    if (RobotBase.isReal()) {
      

      double crtPosition = calculateTurretPosition();

      SparkMaxConfig motorConfig = new SparkMaxConfig();
      motorConfig.encoder.positionConversionFactor(360);
      motorConfig.idleMode(IdleMode.kBrake);
      m_turretMotor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

      m_turretMotor.getEncoder().setPosition(crtPosition);
      m_turretPID.setSetpoint(crtPosition);
    } 
    else {
      SparkMaxConfig motorConfig = new SparkMaxConfig();
      motorConfig.encoder.positionConversionFactor(360);
      motorConfig.idleMode(IdleMode.kBrake);
      m_turretMotor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      m_turretMotor.getEncoder().setPosition(0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double pos = calculateTurretPosition();
    SmartDashboard.putNumber("Turret/Turret Angle", pos);
    SmartDashboard.putNumber("Turret/Encoder 1 Angle", m_encoder1.getAbsolutePosition().getValueAsDouble()*360);
    SmartDashboard.putNumber("Turret/Encoder 2 Angle", m_encoder2.getAbsolutePosition().getValueAsDouble()*360);

    SmartDashboard.putNumber("Turret/Turret Setpoint", m_turretPID.getSetpoint());
    // SmartDashboard.putNumber("Turret Error", getError());
    // SmartDashboard.putNumber("Turret/Turret Output", output);
  }

  public void fastPeriodic() {
    double setpoint = m_turretPID.getSetpoint();
    double output = m_turretPID.calculate(getTurretPosition(), setpoint) 
                    + Math.signum(m_turretPID.getError())
                    * Math.abs(m_turretFeedforward.calculate(relativeAngularVelocity));

    // Apply deadband
    double error = setpoint - getTurretPosition();
    if (Math.abs(error) < TurretConstants.kTurretDeadband) {
      // output = 0;
    }

    output = MathUtil.clamp(output, -TurretConstants.kTurretMaxSpeed, TurretConstants.kTurretMaxSpeed);
    m_turretMotor.set(output);

    SmartDashboard.putNumber("Turret/output", output);
  }

  @Override
  public void simulationPeriodic() {
    // set inputs and update simulation
    m_turretSim.setInput(m_turretMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage());
    m_turretSim.update(0.02);

    // update the SparkMaxSim
    m_turretMotorSim.iterate(
      Units.radiansPerSecondToRotationsPerMinute(m_turretSim.getVelocityRadPerSec()), 
      RobotController.getBatteryVoltage(), 
      0.02);
    m_turretMotorSim.getRelativeEncoderSim().setPosition(Units.radiansToDegrees(m_turretSim.getAngleRads()));

    m_encoder1.getSimState().addPosition(
      Radians.of(
        m_turretSim.getVelocityRadPerSec() 
        * TurretConstants.kEncoder1Ratio));
    m_encoder2.getSimState().addPosition(
      Radians.of(
        m_turretSim.getVelocityRadPerSec() 
        * TurretConstants.kEncoder2Ratio));

    // update battery
    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(m_turretSim.getCurrentDrawAmps()));
  }

  // Calculates and sets the turret setpoint
  public void calculateSetpoint(Pose2d robotPose, Pose2d targetPose, double robotAngularVelocity) {
    // Calculate turret location on the field
    Pose2d rotatedOffset = TurretConstants.kTurretOffset.rotateBy(robotPose.getRotation());
    Pose2d turretLocation = new Pose2d(
      robotPose.getTranslation().plus(rotatedOffset.getTranslation()),
      robotPose.getRotation());
    Pose2d targetDistance = targetPose.relativeTo(turretLocation);
    double robotRelativeAngle = targetDistance.getTranslation().getAngle().getDegrees();

    // Normalize to -180 to 180 range
    robotRelativeAngle = MathUtil.inputModulus(robotRelativeAngle, -180, 180);

    // Find best target angle
    double currentPosition = getTurretPosition();
    double targetAngle = findBestAngle(robotRelativeAngle, currentPosition);
    targetAngle = MathUtil.clamp(targetAngle, -TurretConstants.kTurretMaxRotation / 2, TurretConstants.kTurretMaxRotation / 2);
    m_turretPID.setSetpoint(targetAngle);

    // Calculate feedforward velocity
    relativeAngularVelocity = (targetAngle - previousTargetAngle) / 0.02 - robotAngularVelocity;
    previousTargetAngle = targetAngle;

    // Advantagescope visualization
    double currentFieldHeading = robotPose.getRotation().getDegrees() + getTurretPosition();
    double targetFieldHeading = robotPose.getRotation().getDegrees() + targetAngle;
    Pose3d turret3d = new Pose3d(
      turretLocation.getX(),
      turretLocation.getY(),
      0.5,
      new Rotation3d(0, 0, Units.degreesToRadians(currentFieldHeading)));
    Pose3d turretTarget3d = new Pose3d(
      turretLocation.getX(),
      turretLocation.getY(),
      0.6, 
      new Rotation3d(0, 0, Units.degreesToRadians(targetFieldHeading)));
    m_turretCurrentPublisher.set(turret3d);
    m_turretTargetPublisher.set(turretTarget3d);
  }

  // Finds best angle within physical limits
  private double findBestAngle(double targetAngle, double currentPosition) {
    double maxRotation = TurretConstants.kTurretMaxRotation / 2;
    double[] candidates = {
      targetAngle - 360,
      targetAngle,
      targetAngle + 360
    };
    double bestInBounds = Double.NaN;
    double minDist = Double.MAX_VALUE;
    for (double candidate : candidates) {
      if (Math.abs(candidate) <= maxRotation) {
        double moveDist = Math.abs(candidate - currentPosition);
        if (moveDist < minDist) {
          minDist = moveDist;
          bestInBounds = candidate;
        }
      }
    }
    if (!Double.isNaN(bestInBounds)) {
      return bestInBounds;
    }
    // If is out of bounds, figure out where we want to be anyways
    if (currentPosition > 0) {
      double targetOnNegativeSide = targetAngle < 0 ? targetAngle : targetAngle - 360;
      if (targetOnNegativeSide < -(maxRotation - 90.0)) {
        return -maxRotation;
      }
    } else {
      double targetOnPositiveSide = targetAngle > 0 ? targetAngle : targetAngle + 360;
      if (targetOnPositiveSide > (maxRotation - 90.0)) {
        return maxRotation;
      }
    }

    return currentPosition;
  }

  public double getTurretPosition() {
    // return m_turretMotor.getEncoder().getPosition();
    return calculateTurretPosition();
  }

  // Returns the current error between setpoint and position
  public double getError() {
    return m_turretPID.getSetpoint() - getTurretPosition();
  }

  // Set turret setpoint
  public void setSetpoint(double angle) {
    double clampedAngle = MathUtil.clamp(angle, -TurretConstants.kTurretMaxRotation / 2, TurretConstants.kTurretMaxRotation / 2);
    m_turretPID.setSetpoint(clampedAngle);
  }

  public double getSetpoint() {
    return m_turretPID.getSetpoint();
  }

  // Reset turret encoder
  public void resetEncoder() {
    m_turretMotor.getEncoder().setPosition(0);
  }

  public boolean atSetpoint() {
    return Math.abs(getError()) < TurretConstants.kTurretTolerance;
  }

  // Calculates turret position with whatever this method is called
  private double calculateTurretPosition() {
    if (m_encoder1 == null || m_encoder2 == null) {
      return 0.0;
    }
    // Read encoder positions
    double enc1 = m_encoder1.getAbsolutePosition().getValueAsDouble(); //* 360.0 - TurretConstants.kEncoder1OffsetDegrees;
    double enc2 = m_encoder2.getAbsolutePosition().getValueAsDouble(); //* 360.0 - TurretConstants.kEncoder2OffsetDegrees;
    double diff = enc1 - enc2;
    // handle wraparound
    if (diff < 0) {
      diff += 1.0;
    }
    return diff * (1/(TurretConstants.kEncoder1Ratio - TurretConstants.kEncoder2Ratio)) * 360.0;
  }

}
