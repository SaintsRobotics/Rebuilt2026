// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.CANcoder;
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

  //private final SparkFlex m_turretMotor = new SparkFlex(TurretConstants.kTurretMotorPort, MotorType.kBrushless);
  private final PIDController m_turretPID = new PIDController(TurretConstants.kTurretP, TurretConstants.kTurretI, TurretConstants.kTurretD);
  private final SimpleMotorFeedforward m_turretFeedforward = new SimpleMotorFeedforward(TurretConstants.kTurretS, TurretConstants.kTurretV);

  // CRT Encoders
  private final CANcoder m_encoder1;
  private final CANcoder m_encoder2;

  private double relativeAngularVelocity;
  private double previousTargetAngle;  
  private double lastCalculatedPosition;

  // simulation classes
  private final DCMotor m_motorSim = DCMotor.getNeoVortex(1);
  private final SingleJointedArmSim m_turretSim = new SingleJointedArmSim(
    m_motorSim,
    TurretConstants.kTurretSimGearRatio, 
    TurretConstants.kTurretSimMOI,    
    TurretConstants.kTurretSimLength,   
    -Units.degreesToRadians(TurretConstants.kTurretMaxRotation / 2),  
    Units.degreesToRadians(TurretConstants.kTurretMaxRotation / 2),  
    false,  
    Units.degreesToRadians(0));  
  // private final SparkFlexSim m_turretMotorSim = new SparkFlexSim(m_turretMotor, m_motorSim);

  private final StructPublisher<Pose3d> m_turretCurrentPublisher =
    NetworkTableInstance.getDefault().getStructTopic("Turret/Current", Pose3d.struct).publish();
  private final StructPublisher<Pose3d> m_turretTargetPublisher =
    NetworkTableInstance.getDefault().getStructTopic("Turret/Target", Pose3d.struct).publish();


  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {
    if (RobotBase.isReal()) {
      m_encoder1 = new CANcoder(TurretConstants.kEncoder1CANId);
      m_encoder2 = new CANcoder(TurretConstants.kEncoder2CANId);

      double crtPosition = calculateCRTPosition();

      SparkFlexConfig motorConfig = new SparkFlexConfig();
      motorConfig.encoder.positionConversionFactor(360);
      motorConfig.idleMode(IdleMode.kBrake);
      //m_turretMotor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

      // m_turretMotor.getEncoder().setPosition(crtPosition);
    } 
    else {
      m_encoder1 = null;
      m_encoder2 = null;

      SparkFlexConfig motorConfig = new SparkFlexConfig();
      motorConfig.encoder.positionConversionFactor(360);
      motorConfig.idleMode(IdleMode.kBrake);
      // m_turretMotor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      // m_turretMotor.getEncoder().setPosition(0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double setpoint = m_turretPID.getSetpoint();
    double output = m_turretPID.calculate(getTurretPosition(), setpoint) +
                    m_turretFeedforward.calculate(relativeAngularVelocity);

    // Apply deadband
    double error = setpoint - getTurretPosition();
    if (Math.abs(error) < TurretConstants.kTurretDeadband) {
      output = 0;
    }

    output = MathUtil.clamp(output, -TurretConstants.kTurretMaxSpeed, TurretConstants.kTurretMaxSpeed);
    // m_turretMotor.set(output);

    double pos = calculateCRTPosition();
    // if (Math.abs(lastCalculatedPosition - pos) > TurretConstants.kEncoderMaxDelta) {
    //   pos = lastCalculatedPosition;
    // }
    // lastCalculatedPosition = pos;

    SmartDashboard.putNumber("Turret Angle", pos);
    SmartDashboard.putNumber("Turret Error", calculateCRTError());
    SmartDashboard.putNumber("Encoder 1 Angle", m_encoder1.getAbsolutePosition().getValueAsDouble()*360);
    SmartDashboard.putNumber("Encoder 2 Angle", m_encoder2.getAbsolutePosition().getValueAsDouble()*360);

    // SmartDashboard.putNumber("Turret Setpoint", m_turretPID.getSetpoint());
    // SmartDashboard.putNumber("Turret Error", getError());
    // SmartDashboard.putNumber("Turret Output", output);

    //double period1 = 360.0 / TurretConstants.kEncoder1Ratio; // 360 / (90/13) = 52
    //double period2 = 360.0 / TurretConstants.kEncoder2Ratio; // 360 / (90/14) = 56
    // // Search for solution
    // double bestSolution = 0.0;
    // double minError = Double.MAX_VALUE;
    // for (double candidate = -364; candidate <= 364; candidate += period1) {
    //   double testAngle = turretFromEnc1 + candidate;
    //   double error2 = Math.abs(((testAngle - turretFromEnc2) % period2 + period2) % period2);
    //   error2 = Math.min(error2, period2 - error2);
    //   if (error2 < minError) {
    //     minError = error2;
    //     bestSolution = testAngle;
    //   }
    // }
  }

  @Override
  public void simulationPeriodic() {
    // set inputs and update simulation
    // m_turretSim.setInput(m_turretMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage());
    m_turretSim.update(0.02);

    // update turret the SparkFlexSim
    // m_turretMotorSim.iterate(
    //   Units.radiansPerSecondToRotationsPerMinute(m_turretSim.getVelocityRadPerSec()), 
    //   RobotController.getBatteryVoltage(), 
    //   0.02);
    // m_turretMotorSim.getRelativeEncoderSim().setPosition(Units.radiansToDegrees(m_turretSim.getAngleRads()));

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
    return 0;
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

  // Reset turret encoder
  public void resetEncoder() {
    // m_turretMotor.getEncoder().setPosition(0);
  }

  public boolean atSetpoint() {
    return Math.abs(getError()) < TurretConstants.kTurretTolerance;
  }

  // Calculates turret position with CRT
  private double calculateCRTPosition() {
    if (m_encoder1 == null || m_encoder2 == null) {
      return 0.0;
    }

    // Read encoder positions
    double enc1 = m_encoder1.getAbsolutePosition().getValueAsDouble() * 360.0 - TurretConstants.kEncoder1OffsetDegrees;
    double enc2 = m_encoder2.getAbsolutePosition().getValueAsDouble() * 360.0 - TurretConstants.kEncoder2OffsetDegrees;

    // Normalize range
    enc1 = ((enc1 % 360.0) + 360.0) % 360.0;
    enc2 = ((enc2 % 360.0) + 360.0) % 360.0;

    // Calculate periods
    double turretFromEnc1 = enc1 / TurretConstants.kEncoder1Ratio;
    double turretFromEnc2 = enc2 / TurretConstants.kEncoder2Ratio;
    double period1 = 360.0 / TurretConstants.kEncoder1Ratio; // 360 / (90/13) = 52
    double period2 = 360.0 / TurretConstants.kEncoder2Ratio; // 360 / (90/14) = 56

    // Search for solution
    double bestSolution = 0.0;
    double minError = Double.MAX_VALUE;

    // for (double rotations = -10; rotations <= 10; rotations++){
    //   double enc1Rot = enc1/360 + rotations;
    //   double turretRot = enc1Rot / TurretConstants.kEncoder1Ratio;
    //   double enc2Rot = turretRot * TurretConstants.kEncoder2Ratio;
    //   double error = Math.abs((enc2Rot - Math.floor(enc2Rot)) - enc2/360);
    //   if (error < minError) {
    //     minError = error;
    //     bestSolution = turretRot * 360;
    //   }
    // }
    for (double candidate = -364; candidate <= 364; candidate += period1) {
      double testAngle = turretFromEnc1 + candidate;
      double error2 = Math.abs(((testAngle - turretFromEnc2) % period2 + period2) % period2);
      error2 = Math.min(error2, period2 - error2);
      if (error2 < minError) {
        minError = error2;
        bestSolution = testAngle;
      }
    }
    return bestSolution;
  }

    // Calculates turret position with CRT
  private double calculateCRTError() {
    if (m_encoder1 == null || m_encoder2 == null) {
      return 0.0;
    }

    // Read encoder positions
    double enc1 = m_encoder1.getAbsolutePosition().getValueAsDouble() * 360.0 - TurretConstants.kEncoder1OffsetDegrees;
    double enc2 = m_encoder2.getAbsolutePosition().getValueAsDouble() * 360.0 - TurretConstants.kEncoder2OffsetDegrees;

    // Normalize range
    enc1 = ((enc1 % 360.0) + 360.0) % 360.0;
    enc2 = ((enc2 % 360.0) + 360.0) % 360.0;

    // Calculate periods
    double turretFromEnc1 = enc1 / TurretConstants.kEncoder1Ratio;
    double turretFromEnc2 = enc2 / TurretConstants.kEncoder2Ratio;
    double period1 = 360.0 / TurretConstants.kEncoder1Ratio; // 360 / (90/13) = 52
    double period2 = 360.0 / TurretConstants.kEncoder2Ratio; // 360 / (90/14) = 56

    // Search for solution
    double bestSolution = 0.0;
    double minError = Double.MAX_VALUE;
    for (double candidate = -364; candidate <= 364; candidate += period1) {
      double testAngle = turretFromEnc1 + candidate;
      double error2 = Math.abs(((testAngle - turretFromEnc2) % period2 + period2) % period2);
      error2 = Math.min(error2, period2 - error2);
      if (error2 < minError) {
        minError = error2;
        bestSolution = testAngle;
      }
    }
    return minError;
  }

}
