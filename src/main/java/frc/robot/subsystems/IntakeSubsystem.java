// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.revrobotics.PersistMode; 

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;

import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;


public class IntakeSubsystem extends SubsystemBase {
  private final SparkFlex m_intakeMotor = new SparkFlex(IntakeConstants.kIntakeMotorID, MotorType.kBrushless);
  private final SparkFlex m_armMotor = new SparkFlex(IntakeConstants.kArmMotorID  , MotorType.kBrushless);

  private final PIDController m_armPID = new PIDController(0.02, 0, 0);

  private CANcoder m_armEncoder = new CANcoder(IntakeConstants.kArmEncoderChannel);


  private ArmPosition m_armPosition = ArmPosition.Retracted;

  private final DCMotor m_motorsim = DCMotor.getNeoVortex(1);
  //fix params if needed
  private final SingleJointedArmSim m_intakeSim = new SingleJointedArmSim(m_motorsim,
  IntakeConstants.kArmReduction,       
  IntakeConstants.kArmMOI,
  IntakeConstants.kIntakeLength,
  Units.degreesToRadians(IntakeConstants.kIntakeRaisedAngle), 
  Units.degreesToRadians(IntakeConstants.kIntakeLoweredAngle), 
  true, 
  getArmPosition());
  private final SparkFlexSim m_armMotorSim = new SparkFlexSim(m_armMotor, m_motorsim);
  //private final SparkFlexSim m_rollerMotorSim = new SparkFlexSim(m_intakeMotor, m_motorsim);
  private final CANcoderSimState m_intakeEncoderSim = new CANcoderSimState(m_armEncoder);
  private final MechanismLigament2d m_armMech;
  //private final MechanismLigament2d m_rollerMech;

  private double m_intakeSpeed = 0;
  private double m_armSetpoint = IntakeConstants.kIntakeRaisedAngle;

  /** Creates a new IntakeSubsystem */
  public IntakeSubsystem() {
    SparkFlexConfig intakeConfig = new SparkFlexConfig();
    intakeConfig.idleMode(IdleMode.kCoast);

    m_armEncoder.getConfigurator().apply(new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.Clockwise_Positive));
    //m_armPID.enableContinuousInput(0, 360);

    m_intakeMotor.configure(intakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkFlexConfig armConfig = new SparkFlexConfig();
    armConfig.inverted(true);
    armConfig.idleMode(IdleMode.kBrake);
    m_armMotor.configure(armConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);    

    m_armPID.setTolerance(10);

    m_armSetpoint = 10;

    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.MagnetOffset = IntakeConstants.kArmEncoderOffset;
    m_armEncoder.getConfigurator().apply(config);

    //sim
    Mechanism2d intakeMech = new Mechanism2d(1, 1);
    MechanismRoot2d intakeMechRoot = intakeMech.getRoot("pivot", 0.7, 0.7); //position is weird but it doesn't really matter

    
    m_armMech = intakeMechRoot.append(new MechanismLigament2d("Arm", Units.inchesToMeters(IntakeConstants.kIntakeLength), 60, 5, new Color8Bit(Color.kBlue))); //INWERT PARAMS LATER

    //m_rollerMech = m_armMech.append(new MechanismLigament2d("Intake Bar", 0.3, 30, 9, new Color8Bit(Color.kYellow))); //IDK???!

    SmartDashboard.putData("Intake Mechanism", intakeMech);
  }

  @Override
  public void periodic() {
    m_armPID.setTolerance(10);
    m_armPID.setP(0.002);
    m_armPID.setD(0);


    m_armPID.setSetpoint(m_armSetpoint);
    double armMotorSpeed = m_armPID.atSetpoint() ? 0 : MathUtil.clamp(m_armPID.calculate(m_armEncoder.getAbsolutePosition().getValueAsDouble()), -0.3, 0.3);

    SmartDashboard.putNumber("Motor Speed", armMotorSpeed);
    SmartDashboard.putNumber("Arm Angle", m_armEncoder.getAbsolutePosition().getValueAsDouble());

    // SmartDashboard.putNumber("pid output", armMotorSpeed);

    m_armMotor.set(armMotorSpeed);
    m_intakeMotor.set(m_intakeSpeed);
  }

  @Override
  public void simulationPeriodic() {
    m_intakeSim.setInput(m_armMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage());
    m_intakeSim.update(0.02);

    m_armMotorSim.iterate(Units.radiansPerSecondToRotationsPerMinute(m_intakeSim.getVelocityRadPerSec()), 
    RobotController.getBatteryVoltage(), 
    0.02);


    //double armDegrees = Units.radiansToDegrees(m_intakeSim.getAngleRads());
    m_armMech.setAngle(m_armSetpoint);
    //m_rollerMech.setAngle(-armDegrees);

    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(m_intakeSim.getCurrentDrawAmps()));

    /*   
     m_rollerMotorSim.setInputVoltage(m_intakeMotor.get() * RobotController.getBatteryVoltage());


    m_rollerMotorSim.update(0.020);

    double simVelocity = m_rollerMotorSim.getAngularVelocityRadPerSec();

    if (Math.abs(simVelocity) > 1.0) { 
        m_rollerLigament.setColor(new Color8Bit(Color.kGreen));
    } else {
        m_rollerLigament.setColor(new Color8Bit(Color.kYellow));
    } */
    


  }
  

  public void reset() {
    m_intakeSpeed = 0;
    m_armSetpoint = getArmPosition();
  }

  public void setArmPosition(ArmPosition position) {
    m_armPosition = position;
    switch (position) {
      case Extended:
        m_armSetpoint = IntakeConstants.kIntakeLoweredAngle;
        break;
      case Retracted:
        m_armSetpoint = IntakeConstants.kIntakeRaisedAngle;
      default:
        break;
    }

    m_armPID.setSetpoint(m_armSetpoint);
  }

  public double getArmPosition() {
    return m_armSetpoint;
  }

  public boolean armAtSetpoint() {
    return m_armPID.atSetpoint();
  }

  public void intake() {
    m_intakeSpeed = IntakeConstants.kIntakeSpeed;
  }

  public void stopIntake() {
    m_intakeSpeed = 0;
  }

  public void setIntakeMotor(double speed) {
    m_intakeMotor.set(speed);
  }



  public static enum ArmPosition {
    Extended,
    Retracted
  }
}

