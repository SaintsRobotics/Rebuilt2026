// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ShooterConstants;



public class ShooterSubsystem extends SubsystemBase {

    //shooter motors
    SparkFlex m_shooterMotorLeft = new SparkFlex(ShooterConstants.kShooterMotorLeftPort, MotorType.kBrushless);
    SparkFlex m_shooterMotorRight = new SparkFlex(ShooterConstants.kShooterMotorRightPort, MotorType.kBrushless);

    //hood motor
    SparkMax m_hoodMotor = new SparkMax(ShooterConstants.kHoodMotorPort, MotorType.kBrushless);
    private DutyCycleEncoder m_hoodEncoder = new DutyCycleEncoder(ShooterConstants.kHoodEncoderChannel, ShooterConstants.kHoodAngleMax, ShooterConstants.kHoodAngleMin);    
    
    //PID controllers and feedforward for shooter speed and hood angle
    private final PIDController m_shooterPID = new PIDController(ShooterConstants.kShooterP, ShooterConstants.kShooterI, ShooterConstants.kShooterD);
    private final SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(ShooterConstants.kShooterS, ShooterConstants.kShooterV);

    private final PIDController m_hoodAnglePID = new PIDController(ShooterConstants.kHoodAngleP, ShooterConstants.kHoodAngleI, ShooterConstants.kHoodAngleD);

    // Simulation classes
    private final DCMotor m_flywheelDCMotor = DCMotor.getNeoVortex(2);
    private final FlywheelSim m_flywheelSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            m_flywheelDCMotor, 
            ShooterConstants.kFlywheelMOI, 
            ShooterConstants.kFlywheelGearing), 
        m_flywheelDCMotor);
    private final SparkFlexSim m_leftMotorSim = new SparkFlexSim(m_shooterMotorLeft, DCMotor.getNeoVortex(1));
    private final SparkFlexSim m_rightMotorSim = new SparkFlexSim(m_shooterMotorRight, DCMotor.getNeoVortex(1));

    //constructor
    public ShooterSubsystem() {

        SparkFlexConfig motorConfig = new SparkFlexConfig();
        motorConfig.idleMode(IdleMode.kCoast);
        m_shooterMotorLeft.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        motorConfig.inverted(true);
        m_shooterMotorRight.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    }

    //resets shooter
    public void reset() {
        
        shooterStop();

    }

    //stops shooter
    public void shooterStop() {

        m_shooterMotorLeft.set(0);
        m_shooterMotorRight.set(0);

    }

    //sets shooter parameters for PIDF control
    public void setShooterParameters(double flywheelSpeed, double hoodAngle) {

        m_shooterPID.setSetpoint(flywheelSpeed);
        m_hoodAnglePID.setSetpoint(hoodAngle);

    }

    //sets the shooter motors at given speed
    public void setShooterMotors(double speed) {

        m_shooterMotorLeft.set(speed);
        m_shooterMotorRight.set(speed);

    }

    //returns speed of the shooter as a average of the speeds of the two motors
    public double getAvgShooterSpeed() {
        return (m_shooterMotorLeft.get() + m_shooterMotorRight.get())/2;
    }

    //returns speed of the left shooter motor
    public double getLeftShooterSpeed() {
        return m_shooterMotorLeft.get();
    }

    //returns speed of the right shooter motor
    public double getRightShooterSpeed() {
        return m_shooterMotorRight.get();
    }

    //sets hood angle by setting the setpoint of the hood angle PID controller
    public void setHoodAngle(double angle) {

        m_hoodAnglePID.setSetpoint(angle);

    }

    //sets hood motor to given speed
    public void setHoodMotor(double speed) {

        m_hoodMotor.set(speed);

    }

    //returns hood angle from the hood encoder
    public double getHoodAngle() {
        return m_hoodEncoder.get();
    }

    //periodic
    public void periodic() {

        //calculates PIDF output and sets shooter motors to that output
        double flywheelPIDValue = m_shooterPID.calculate(getLeftShooterSpeed());
        double ffSpeed = m_shooterFeedforward.calculate(m_shooterPID.getSetpoint());

        setShooterMotors(MathUtil.clamp(flywheelPIDValue + ffSpeed, ShooterConstants.kMinSpeed, ShooterConstants.kMaxSpeed));

        //calculates PID output for hood angle and sets hood motor to that output
        double hoodPIDValue = m_hoodAnglePID.calculate(getHoodAngle());
        setHoodMotor(hoodPIDValue);

    }

    @Override
    public void simulationPeriodic() {
        // set inputs and update by one timestep
        m_flywheelSim.setInput(m_leftMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage());
        m_flywheelSim.update(0.02);

        // update SparkFlexSim
        m_leftMotorSim.iterate(m_flywheelSim.getAngularVelocityRPM(), RobotController.getBatteryVoltage(), 0.02);
        m_rightMotorSim.iterate(m_flywheelSim.getAngularVelocityRPM(), RobotController.getBatteryVoltage(), 0.02);

        // Update battery
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_flywheelSim.getCurrentDrawAmps()));
    }

}

