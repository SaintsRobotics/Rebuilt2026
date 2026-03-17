// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Robot;
import frc.robot.Constants.ShooterConstants;



public class ShooterSubsystem extends SubsystemBase {

    //shooter motors
    private final SparkFlex m_shooterMotorLeft = new SparkFlex(ShooterConstants.kShooterMotorLeftPort, MotorType.kBrushless);
    private final SparkFlex m_shooterMotorRight = new SparkFlex(ShooterConstants.kShooterMotorRightPort, MotorType.kBrushless);

    //hood motor
    private final SparkMax m_hoodMotor = new SparkMax(ShooterConstants.kHoodMotorPort, MotorType.kBrushless);
    private final CANcoder m_hoodEncoder = new CANcoder(ShooterConstants.kHoodEncoderChannel);    
    
    // spindexer and transfer
    private final SparkFlex m_spindexerMotor = new SparkFlex(ShooterConstants.kSpindexerPort, MotorType.kBrushless);
    private final SparkFlex m_transferMotor = new SparkFlex(ShooterConstants.kTransferPort, MotorType.kBrushless);

    //PID controllers and feedforward for shooter speed and hood angle
    private final PIDController m_shooterPID = new PIDController(ShooterConstants.kShooterP, ShooterConstants.kShooterI, ShooterConstants.kShooterD, Constants.kFastPeriodicPeriod);
    private final SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(ShooterConstants.kShooterS, ShooterConstants.kShooterV);

    // TODO: tune these lol
    private final PIDController m_spindexerPID = new PIDController(0.01, 0, 0);
    private final SimpleMotorFeedforward m_spindexerFF = new SimpleMotorFeedforward(0, 0.01);

    private final PIDController m_transferPID = new PIDController(0.01, 0, 0);
    private final SimpleMotorFeedforward m_transferFF = new SimpleMotorFeedforward(0, 0.01);

    private final PIDController m_hoodAnglePID = new PIDController(ShooterConstants.kHoodAngleP, ShooterConstants.kHoodAngleI, ShooterConstants.kHoodAngleD, Constants.kFastPeriodicPeriod);

    private final Supplier<Pose2d> m_poseSupplier;

    private boolean spindexerOn = false;
    private boolean transferOn = false;
    private double spindexerSpeed = 0;
    private double transferSpeed = 0;

    // Simulation classes
    private final DCMotor m_flywheelDCMotor = DCMotor.getNeoVortex(2);
    private final FlywheelSim m_flywheelSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            m_flywheelDCMotor, 
            ShooterConstants.kFlywheelMOI, 
            ShooterConstants.kFlywheelGearing), 
        m_flywheelDCMotor);
    private final SparkFlexSim m_sparkFlexSim = new SparkFlexSim(m_shooterMotorLeft, m_flywheelDCMotor);

    //constructor
    public ShooterSubsystem(Supplier<Pose2d> poseSupplier) {
        m_poseSupplier = poseSupplier;

        SparkFlexConfig motorConfig = new SparkFlexConfig();
        motorConfig.idleMode(IdleMode.kCoast);
        m_shooterMotorLeft.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkFlexConfig followerConfig = new SparkFlexConfig();
        followerConfig
            .follow(m_shooterMotorLeft, true)
            .idleMode(IdleMode.kCoast);
        m_shooterMotorRight.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkFlexConfig hoodMotorConfig = new SparkFlexConfig();
        hoodMotorConfig
            .idleMode(IdleMode.kBrake)
            .absoluteEncoder.positionConversionFactor(360)
            .inverted(true);
        m_hoodMotor.configure(hoodMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkFlexConfig spindexerConfig = new SparkFlexConfig();
        spindexerConfig.idleMode(IdleMode.kCoast);
        m_spindexerMotor.configure(spindexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_transferMotor.configure(spindexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_shooterPID.setTolerance(150);
        
        // SmartDashboard.putNumber("Shooter/Set Hood Output", 0);
        // SmartDashboard.putNumber("Shooter/Input Shooter Speed", 0);
        // SmartDashboard.putNumber("Shooter/Hood Angle Input", 0);
    }

    //resets shooter
    public void reset() {
        
        shooterStop();
        setHoodAngle(ShooterConstants.kHoodAngleMin);

    }

    //stops shooter
    public void shooterStop() {

        m_shooterPID.setSetpoint(0);
        m_spindexerPID.setSetpoint(0);
        m_transferPID.setSetpoint(0);

    }

    //sets shooter parameters for PIDF control
    public void setShooterParameters(double flywheelSpeed, double hoodAngle) {

        m_shooterPID.setSetpoint(flywheelSpeed);
        m_hoodAnglePID.setSetpoint(hoodAngle);

    }

    //sets the shooter motors at given speed
    public void setShooterMotors(double speed) {

        m_shooterMotorLeft.set(speed);

    }

    //returns speed of the shooter as a average of the speeds of the two motors
    public double getAvgShooterSpeed() {
        return Robot.isReal() ? (getLeftShooterSpeed() + getRightShooterSpeed())/2 : m_flywheelSim.getAngularVelocityRPM();
    }

    //returns speed of the left shooter motor
    public double getLeftShooterSpeed() {
        return m_shooterMotorLeft.getEncoder().getVelocity();
    }

    //returns speed of the right shooter motor
    public double getRightShooterSpeed() {
        return m_shooterMotorRight.getEncoder().getVelocity();
    }

    public boolean isShooterReady() {
        return m_shooterPID.atSetpoint();
    }

    public boolean isHoodReady() {
        return Math.abs(m_hoodAnglePID.getSetpoint() - getHoodAngle()) < 0.1;
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
        double angle = m_hoodEncoder.getAbsolutePosition().getValueAsDouble();
        if (angle > ShooterConstants.kHoodAngleMax + 0.1) {
            angle -= 1;
        }
        return angle;
        // return m_hoodAnglePID.getSetpoint();
    }

    public void setSpindexer(double setpoint) {
        spindexerSpeed = setpoint;
    }

    public void setSpindexer(boolean on) {
        spindexerSpeed = on ? 1.0 : 0;
    }

    public void setTransfer(double setpoint) {
        transferSpeed = setpoint;
    }

    public void setTransfer(boolean on) {
        transferSpeed = on ? 0.5 : 0;
    }

    //periodic
    public void periodic() {

        // SmartDashboard.putNumber("Shooter Hood Angle", getHoodAngle());
        SmartDashboard.putBoolean("Shooter/Shooter Ready?", isShooterReady());
        SmartDashboard.putNumber("Shooter/Flywheel Setpoint", m_shooterPID.getSetpoint());
        SmartDashboard.putNumber("Shooter/Flywheel Speed", Robot.isReal() ? m_shooterMotorLeft.getEncoder().getVelocity() : m_flywheelSim.getAngularVelocityRPM());
        SmartDashboard.putNumber("Shooter/Hood Angle", getHoodAngle());
        SmartDashboard.putNumber("Shooter/Motor speed", m_shooterMotorLeft.get());
    }

    @Override
    public void simulationPeriodic() {
        // set inputs and update by one timestep
        m_flywheelSim.setInput(m_sparkFlexSim.getAppliedOutput() * RobotController.getBatteryVoltage());
        m_flywheelSim.update(0.02);

        // update SparkFlexSim
        m_sparkFlexSim.iterate(m_flywheelSim.getAngularVelocityRPM(), RobotController.getBatteryVoltage(), 0.02);

        // Update battery
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_flywheelSim.getCurrentDrawAmps()));

        // SmartDashboard.putNumber("flywheel speed", m_flywheelSim.getAngularVelocityRPM());
    }

    /** Runs every 10ms (100Hz), faster than the regular periodic loop */
    public void fastPeriodic() {

        //calculates PIDF output and sets shooter motors to that output
        double flywheelPIDValue = m_shooterPID.calculate(getLeftShooterSpeed());
        double ffSpeed = m_shooterFeedforward.calculate(m_shooterPID.getSetpoint());

        setShooterMotors(MathUtil.clamp(flywheelPIDValue + ffSpeed, ShooterConstants.kMinSpeed, ShooterConstants.kMaxSpeed));

        // If in a trench, force hood flat to prevent mechanical collision
        if (FieldConstants.kTrenchesRegion.isInRegion(m_poseSupplier.get())) {
            m_hoodAnglePID.setSetpoint(ShooterConstants.kHoodAngleMin);
        }

        //calculates PID output for hood angle and sets hood motor to that output
        double hoodOutput = m_hoodAnglePID.calculate(getHoodAngle());
        setHoodMotor(MathUtil.clamp(-hoodOutput, -ShooterConstants.kHoodSpeedMax, ShooterConstants.kHoodSpeedMax));
        // setHoodMotor(SmartDashboard.getNumber("Shooter/Set Hood Output", 0));
        SmartDashboard.putNumber("Shooter/Hood Output", hoodOutput);

        // double spindexerOutput = m_spindexerPID.calculate(
        //     m_spindexerMotor.getEncoder().getVelocity()) 
        //     + m_spindexerFF.calculate(m_spindexerPID.getSetpoint());
        // m_spindexerMotor.set(MathUtil.clamp(spindexerOutput, -ShooterConstants.kSpindexerMaxSpd, ShooterConstants.kSpindexerMaxSpd));
        m_spindexerMotor.set(spindexerSpeed);

        // double transferOutput = m_transferPID.calculate(
        //     m_transferMotor.getEncoder().getVelocity()) 
        //     + m_transferFF.calculate(m_transferPID.getSetpoint());
        // m_transferMotor.set(MathUtil.clamp(transferOutput, -ShooterConstants.kTransferMaxSpd, ShooterConstants.kTransferMaxSpd));
        m_transferMotor.set(transferSpeed);
    }

}

