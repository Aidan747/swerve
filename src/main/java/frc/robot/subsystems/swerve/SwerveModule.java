// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import java.util.Optional;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {

  private CANSparkMax turnMotor;
  private CANSparkMax driveMotor;

  private RelativeEncoder driveMotorEncoder;
  private RelativeEncoder turnMotorEncoder;

  private CANcoder encoder;
  private StatusSignal<Double> encoder_signal;
  public SparkMaxPIDController turnController;

  private double turnPIDOut;
  private double absEncoderOffset;
  private int mod_number;
  private double absOffset;

  private SwerveModuleState desiredState = new SwerveModuleState(0, new Rotation2d());

  public SwerveModule(int driveMotorID, int turnMotorID, int encoderID, int mod_number, double absOffset) {
    turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
    driveMotor =  new CANSparkMax(driveMotorID, MotorType.kBrushless);
    encoder = new CANcoder(encoderID);

    this.mod_number = mod_number;
    this.absOffset = absOffset;

    driveMotorEncoder = driveMotor.getEncoder();
    encoder_signal = encoder.getAbsolutePosition();
    turnMotorEncoder = turnMotor.getEncoder();
    

    turnController = turnMotor.getPIDController();
    turnController.setP(Constants.SwerveConstants.SwervePIDConstants.kP);
    turnController.setI(Constants.SwerveConstants.SwervePIDConstants.kI);
    turnController.setD(Constants.SwerveConstants.SwervePIDConstants.kD);
    turnController.setPositionPIDWrappingEnabled(true);
    turnController.setPositionPIDWrappingMaxInput(2 * Math.PI);
    turnController.setPositionPIDWrappingMinInput(0);
    turnController.setOutputRange(-1, 1);

    driveMotorEncoder.setPositionConversionFactor(Constants.SwerveConstants.Conversions.rotationToMeters);
    driveMotorEncoder.setVelocityConversionFactor(Constants.SwerveConstants.Conversions.RPMToMPS);


  }
  public void resetEncoders() {
    driveMotorEncoder.setPosition(0);
    turnMotorEncoder.setPosition(getAbsTurnPosition());
  }
  

  public void setDriveMotor(double input) {
      if(Math.abs(input) > 1.0) return;
      driveMotor.set(input);
  }

  public void setState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.005) {
      stop();
      return;
    }
    state = SwerveModuleState.optimize(state, getModuleState().angle);
    driveMotor.set(state.speedMetersPerSecond / Constants.SwerveConstants.maxSpeedMpS);
    turnController.setReference(state.angle.getRadians(), ControlType.kPosition);
  }

  public double getDriveVelocity() {
    return driveMotor.getEncoder().getVelocity();
  }

  public double getAbsTurnPosition() {
    return encoder_signal.getValue();
  }

  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAbsTurnPosition()));
  }

  public SwerveModulePosition getModulePosition() {
    Rotation2d rot = new Rotation2d(encoder_signal.getValue());
    return new SwerveModulePosition(driveMotorEncoder.getPosition(), rot);
  }

  public void stop() {
    driveMotor.set(0);
    turnMotor.set(0);
  }

  @Override
  public void periodic() {
    encoder_signal.refresh();
    SmartDashboard.putNumber("Encoder" + Integer.toString(mod_number), encoder_signal.getValue());
    SmartDashboard.putNumber("Speed" + Double.toString(driveMotor.get()), driveMotor.get());
  } 
}
