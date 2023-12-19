// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.hardware.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDrive extends SubsystemBase {


  
  private SwerveModule[] mods;
  private SwerveModuleState[] states;
  private SwerveModulePosition[] positions;
  private Pigeon2 gyro;
  private final SwerveDriveOdometry odometer;

  public SwerveDrive(SwerveModule[] mods, Pigeon2 gyro) {
    this.mods = mods;
    this.gyro = gyro;

    odometer = new SwerveDriveOdometry(
      Constants.SwerveConstants.kDriveKinematics,
      new Rotation2d(0),
      getModulePositions());
  }

  private void zeroGyroHeading() {
    gyro.reset();
  } 

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for(int i = 0; i < mods.length; i++) {
      positions[i] = mods[i].getModulePosition(); 
    }
    return positions;
   } 
   public void drive(Translation2d deltaPos, double rotationRate) {
      SwerveModuleState[] newStates = Constants.SwerveConstants.kDriveKinematics
        .toSwerveModuleStates(
          //new ChassisSpeeds(deltaPos.getX(), deltaPos.getY(), rotationRate)
          ChassisSpeeds.fromFieldRelativeSpeeds(deltaPos.getX(), deltaPos.getY(), rotationRate, gyro.getRotation2d())
        );
      setStates(newStates);
   } 

   public void setStates(SwerveModuleState[] states) {
    for(int i = 0; i < mods.length; i++) {
      mods[i].setState(states[i]);
    }
   }

  @Override
  public void periodic() {
    
  }
}
