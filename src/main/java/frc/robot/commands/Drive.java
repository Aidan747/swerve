// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveDrive;

public class Drive extends CommandBase {
  /** Creates a new Drive. */
  private SwerveDrive m_drivetrain;
  private XboxController m_driveController;
  private Translation2d deltaPosition;
  private double rotation;

  public Drive(SwerveDrive m_drivetrain, XboxController m_driveController) {
    this.m_drivetrain = m_drivetrain;
    this.m_driveController = m_driveController;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = (m_driveController.getLeftY() < 0.05) ? m_driveController.getLeftY() : 0;
    double y = (m_driveController.getLeftX() < 0.05) ? m_driveController.getLeftX() : 0;
    deltaPosition = new Translation2d(x * Constants.SwerveConstants.maxSpeedMpS, y);
    rotation = m_driveController.getRightX() * Constants.SwerveConstants.maxAnglularVelocity;
    m_drivetrain.drive(deltaPosition, rotation);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
