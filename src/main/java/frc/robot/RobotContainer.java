// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Drive;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveModule;

import com.ctre.phoenix6.hardware.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  XboxController driverController = new XboxController(0);

  //add device numbers
  public static Pigeon2 gyro = new Pigeon2(14);

  public SwerveModule[] modules = {
    /* all orientations asssume the battery as a point of reference
       (i.e the battery is assumed to be in the back, with all orientations 
       appearing as if you were looking from the battery to the front of the
       robot )
    */

    //back left 
    new SwerveModule(
      Constants.SwerveConstants.backLeftDriveMotorID,
      Constants.SwerveConstants.backLeftTurnMotorID,
      Constants.SwerveConstants.backLeftmodEncoderID,
      1,
      0
    ),
    //back right
    new SwerveModule(
      Constants.SwerveConstants.backRightDriveMotorID,
      Constants.SwerveConstants.backRightTurnMotorID,
      Constants.SwerveConstants.backRightmodEncoderID,
      2,
      0.4626
    ),
    //front left
    new SwerveModule(
      Constants.SwerveConstants.frontLeftDriveMotorID,
      Constants.SwerveConstants.frontLeftTurnMotorID,
      Constants.SwerveConstants.frontLeftmodEncoderID,
      3,
      0
    ),
    //front right
    new SwerveModule(
      Constants.SwerveConstants.frontRightDriveMotorID,
      Constants.SwerveConstants.frontRightTurnMotorID,
      Constants.SwerveConstants.frontRightmodEncoderID,
      4,
      0
    )
  };

  public SwerveDrive drivetrain = new SwerveDrive(modules, gyro);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    drivetrain.setDefaultCommand(new Drive(drivetrain, driverController));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new InstantCommand(() -> {});
  }
}
