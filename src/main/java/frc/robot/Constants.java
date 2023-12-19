// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double kTrackWidth = Units.inchesToMeters(27.75);
  public static final double kTrackLength = Units.inchesToMeters(34);

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static final class SwerveConstants {
    public static final int frontRightTurnMotorID = 13;
    public static final int frontRightDriveMotorID = 12;
    public static final int frontLeftTurnMotorID = 6;
    public static final int frontLeftDriveMotorID = 7;
    public static final int backRightTurnMotorID = 10;
    public static final int backRightDriveMotorID = 11;
    public static final int backLeftTurnMotorID = 9;
    public static final int backLeftDriveMotorID = 8;
    public static final int frontRightmodEncoderID = 3;
    public static final int frontLeftmodEncoderID = 2;
    public static final int backLeftmodEncoderID = 5;
    public static final int backRightmodEncoderID = 4;
    public static final double maxSpeedMpS = 3.96;
    public static final double maxAnglularVelocity = 2*Math.PI;
    public static final class SwervePIDConstants {
      public static final double kP = 0.3;
      public static final double kI = 0.0001;
      public static final double kD = 1.5;
    }
    public static final class Conversions {
      public static final double gearRatio = 8.41;
      public static final double wheelCircumfrence = Units.inchesToMeters(12.56);
      public static final double rotationToMeters = gearRatio * wheelCircumfrence;
      public static final double RPMToMPS = rotationToMeters / 60;
      public static final double TurningMotorReduction = 150 / 7;
    }
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kTrackLength / 2, kTrackWidth / 2 ),
      new Translation2d(kTrackLength / 2, kTrackWidth / 2 ),
      new Translation2d(kTrackLength / 2, kTrackWidth / 2 ),
      new Translation2d(kTrackLength / 2, kTrackWidth / 2 )
    );
  }
}
