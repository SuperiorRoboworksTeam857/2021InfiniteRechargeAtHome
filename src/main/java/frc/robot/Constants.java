/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

public final class Constants {
  public static final class DriveConstants {
    public static final int kLeftMotor1Port = 30;//30
    public static final int kLeftMotor2Port = 31;//31
    public static final int kRightMotor1Port = 32;//32
    public static final int kRightMotor2Port = 33;//33

    public static final double kTrackwidthMeters = 0.5797;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final int kEncoderCPR = 1;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) (kEncoderCPR * 10.71);

    public static final boolean kGyroReversed = true;


    public static final double ksVolts = 0.194;
    public static final double kvVoltSecondsPerMeter = 2.79;
    public static final double kaVoltSecondsSquaredPerMeter = 0.381;

    public static final double kPDriveVel = 2.28;
  }

  public static final class IntakeConstants {
    public static final int kIntakeMotorPort = 34;
    public static final int kPCMID = 2;
    public static final int kIntakeForwardPort = 0;
    public static final int kIntakeReversePort = 1;
  }

  public static final class SerializerConstants {
    public static final int kSerializerMotorPort = 36;
  }

  public static final class EsophagusConstants {
    public static final int kEsophagusMotorPort = 37;
    public static final int kEsophagusBottomBeamBreakPort = 0;
    public static final int kEsophagusTopBeamBreakPort = 1;
  }

  public static final class ShooterConstants {
    public static final int krightMotorPort = 38;
    public static final int kleftMotorPort = 39;
  }

  public static final class LimelightConstants {
    public static final double kLimelightHeight = 41.0; // Inches
    public static final double kLimelightAngle = -91.0; // Degrees 
    public static final double kTargetHeight = 98.25;
  }

  public static final class OIConstants {
    public static final int kOperatorControllerPort = 0;
    public static final int kDriverStickPort = 1;

    public static final int kLeftStickPort = 2;
    public static final int kRightStickPort = 3;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1.5; // fairly fast for Francois XXI
    public static final double kFastMaxSpeedMetersPerSecond = 3.5; // REALLY fast for Francois XXI
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }
}
