/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.
  CANSparkMax m_frontLeftMotor = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless);
  CANSparkMax m_backLeftMotor = new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushless);

  private final SpeedControllerGroup m_leftMotors =
      new SpeedControllerGroup(m_frontLeftMotor, m_backLeftMotor);

  // The motors on the right side of the drive.
  CANSparkMax m_frontRightMotor = new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless);
  CANSparkMax m_backRightMotor = new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless);


  private final SpeedControllerGroup m_rightMotors =
      new SpeedControllerGroup(m_frontRightMotor,m_backRightMotor);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // The gyro sensor
  private final Gyro m_gyro = new ADXRS450_Gyro();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    // Sets the distance per pulse for the encoders
    m_frontLeftMotor.getEncoder().setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    m_frontRightMotor.getEncoder().setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);


    m_frontLeftMotor.getEncoder().setVelocityConversionFactor(DriveConstants.kEncoderDistancePerPulse/60);
    m_frontRightMotor.getEncoder().setVelocityConversionFactor(DriveConstants.kEncoderDistancePerPulse/60);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_frontLeftMotor.getEncoder().getPosition(),
                      -m_frontRightMotor.getEncoder().getPosition());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    SmartDashboard.putNumber("x", m_odometry.getPoseMeters().getTranslation().getX());
    SmartDashboard.putNumber("y", m_odometry.getPoseMeters().getTranslation().getY());

    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    SmartDashboard.putNumber("left speed", m_frontLeftMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("right speed", m_frontRightMotor.getEncoder().getVelocity());
    return new DifferentialDriveWheelSpeeds( m_frontLeftMotor.getEncoder().getVelocity(),
                                             -m_frontRightMotor.getEncoder().getVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }
  public void arcadeDrive(double fwd, double rot, boolean squareInputs) {
    m_drive.arcadeDrive(fwd, rot, squareInputs);
  }
  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(-rightVolts);
    SmartDashboard.putNumber("left", leftVolts);
    SmartDashboard.putNumber("right", rightVolts);
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    m_frontLeftMotor.getEncoder().setPosition(0);
    m_frontRightMotor.getEncoder().setPosition(0);
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    SmartDashboard.putNumber("heading", Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0));
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
