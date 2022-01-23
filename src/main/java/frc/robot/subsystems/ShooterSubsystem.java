/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  CANSparkMax m_leftMotor = new CANSparkMax(ShooterConstants.kleftMotorPort, MotorType.kBrushless);
  CANSparkMax m_rightMotor = new CANSparkMax(ShooterConstants.krightMotorPort, MotorType.kBrushless);
  
  public ShooterSubsystem() {
    
  }

  @Override
  public void periodic() {
    
  }

  public void runShooter() {
    m_leftMotor.set(-0.9);
    m_rightMotor.set(-0.9);
  }

  public void runShooterFast() {
    m_leftMotor.set(-1);
    m_rightMotor.set(-1);
  }

  public void stopShooter() {
    m_leftMotor.set(0);
    m_rightMotor.set(0);
  }

  public boolean isShooterAtSpeed() {
    return m_rightMotor.getEncoder().getVelocity() <= -4000;
  }
}
  

