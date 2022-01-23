/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  WPI_VictorSPX m_intakeMotor = new WPI_VictorSPX(IntakeConstants.kIntakeMotorPort);
  DoubleSolenoid m_intakeSolenoid = new DoubleSolenoid(IntakeConstants.kPCMID, IntakeConstants.kIntakeForwardPort, IntakeConstants.kIntakeReversePort);

  public IntakeSubsystem() {
    
  }

  @Override
  public void periodic() {
    
  }

  public void runIntake(double speed) {
    if (m_intakeSolenoid.get().equals(Value.kForward)) {
      m_intakeMotor.set(speed);
    } else {
      m_intakeMotor.set(0);
    }
  }

  public boolean isIntakeRunning() {
    return m_intakeMotor.get() !=0;
  }

  public void toggleIntake() {
    if (m_intakeSolenoid.get().equals(Value.kReverse)) {
      m_intakeSolenoid.set(Value.kForward);
    } else {
      m_intakeSolenoid.set(Value.kReverse);
    }
  }

  public void raiseIntake() {
    m_intakeSolenoid.set(Value.kReverse);
  }

  public void lowerIntake() {
    m_intakeSolenoid.set(Value.kForward);
  }
}
