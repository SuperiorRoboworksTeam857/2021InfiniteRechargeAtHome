/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import frc.robot.Constants.EsophagusConstants;

public class EsophagusSubsystem extends SubsystemBase {
  WPI_VictorSPX m_esophagusMotor = new WPI_VictorSPX(EsophagusConstants.kEsophagusMotorPort);
  DigitalInput m_bottomBeamBreak = new DigitalInput(EsophagusConstants.kEsophagusBottomBeamBreakPort);
  DigitalInput m_topBeamBreak = new DigitalInput(EsophagusConstants.kEsophagusTopBeamBreakPort);

  public EsophagusSubsystem() {
    
  }

  @Override
  public void periodic() {
    
  }

  public void runEsophagus() {
    m_esophagusMotor.set(-1.0);
  }

  public void stopEsophagus() {
    m_esophagusMotor.set(0);
  }

  public void autoRunEsophagus() {
    if (!m_bottomBeamBreak.get() && m_topBeamBreak.get()) {
      runEsophagus();
    } else {
      stopEsophagus();
    }
  }

  public void runEsophagusToTop() {
    if (m_topBeamBreak.get()) {
      runEsophagus();
    } else {
      stopEsophagus();
    }
  }

  public boolean isEsophagusRunning() {
    return m_esophagusMotor.get()!= 0;
  }
}
  

