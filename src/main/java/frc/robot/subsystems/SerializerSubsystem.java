/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import frc.robot.Constants.SerializerConstants;

public class SerializerSubsystem extends SubsystemBase {
  WPI_VictorSPX m_serializerMotor = new WPI_VictorSPX(SerializerConstants.kSerializerMotorPort);

  public SerializerSubsystem() {
    
  }

  @Override
  public void periodic() {
    
  }

  public void runSerializer() {
    m_serializerMotor.set(-0.3);
  }

  public void stopSerializer() {
    m_serializerMotor.set(0);
  }
}
