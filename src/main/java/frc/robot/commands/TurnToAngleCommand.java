package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;


public class TurnToAngleCommand extends CommandBase {

    private final DriveSubsystem m_robotDrive;
    private boolean complete = false;
    private double angle;
    private Timer timer = new Timer();
    private double timeout;
    public TurnToAngleCommand(DriveSubsystem subsystem, double degrees, double timeoutS){
        m_robotDrive = subsystem;
        angle = degrees;
        timeout = timeoutS;
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){
        timer.reset();
        timer.start();
        complete = false;
    }
    
    @Override
    public void execute(){
        double gyroAngle = m_robotDrive.getHeading();

        final double kP = 0.005;
        SmartDashboard.putNumber("gyroAngle", gyroAngle);
    
        if (angle > 180) {
            angle = -(360 - angle);
        } else if (angle < -180) {
            angle = 360 + angle;
        }
    
        double err = angle - gyroAngle;
    
        double speed = MathUtil.clamp(err * kP, -0.4, 0.4);
    
        if (Math.abs(err) > 2 && timer.get() < timeout) {
            m_robotDrive.arcadeDrive(0, -speed, false);
        } else {
            complete = true;
        }
    }

    @Override
    public void end(boolean inturrupted){
        m_robotDrive.arcadeDrive(0, 0);
        timer.stop();
    }

    @Override
    public boolean isFinished(){
        return complete;
    }
}