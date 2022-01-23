/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;
import java.util.Map;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.commands.TurnToTargetCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EsophagusSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SerializerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static edu.wpi.first.wpilibj.XboxController.Button;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final SerializerSubsystem m_serializer = new SerializerSubsystem();
  private final EsophagusSubsystem m_esophagus = new EsophagusSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final LimelightSubsystem m_limelight = new LimelightSubsystem();
  public final Pixy2Handler m_pixy = new Pixy2Handler();

  // The driver's controller
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
  Joystick m_driverStick = new Joystick(OIConstants.kDriverStickPort);

  Joystick m_leftStick = new Joystick(OIConstants.kLeftStickPort);
  Joystick m_rightStick = new Joystick(OIConstants.kRightStickPort);

  SlewRateLimiter forwardLimiter = new SlewRateLimiter(2);
  SlewRateLimiter turnLimiter = new SlewRateLimiter(5);

  private double topForwardSpeed = 0.7;


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    CameraServer.getInstance().startAutomaticCapture();

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(() -> m_robotDrive.arcadeDrive(-forwardLimiter.calculate(topForwardSpeed*m_driverStick.getY()),
                                                      //0.55*turnLimiter.calculate(m_driverStick.getThrottle())),
                                                      0.55*m_driverStick.getThrottle()),
                                                      m_robotDrive));
        // new RunCommand(() -> m_robotDrive.tankDriveVolts(-5*m_leftStick.getY(),
        //                                                  -5*m_rightStick.getY()),
        //                                                  m_robotDrive));
    
    m_intake.setDefaultCommand(new RunCommand(() -> m_intake.runIntake(0), m_intake));

    m_serializer.setDefaultCommand(new RunCommand(() -> m_serializer.stopSerializer(), m_serializer));

    m_esophagus.setDefaultCommand(new RunCommand(() -> m_esophagus.autoRunEsophagus(), m_esophagus));

    m_shooter.setDefaultCommand(new RunCommand(() -> m_shooter.stopShooter(), m_shooter));


    
    m_limelight.setDefaultCommand(new RunCommand(() -> m_limelight.enableLimelight(false), m_limelight));
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * calling passing it to a {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drive at half speed when the right bumper is held
    //new JoystickButton(m_operatorController, Button.kBumperRight.value).whenPressed(() -> m_robotDrive.setMaxOutput(0.5))
        //.whenReleased(() -> m_robotDrive.setMaxOutput(1));

    // new JoystickButton(m_operatorController, 3).whileHeld(
    //                     new RunCommand(() -> m_robotDrive.arcadeDrive(-forwardLimiter.calculate(m_driverStick.getY()),
    //                     //0.55*turnLimiter.calculate(m_driverStick.getThrottle())),
    //                     0.55*m_driverStick.getThrottle()),
    //                     m_robotDrive));
    new JoystickButton(m_driverStick, 2).whileHeld( () -> topForwardSpeed = 1)
                                               .whenReleased( () -> topForwardSpeed = 0.7);



    new JoystickButton(m_driverStick, 1).whileHeld(() -> m_intake.runIntake(1), m_intake);//.whenReleased(() -> m_intake.runIntake(0));
    //new JoystickButton(m_driverStick, 2).whileHeld(() -> m_intake.runIntake(-1), m_intake);//.whenReleased(() -> m_intake.runIntake(0));
    new JoystickButton(m_driverStick, 4).whenPressed(() -> m_intake.toggleIntake(), m_intake);

    Trigger POVTrigger = new Trigger(() -> m_driverStick.getPOV() == 180);
    POVTrigger.whileActiveContinuous(() -> m_serializer.runSerializer(), m_serializer);
    new Trigger(() -> m_intake.isIntakeRunning()).whileActiveContinuous(() -> m_serializer.runSerializer(), m_serializer);
    new Trigger(() -> m_esophagus.isEsophagusRunning()).whileActiveContinuous(() -> m_serializer.runSerializer(), m_serializer);


    new JoystickButton(m_operatorController, 4).whileHeld(() -> m_esophagus.runEsophagus(), m_esophagus);
    new Trigger(() -> m_operatorController.getTriggerAxis(GenericHID.Hand.kRight) > 0.2 && m_shooter.isShooterAtSpeed())
      .whileActiveContinuous(() -> m_esophagus.runEsophagus(), m_esophagus);
    
    new Trigger(() -> m_operatorController.getTriggerAxis(GenericHID.Hand.kRight) > 0.2 && !m_shooter.isShooterAtSpeed())
      .whileActiveContinuous(() -> m_esophagus.runEsophagusToTop(), m_esophagus);


    new Trigger(() -> m_operatorController.getTriggerAxis(GenericHID.Hand.kLeft) > 0.2)
      .whileActiveContinuous(() -> m_shooter.runShooter(), m_shooter);
    new JoystickButton(m_operatorController, 5).whileHeld(() -> m_shooter.runShooterFast(), m_shooter);
    

    new JoystickButton(m_driverStick, 3).whenPressed(() -> m_limelight.toggleDriverCam(), m_limelight);
    new JoystickButton(m_driverStick, 5)
      .whenPressed(new SequentialCommandGroup(    
            new InstantCommand(() -> m_limelight.enableLimelight(true), m_limelight),
            new TurnToTargetCommand(m_robotDrive, m_limelight, m_driverStick, 50)))
      .whenReleased(() -> m_limelight.enableLimelight(false), m_limelight);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousBarrelRaceCommand() {

    m_robotDrive.resetEncoders();
    m_robotDrive.zeroHeading();
    m_robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
    m_intake.raiseIntake();
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, 10);

    // Create config for trajectory
    TrajectoryConfig turnconfig = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max spb
            .addConstraint(autoVoltageConstraint).setStartVelocity(AutoConstants.kMaxSpeedMetersPerSecond)
            .setEndVelocity(AutoConstants.kMaxSpeedMetersPerSecond);
    TrajectoryConfig firstconfig = new TrajectoryConfig(AutoConstants.kFastMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max spb
            .addConstraint(autoVoltageConstraint).setEndVelocity(AutoConstants.kMaxSpeedMetersPerSecond);
    TrajectoryConfig middleconfig = new TrajectoryConfig(AutoConstants.kFastMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max spb
            .addConstraint(autoVoltageConstraint).setStartVelocity(AutoConstants.kMaxSpeedMetersPerSecond)
            .setEndVelocity(AutoConstants.kMaxSpeedMetersPerSecond);

    TrajectoryConfig lastconfig = new TrajectoryConfig(AutoConstants.kFastMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max spb
            .addConstraint(autoVoltageConstraint).setStartVelocity(AutoConstants.kMaxSpeedMetersPerSecond);

    Trajectory firstTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction        
        List.of(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(120 * 2.54 / 100, 0, new Rotation2d(0))), firstconfig);

    Trajectory secondTrajectory = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(120 * 2.54 / 100, 0, new Rotation2d(0)),
            new Pose2d(150 * 2.54 / 100, -30 * 2.54 / 100, new Rotation2d(-Math.PI / 2)),
            new Pose2d(120 * 2.54 / 100, -60 * 2.54 / 100, new Rotation2d(-Math.PI)),
            new Pose2d(90 * 2.54 / 100, -30 * 2.54 / 100, new Rotation2d(-Math.PI * 3 / 2)),
            new Pose2d(120 * 2.54 / 100, 0, new Rotation2d(-Math.PI * 2))), turnconfig);

    Trajectory thirdTrajectory = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(120 * 2.54 / 100, 0, new Rotation2d(-Math.PI * 2)),
            new Pose2d(210 * 2.54 / 100, 0, new Rotation2d(-Math.PI * 2))), middleconfig);

    Trajectory fourthTrajectory = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(210 * 2.54 / 100, 0, new Rotation2d(-Math.PI * 2)),
            new Pose2d(240 * 2.54 / 100, 30 * 2.54 / 100, new Rotation2d(-Math.PI * 3 / 2)),
            new Pose2d(210 * 2.54 / 100, 60 * 2.54 / 100, new Rotation2d(-Math.PI)),
            new Pose2d(180 * 2.54 / 100, 30 * 2.54 / 100, new Rotation2d(-Math.PI / 2))
            
        ),
        // Pass config
        turnconfig);

    Trajectory fifthTrajectory = TrajectoryGenerator.generateTrajectory(
        List.of(new Pose2d(180 * 2.54 / 100, 30 * 2.54 / 100, new Rotation2d(-Math.PI / 2)),
        new Pose2d(270 * 2.54 / 100, -60 * 2.54 / 100, new Rotation2d(0))
        ),
        middleconfig
    );

    Trajectory sixthTrajectory = TrajectoryGenerator.generateTrajectory(
        List.of(new Pose2d(270 * 2.54 / 100, -60 * 2.54 / 100, new Rotation2d(0)),
        new Pose2d(300 * 2.54 / 100, -30 * 2.54 / 100, new Rotation2d(Math.PI / 2)),
        new Pose2d(270 * 2.54 / 100, 0, new Rotation2d(Math.PI))),
        turnconfig
    );

    Trajectory seventhTrajectory = TrajectoryGenerator
        .generateTrajectory(List.of(new Pose2d(270 * 2.54 / 100, 0, new Rotation2d(Math.PI)),
            new Pose2d(0 * 2.54 / 100, 0, new Rotation2d(Math.PI))

        ), lastconfig);

    RamseteCommand firstramseteCommand = new RamseteCommand(firstTrajectory, m_robotDrive::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts, m_robotDrive);

    RamseteCommand secondramseteCommand = new RamseteCommand(secondTrajectory, m_robotDrive::getPose,
    new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
    new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
        DriveConstants.kaVoltSecondsSquaredPerMeter),
    DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
    new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
    // RamseteCommand passes volts to the callback
    m_robotDrive::tankDriveVolts, m_robotDrive);

    RamseteCommand thirdramseteCommand = new RamseteCommand(thirdTrajectory, m_robotDrive::getPose,
    new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
    new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
        DriveConstants.kaVoltSecondsSquaredPerMeter),
    DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
    new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
    // RamseteCommand passes volts to the callback
    m_robotDrive::tankDriveVolts, m_robotDrive);

    RamseteCommand fourthramseteCommand = new RamseteCommand(fourthTrajectory, m_robotDrive::getPose,
    new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
    new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
        DriveConstants.kaVoltSecondsSquaredPerMeter),
    DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
    new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
    // RamseteCommand passes volts to the callback
    m_robotDrive::tankDriveVolts, m_robotDrive);

    RamseteCommand fifthramseteCommand = new RamseteCommand(fifthTrajectory, m_robotDrive::getPose,
    new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
    new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
        DriveConstants.kaVoltSecondsSquaredPerMeter),
    DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
    new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
    // RamseteCommand passes volts to the callback
    m_robotDrive::tankDriveVolts, m_robotDrive);

    RamseteCommand sixthramseteCommand = new RamseteCommand(sixthTrajectory, m_robotDrive::getPose,
    new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
    new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
        DriveConstants.kaVoltSecondsSquaredPerMeter),
    DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
    new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
    // RamseteCommand passes volts to the callback
    m_robotDrive::tankDriveVolts, m_robotDrive);

    RamseteCommand seventhramseteCommand = new RamseteCommand(seventhTrajectory, m_robotDrive::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts, m_robotDrive);

    // Run path following command, then stop at the end.
    // return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));

    return new SequentialCommandGroup(
        firstramseteCommand,
        secondramseteCommand,
        thirdramseteCommand,
        fourthramseteCommand,
        fifthramseteCommand,
        sixthramseteCommand,
        seventhramseteCommand,
        new InstantCommand(() -> m_robotDrive.tankDriveVolts(0, 0), m_robotDrive));
  }

  public Command getAutonomousSlalomCommand() {

    m_robotDrive.resetEncoders();
    m_robotDrive.zeroHeading();
    m_robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
    m_intake.raiseIntake();
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, 10);

    // Create config for trajectory
    TrajectoryConfig startconfig = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max spb
            .addConstraint(autoVoltageConstraint)
            .setEndVelocity(AutoConstants.kMaxSpeedMetersPerSecond);
    TrajectoryConfig straightconfig = new TrajectoryConfig(AutoConstants.kFastMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max spb
            .addConstraint(autoVoltageConstraint)
            .setStartVelocity(AutoConstants.kMaxSpeedMetersPerSecond)
            .setEndVelocity(AutoConstants.kMaxSpeedMetersPerSecond);
    TrajectoryConfig turnconfig = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max spb
            .addConstraint(autoVoltageConstraint)
            .setStartVelocity(AutoConstants.kMaxSpeedMetersPerSecond)
            .setEndVelocity(AutoConstants.kMaxSpeedMetersPerSecond);
    TrajectoryConfig endconfig = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max spb
            .addConstraint(autoVoltageConstraint)
            .setStartVelocity(AutoConstants.kMaxSpeedMetersPerSecond);

    Trajectory firstTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        List.of(new Pose2d(0, 0, new Rotation2d(0)),
            new Pose2d(30 * 2.54 / 100, 0, new Rotation2d(0)),
            new Pose2d(60 * 2.54 / 100, 30 * 2.54 / 100, new Rotation2d(Math.PI / 2)),
            new Pose2d(90 * 2.54 / 100, 60 * 2.54 / 100, new Rotation2d(0))
        ),

        // Pass config
        startconfig);
    
    Trajectory secondTrajectory = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      List.of(
              new Pose2d(90 * 2.54 / 100, 60 * 2.54 / 100, new Rotation2d(0)),
              new Pose2d(210 * 2.54 / 100, 60 * 2.54 / 100, new Rotation2d(0))
          ),
          straightconfig);

    Trajectory thirdTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            List.of(
                new Pose2d(210 * 2.54 / 100, 60 * 2.54 / 100, new Rotation2d(0)),
                new Pose2d(240 * 2.54 / 100, 30 * 2.54 / 100, new Rotation2d(-Math.PI / 2)),
                new Pose2d(270 * 2.54 / 100, 0, new Rotation2d(0)),
                new Pose2d(300 * 2.54 / 100, 30 * 2.54 / 100, new Rotation2d(Math.PI / 2)),
                new Pose2d(270 * 2.54 / 100, 60 * 2.54 / 100, new Rotation2d(Math.PI)),
                new Pose2d(240 * 2.54 / 100, 30 * 2.54 / 100, new Rotation2d(Math.PI * 3 / 2)),
                new Pose2d(210 * 2.54 / 100, 0, new Rotation2d(Math.PI))
            ),
            turnconfig);

    Trajectory fourthTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            List.of(
                new Pose2d(210 * 2.54 / 100, 0, new Rotation2d(Math.PI)),
                new Pose2d(90 * 2.54 / 100, 0, new Rotation2d(Math.PI))
      
              ),
              straightconfig);

    Trajectory fifthTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            List.of(
                new Pose2d(90 * 2.54 / 100, 0, new Rotation2d(Math.PI)),
                new Pose2d(60 * 2.4 / 100, 30 * 2.54 / 100, new Rotation2d(Math.PI / 2)),
                new Pose2d(30 * 2.54 / 100, 60 * 2.54 / 100, new Rotation2d(Math.PI))
            ),
            endconfig);
    
    RamseteCommand firstramseteCommand = new RamseteCommand(firstTrajectory, m_robotDrive::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts, m_robotDrive);
    
    RamseteCommand secondramseteCommand = new RamseteCommand(secondTrajectory, m_robotDrive::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts, m_robotDrive);
    
    RamseteCommand thirdramseteCommand = new RamseteCommand(thirdTrajectory, m_robotDrive::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts, m_robotDrive);

    RamseteCommand fourthramseteCommand = new RamseteCommand(fourthTrajectory, m_robotDrive::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts, m_robotDrive);

    RamseteCommand fifthramseteCommand = new RamseteCommand(fifthTrajectory, m_robotDrive::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts, m_robotDrive);

    // Run path following command, then stop at the end.
    // return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));

    return firstramseteCommand.andThen(
      secondramseteCommand.andThen(
        thirdramseteCommand.andThen(
          fourthramseteCommand.andThen(
            fifthramseteCommand.andThen(
              () -> m_robotDrive.tankDriveVolts(0, 0))))));
  }

  public Command getAutonomousBounceCommand() {

    m_robotDrive.resetEncoders();
    m_robotDrive.zeroHeading();
    m_robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
    m_intake.raiseIntake();
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, 10);

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max spb
            .addConstraint(autoVoltageConstraint);

    Trajectory firstTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        List.of(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(30 * 2.54 / 100, 0, new Rotation2d(0)),
            new Pose2d(60 * 2.54 / 100, 30 * 2.54 / 100, new Rotation2d(Math.PI / 2)),
            new Pose2d(60 * 2.54 / 100, 55 * 2.54 / 100, new Rotation2d(Math.PI / 2))

        ),

        // Pass config
        config);

    Trajectory secondTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        List.of(new Pose2d(60 * 2.54 / 100, 55 * 2.54 / 100, new Rotation2d(Math.PI / 2)),
            new Pose2d(90 * 2.54 / 100, -30 * 2.54 / 100, new Rotation2d(Math.PI / 2)),
            new Pose2d(120 * 2.54 / 100, -60 * 2.54 / 100, new Rotation2d(Math.PI)),
            new Pose2d(150 * 2.54 / 100, -30 * 2.54 / 100, new Rotation2d(Math.PI * 3 / 2)),
            new Pose2d(150 * 2.54 / 100, 55 * 2.54 / 100, new Rotation2d(Math.PI * 3 / 2))),

        // Pass config
        config.setReversed(true));

    Trajectory thirdTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        List.of(new Pose2d(150 * 2.54 / 100, 55 * 2.54 / 100, new Rotation2d(Math.PI * 3 / 2)),
            new Pose2d(150 * 2.54 / 100, -30 * 2.54 / 100, new Rotation2d(Math.PI * 3 / 2)),
            new Pose2d(180 * 2.54 / 100, -60 * 2.54 / 100, new Rotation2d(Math.PI * 2)),
            new Pose2d(210 * 2.54 / 100, -60 * 2.54 / 100, new Rotation2d(Math.PI * 2)),
            new Pose2d(240 * 2.54 / 100, -30 * 2.54 / 100, new Rotation2d(Math.PI * 5 / 2)),
            new Pose2d(240 * 2.54 / 100, 55 * 2.54 / 100, new Rotation2d(Math.PI * 5 / 2))),

        // Pass config
        config.setReversed(false));

    Trajectory fourthTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        List.of(new Pose2d(240 * 2.54 / 100, 55 * 2.54 / 100, new Rotation2d(Math.PI * 5 / 2)),
            new Pose2d(270 * 2.54 / 100, 0, new Rotation2d(Math.PI * 11 / 4))),

        // Pass config
        config.setReversed(true));

    RamseteCommand firstramseteCommand = new RamseteCommand(firstTrajectory, m_robotDrive::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts, m_robotDrive);
    RamseteCommand secondramseteCommand = new RamseteCommand(secondTrajectory, m_robotDrive::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts, m_robotDrive);

    RamseteCommand thirdramseteCommand = new RamseteCommand(thirdTrajectory, m_robotDrive::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts, m_robotDrive);

    RamseteCommand fourthramseteCommand = new RamseteCommand(fourthTrajectory, m_robotDrive::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts, m_robotDrive);
    // Run path following command, then stop at the end.
    // return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));

    return firstramseteCommand.andThen(secondramseteCommand
        .andThen(thirdramseteCommand.andThen(fourthramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0)))));
  }

  public Command getTrenchRunCommand() {

    m_robotDrive.resetEncoders();
    m_robotDrive.zeroHeading();
    m_robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
    m_intake.raiseIntake();

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, 10);

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(1.5,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .addConstraint(autoVoltageConstraint);
    TrajectoryConfig fastconfig = new TrajectoryConfig(2,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .addConstraint(autoVoltageConstraint).setReversed(true);

    Trajectory firstTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        List.of(
            new Pose2d(0, 0, new Rotation2d(-Math.PI/2)),
            new Pose2d(-86 * 2.54 / 100, -60 * 2.54 / 100, new Rotation2d(-Math.PI)),
            new Pose2d(-180 * 2.54 / 100, -60 * 2.54 / 100, new Rotation2d(-Math.PI))
        ),
        config);
    Trajectory secondTrajectory = TrajectoryGenerator.generateTrajectory(
        List.of(
            new Pose2d(-180 * 2.54 / 100, -60 * 2.54 / 100, new Rotation2d(-Math.PI)),
            new Pose2d(-86 * 2.54 / 100, -60 * 2.54 / 100, new Rotation2d(-Math.PI)),
            new Pose2d(0, 0, new Rotation2d(-Math.PI/2))
        ),
        fastconfig);

    RamseteCommand firstRamseteCommand = new RamseteCommand(firstTrajectory, m_robotDrive::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts, m_robotDrive);
    RamseteCommand secondRamseteCommand = new RamseteCommand(secondTrajectory, m_robotDrive::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts, m_robotDrive);

    // Run path following command, then stop at the end.
    // return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));

    // shoot, turn right, intake down and intake in and sweep over power cells, back up, intake up, turn left, shoot
    
    // m_esophagus.setDefaultCommand(new RunCommand(() -> m_esophagus.autoRunEsophagus(), m_esophagus))
    // new Trigger(() -> m_intake.isIntakeRunning()).whileActiveContinuous(() -> m_serializer.runSerializer(), m_serializer);
    // new Trigger(() -> m_esophagus.isEsophagusRunning()).whileActiveContinuous(() -> m_serializer.runSerializer(), m_serializer);


    // TODO: why do default commands and triggers not run in auto?
    return new SequentialCommandGroup(
        // Speed up shooter
        new RunCommand(() -> m_shooter.runShooter(), m_shooter).withInterrupt(m_shooter::isShooterAtSpeed),
        // Shoot and run esophagus
        new ParallelRaceGroup(
            new RunCommand(() -> m_shooter.runShooter(), m_shooter),
            new RunCommand(() -> m_esophagus.runEsophagus(), m_esophagus),
            new WaitCommand(3)
        ),
        new InstantCommand(() -> m_shooter.stopShooter(), m_shooter),
        new InstantCommand(() -> m_esophagus.stopEsophagus(), m_esophagus),
        new TurnToAngleCommand(m_robotDrive, -90, 5),
        new ParallelRaceGroup(
            new RunCommand(() -> m_esophagus.autoRunEsophagus(), m_esophagus),
            new SequentialCommandGroup(
                new InstantCommand(() -> m_intake.lowerIntake(), m_intake),
                new ParallelRaceGroup(
                    firstRamseteCommand,
                    new RunCommand(() -> m_intake.runIntake(1), m_intake)
                ),
                new InstantCommand(() -> m_intake.runIntake(0), m_intake),
                secondRamseteCommand,
                new InstantCommand(() -> m_robotDrive.arcadeDrive(0, 0), m_robotDrive)
            )
        ),
        new ParallelRaceGroup(
            new TurnToAngleCommand(m_robotDrive, 0, 5),
            new RunCommand(() -> m_esophagus.runEsophagusToTop(), m_esophagus)
        ),
        new InstantCommand(() -> m_esophagus.stopEsophagus(), m_esophagus),
        new RunCommand(() -> m_shooter.runShooter(), m_shooter).withInterrupt(m_shooter::isShooterAtSpeed),
        new ParallelRaceGroup(
            new RunCommand(() -> m_shooter.runShooter(), m_shooter),
            new RunCommand(() -> m_esophagus.runEsophagus(), m_esophagus),
            new WaitCommand(3)
        ),
        new InstantCommand(() -> m_robotDrive.arcadeDrive(0, 0), m_robotDrive),
        new InstantCommand(() -> m_shooter.stopShooter(), m_shooter),
        new InstantCommand(() -> m_esophagus.stopEsophagus(), m_esophagus)
    );

    //return ramseteCommand.andThen(StraightramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0)));
  }

  public Command getFiveBallTrenchRunCommand() {

    m_robotDrive.resetEncoders();
    m_robotDrive.zeroHeading();
    m_robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
    m_intake.raiseIntake();

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, 10);

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(1.5,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .addConstraint(autoVoltageConstraint);
    TrajectoryConfig fastconfig = new TrajectoryConfig(2,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .addConstraint(autoVoltageConstraint).setReversed(true);

    Trajectory firstTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        List.of(
            new Pose2d(0, 0, new Rotation2d(0)),
            new Pose2d(125 * 2.54 / 100, 0, new Rotation2d(0))
        ),
        config);
    Trajectory secondTrajectory = TrajectoryGenerator.generateTrajectory(
        List.of(
            new Pose2d(125 * 2.54 / 100, 0, new Rotation2d(0)),
            new Pose2d(60 * 2.54 / 100, 0, new Rotation2d(0)),
            new Pose2d(0, -60 * 2.54 / 100, new Rotation2d(Math.PI/2)) // better alignment is 67, but rebounds
        ),
        fastconfig);

    RamseteCommand firstRamseteCommand = new RamseteCommand(firstTrajectory, m_robotDrive::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts, m_robotDrive);
    RamseteCommand secondRamseteCommand = new RamseteCommand(secondTrajectory, m_robotDrive::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts, m_robotDrive);

    // Run path following command, then stop at the end.
    // return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));

    // shoot, turn right, intake down and intake in and sweep over power cells, back up, intake up, turn left, shoot
    
    // m_esophagus.setDefaultCommand(new RunCommand(() -> m_esophagus.autoRunEsophagus(), m_esophagus))
    // new Trigger(() -> m_intake.isIntakeRunning()).whileActiveContinuous(() -> m_serializer.runSerializer(), m_serializer);
    // new Trigger(() -> m_esophagus.isEsophagusRunning()).whileActiveContinuous(() -> m_serializer.runSerializer(), m_serializer);


    return new SequentialCommandGroup(
        new ParallelRaceGroup(
            new RunCommand(() -> m_esophagus.autoRunEsophagus(), m_esophagus),
            new SequentialCommandGroup(
                new InstantCommand(() -> m_intake.lowerIntake(), m_intake),
                new ParallelRaceGroup(
                    firstRamseteCommand,
                    new RunCommand(() -> m_intake.runIntake(1), m_intake)
                ),
                new ParallelRaceGroup(
                    secondRamseteCommand,
                    new RunCommand(() -> m_intake.runIntake(1), m_intake)
                ),
                new InstantCommand(() -> m_intake.runIntake(0), m_intake),
                new InstantCommand(() -> m_robotDrive.arcadeDrive(0, 0), m_robotDrive)
            )
        ),
        new ParallelRaceGroup(
            new TurnToAngleCommand(m_robotDrive, 180, 5),
            new RunCommand(() -> m_esophagus.runEsophagusToTop(), m_esophagus)
        ),
        new InstantCommand(() -> m_esophagus.stopEsophagus(), m_esophagus),
        new ParallelCommandGroup(
            new TurnToTargetCommand(m_robotDrive, m_limelight, m_driverStick, 50).withInterrupt(m_limelight::isTurnedToTarget),
            new SequentialCommandGroup(
                new RunCommand(() -> m_shooter.runShooter(), m_shooter).withInterrupt(m_shooter::isShooterAtSpeed),
                new ParallelRaceGroup(
                    new RunCommand(() -> m_shooter.runShooter(), m_shooter),
                    new RunCommand(() -> m_esophagus.runEsophagus(), m_esophagus),
                    new WaitCommand(5)
                )
            )
        ),
        new InstantCommand(() -> m_robotDrive.arcadeDrive(0, 0), m_robotDrive),
        new InstantCommand(() -> m_shooter.stopShooter(), m_shooter),
        new InstantCommand(() -> m_esophagus.stopEsophagus(), m_esophagus)
    );

    //return ramseteCommand.andThen(StraightramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0)));
  }

  public Command getGalacticSearchCommand() {

    m_robotDrive.resetEncoders();
    m_robotDrive.zeroHeading();
    m_robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
    m_intake.lowerIntake();

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, 10);

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond*0.5,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .addConstraint(autoVoltageConstraint);
    TrajectoryConfig fastconfig = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond*0.9,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .addConstraint(autoVoltageConstraint);
    TrajectoryConfig firstconfig = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond*1,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .addConstraint(autoVoltageConstraint);//.setEndVelocity(AutoConstants.kMaxSpeedMetersPerSecond*1);
    TrajectoryConfig secondconfig = new TrajectoryConfig(2.5,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .addConstraint(autoVoltageConstraint)//.setStartVelocity(AutoConstants.kMaxSpeedMetersPerSecond*1)
            .setReversed(true);
    TrajectoryConfig redBfirstconfig = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond*0.9,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setEndVelocity(AutoConstants.kMaxSpeedMetersPerSecond*0.9);
    TrajectoryConfig redBsecondconfig = new TrajectoryConfig(4.5,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setStartVelocity(AutoConstants.kMaxSpeedMetersPerSecond*0.9);


    Trajectory redAaTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        List.of(
            new Pose2d(0, 0, new Rotation2d(0)),
            new Pose2d(50 * 2.54 / 100, 0, new Rotation2d(0)),
            new Pose2d(110 * 2.54 / 100, -30 * 2.54/100, new Rotation2d(0)),
            new Pose2d(160 * 2.54 / 100, 40 * 2.54 / 100, new Rotation2d(Math.PI/2))
            //new Pose2d(200 * 2.54 / 100, 60 * 2.54 / 100, new Rotation2d(0))
        ),
        firstconfig);
        Trajectory redAbTrajectory = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(160 * 2.54 / 100, 40 * 2.54 / 100, new Rotation2d(Math.PI/2)),
                new Pose2d(250 * 2.54 / 100, 0 * 2.54 / 100, new Rotation2d(Math.PI)),
                new Pose2d(320 * 2.54 / 100, 0 * 2.54 / 100, new Rotation2d(Math.PI))

                // new Pose2d(200 * 2.54 / 100, 60 * 2.54 / 100, new Rotation2d(0)),
                // new Pose2d(300 * 2.54 / 100, 0, new Rotation2d(0)),
                // new Pose2d(320 * 2.54 / 100, 0, new Rotation2d(0))

            ),
            secondconfig);
    Trajectory redBTrajectory = TrajectoryGenerator.generateTrajectory(
        List.of(
            new Pose2d(0, 0, new Rotation2d(0)),
            new Pose2d(45 * 2.54 / 100, 30 * 2.54 / 100, new Rotation2d(0)),
            new Pose2d(105 * 2.54 / 100, -30 * 2.54 / 100, new Rotation2d(0)),
            new Pose2d(165 * 2.54 / 100, 30 * 2.54 / 100, new Rotation2d(0))
        ),
        redBfirstconfig);
        Trajectory redBbTrajectory = TrajectoryGenerator.generateTrajectory(
        List.of(
            new Pose2d(165 * 2.54 / 100, 30 * 2.54 / 100, new Rotation2d(0)),
            new Pose2d(335 * 2.54 / 100, 0, new Rotation2d(0))
        ),
        redBsecondconfig);
    Trajectory blueStartTrajectory = TrajectoryGenerator.generateTrajectory(
        List.of(
            new Pose2d(0, 0, new Rotation2d(0)),
            new Pose2d(75 * 2.54 / 100, -60 * 2.54 / 100, new Rotation2d(0)),
            new Pose2d(90 * 2.54 / 100, -60 * 2.54 / 100, new Rotation2d(0))
        ),
        config);
    Trajectory blueATrajectory = TrajectoryGenerator.generateTrajectory(
        List.of(
            new Pose2d(90 * 2.54 / 100, -60 * 2.54/100, new Rotation2d(0)),
            new Pose2d(140 * 2.54 / 100, -60 * 2.54/100, new Rotation2d(0)),
            new Pose2d(190 * 2.54 / 100, 10 * 2.54 / 100, new Rotation2d(Math.PI/2)),
            new Pose2d(220 * 2.54 / 100, 30 * 2.54 / 100, new Rotation2d(0)),
            new Pose2d(240 * 2.54 / 100, 15 * 2.54 / 100, new Rotation2d(-Math.PI/4)),
            new Pose2d(320 * 2.54 / 100, -5 * 2.54 / 100, new Rotation2d(0))
        ),
        config);
    Trajectory blueBTrajectoryFirstHalf = TrajectoryGenerator.generateTrajectory(
        List.of(
            new Pose2d(90 * 2.54 / 100, -60 * 2.54/100, new Rotation2d(0)),
            new Pose2d(135 * 2.54 / 100, -30 * 2.54 / 100, new Rotation2d(0)),
            new Pose2d(195 * 2.54 / 100, 30 * 2.54 / 100, new Rotation2d(0)),
            new Pose2d(255 * 2.54 / 100, -30 * 2.54 / 100, new Rotation2d(0))
        ),
        config);
        Trajectory blueBTrajectorySecondHalf = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(255 * 2.54 / 100, -30 * 2.54 / 100, new Rotation2d(0)),
                new Pose2d(300 * 2.54 / 100, 0, new Rotation2d(Math.PI/4))
        ),
        config);

    RamseteCommand redAStartRamseteCommand = new RamseteCommand(redAaTrajectory, m_robotDrive::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts, m_robotDrive);
        RamseteCommand redARamseteCommandSecondHalf = new RamseteCommand(redAbTrajectory, m_robotDrive::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts, m_robotDrive);
    RamseteCommand redBRamseteCommand = new RamseteCommand(redBTrajectory, m_robotDrive::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts, m_robotDrive);
        RamseteCommand redBRamseteCommandend = new RamseteCommand(redBbTrajectory, m_robotDrive::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts, m_robotDrive);

    RamseteCommand blueStartRamseteCommand = new RamseteCommand(blueStartTrajectory, m_robotDrive::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts, m_robotDrive);

    RamseteCommand blueARamseteCommand = new RamseteCommand(blueATrajectory, m_robotDrive::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts, m_robotDrive);

    RamseteCommand blueBRamseteCommandFirstHalf = new RamseteCommand(blueBTrajectoryFirstHalf, m_robotDrive::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts, m_robotDrive);
        RamseteCommand blueBRamseteCommandSecondHalf = new RamseteCommand(blueBTrajectorySecondHalf, m_robotDrive::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts, m_robotDrive);

    // Run path following command, then stop at the end.
    // return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));

    // shoot, turn right, intake down and intake in and sweep over power cells, back up, intake up, turn left, shoot
    
    // m_esophagus.setDefaultCommand(new RunCommand(() -> m_esophagus.autoRunEsophagus(), m_esophagus))
    // new Trigger(() -> m_intake.isIntakeRunning()).whileActiveContinuous(() -> m_serializer.runSerializer(), m_serializer);
    // new Trigger(() -> m_esophagus.isEsophagusRunning()).whileActiveContinuous(() -> m_serializer.runSerializer(), m_serializer);



    return new SequentialCommandGroup(
        new WaitCommand(1.5),
        new InstantCommand(() -> m_intake.runIntake(1), m_intake),
        // do we see ball, and where?
        //new SelectCommand(commands, selector)
        new ParallelRaceGroup(
            new SelectCommand(Map.ofEntries(
                Map.entry(Path.redA, redAStartRamseteCommand.andThen(redARamseteCommandSecondHalf)),
                Map.entry(Path.redB, redBRamseteCommand.andThen(redBRamseteCommandend)),
                Map.entry(Path.blueUnknown, new SequentialCommandGroup(
                    blueStartRamseteCommand,
                    new SelectCommand(Map.ofEntries(
                        Map.entry(Path.blueA, blueARamseteCommand),
                        Map.entry(Path.blueB, new SequentialCommandGroup(
                            blueBRamseteCommandFirstHalf,
                            new InstantCommand(() -> m_intake.runIntake(0), m_intake),
                            blueBRamseteCommandSecondHalf
                        )),
                        Map.entry(Path.blueUnknown, new PrintCommand("unknown"))),
                        this::detectSecondBall
                        )
                    ))
            ), this::detectFirstBall),
            new RunCommand(() -> m_esophagus.autoRunEsophagus(), m_esophagus)
        ),
        new InstantCommand(() -> m_robotDrive.arcadeDrive(0, 0), m_robotDrive),
        new InstantCommand(() -> m_shooter.stopShooter(), m_shooter),
        new InstantCommand(() -> m_esophagus.stopEsophagus(), m_esophagus)
    );

    }
    private enum Path {
        redA,
        redB,
        blueA,
        blueB,
        blueUnknown
    }

    private Path detectFirstBall() {
        System.out.println(m_pixy.ballDetected);
        if (m_pixy.ballDetected){
            if (m_pixy.x() < 180 && m_pixy.x() > 140) {
                System.out.println("red A");
                return Path.redA;
            } else if (m_pixy.x() < 140) {
                System.out.println("red B");
                return Path.redB;
            } else {
                System.out.println("impossible first");  
                return Path.blueUnknown;
            }
        } else {
            System.out.println("blue");
            return Path.blueUnknown;
        }
    }

    private Path detectSecondBall() {
        System.out.println(m_pixy.ballDetected);
        if (m_pixy.ballDetected){
            if (m_pixy.x() < 200 && m_pixy.x() > 120) {
                System.out.println("blue A");
                return Path.blueA;
            } else if (m_pixy.x() < 120) {
                System.out.println("blue B");
                return Path.blueB;
            } else {
                System.out.println("impossible second");  
                return Path.blueUnknown;
            }
        } else {
            System.out.println("impossible second no ball detected");
            return Path.blueUnknown;
        }
    }
}