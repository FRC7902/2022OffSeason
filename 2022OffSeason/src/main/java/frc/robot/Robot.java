// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private PWMVictorSPX left1 = new PWMVictorSPX(0);
  private PWMVictorSPX left2 = new PWMVictorSPX(1);
  private PWMVictorSPX right1 = new PWMVictorSPX(2);
  private PWMVictorSPX right2 = new PWMVictorSPX(3);

  private SpeedControllerGroup left = new SpeedControllerGroup(left1, left2);
  private SpeedControllerGroup right = new SpeedControllerGroup(right1, right2);

  private DifferentialDrive drive = new DifferentialDrive(left, right);

  private Timer timer = new Timer();
  private Joystick joystick = new Joystick(0);
  

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private static final KitbotWheelSize KitbotWheelSize = null;
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // Encoders
  private final Encoder m_leftEncoder = new Encoder(0, 1, 2);
  private final Encoder m_rightEncoder = new Encoder(3, 4, 5);

  // Gyro
  private final AnalogGyro m_gyro = new AnalogGyro(1);

  // Simulation Stuff
  private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
      Rotation2d.fromDegrees(getHeading()), new Pose2d(4, 5, new Rotation2d()));;
  private EncoderSim m_leftEncoderSim;
  private EncoderSim m_rightEncoderSim;
  private Field2d m_fieldSim;
  private AnalogGyroSim m_gyroSim;
  public DifferentialDrivetrainSim m_driveTrainSim;
  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    right.setInverted(true);
    
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // Set Encoder pulses
    m_leftEncoder.setDistancePerPulse((0.1524 * Math.PI) / (double) 1024);
    m_rightEncoder.setDistancePerPulse((0.1524 * Math.PI) / (double) 1024);
    m_leftEncoder.reset();
    m_rightEncoder.reset();

    if (!RobotBase.isReal()) {
      // Set up robot simulation
      m_driveTrainSim = DifferentialDrivetrainSim.createKitbotSim(KitbotMotor.kDualCIMPerSide, KitbotGearing.k10p71,
          KitbotWheelSize, null);
      m_fieldSim = new Field2d();
      SmartDashboard.putData("Field", m_fieldSim);

      // Connect the simulators with their counterparts
      m_leftEncoderSim = new EncoderSim(m_leftEncoder);
      m_rightEncoderSim = new EncoderSim(m_rightEncoder);
      m_gyroSim = new AnalogGyroSim(m_gyro);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
      
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        if(timer.get() < 2){
          drive.arcadeDrive(0, 1);
        }else if(timer.get() < 7){
          drive.arcadeDrive(1, 0);
        }
        else{
          drive.arcadeDrive(0, 0);
        }
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;}
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    drive.arcadeDrive(-joystick.getX()*0.5, -joystick.getY() );
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    m_driveTrainSim.setInputs(left.get() * RobotController.getBatteryVoltage(),
    right.get() * RobotController.getBatteryVoltage());

// Run and update simulation
m_driveTrainSim.update(0.02);
m_leftEncoderSim.setDistance(m_driveTrainSim.getLeftPositionMeters());
m_leftEncoderSim.setRate(m_driveTrainSim.getLeftVelocityMetersPerSecond());
m_rightEncoderSim.setDistance(m_driveTrainSim.getRightPositionMeters());
m_rightEncoderSim.setRate(m_driveTrainSim.getRightVelocityMetersPerSecond());
m_gyroSim.setAngle(-m_driveTrainSim.getHeading().getDegrees());

m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
m_fieldSim.setRobotPose(getPose());
  }

  public Pose2d getPose() {
    DifferentialDriveOdometry m_odometry;
    return m_odometry.getPoseMeters();
  }
}
