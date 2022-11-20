// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private final PWMVictorSPX m_leftLeader = new PWMVictorSPX(Constants.DriveConstants.DrivetrainLeftLeaderCAN);
  private final PWMVictorSPX m_leftFollower = new PWMVictorSPX(Constants.DriveConstants.DrivetrainLeftFollowerCAN);
  private final PWMVictorSPX m_rightLeader = new PWMVictorSPX(Constants.DriveConstants.DrivetrainRightLeaderCAN);
  private final PWMVictorSPX m_rightFollower = new PWMVictorSPX(Constants.DriveConstants.DrivetrainRightFollowerCAN);

  private final MotorControllerGroup left = new MotorControllerGroup(m_leftLeader, m_leftFollower);
  private final MotorControllerGroup right = new MotorControllerGroup(m_rightLeader, m_rightFollower);

  private final DifferentialDrive drive = new DifferentialDrive(left, right);

  // // Encoders
  // private final Encoder m_leftEncoder = new Encoder(
  //     Constants.DriveConstants.kLeftEncoderA,
  //     Constants.DriveConstants.kLeftEncoderB,
  //     Constants.DriveConstants.kLeftEncoderIndex);
  // private final Encoder m_rightEncoder = new Encoder(
  //     Constants.DriveConstants.kRightEncoderA,
  //     Constants.DriveConstants.kRightEncoderB,
  //     Constants.DriveConstants.kRightEncoderIndex);

  // Gyro
  // private final AnalogGyro m_gyro = new AnalogGyro(Constants.DriveConstants.gyroChannel);

  // Simulation Stuff
  private EncoderSim m_leftEncoderSim;
  private EncoderSim m_rightEncoderSim;
  private Field2d m_fieldSim;
  private AnalogGyroSim m_gyroSim;
  public DifferentialDrivetrainSim m_driveTrainSim;
  // private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
  //     Rotation2d.fromDegrees(getHeading()),
  //     new Pose2d(4, 5, new Rotation2d()));

  public DriveSubsystem() {

    right.setInverted(true);


    // Set Encoder pulses
    // m_leftEncoder.setDistancePerPulse((0.1524 * Math.PI) / (double) 1024);
    // m_rightEncoder.setDistancePerPulse((0.1524 * Math.PI) / (double) 1024);
    // m_leftEncoder.reset();
    // m_rightEncoder.reset();
    
    
    if (RobotBase.isSimulation()) {

      // Set up robot simulation
      m_driveTrainSim = DifferentialDrivetrainSim.createKitbotSim(KitbotMotor.kDualCIMPerSide, KitbotGearing.k10p71,
          KitbotWheelSize.kSixInch, null);
      m_fieldSim = new Field2d();
      SmartDashboard.putData("Field", m_fieldSim);

      // Connect the simulators with their counterparts
      // m_leftEncoderSim = new EncoderSim(m_leftEncoder);
      // m_rightEncoderSim = new EncoderSim(m_rightEncoder);
      // m_gyroSim = new AnalogGyroSim(m_gyro);
    }

  }

  public void driveArcade(double xForward, double zRotation) {

    drive.arcadeDrive(xForward, zRotation);

  }

  // public Pose2d getPose() {
  //   return m_odometry.getPoseMeters();
  // }

  // get the heading angle from the gyro
  // public double getHeading() {
  //   return Math.IEEEremainder(m_gyro.getAngle(), 360);
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // if (RobotBase.isSimulation()){
    //   m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    //   m_fieldSim.setRobotPose(getPose());
    // }

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

    // Run and update simulation
    m_driveTrainSim.update(0.02);
    m_leftEncoderSim.setDistance(m_driveTrainSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_driveTrainSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_driveTrainSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_driveTrainSim.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_driveTrainSim.getHeading().getDegrees());

    
  }

}