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

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private final WPI_VictorSPX m_leftLeader = new WPI_VictorSPX(Constants.DriveConstants.DrivetrainLeftLeaderCAN);
  private final WPI_VictorSPX m_leftFollower = new WPI_VictorSPX(Constants.DriveConstants.DrivetrainLeftFollowerCAN);
  private final WPI_VictorSPX m_rightLeader = new WPI_VictorSPX(Constants.DriveConstants.DrivetrainRightLeaderCAN);
  private final WPI_VictorSPX m_rightFollower = new WPI_VictorSPX(Constants.DriveConstants.DrivetrainRightFollowerCAN);

  private final MotorControllerGroup left = new MotorControllerGroup(m_leftLeader, m_leftFollower);
  private final MotorControllerGroup right = new MotorControllerGroup(m_rightLeader, m_rightFollower);

  private final DifferentialDrive drive = new DifferentialDrive(left, right);


  private EncoderSim m_leftEncoderSim;
  private EncoderSim m_rightEncoderSim;
  private Field2d m_fieldSim;
  private AnalogGyroSim m_gyroSim;
  public DifferentialDrivetrainSim m_driveTrainSim;

  public DriveSubsystem() {

    right.setInverted(true);

    
    if (RobotBase.isSimulation()) {

      // Set up robot simulation
      m_driveTrainSim = DifferentialDrivetrainSim.createKitbotSim(KitbotMotor.kDualCIMPerSide, KitbotGearing.k10p71,
          KitbotWheelSize.kSixInch, null);
      m_fieldSim = new Field2d();
      SmartDashboard.putData("Field", m_fieldSim);

    }

  }

  public void driveArcade(double xForward, double zRotation) {

    drive.arcadeDrive(xForward, zRotation);

  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {

    m_driveTrainSim.update(0.02);
    m_leftEncoderSim.setDistance(m_driveTrainSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_driveTrainSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_driveTrainSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_driveTrainSim.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_driveTrainSim.getHeading().getDegrees());

    
  }

}