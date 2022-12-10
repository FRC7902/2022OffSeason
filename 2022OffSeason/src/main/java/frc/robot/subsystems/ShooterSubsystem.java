// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final WPI_VictorSPX m_leader = new WPI_VictorSPX(ShooterConstants.kShooterLeaderCAN);
  private final WPI_VictorSPX m_follower = new WPI_VictorSPX(ShooterConstants.kShooterFollowerCAN);



  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    m_follower.follow(m_leader);
    m_leader.setInverted(false);
    m_follower.setInverted(InvertType.FollowMaster);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
