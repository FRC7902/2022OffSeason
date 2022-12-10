// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class TransferSubsystem extends SubsystemBase {
  public final WPI_VictorSPX transferMotor = new WPI_VictorSPX(Constants.TransferConstants.kTransferCAN);

  /** Creates a new TransferSubsystem. */
  public TransferSubsystem() {
    transferMotor.setInverted(true);
    transferMotor.configOpenloopRamp(Constants.TransferConstants.kRampTime);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
