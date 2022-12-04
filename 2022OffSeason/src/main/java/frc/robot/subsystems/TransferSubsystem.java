// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class TransferSubsystem extends SubsystemBase {
  private final WPI_VictorSPX transferMotor = new WPI_VictorSPX(Constants.TransferConstants.TransferCAN);
  public TransferSubsystem() {
    transferMotor.setInverted(false);
  }

  public void setPower(double power){
      transferMotor.set(power);
  }
  public void stopMotor(){
      transferMotor.stopMotor();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    // Example message
  }
}
