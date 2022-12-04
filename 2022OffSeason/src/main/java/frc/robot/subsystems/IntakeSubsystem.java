// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class IntakeSubsystem extends SubsystemBase {

  private final WPI_VictorSPX powerMotor = new WPI_VictorSPX(Constants.IntakeConstants.IntakePowerCAN);
  private final WPI_VictorSPX deploymentMotor = new WPI_VictorSPX(Constants.IntakeConstants.IntakeDeploymentCAN);

  public IntakeSubsystem() {
    powerMotor.setInverted(true);
    deploymentMotor.setInverted(true);
  }

  public void setIntakePower(double power){
      powerMotor.set(power);
  }
  public void setDeploymentPower(double power){
      deploymentMotor.set(power);
  }
  public void stopPowerMotor(){
      powerMotor.stopMotor();
  }
  public void stopDeploymentMotor(){
      deploymentMotor.stopMotor();
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
