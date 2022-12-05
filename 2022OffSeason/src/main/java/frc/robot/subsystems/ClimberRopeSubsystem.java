// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class ClimberRopeSubsystem extends SubsystemBase {
  public final CANSparkMax mainClimberRopeMotor = new CANSparkMax(Constants.ClimberRopeConstants.Rope1CAN, MotorType.kBrushless);
  public final WPI_VictorSPX adjustmentRopeMotor = new WPI_VictorSPX(Constants.ClimberRopeConstants.Rope2CAN);

  public ClimberRopeSubsystem() {
      mainClimberRopeMotor.setSmartCurrentLimit(Constants.ClimberRopeConstants.Rope1CurrentLimit);
      mainClimberRopeMotor.setInverted(true);
      adjustmentRopeMotor.setInverted(true);
  }

  public void setMainClimberPower(double power){
    mainClimberRopeMotor.set(power);
  }
  public void setAdjustmentPower(double power){
      adjustmentRopeMotor.set(power);
  }
  public void stopMainClimberMotor(){
    mainClimberRopeMotor.stopMotor();
  }
  public void stopAdjustmentMotor(){
      adjustmentRopeMotor.stopMotor();
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