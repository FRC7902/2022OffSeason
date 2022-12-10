// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ElevatorConstants; 

public class ElevatorSubsystem extends SubsystemBase {
  private final CANSparkMax m_elevator = new CANSparkMax(ElevatorConstants.kElevatorCAN, MotorType.kBrushless);
  String status = "Off";
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    m_elevator.restoreFactoryDefaults();
    m_elevator.setSmartCurrentLimit(ElevatorConstants.kCurrentLimit);
    m_elevator.setInverted(false);
  }
  public void startElevator(double power){
    m_elevator.set(power);
    if (power>0){
      status = "Rising";
    } else if (power<0){
      status = "Loweringg";
    } else{
      status = "Off";
    }
  }

  public void stopElevator(){
    m_elevator.set(ElevatorConstants.kStopValue);
    m_elevator.stopMotor();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
