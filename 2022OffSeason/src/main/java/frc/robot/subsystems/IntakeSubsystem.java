// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.Constants.IntakeConstants;;

public class IntakeSubsystem extends SubsystemBase {
  public WPI_VictorSPX intakePower = new WPI_VictorSPX(IntakeConstants.kIntakePowerCAN);
  public WPI_VictorSPX intakeDep = new WPI_VictorSPX(IntakeConstants.kIntakeDepCAN);
  String status = "Off";

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakePower.setInverted(true);
    intakeDep.setInverted(true);

    intakePower.configOpenloopRamp(IntakeConstants.kPowerRampTime);
    intakeDep.configOpenloopRamp(IntakeConstants.kDeplRampTime);

  }

  public void setIntakePower(double speed){
    intakePower.set(speed);
    if (speed>0){
      status = "Sucking";
    } else if(speed<0){
      status = "Returning";
    } else{
      status = "Off";
    }
  }

  public void stopIntakePower(){
    intakePower.stopMotor();
    status = "Off";
  }

  public void setIntakeArm(double speed){
    intakeDep.set(speed);
  }

  public void stopIntakeArm(){
    intakeDep.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
