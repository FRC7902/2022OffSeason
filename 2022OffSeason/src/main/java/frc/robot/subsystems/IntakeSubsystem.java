package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    

    public final WPI_VictorSPX intakePowerController = new WPI_VictorSPX(Constants.IntakeConstants.IntakePowerCAN);
    public final WPI_VictorSPX intakeDeploymentController = new WPI_VictorSPX(Constants.IntakeConstants.IntakeDeploymentCAN);
    public final String intakeStatus = "";


    public IntakeSubsystem(){

        intakePowerController.setInverted(true);

    }


    public void DeployIntake(){

        intakeDeploymentController.set(Constants.IntakeConstants.IntakeDeploymentPercentOut);
    }

    public void RetractIntake(){
        intakeDeploymentController.set(-Constants.IntakeConstants.IntakeDeploymentPercentOut);
    }

    public void setIntakePower (double speed){
        intakePowerController.set(speed);
    }

    public void stopIntakePower (){
        intakePowerController.stopMotor();
    }

    public String getIntakeDirection (){
        if (intakePowerController.get() < 0){
            return "Ejecting";
        }
        else if (intakePowerController.get() > 0){
            return "Intaking";
        }
        else{
            return "Stopped";
        }
    }


}
