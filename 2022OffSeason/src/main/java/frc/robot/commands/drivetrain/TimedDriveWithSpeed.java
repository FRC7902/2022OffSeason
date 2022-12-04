package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TimedDriveWithSpeed extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveSubsystem m_DriveSubsystem;
    Timer timer = new Timer();
    double time;
    double speed;

    public TimedDriveWithSpeed(DriveSubsystem driveSubsystem, double speed, double time) {
        m_DriveSubsystem = driveSubsystem;
        this.speed = speed;
        this.time = time;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize(){
        m_DriveSubsystem.driveArcade(0,0);
        timer.reset();
        timer.start();
    }

    @Override
    public void execute(){
        if(!isFinished()){
            m_DriveSubsystem.driveArcade(speed, 0);
        }
        else{
            m_DriveSubsystem.driveArcade(0,0);
        }
    }
    @Override
    public boolean isFinished(){
        return timer.get() >= time;
    }
    
}