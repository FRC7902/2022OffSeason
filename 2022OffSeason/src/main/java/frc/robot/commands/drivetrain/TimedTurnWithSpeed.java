package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TimedTurnWithSpeed extends CommandBase {
    DriveSubsystem m_driveSubsystem;
    Timer timer = new Timer();
    double time;
    double speed;

    public TimedTurnWithSpeed (double time, double speed, DriveSubsystem subsystem){
        m_driveSubsystem = subsystem;
        this.time = time; 
        this.speed = speed;

        addRequirements(subsystem);
    }

    @Override
    public void initialize(){
        m_driveSubsystem.driveArcade(0, 0);
        timer.reset();
        timer.start();
    }

    @Override
    public void execute(){
        m_driveSubsystem.driveArcade(0, speed);
    }

    @Override
    public void end(boolean interrupted){
        m_driveSubsystem.driveArcade(0,0);
    }

    @Override
    public boolean isFinished(){
        return timer.get() >= time;
    }
}
