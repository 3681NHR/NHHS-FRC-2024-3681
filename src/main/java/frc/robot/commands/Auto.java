package frc.robot.commands;

import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.enums.LauncherState;
import frc.robot.enums.RollerState;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LauncherSwingSubsystem;

public class Auto extends Command{
    
    LauncherSubsystem m_launcherSubsystem;
    LauncherSwingSubsystem m_launcherSwingSubsystem;
    DriveSubsystem m_DriveSubsystem;
    FeederSubsystem m_FeederSubsystem;

    boolean startingFOD;

    int ticks = 0;

    SendableChooser<Pose2d> startingPoses;
    int driveStop = 250;

    public Auto(LauncherSwingSubsystem launcherSwing, LauncherSubsystem launcher, DriveSubsystem drive, FeederSubsystem feeder, SendableChooser<Pose2d> startingPoses){
        m_launcherSubsystem      = launcher;
        m_launcherSwingSubsystem = launcherSwing;
        m_DriveSubsystem         = drive;
        m_FeederSubsystem       = feeder;

        addRequirements(launcher);
        addRequirements(launcherSwing);
        addRequirements(m_DriveSubsystem);
        addRequirements(m_FeederSubsystem);

        this.startingPoses = startingPoses;
    }
    @Override
    public void initialize(){
        ticks = 0;
        m_DriveSubsystem.zero(Degree.of(startingPoses.getSelected().getRotation().getDegrees()));

        driveStop = (int)startingPoses.getSelected().getX();
    }

    @Override
    public void end(boolean e){
        m_launcherSubsystem.setSpeed(Constants.LAUNCHER.LAUNCH_SPEED_RPM);
        m_FeederSubsystem.setspeed(Constants.INDEXER.RECV_OUTPUT);
        m_DriveSubsystem   .stop();
    }

    @Override
    public void execute(){
        if(m_feederSubsystem.isHolding()){
            m_feederSubsystem.setSpeed(LauncherState.LAUNCHING);
        } else {
            m_launcherSubsystem.setSpeed(LauncherState.IDLE);
        }

        m_launcherSwingSubsystem.setAngle(Constants.LAUNCHER_SWING.LAUNCH_POSITION);

        if(m_launcherSubsystem.atspeed()){
            if(m_launcherSubsystem.isHolding()){
                m_launcherSubsystem.setRoller(RollerState.LAUNCH);
            } else {
                m_launcherSubsystem.setRoller(RollerState.IDLE);
            }
        }
        if(m_launcherSubsystem.switchEnabled() ? (!m_launcherSubsystem.isHolding()  && ticks <= driveStop) : (ticks >= 150 && ticks <= driveStop)){
            m_DriveSubsystem.setAutoMotion(0.5, 0, 0);
        } else {
            m_DriveSubsystem.setAutoMotion(0, 0, 0);
        }

        ticks++;
    }

    @Override
    public boolean isFinished(){
        return ticks >= 200 + driveStop;//50ticks = 1sec
    }
}
