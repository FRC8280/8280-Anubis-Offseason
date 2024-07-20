package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends Command {
  private final Shooter m_shooterSubsystem;
  private final Intake m_intakeSubsystem;
  private Timer m_shotTimer;


  public ShootCommand(Shooter shooterSubsystem, Intake intakeSubsystem) {
    m_shooterSubsystem = shooterSubsystem;
    m_intakeSubsystem = intakeSubsystem;

    m_shotTimer = new Timer();
    addRequirements(m_shooterSubsystem,m_intakeSubsystem);
  }

  @Override
  public void initialize() {
    //Start up the motor
    m_shotTimer.reset();
    m_shooterSubsystem.SpinShootingMotorsDynamic();
  }

  @Override
  public void execute() {
    //When motors hit RPM or time start the indexer
    if(m_shooterSubsystem.AtTargetSpeed())
    {
        m_intakeSubsystem.ReverseIntake();
        m_shooterSubsystem.StartIndexMotor();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.StopAllMotors();
    m_intakeSubsystem.StopAllMotors();
  }

  @Override
  public boolean isFinished() {

    if(m_shotTimer.hasElapsed(Constants.Shooter.kAutoShotDuration))
        return true;
    else
        return false;
  }
    
}
