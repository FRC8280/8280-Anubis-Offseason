package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;

public class ExtendIntake extends Command {
  private final Intake m_intakeSubsystem;
  private final Shooter m_shooterSubsystem;

  public ExtendIntake(Intake intakeSubsystem, Shooter shooterSubsystem) {
    m_intakeSubsystem = intakeSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    addRequirements(m_shooterSubsystem,m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterSubsystem.SetAmpArmIntake();
    m_intakeSubsystem.goToGround();
  }

   // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // No need to do anything here
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

    // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
    
}
