// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

//import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ExtendIntake;
import frc.robot.commands.RetractIntake;
import frc.robot.commands.ShootCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;

public class RobotContainer {
  public double m_TargetYaw = Constants.Vision.NoTarget;
  public boolean m_AutoAlign = false;
  public boolean singleDriver = false;
  public boolean demoMode = false;

  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 3.0 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driver = new CommandXboxController(0); // My joystick
  private final CommandXboxController operator = new CommandXboxController(1);
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  /* Subsytems */
  public final Climber m_Climber = Climber.getInstance();
  private final Shooter m_Shooter = Shooter.getInstance();
  private final Intake m_Intake = Intake.getInstance();
  //private static final Blinkin m_blinkin = new Blinkin(0);

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  //private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  //private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  //private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private ShootCommand m_ShootCommand;
  private RetractIntake m_RetractIntakeCommand;
  private ExtendIntake m_ExtendIntakeCommand;

  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    m_ShootCommand = new ShootCommand(m_Shooter, m_Intake);
    m_RetractIntakeCommand = new RetractIntake(m_Intake);
    m_ExtendIntakeCommand = new ExtendIntake(m_Intake, m_Shooter);

    // Register Named Commands
    NamedCommands.registerCommand("Shoot Note", m_ShootCommand);
    NamedCommands.registerCommand("Extend Intake", m_ExtendIntakeCommand);
    NamedCommands.registerCommand("Retract Intake", m_RetractIntakeCommand);
    NamedCommands.registerCommand("Shutdown Intake", new InstantCommand(() -> m_Intake.StopAllMotors()));
    NamedCommands.registerCommand("Shutdown Shooter",  new InstantCommand(() -> m_Shooter.StopAllMotors()));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  public void PeriodicCall()
  {
    //SmartDashboard.putNumber("AutoPivot Rot", calculateRotation());
    //SmartDashboard.putNumber("Right Stick",driver.getRightX());
    //SmartDashboard.putNumber("Target Yaw",m_TargetYaw);
    //SmartDashboard.putBoolean("AutoAlign",m_AutoAlign);
    }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return autoChooser.getSelected();
  }

  public void logPosition()
  {
    drivetrain.logPosition();
  }

  public void SetShooterRange(double range)
  {
    m_Shooter.m_Range = range;
  }
  
  public void FixArm()
  {
    m_Shooter.StowAmpArmIntake();
  }
  
  private boolean TriggerOrBumperDown()
  {

    XboxController pointer;
    if (singleDriver)
      pointer = driver.getHID();
    else
      pointer = operator.getHID();

    if(pointer.getLeftBumper() || (pointer.getLeftTriggerAxis() > 0.65) )
      return true;
    
    return false;
  }

  private boolean rightStickUsed()
  {
    if(Math.abs(driver.getHID().getRightX()) > 0.1)
      return true;
    
    return false;
  }
  private double calculateRotation(){

    //If the auto alignment flag isn't set or their is no target. Simply use the joystick data. 
   if(!m_AutoAlign || m_TargetYaw == Constants.Vision.NoTarget)
      return -driver.getRightX()* MaxAngularRate;

    double Kp = -0.005f;
    double min_command = 0.05f;

    double heading_error = -m_TargetYaw;
    double steering_adjust = 0.0f;
    if (Math.abs(heading_error) > 1.0) 
    {
        if (heading_error < 0) 
        {
            steering_adjust = Kp*heading_error + min_command;
        } 
        else 
        {
            steering_adjust = Kp*heading_error - min_command;
        }
    } 
    return -steering_adjust* MaxAngularRate;
  }

  private void EnableAutoAlign()
  {
    m_AutoAlign = true;
  }

  private void DisableAutoAlign()
  {
    m_AutoAlign = false;
  }
  
  private void configureBindings() {
    
    CommandXboxController pointer;
    if (singleDriver)
      pointer = driver;
    else
      pointer = operator;

    double adjustment = 1.0;
    if(demoMode)
      adjustment = 0.5;
    
    MaxSpeed = MaxSpeed*adjustment;
    MaxAngularRate = MaxAngularRate*adjustment;

    //Todo: Set automatic alignment code. 
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(calculateRotation()) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));

    driver.a()
     .onTrue(new InstantCommand(() -> EnableAutoAlign()));

    Trigger rightStickUsed = new Trigger( ()-> this.rightStickUsed());
    rightStickUsed.onTrue(new InstantCommand(() -> DisableAutoAlign()));

    //Original breaking code from CTRE
    /*joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));*/

    // reset the field-centric heading on left bumper press
    driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    drivetrain.registerTelemetry(logger::telemeterize);

    //Hat adjustment at low speed. 
    //joystick.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    //joystick.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

    /* Bindings for drivetrain characterization */
    /* These bindings require multiple buttons pushed to swap between quastatic and dynamic */
    /* Back/Start select dynamic/quasistatic, Y/X select forward/reverse direction */
    driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
   

    /*
     * driver.leftTrigger()
     * .onTrue(new InstantCommand(() -> s_Swerve.goSlow()))
     * .onFalse(new InstantCommand(() -> s_Swerve.NormalSpeed()));
     */

    //Operator Commands
    //Todo Xavier are you using these?
    /*Trigger safety = new Trigger( () -> ( (operator.getLeftX() > 0.75) || (operator.getLeftY() > 0.75)) );
      safety.whileTrue(new InstantCommand(()->m_Shooter.SetShooterAngle(35)));

    operator.back()
      .onTrue(Commands.parallel(new InstantCommand(() -> m_Shooter.DisableOverride()),
            new InstantCommand(() -> m_Climber.SetClimberCruise())));*/
      
    // Operator Commands
    pointer.leftTrigger() // intake note
        .onTrue(Commands.parallel(new InstantCommand(()->m_Shooter.SetAmpArmIntake()),new InstantCommand(() -> m_Intake.goToGround()),
            new InstantCommand(() -> m_Shooter.pivotLoaded())));

    Trigger retractIntake = new Trigger( ()-> this.TriggerOrBumperDown());
    retractIntake.onFalse(new InstantCommand(() -> m_Intake.goToStow()));

    pointer.leftBumper() // eject note
      .onTrue(Commands.parallel(new InstantCommand(() -> m_Intake.goToGroundForAmp()),
          new InstantCommand(() -> m_Shooter.pivotLoaded())));

    Trigger stowedIntakeAmpArmUp = new Trigger( () -> ( (m_Intake.isPivotStowed()) && (m_Shooter.IsArmInTakePosition())) );
    stowedIntakeAmpArmUp.onTrue(new InstantCommand(() -> m_Shooter.ShutDownAmpArm()));

    // Todo Trigger #1 if intake is in position then move to indexer
    Trigger intakeTrigger = new Trigger(() -> m_Intake.NoteReadyToTransfer());  
    intakeTrigger.onTrue(new WaitCommand(0.3)
         .andThen(Commands.parallel(//new InstantCommand(() -> m_Shooter.ShutDownAmpArm()),
                                    new InstantCommand(() -> m_Intake.ReverseIntake()),
                                    new InstantCommand(() -> m_Shooter.IntakeIndexer())))


        //.andThen(new WaitCommand(0.2))//.46
        .andThen(new WaitUntilCommand(()->m_Shooter.getShooterHasNote()))
        //Todo replace this wait command with sensor

        .andThen(Commands.parallel(new InstantCommand(() -> m_Intake.StopIntakeMotor()), //stop motors
            new InstantCommand(() -> m_Shooter.StopIndexMotor())))

        .andThen(new InstantCommand(() -> m_Shooter.AdjustIndexPostIntake())) // back the note up using sensor
        .andThen(new WaitCommand(0.45)) // Simple timer  //37
        
        .andThen(new InstantCommand(() -> m_Shooter.StopIndexMotor())) // stop the adjustment
        .andThen(new InstantCommand(() -> m_Shooter.DisableOverride())) // disable override
        );

    Trigger inTakeEmpty = new Trigger(()->m_Intake.intakeEmpty());

    // Take the normal shot
    pointer.rightTrigger()
      .and(inTakeEmpty)  
      .onTrue(new InstantCommand(() -> m_Shooter.SpinShootingMotorsDynamic()) // Turn on the shooter
            .andThen(new WaitUntilCommand(()->m_Shooter.AtTargetSpeed()))
             .andThen(Commands.parallel(new InstantCommand(() -> m_Intake.ReverseIntake()),
                                        new InstantCommand(() ->m_Shooter.StartIndexMotor())))
            
            .andThen(new WaitCommand(0.25)) // Wait for note to clear
            .andThen(Commands.parallel(new InstantCommand(() -> m_Intake.StopIntakeMotor()),
                                       //new InstantCommand(() -> m_Shooter.ManualSetShooterSpeed(0)),
                                       new InstantCommand(() -> m_Shooter.OverrideShooterToZero()),
                                       new InstantCommand(() -> m_Shooter.StopIndexMotor())))
            .andThen(new InstantCommand(() -> m_Shooter.pivotLoaded())));


    //Try to transfer the note as it's not empty first
    Trigger inTakeNotEmpty = new Trigger(()->m_Intake.getIntakeHasNote());
    pointer.rightTrigger()
      .and(inTakeNotEmpty)  
      //Run the transfer
        .onTrue(Commands.parallel(new InstantCommand(() -> m_Intake.ReverseIntake()),
                                    new InstantCommand(() -> m_Shooter.IntakeIndexer()))

        .andThen(new WaitCommand(0.45))

        .andThen(Commands.parallel(new InstantCommand(() -> m_Intake.StopIntakeMotor()), //stop motors
            new InstantCommand(() -> m_Shooter.StopIndexMotor())))

        .andThen(new InstantCommand(() -> m_Shooter.AdjustIndexPostIntake())) // back the note up using sensor
        .andThen(new WaitCommand(0.45)) // Simple timer
        
        .andThen(new InstantCommand(() -> m_Shooter.StopIndexMotor())) // stop the adjustment
        .andThen(new InstantCommand(() -> m_Shooter.DisableOverride())) // disable override
        
      //Normal shoot
      .andThen(new InstantCommand(() -> m_Shooter.SpinShootingMotorsDynamic()) // Turn on the shooter
            .andThen(new WaitUntilCommand(()->m_Shooter.AtTargetSpeed()))
             .andThen(Commands.parallel(new InstantCommand(() -> m_Intake.ReverseIntake()),
                                        new InstantCommand(() ->m_Shooter.StartIndexMotor())))
            
            .andThen(new WaitCommand(0.25)) // Wait for note to clear
            .andThen(Commands.parallel(new InstantCommand(() -> m_Intake.StopIntakeMotor()),
                                       new InstantCommand(() -> m_Shooter.OverrideShooterToZero()),
                                       new InstantCommand(() -> m_Shooter.StopIndexMotor())))
            .andThen(new InstantCommand(() -> m_Shooter.pivotLoaded()))));
            
    pointer.rightBumper()
        .whileTrue(Commands.parallel(new InstantCommand(() -> m_Shooter.SetAmpArmIntake()),
                                       new InstantCommand(() -> m_Intake.ejectNote())));

    // climber commands
    pointer.povDown()
        .onTrue(Commands.parallel(new InstantCommand(() -> m_Climber.DecreasePower()),
                                  new InstantCommand(() -> m_Shooter.PivotFlat())))
        .onFalse(new InstantCommand(() -> m_Climber.NoPower()));
    pointer.povUp()
        .onTrue(Commands.parallel(new InstantCommand(() -> m_Climber.IncreasePower()),
                                  new InstantCommand(() -> m_Shooter.PivotFlat())))
        .onFalse(new InstantCommand(() -> m_Climber.NoPower()));

    // Manually change the pivots.
    
    Trigger povSubWooferShotTrigger = new Trigger ( () -> ( m_Intake.isPivotStowed() ));
    pointer.povRight()
     .and(povSubWooferShotTrigger)
        .whileTrue(new InstantCommand(() -> m_Shooter.IncreasePivot(Constants.Shooter.k_ShooterPivotIncrement)));
    pointer.povLeft()
     .and(povSubWooferShotTrigger)
        .whileFalse(new InstantCommand(() -> m_Shooter.DecreasePivot(Constants.Shooter.k_ShooterPivotIncrement)));
       
    pointer.x()
     .and(povSubWooferShotTrigger)
        .onTrue(new InstantCommand(() -> m_Shooter.SubWooferShot())
            .andThen(Commands.parallel(new InstantCommand(() -> m_Intake.StopIntakeMotor()),
                new InstantCommand(() -> m_Shooter.ManualSetShooterSpeed(0)),
                new InstantCommand(() -> m_Shooter.StopIndexMotor())))
            .andThen(new InstantCommand(() -> m_Intake.goToStow()))); 

      /*New Amp Shot */
      pointer.b() //Amp shot 
      .and(inTakeEmpty)  
      //Todo: move the arm into the right position and wait until its ready. 
      .onTrue(new InstantCommand(()->m_Shooter.PivotAmpShot()) // Turn on the shooter
            .andThen(new InstantCommand(()->m_Shooter.SetAmpArmScore()))
            .andThen(new WaitCommand(0.7)) // get up to speed
            .andThen(new InstantCommand(() -> m_Shooter.SpinShootingMotorsBoth(Constants.Shooter.k_AmpTopRPM,Constants.Shooter.k_AmpBottomRPM)))
            .andThen(new WaitUntilCommand(()->m_Shooter.AtTargetSpeed()))
             .andThen(Commands.parallel(new InstantCommand(() -> m_Intake.ReverseIntake()),
                                        new InstantCommand(() ->m_Shooter.StartIndexMotor())))
            
            .andThen(new WaitCommand(1.25)) // Wait for note to clear
            .andThen(Commands.parallel(new InstantCommand(() -> m_Intake.StopIntakeMotor()),
                                       //new InstantCommand(() -> m_Shooter.ManualSetShooterSpeed(0)),
                                       new InstantCommand(() -> m_Shooter.OverrideShooterToZero()),
                                       new InstantCommand(() -> m_Shooter.StopIndexMotor())))
            .andThen(new InstantCommand(()->m_Shooter.StowAmpArmIntake()))
            .andThen(new InstantCommand(() -> m_Shooter.pivotLoaded())));
            //Todo: Add a return the Amp arm to original position


      if (singleDriver == false){
      pointer.y()
        .onTrue(new InstantCommand(()-> m_Shooter.FeedShot())
      .andThen(new WaitCommand(1.0))
      .andThen(new InstantCommand(()-> m_Shooter.SpinShootingMotorsBoth(Constants.Shooter.k_FeedTopRPM,Constants.Shooter.k_FeedBottomRPM)))
      .andThen(new WaitCommand(1))       //Wait for note to clear
      .andThen(Commands.parallel(new InstantCommand(() -> m_Intake.ReverseIntakeNudge()),
                                        new InstantCommand(() ->m_Shooter.StartIndexMotor())))  
      .andThen(new WaitCommand(2))       //Wait for note to clear
      .andThen(Commands.parallel(new InstantCommand(() -> m_Intake.StopIntakeMotor()),
                                 new InstantCommand(() ->m_Shooter.OverrideShooterToZero()),
                                 new InstantCommand(() ->m_Shooter.StopIndexMotor())))
      .andThen(new InstantCommand(() -> m_Shooter.pivotLoaded()))
      );
      }
  }


}
