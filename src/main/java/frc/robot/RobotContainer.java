// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

//import java.util.List;
//import java.util.Optional;

//import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

//import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Rotation3d;
//import edu.wpi.first.math.geometry.Translation3d;
//import edu.wpi.first.math.util.Units;
//import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
//import frc.robot.subsystems.Blinkin;

public class RobotContainer {

  public boolean singleDriver = false;
  public boolean demoMode = false;

  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 3.0 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
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
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  //private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  /* Path follower */
  private Command runAuto = drivetrain.getAutoPath("Tests");

  private final Telemetry logger = new Telemetry(MaxSpeed);

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
      pointer = joystick.getHID();
    else
      pointer = operator.getHID();

    if(pointer.getLeftBumper() || (pointer.getLeftTriggerAxis() > 0.65) )
      return true;
    
    return false;
  }

  private void configureBindings() {
    
    CommandXboxController pointer;
    if (singleDriver)
      pointer = joystick;
    else
      pointer = operator;

    double adjustment = 1.0;
    if(demoMode)
      adjustment = 0.5;
    
    MaxSpeed = MaxSpeed*adjustment;
    MaxAngularRate = MaxAngularRate*adjustment;

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    //drivetrain.registerTelemetry(logger::telemeterize);

    //Hat adjustment at low speed. 
    //joystick.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    //joystick.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

    /* Bindings for drivetrain characterization */
    /* These bindings require multiple buttons pushed to swap between quastatic and dynamic */
    /* Back/Start select dynamic/quasistatic, Y/X select forward/reverse direction */
    /*joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));*/
   

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

        .andThen(new WaitCommand(0.25))//.46

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
            //.andThen(new WaitCommand(0.7)) // get up to speed
            .andThen(new WaitUntilCommand(()->m_Shooter.AtTargetSpeed()))
             .andThen(Commands.parallel(new InstantCommand(() -> m_Intake.ReverseIntake()),
                                        new InstantCommand(() ->m_Shooter.StartIndexMotor())))
            
            //.andThen(new InstantCommand(() -> m_Shooter.StartIndexMotor())) // start indexer
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
            //.andThen(new WaitCommand(0.7)) // get up to speed
            .andThen(new WaitUntilCommand(()->m_Shooter.AtTargetSpeed()))
             .andThen(Commands.parallel(new InstantCommand(() -> m_Intake.ReverseIntake()),
                                        new InstantCommand(() ->m_Shooter.StartIndexMotor())))
            
            //.andThen(new InstantCommand(() -> m_Shooter.StartIndexMotor())) // start indexer
            .andThen(new WaitCommand(0.25)) // Wait for note to clear
            .andThen(Commands.parallel(new InstantCommand(() -> m_Intake.StopIntakeMotor()),
                                       //new InstantCommand(() -> m_Shooter.ManualSetShooterSpeed(0)),
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
            
            //.andThen(new InstantCommand(() -> m_Shooter.StartIndexMotor())) // start indexer
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
      //.andThen(new InstantCommand(()-> m_Shooter.ManualSetShooterSpeedboth(0.07,0.15)))
      .andThen(new InstantCommand(()-> m_Shooter.SpinShootingMotorsBoth(Constants.Shooter.k_FeedTopRPM,Constants.Shooter.k_FeedBottomRPM)))
      //.andThen(new WaitUntilCommand(()->m_Shooter.AtTargetSpeed()))
      .andThen(new WaitCommand(1))       //Wait for note to clear
      //.andThen(new InstantCommand(() -> m_Shooter.StartIndexMotor())) //start indexer
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

  public RobotContainer() {

    configureBindings();
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return runAuto;
  }
}
