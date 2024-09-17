package frc.robot.subsystems;

import java.math.RoundingMode;
import java.text.DecimalFormat;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
//import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import frc.robot.subsystems.Intake.IntakeState;
//import frc.robot.subsystems.Intake.PivotTarget;

public class Shooter extends SubsystemBase {

  private Timer m_EmergencyTimer;
  private Timer m_PreShotTimer;
  private Timer m_BarClearTimer;
  private Timer m_AdjustmentTimer;

  private CANSparkFlex m_topMotor;
  private CANSparkFlex m_bottomMotor;
  private CANSparkMax m_indexMotor;
  private CANSparkMax m_sPivotMotor;
  private CANSparkMax m_ampArmMotor;

  //Control for amp arm motor

  public enum ampArmStates {
    kScoringPosition,
    kIntakePositionUp,
    kStowed
  }

  private ampArmStates ampArmState = ampArmStates.kStowed;
  //private boolean m_AmpArmActive = false;
  //private boolean m_StowAmpArm = false;
  private double currentAmpArmPosition = Constants.Shooter.k_AmpArmHalfWayPointAbs;

  private SparkPIDController m_ampPidController;
  
  private final SparkAbsoluteEncoder m_ampAbsoluteEncoder;
  private RelativeEncoder m_ampArmEncoder;

  private SparkPIDController m_PidController;
  private RelativeEncoder m_topMotorEncoder;
  private RelativeEncoder m_bottomMotorEncoder;

  //private RelativeEncoder mEncoder;
  private final SparkAbsoluteEncoder m_pAbsoluteEncoder; 

  private SparkPIDController m_TopPidController;
  private SparkPIDController m_BottomPidController;

  //private boolean m_pivotRunning;
  private boolean m_shooterRunning;
  public double m_TopShooterMotorSpeed = 0;
  public double m_BottomShooterMotorSpeed = 0;
  //private boolean m_Indexing;
  public double m_Range = Constants.Vision.NoTarget;
  private boolean m_Override = false;
  private boolean m_DemoMode = false;

  private static final double k_sPivotMotorP =  2;
  private static final double k_sPivotMotorI = 0.0;
  private static final double k_sPivotMotorD = 0;//0.001;
  private static final double k_sPivotMaxOutput = 0.35;
  private static final double k_sPivotMinOutput = -0.35;
  private static final double kIz = 0; 
  private static final double kFF = 0; 
  private double currentSPivotPosition = Constants.Shooter.k_ShooterPivotLoaded;
  //private double currentAPivotPosition = Constats.Amp.[Insert here idk yet]

  //Amp Arm PID
  private static final double k_AmpArmP =  1;//2;
  private static final double k_AmpArmI = 0.0;
  private static final double k_AmpArmD = 0.01;//0.001
  private static final double k_AmpArmMaxOutput = 1;
  private static final double k_AmpArmMinOutput = -1;
  //private static final double kAmpArmIz = 0; 
  //private static final double kAmpArmFF = 0; 


  //Shooter PID
  private static final double k_sShooterMotorP =  0.0006;//0.00006;
  private static final double k_sShooterMotorI = 0.0;
  private static final double k_sShooterMotorD = 0.000;
  private static final double k_sShooterMaxOutput = 1;
  private static final double k_sShooterMinOutput = -1;
  private static final double kShooterIz = 0; 
  private static final double kShooterFF = 0.000159; 

  private TimeOfFlight distanceSensor;

  private static Shooter mInstance;
  private double lastPivot;

  public static Shooter getInstance() {
    if (mInstance == null) {
      mInstance = new Shooter();
    }
    return mInstance;
  }

  /** Creates a new ShooterSubsystem. */
  public Shooter() {

    m_EmergencyTimer = new Timer();
    m_BarClearTimer = new Timer();
    m_AdjustmentTimer = new Timer();
    m_PreShotTimer = new Timer();
    
    m_AdjustmentTimer.reset();
    m_BarClearTimer.reset();
    m_BarClearTimer.start();

    // create two new SPARK MAXs and configure them
    m_sPivotMotor =
      new CANSparkMax(Constants.Shooter.kPivotShooterCanId, CANSparkLowLevel.MotorType.kBrushless);
    m_sPivotMotor.setInverted(false);
    m_sPivotMotor.setSmartCurrentLimit(Constants.Shooter.kSPivotCurrentLimit);
    m_sPivotMotor.setIdleMode(IdleMode.kCoast);
    m_sPivotMotor.burnFlash();

    m_PidController = m_sPivotMotor.getPIDController();
    m_PidController.setP(k_sPivotMotorP);
    m_PidController.setI(k_sPivotMotorI);
    m_PidController.setD(k_sPivotMotorD);
    m_PidController.setIZone(kIz);
    m_PidController.setFF(kFF);
    m_PidController.setOutputRange(k_sPivotMinOutput, k_sPivotMaxOutput);

    m_pAbsoluteEncoder = m_sPivotMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    m_PidController.setFeedbackDevice(m_pAbsoluteEncoder);

    m_topMotor =
        new CANSparkFlex(Constants.Shooter.kTopCanId, CANSparkLowLevel.MotorType.kBrushless);
    m_topMotor.restoreFactoryDefaults();
    m_topMotor.setInverted(false);
    m_topMotor.setSmartCurrentLimit(Constants.Shooter.kCurrentLimit);
    m_topMotor.setIdleMode(IdleMode.kBrake);
    m_topMotorEncoder = m_topMotor.getEncoder();

    m_bottomMotor =
        new CANSparkFlex(Constants.Shooter.kBottomCanId, CANSparkLowLevel.MotorType.kBrushless);
    m_bottomMotor.restoreFactoryDefaults();
    m_bottomMotor.setInverted(false);
    m_bottomMotor.setSmartCurrentLimit(Constants.Shooter.kCurrentLimit);
    m_bottomMotor.setIdleMode(IdleMode.kBrake);
    m_bottomMotorEncoder = m_bottomMotor.getEncoder();

    m_indexMotor =
      new CANSparkMax(Constants.Shooter.kIndexCanId, CANSparkLowLevel.MotorType.kBrushless);
    m_indexMotor.setInverted(false);
    m_indexMotor.setSmartCurrentLimit(Constants.Shooter.kIndexCurrentLimit);
    m_indexMotor.setIdleMode(IdleMode.kBrake);
    m_indexMotor.burnFlash();

    m_ampArmMotor =
      new CANSparkMax(Constants.Shooter.kAmpArmId, CANSparkLowLevel.MotorType.kBrushless);
      m_ampArmMotor.restoreFactoryDefaults();
    m_ampArmMotor.setInverted(false);
    m_ampArmMotor.setIdleMode(IdleMode.kCoast);
    m_ampArmMotor.burnFlash();

    m_ampPidController = m_ampArmMotor.getPIDController();
    m_ampPidController.setP(k_AmpArmP);
    m_ampPidController.setI(k_AmpArmI);
    m_ampPidController.setD(k_AmpArmD);
    m_ampPidController.setIZone(0);
    m_ampPidController.setFF(0);
    m_ampPidController.setOutputRange(k_AmpArmMinOutput, k_AmpArmMaxOutput);
    /*Replacing with absolute encoder*/
    m_ampArmEncoder = m_ampArmMotor.getEncoder();
    m_ampArmEncoder.setPosition(0);

    m_ampAbsoluteEncoder = m_ampArmMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    m_ampPidController.setFeedbackDevice(m_ampAbsoluteEncoder);
    
    m_TopPidController = m_topMotor.getPIDController();
    m_TopPidController.setP(k_sShooterMotorP);
    m_TopPidController.setI(k_sShooterMotorI);
    m_TopPidController.setD(k_sShooterMotorD);
    m_TopPidController.setIZone(kShooterIz);
    m_TopPidController.setFF(kShooterFF);
    m_TopPidController.setOutputRange(k_sShooterMinOutput, k_sShooterMaxOutput);

    m_BottomPidController = m_bottomMotor.getPIDController();
    m_BottomPidController.setP(k_sShooterMotorP);
    m_BottomPidController.setI(k_sShooterMotorI);
    m_BottomPidController.setD(k_sShooterMotorD);
    m_BottomPidController.setIZone(kShooterIz);
    m_BottomPidController.setFF(kShooterFF);
    m_BottomPidController.setOutputRange(k_sShooterMinOutput, k_sShooterMaxOutput);

    m_topMotor.burnFlash();
    m_bottomMotor.burnFlash();

    //m_pivotRunning = false;
    m_shooterRunning = false;
    distanceSensor = new TimeOfFlight(40);

    //m_Indexing = false;
    lastPivot = Constants.Shooter.k_ShooterPivotLoaded;
  }

  public void EnableOverride()
  {
    m_Override = true;
  }

  public void DisableOverride()
  {
    m_Override = false;
  }
  /**
   * Turns the shooter on. Can be run once and the shooter will stay running or run continuously
   * in a {@code RunCommand}.
   */
  public void runShooter() {
    m_shooterRunning = true;
  }

   public void SpinShootingMotorsDynamic()  //Run this to base on range. 
   {
      //I see nothing
      if(m_Range == Constants.Vision.NoTarget)
      {  
        SpinShootingMotors(Constants.Shooter.k_ShooterRPM);
        return;
      }

      if(m_Range <= 1.5)
        SpinShootingMotors(Constants.Shooter.k_ShooterRPM);
      else
        SpinShootingMotors(Constants.Shooter.k_ShooterMAXRPM);
   }
  //Spins motors at setrpm and not power
  public void SpinShootingMotors(double motorspeed)  //Fires a shot at max velocity without changing angle
  {
    if(m_DemoMode)
    motorspeed = motorspeed *0.5;
      m_EmergencyTimer.reset();
      m_EmergencyTimer.start();

      m_shooterRunning = true;
      m_TopShooterMotorSpeed = motorspeed;
      m_BottomShooterMotorSpeed = motorspeed;
  }

  //Spins motors at setrpm and not power
  public void SpinShootingMotorsBoth(double TopMotorspeed,double BottomMotorspeed)  //Fires a shot at max velocity without changing angle
  {
      m_EmergencyTimer.reset();
      m_EmergencyTimer.start();

      m_shooterRunning = true;
      m_TopShooterMotorSpeed = TopMotorspeed;
      m_BottomShooterMotorSpeed = BottomMotorspeed;
  }

  //sets motor power
  public void ManualSetShooterSpeed(double x)
  {
    m_topMotor.set(x);
    m_bottomMotor.set(x);
  }

  public Boolean AtTargetSpeed()  //Returns True if the motors have reached the velocity
  {   
      //Emergency stop if the battery is too low and we can't hit the velocity. 
      if(m_EmergencyTimer.hasElapsed(Constants.Shooter.k_RPMShooterTimeLimit))
      {
        m_EmergencyTimer.stop();
        return true;
      }  
      //Note change to fire when either roller hits 90% of target velocity
      if( ((m_topMotorEncoder.getVelocity() / m_TopShooterMotorSpeed ) >= 0.9) 
      && ((m_bottomMotorEncoder.getVelocity() / m_BottomShooterMotorSpeed ) >= 0.9))
        return true;
      else
        return false;
  }
  //sets motor power
  public void ManualSetShooterSpeedboth(double top, double bottom)
  {
    m_topMotor.set(top);
    m_bottomMotor.set(bottom);
  }
  /**
   * Turns the shooter off. Can be run once and the shooter will stay running or run continuously
   * in a {@code RunCommand}.
   */
  public void stopShooter() {
    m_shooterRunning = false;
  }

  public void IncreasePivot(double increment)
  {
      System.out.printf("******Increase Pivot****** %f + %f\n",currentSPivotPosition,increment);

      if(currentSPivotPosition + increment < 0.75 )
        currentSPivotPosition+= increment;

      lastPivot = currentSPivotPosition;
  }

  public void DecreasePivot(double increment)
  {
      System.out.printf("******Decrease Pivot****** %f + %f\n",currentSPivotPosition,increment);
      if(currentSPivotPosition - increment > 0.06 )
        currentSPivotPosition -= increment;
      
      lastPivot = currentSPivotPosition;
  }
  //manual pivot commands to test launcher

  public void LastManualPivot()
  {
    currentSPivotPosition = lastPivot;
  }

  public void SubWooferShot()
  {
     currentSPivotPosition = Constants.Shooter.k_SubWoofer;  
     EnableOverride();
  }

  public void SubWooferShotAuton()
  {
     currentSPivotPosition = Constants.Shooter.k_SubWooferAuton;  
  }

  public void PivotAmpShot()
  {
     currentSPivotPosition = Constants.Shooter.k_AmpShot;  
     EnableOverride();
  }

  public void PivotFlat()
  {
     currentSPivotPosition = Constants.Shooter.k_PivotNinety;  
     EnableOverride();
  }

  public void WingShot()
  {
    currentSPivotPosition = Constants.Shooter.k_WingShot;  
  }
  public void StageShot()
  {
     currentSPivotPosition = Constants.Shooter.k_StageShot;
  }
  public void FeedShot()
  {
     currentSPivotPosition = Constants.Shooter.k_FeedShot;
  }
  
  public void pivotZero()
  {
      currentSPivotPosition = Constants.Shooter.k_PivotZero;  
  }

  public void pivotLoaded()
  {
    currentSPivotPosition = Constants.Shooter.k_ShooterPivotLoaded;
    EnableOverride();
  }

  public void pivotFortyFive()
  {
      currentSPivotPosition = Constants.Shooter.k_PivotMiddle;
  }

  public void pivotNinety()
  {
    currentSPivotPosition = Constants.Shooter.k_PivotNinety;
  }

  public void SetAmpArmScore()
  {
     currentAmpArmPosition = Constants.Shooter.k_AmpArmScorAbs;  
     //m_AmpArmActive = true;
     //m_StowAmpArm = false;

     ampArmState = ampArmStates.kScoringPosition;
  }

  public void SetAmpArmIntake()
  {
    m_BarClearTimer.reset();
    m_BarClearTimer.start();

    //m_AmpArmActive = true;
    //m_StowAmpArm = false;
    ampArmState = ampArmStates.kIntakePositionUp;
    currentAmpArmPosition = Constants.Shooter.k_AmpArmIntakePositionAbs;
  }

  public boolean IsArmInTakePosition()
  {
    if (ampArmState == ampArmStates.kIntakePositionUp)
      return true;
    else 
      return false;

  }
  public void StowAmpArmIntake()
  {
      ampArmState = ampArmStates.kStowed;
      currentAmpArmPosition = Constants.Shooter.k_AmpArmHalfWayPointAbs;
      //m_AmpArmActive = false;
      //m_StowAmpArm = true;
  }
  public void ShutDownAmpArm()
  {
    if(!m_BarClearTimer.hasElapsed(0.5))
      return;

     ampArmState = ampArmStates.kStowed;
     //m_AmpArmActive = false;
     //m_StowAmpArm = false;
     currentAmpArmPosition = 0;  
     m_ampArmMotor.set(0.0);
  }

  public enum PivotTarget {
    NONE,
    LOW,
    MEDIUM,
    HIGH,
    LOADED
  }

  /*  public boolean IndexComplete()
  {
    if(m_Indexing && getShooterHasNote())
    {
      m_Indexing =false;
      return true;
    }  
    else
      return false;
  }*/

  public boolean getShooterHasNote() {

    if(distanceSensor.getRange() < Constants.Shooter.kNoteDetectionDistance)
      return true;
    else 
      return false; 
  } 

    public boolean getShooterNoteAdjusted() {

    if(m_AdjustmentTimer.hasElapsed(0.60))  //Override don't go too long. 
    {
       m_AdjustmentTimer.stop();
       m_AdjustmentTimer.reset();
       return true;
    }

     if(distanceSensor.getRange() > Constants.Shooter.kNoteTooCloseShooters)
        return true;
      else 
        return false; 
  } 

  public void PreShotAdjustment()
  {
    if(distanceSensor.getRange() > Constants.Shooter.kNoteTooCloseShooters)
      return;
    else
      {
        m_PreShotTimer.reset();
        m_PreShotTimer.start();
        m_indexMotor.set(-0.5);
      }
  }

  public boolean PreShotClear()
  {
    if(m_PreShotTimer.hasElapsed(Constants.Shooter.kPreShotLimit))
      return true;
   
    if(distanceSensor.getRange() > Constants.Shooter.kNoteTooCloseShooters)
      return true;
    else
      return false;
  }

  public boolean testAdjustedNote()
  {
    if(distanceSensor.getRange() > Constants.Shooter.kNoteDetectionDistanceAdjusted)
      return true;
    else 
      return false; 
  }
  /*public boolean getShooterIsAdjusted() {

   if(distanceSensor.getRange() < 70)
    {
      //System.out.printf("******Adjusted value over 70******\n");
      return true;
    }
    else 
      return false;
  }*/

  public void SetShooterAngle(double angle)
  {
    return;
    //Todo - fix this so it's save
    //currentSPivotPosition = angle;
    //EnableOverride();
  }

  public void OverrideShooterToZero()
 {
  m_topMotor.set(0.0);
    m_bottomMotor.set(0.0);
    m_shooterRunning = false;
    m_TopShooterMotorSpeed = 0;
    m_BottomShooterMotorSpeed = 0;
 }

  public void StopAllMotors()
  {
    m_sPivotMotor.set(0);
    m_indexMotor.set(0.0);
    m_topMotor.set(0.0);
    m_bottomMotor.set(0.0);
  }

  public void AdjustIndexPostIntake()
  {
    //if(testAdjustedNote()==true)
   //   return;

    m_AdjustmentTimer.reset();
    m_AdjustmentTimer.start();

    m_indexMotor.set(-0.40);
  }

  public void IntakeIndexerModified()
  {
    m_indexMotor.set(Constants.Shooter.kIndexPowerSlow);
  }
  public void IntakeIndexer()
  {
    //m_Indexing = true;
    m_indexMotor.set(Constants.Shooter.kIndexPower);
  }

  public void ReverseIndexer()
  {
    //m_Indexing = true;
    m_indexMotor.set(-Constants.Shooter.kIndexPower);
  }

public void ReverseIndexerLight()
  {
    //m_Indexing = true;
    m_indexMotor.set(-Constants.Shooter.kIndexPower/2);
  }

  public void StartIndexMotor()
  {
    m_indexMotor.set(Constants.Shooter.kIndexPower);
  }

  public void StopAllShooterMotors()
  {
    StopIndexMotor();
    StopShootingMotor();
  }

  public void StopIndexMotor()
  {
    m_indexMotor.set(0);
  }

  public void StopShootingMotor()
  {
    m_shooterRunning = false;
    m_TopShooterMotorSpeed = 0;
    m_BottomShooterMotorSpeed = 0;
  }

  public double CalculateReference(double distance)
  {

    //Calculates the correct encoder value based on distacne to the target.
    //\double x =  Constants.Shooter.kAutoElevationConstant*(distance - Constants.Shooter.kBaseVisionDistance ) + Constants.Shooter.kBaseShooterElevation;
   // System.out.printf("******Calculated Pivot****** %f\n",x);

    //double adjustment = -4;

    if(distance <= 0.68)
      return Constants.Shooter.k_SubWoofer;
    else if( (distance > 0.68) && (distance <0.74) )//.8
      return 0.247;//14.183;// + adjustment;
    else if( (distance >= 0.74) && (distance <0.95) )
      return 0.37;//17.492;// + adjustment;
    else if( (distance >= 0.95) && (distance <1.05) )
      return 0.39;//18.588;// + adjustment;
    else if( (distance >= 1.05) && (distance <1.15) )
      return 0.44;//19.683;// + adjustment ;

    else if( (distance >= 1.15) && (distance <1.27) )
      return 0.46;//24.57;// + adjustment-.5;
    else if( (distance >= 1.27) && (distance <1.34) )
      return 0.47;//25.12;// + adjustment;
    else if( (distance >= 1.34) && (distance <1.35) )
      return 0.48;//25.6696;// + adjustment;
    else if( (distance >= 1.35) && (distance <1.43) )
      return 0.50;//26.217;//+ + adjustment;
    else if( (distance >= 1.43) && (distance <1.46) ) //old
      return 0.51;//26.759;// + adjustment;
    else if( (distance >= 1.46) && (distance <1.49) )
      return 0.54;//26.759;// + adjustment;
    else if( (distance >= 1.49) && (distance <1.53) )
      return 0.56;//26.759;// + adjustment;
    else if( (distance >= 1.52) && (distance <1.55) )
      return 0.58;//26.759;// + adjustment;
    else if( (distance >= 1.55) && (distance <1.58) )
      return 0.56;//26.759;// + adjustment;
    else if( (distance >= 1.58) && (distance <1.61) )
      return 0.56;//26.759;// + adjustment;
    else if( (distance >= 1.61) && (distance <1.64) )  //2
      return 0.56;//26.759;// + adjustment;
    else if( (distance >= 1.64) && (distance <1.67) )
      return 0.56;//26.759;// + adjustment;
   /* else if( (distance >= 1.67) && (distance <1.70) )
      return 0.56;//26.759;// + adjustment;
    else if( (distance >= 1.70) && (distance <1.73) )
      return 0.56;//26.759;// + adjustment;
    else if( (distance >= 1.73) && (distance <1.76) )
      return 0.56;//26.759;// + adjustment;
    else if( (distance >= 1.76) && (distance <1.79) )
      return 0.56;//26.759;// + adjustment;
    else if( (distance >= 1.79) && (distance <1.82) )
      return 0.56;//26.759;// + adjustment;
    else if( (distance >= 1.82) && (distance <1.85) )
      return 0.56;//26.759;// + adjustment;*/
    else
      return Constants.Shooter.k_SubWoofer;//0.247;
  }
  public double GetAdjustedAbsAmpEncoder()
  {
    return m_ampAbsoluteEncoder.getPosition();
  }

  @Override
  public void periodic() { 
    
    //auto targeting be calulate reference
    
    if((m_Range != Constants.Vision.NoTarget) && !m_Override)
    { 
        currentSPivotPosition = CalculateReference(m_Range);
       
        //Pivot Range check
        if(currentSPivotPosition < Constants.Shooter.k_SubWoofer)
          currentSPivotPosition = Constants.Shooter.k_SubWoofer;
        else if(currentSPivotPosition > Constants.Shooter.k_WingShot)
          currentSPivotPosition = Constants.Shooter.k_WingShot;
    }   

    //m_ampPidController.setReference(0.7, CANSparkMax.ControlType.kPosition);
     if( (ampArmState == ampArmStates.kScoringPosition) || (ampArmState == ampArmStates.kIntakePositionUp))
        m_ampPidController.setReference(currentAmpArmPosition, CANSparkMax.ControlType.kPosition);
    else if(ampArmState == ampArmStates.kStowed)
    {
        if(GetAdjustedAbsAmpEncoder() > Constants.Shooter.k_AmpArmHalfWayPointAbs)
          ShutDownAmpArm();
        else
          m_ampPidController.setReference(currentAmpArmPosition, CANSparkMax.ControlType.kPosition);
    }

    m_PidController.setReference(currentSPivotPosition, CANSparkMax.ControlType.kPosition);

    if(m_shooterRunning)
    {  
      m_TopPidController.setReference(m_TopShooterMotorSpeed, CANSparkFlex.ControlType.kVelocity);
      m_BottomPidController.setReference(m_BottomShooterMotorSpeed, CANSparkFlex.ControlType.kVelocity);
    }

    SmartDashboard.putNumber("Target Range", m_Range);
    SmartDashboard.putNumber("Pivot Position", m_pAbsoluteEncoder.getPosition()); //mEncoder.getPosition());
    SmartDashboard.putNumber("Amp Arm Encoder", GetAdjustedAbsAmpEncoder()); //mEncoder.getPosition());
    
    String x = "nada";
    if(ampArmState == ampArmStates.kScoringPosition)
       x = "Scoring Position";
    else if (ampArmState == ampArmStates.kIntakePositionUp)
      x = "Intake Position";
    else if (ampArmState == ampArmStates.kStowed)
      x = "Stowed Position";
    SmartDashboard.putString("AmpArmSTate", x);

    SmartDashboard.putNumber("Shooter Sensors Distance", distanceSensor.getRange());
    SmartDashboard.putNumber("Motor Speed", m_TopShooterMotorSpeed);
    SmartDashboard.putNumber("Top Speed", m_topMotorEncoder.getVelocity());
    SmartDashboard.putNumber("Bottom Speed", m_bottomMotorEncoder.getVelocity());

    SmartDashboard.putNumber("Top %", (m_topMotorEncoder.getVelocity() / m_TopShooterMotorSpeed ));
    SmartDashboard.putNumber("Bottom %", (m_bottomMotorEncoder.getVelocity() / m_BottomShooterMotorSpeed ));

    }
  }
