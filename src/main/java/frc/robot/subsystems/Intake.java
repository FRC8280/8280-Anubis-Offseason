package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private TimeOfFlight distanceSensor;
  //private boolean emergencyEject = false;
  //configuration values for pivot motor 
  private static final double k_pivotMotorP = 0.042;//0.032; 
  private static final double k_pivotMotorI = 0.0;
  private static final double k_pivotMotorD = 0.001;
  private static final double k_pivotMaxOutput = 1;
  private static final double k_pivotMinOutput = -1;
  private static final double kIz = 0; 
  private static final double kFF = 0; 
  private double currentPivotPosition = Constants.Intake.k_pivotAngleStow;
  public double maxRPM, maxVel, minVel, maxAcc, allowedErr;
  
  private final SparkAbsoluteEncoder m_AbsolutePivotEncoder; 

  //public final LEDs m_leds = LEDs.getInstance();
  
  /*-------------------------------- Private instance variables ---------------------------------*/
  private static Intake mInstance;
  private PeriodicIO m_periodicIO;

  
  public static Intake getInstance() {
    if (mInstance == null) {
      mInstance = new Intake();
    }
    return mInstance;
  } 

  private CANSparkMax mIntakeMotor;
  private CANSparkMax mPivotMotor;
  private SparkPIDController m_pidController;
  private RelativeEncoder m_encoder;

  public boolean m_IntakeDeployed;
  public IntakeState m_IntakeState = IntakeState.NONE;
  public boolean m_TransferOverride = false;

  public Intake() {

    super("Intake");
    mIntakeMotor = new CANSparkMax(Constants.Intake.kIntakeMotorId, MotorType.kBrushless);
    mIntakeMotor.restoreFactoryDefaults();
    mIntakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    mIntakeMotor.burnFlash();

    mPivotMotor = new CANSparkMax(Constants.Intake.kPivotMotorId, MotorType.kBrushless);
    mPivotMotor.restoreFactoryDefaults();
    mPivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    mPivotMotor.setSmartCurrentLimit(30);

    //mPivotMotor.setSoftLimit(SoftLimitDirection.kReverse, -55);
    //mPivotMotor.setSoftLimit(SoftLimitDirection.kForward, 0);
    //mPivotMotor.enableSoftLimit(SoftLimitDirection.kForward,true );
    //mPivotMotor.enableftLimit(SoftLimitDirection.kReverse,true );
    
    m_AbsolutePivotEncoder = mPivotMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    
    m_pidController = mPivotMotor.getPIDController();
    m_encoder = mPivotMotor.getEncoder();
    m_encoder.setPosition(0);//m_AbsolutePivotEncoder.getPosition()*-98);

        // set PID coefficients
    m_pidController.setP(k_pivotMotorP);
    m_pidController.setI(k_pivotMotorI);
    m_pidController.setD(k_pivotMotorD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(k_pivotMinOutput, k_pivotMaxOutput);
    mPivotMotor.burnFlash();

    m_periodicIO = new PeriodicIO();
    m_IntakeDeployed = false;

    distanceSensor = new TimeOfFlight(0);
    m_IntakeState = IntakeState.NONE;

  }

  private static class PeriodicIO {
    // Input: Desired state
    PivotTarget pivot_target = PivotTarget.STOW;
    
    // Output: Motor set values
    //double intake_pivot_voltage = 0.0;
   // double intake_speed = 0.0;
  }

  public enum PivotTarget {
    NONE,
    GROUND,
    SOURCE,
    AMP,
    STOW
  }

  public enum IntakeState {
    NONE,
    INTAKE,
    INTAKE_LITE,
    EJECT,
    EJECT_LITE,
    EJECT_NUDGE,
    PULSE,
    FEED_SHOOTER,
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
    checkAutoTasks();  
    
    //Intake position PID
    m_pidController.setReference(currentPivotPosition, CANSparkMax.ControlType.kPosition);
    mIntakeMotor.set(intakeStateToSpeed(m_IntakeState));  //set the intake speed. 
    outputTelemetry();
  }

   private void checkAutoTasks() {
    // If the intake is set to GROUND, and the intake has a note, and the pivot is
    // close to it's target
    // Stop the intake and go to the SOURCE position

    if (m_periodicIO.pivot_target == PivotTarget.GROUND && getIntakeHasNote()){//) && isPivotAtTarget()) 
      m_periodicIO.pivot_target = PivotTarget.STOW;
      m_IntakeState = IntakeState.NONE;//IntakeState.INTAKE_LITE
      System.out.printf("******Auto note detection turning intake off******");
      currentPivotPosition = Constants.Intake.k_pivotAngleStow;    
      //m_leds.setColor(Color.kGreen);
    }
  }

  public double intakeStateToSpeed(IntakeState state) {
    switch (state) {
      case INTAKE:
        return Constants.Intake.k_intakeSpeed;
      case INTAKE_LITE:
        return Constants.Intake.k_intakeSpeed*0.75;
      case EJECT:
        return Constants.Intake.k_ejectSpeed;
      case EJECT_LITE:
        return Constants.Intake.k_ejectSpeed*.65;
      case EJECT_NUDGE:
        return Constants.Intake.k_ejectSpeedNudge;
      case PULSE:
        // Use the timer to pulse the intake on for a 1/16 second,
        // then off for a 15/16 second
        if (Timer.getFPGATimestamp() % 1.0 < (1.0 / 45.0)) {
          return Constants.Intake.k_intakeSpeed;
        }
        return 0.0;
      case FEED_SHOOTER:
        return Constants.Intake.k_feedShooterSpeed;
      default:
        // "Safe" default
        return 0.0;
    }
  }

  public void ejectNote(){
    currentPivotPosition = Constants.Intake.k_pivotAngleAmp;
    if(m_encoder.getPosition() < -20)
        ReverseIntake();
    else
        StopIntakeMotor();
  }

  public void ActivateOverride()
  {
    m_TransferOverride = true;
  }
    public void DeActivateOverride()
  {
    m_TransferOverride = false;
  }

  public boolean NoteReadyToReturn()
  {
     if(!getIntakeHasNote() && isPivotStowed()&& m_TransferOverride) 
      return true;
    else  
      return false;
  }
  public boolean NoteReadyToTransfer()
  {
    if(getIntakeHasNote() && isPivotStowed() && !m_TransferOverride) 
      return true;
    else  
      return false;
  }

  //@Override
  public void stop() {

    mPivotMotor.stopMotor();
    mIntakeMotor.stopMotor();
  }

 // @Override
  public void outputTelemetry() {
    SmartDashboard.putNumber("Intake Distance", distanceSensor.getRange());
    SmartDashboard.putNumber("Intake position",m_encoder.getPosition()) ;/* 
    putNumber("Pivot Current", mPivotMotor.getOutputCurrent());
    putNumber("Pivot Speed",m_encoder.getVelocity());
    putNumber("Pivot movement status",Math.abs(m_encoder.getPosition() - Constants.Intake.k_pivotAngleStow));*/
  }

  //@Override
  public void reset() {
  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public boolean intakeEmpty()
  {
    return !getIntakeHasNote();
  }
  public boolean getIntakeHasNote() {
    
    if(distanceSensor.getRange() < Constants.Intake.k_NoteInIntakeDistance)
      return true;
    else 
      return false; 
  }
  
  // Pivot helper functions
  public void goToGround() {
    
    m_TransferOverride = false;
    System.out.printf("******Going to ground turning intake on******");
    if(getIntakeHasNote())  //Don't do anything if there's already a note. 
      return;
      
    currentPivotPosition = Constants.Intake.k_pivotAngleGround;
    m_periodicIO.pivot_target = PivotTarget.GROUND;
    m_IntakeState = IntakeState.INTAKE;
    //m_leds.setColor(Color.kYellow);
  }

  public void goToGroundForAmp() {
    
    m_TransferOverride = true;
    System.out.printf("******Going to ground turning intake on******");
    if(getIntakeHasNote())  //Don't do anything if there's already a note. 
      return;
      
    currentPivotPosition = Constants.Intake.k_pivotAngleGround;
    m_periodicIO.pivot_target = PivotTarget.GROUND;
    m_IntakeState = IntakeState.INTAKE;
    //m_leds.setColor(Color.kYellow);
  }


  public void GotoAmpScore() {
    currentPivotPosition = -28;//Constants.Intake.k_pivotAngleSource;
    m_periodicIO.pivot_target = PivotTarget.AMP;
  }
   public void goToGroundEject() {
    
    System.out.printf("******Going to ground turning intake reverse******");
    //if(getIntakeHasNote())  //Don't do anything if there's already a note. 
     // return;
      
    currentPivotPosition = Constants.Intake.k_pivotAngleGround;
    m_periodicIO.pivot_target = PivotTarget.GROUND;
    //m_IntakeState = IntakeState.EJECT;
    //m_leds.setColor(Color.kYellow);
  }


  public void goToStow() {
    currentPivotPosition = Constants.Intake.k_pivotAngleStow;
    m_periodicIO.pivot_target = PivotTarget.STOW;

    if(m_IntakeState!= IntakeState.EJECT){
      m_IntakeState = IntakeState.NONE;
      System.out.printf("******Stow command turning intake off ******");
    }
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/
 
  
  public void StopAllMotors ()
  {
    m_IntakeState = IntakeState.NONE;
    mPivotMotor.set(0);
    System.out.printf("******Stop motor turning intake off******");
  }

  public void StopIntakeMotor()
  {
    m_IntakeState = IntakeState.NONE;
    System.out.printf("******stop intake command turning intake off******\n");
  }
  public void ReverseIntakeNudge(){
    m_IntakeState = IntakeState.EJECT_NUDGE;
  }
  public void ReverseIntake()
  {
    m_IntakeState = IntakeState.EJECT;
    //mIntakeMotor.set(Constants.Intake.k_ejectSpeed);
  }

   public void ReverseIntakeAmp()
  {
    m_IntakeState = IntakeState.EJECT_LITE;
    //mIntakeMotor.set(Constants.Intake.k_ejectSpeed);
  }
  
  
  public void ActivateIntake()
  {
    m_IntakeState = IntakeState.INTAKE;
  }

   public void ActivateIntakeLite()
  {
    m_IntakeState = IntakeState.INTAKE_LITE;
  }

  public boolean isPivotStowed() {
      return Math.abs(m_encoder.getPosition() - Constants.Intake.k_pivotAngleStow) < 5;
  }

}