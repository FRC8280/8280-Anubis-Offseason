// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import frc.robot.Constants;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

//import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends SubsystemBase {

  private CANSparkMax m_leftClimber;
  private CANSparkMax m_rightClimber;

  private RelativeEncoder m_leftEncoder;
  private RelativeEncoder m_RightEncoder;

  private static final double k_sClimberMotorP =  0.5;
  private static final double k_sClimberMotorI = 0.0;
  private static final double k_sClimberMotorD = 0.001;
  private static final double k_sClimberMaxOutput = 0.9;
  private static final double k_sClimberMinOutput = -0.9;
  private static final double kIz = 0; 
  private static final double kFF = 0; 
  private SparkPIDController m_LeftPidController;
  private SparkPIDController m_RightPidController;

  private double currentClimberPosition = Constants.Climber.cruitseHeight;
  private boolean manualClimber = false;
  private DigitalInput m_leftHallEffectSensor;
  private DigitalInput m_rightHallEffectSensor;

  private boolean climberGoingUp = false;
  private static Climber mInstance;
  public static Climber getInstance() {
    if (mInstance == null) {
      mInstance = new Climber();
    }
    return mInstance;
  } 

  public Climber() {

    m_leftClimber =
      new CANSparkMax(Constants.Climber.leftClimberID, CANSparkLowLevel.MotorType.kBrushless);
    m_rightClimber =
      new CANSparkMax(Constants.Climber.rightClimberID, CANSparkLowLevel.MotorType.kBrushless);
    
      m_leftHallEffectSensor = new DigitalInput(0);
      m_rightHallEffectSensor = new DigitalInput(1);
      
      m_leftEncoder = m_leftClimber.getEncoder();
      m_RightEncoder = m_rightClimber.getEncoder();

      m_leftClimber.enableSoftLimit(SoftLimitDirection.kForward, true);
      m_rightClimber.enableSoftLimit(SoftLimitDirection.kForward, true);
      m_leftClimber.enableSoftLimit(SoftLimitDirection.kReverse, false);
      m_rightClimber.enableSoftLimit(SoftLimitDirection.kReverse, false);
     
      m_leftClimber.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.Climber.maxClimberHeight);
      m_leftClimber.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.Climber.minClimberHeight);
      m_rightClimber.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.Climber.maxClimberHeight);
      m_rightClimber.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.Climber.minClimberHeight); 

      m_LeftPidController = m_leftClimber.getPIDController();
      m_LeftPidController.setP(k_sClimberMotorP);
      m_LeftPidController.setI(k_sClimberMotorI);
      m_LeftPidController.setD(k_sClimberMotorD);
      m_LeftPidController.setIZone(kIz);
      m_LeftPidController.setFF(kFF);
      m_LeftPidController.setOutputRange(k_sClimberMinOutput, k_sClimberMaxOutput);

      m_RightPidController = m_rightClimber.getPIDController();
      m_RightPidController.setP(k_sClimberMotorP);
      m_RightPidController.setI(k_sClimberMotorI);
      m_RightPidController.setD(k_sClimberMotorD);
      m_RightPidController.setIZone(kIz);
      m_RightPidController.setFF(kFF);
      m_RightPidController.setOutputRange(k_sClimberMinOutput, k_sClimberMaxOutput);

      m_RightEncoder.setPosition(0);
      m_leftEncoder.setPosition(0);

      m_rightClimber.burnFlash();
      m_leftClimber.burnFlash();
  }

  public void IncreasePower()
  {
    manualClimber = true;

    climberGoingUp = true;
    if(m_RightEncoder.getPosition() < Constants.Climber.maxClimberHeight)
      m_rightClimber.set(Constants.Climber.climberPower);

    if(m_leftEncoder.getPosition() < Constants.Climber.maxClimberHeight)
      m_leftClimber.set(Constants.Climber.climberPower);
  }

  public void DecreasePower()
  {

    manualClimber = true;
    climberGoingUp = false;

    //if(m_RightEncoder.getPosition() > Constants.Climber.minClimberHeight)
     // m_rightClimber.set(-Constants.Climber.climberPower/4);

   // if(m_leftEncoder.getPosition() > Constants.Climber.minClimberHeight)
      //m_leftClimber.set(-Constants.Climber.climberPower/4);

   if(m_rightHallEffectSensor.get())
      m_rightClimber.set(-Constants.Climber.climberPower);
   else
      m_RightEncoder.setPosition(0);  
    
  if(m_leftHallEffectSensor.get())
    m_leftClimber.set(-Constants.Climber.climberPower);
  else 
    m_leftEncoder.setPosition(0); 

  }

  public void EnableMotorController()
  {
    manualClimber = false;
  }

  public void NoPower()
  {
    climberGoingUp = false;
      m_rightClimber.set(0);
      m_leftClimber.set(0);
  }

  public void SetClimberAmpScorePosition()
  {
    manualClimber = false;
    currentClimberPosition = Constants.Climber.ampScoreHeight;

  }

   public void SetClimberCruise()
  {
    manualClimber = false;
    currentClimberPosition = Constants.Climber.cruitseHeight;

  }

  public boolean ClimbersAtAmpPosition()
  {
    if(currentClimberPosition >= Constants.Climber.ampScoreHeight)
      return true;
    else
      return false;
  }
  @Override
  public void periodic() {

    
    /*if(!m_rightHallEffectSensor.get())
    {  
       if(!climberGoingUp)
       {
        m_rightClimber.set(0);
        m_leftClimber.set(0);
       }
       m_RightEncoder.setPosition(0);
       m_leftEncoder.setPosition(0);
    }*/
   
   
   /* if(!m_leftHallEffectSensor.get())
    {
      if(!climberGoingUp)
        m_leftClimber.set(0);
      m_leftEncoder.setPosition(0); 
    } */

   /* if(manualClimber == false)
    {
      m_LeftPidController.setReference(currentClimberPosition, CANSparkMax.ControlType.kPosition);
      m_RightPidController.setReference(currentClimberPosition, CANSparkMax.ControlType.kPosition);
    }*/
    
    SmartDashboard.putNumber("Climber Pos Left", m_leftEncoder.getPosition());
    SmartDashboard.putNumber("Climber Pos Right", m_RightEncoder.getPosition());
    SmartDashboard.putBoolean("Left Magnet", m_leftHallEffectSensor.get());
    SmartDashboard.putBoolean("Right Magnet", m_rightHallEffectSensor.get());
  }
}
