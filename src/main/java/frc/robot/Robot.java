// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.photonvision.targeting.PhotonPipelineResult;
//import edu.wpi.first.net.PortForwarder;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.PhotonUtils;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private Vision vision;

  private final Field2d m_field = new Field2d();

public PhotonTrackedTarget FindSpeakerTarget(List<PhotonTrackedTarget> targetList){
    Optional<Alliance> ally = DriverStation.getAlliance();
    
    int targetFiducial;
    if (ally.get() == Alliance.Blue) 
      targetFiducial = 7;
    else
      targetFiducial = 4;

    for (PhotonTrackedTarget selectedTarget : targetList )
    {
        if(selectedTarget.getFiducialId() == targetFiducial)
          return selectedTarget;
    }
    return null;
  }
  //????
  @Override
  public void robotInit() {
    PortForwarder.add(5800, "photonvision.local", 5800);

    m_robotContainer = new RobotContainer();
    m_robotContainer.drivetrain.getDaqThread().setThreadPriority(99);
    vision = new Vision();
    //m_robotContainer.FixArm();
    // Do this in either robot or subsystem init
    SmartDashboard.putData("Field", m_field);
  }
  @Override
  public void robotPeriodic() {
    
    m_robotContainer.PeriodicCall();
    
    double range = Constants.Vision.NoTarget;
    CommandScheduler.getInstance().run();
    
    /*Original vision targeting - modified to use vision class */
    //var result = camera.getLatestResult();
    PhotonPipelineResult result = vision.getLatestResult();
    PhotonTrackedTarget target = null;

    //Calculate shooter pivot and autoaim
    if(result.hasTargets())
      {
        List<PhotonTrackedTarget> targetList = result.getTargets();
        target = FindSpeakerTarget(targetList);
      } 
      else
        target = null;
  
      if(target != null)
      {
          //Todo move to the telemetry class
          SmartDashboard.putNumber("Target yaw", target.getYaw());
          SmartDashboard.putNumber("Target ID", target.getFiducialId());

          //Provide target data to swerve
          if( (target.getFiducialId() == 7) || (target.getFiducialId() == 4) )
          {  
            /*todo - this is for the swerve drive, maybe set in the container instead */
            m_robotContainer.m_TargetYaw = target.getYaw();

            range =
                PhotonUtils.calculateDistanceToTargetMeters(
                        Constants.Vision.CAMERA_HEIGHT_METERS,
                        Constants.Vision.TARGET_HEIGHT_METERS,
                        Constants.Vision.CAMERA_PITCH_RADIANS,
                        Units.degreesToRadians(target.getPitch()));
            m_robotContainer.SetShooterRange(range);
          }
          else
          { 
             m_robotContainer.m_TargetYaw = Constants.Vision.NoTarget;
             m_robotContainer.SetShooterRange(Constants.Vision.NoTarget);
          }
      } 
      else
        m_robotContainer.m_TargetYaw = Constants.Vision.NoTarget;
      
      /* Calculate Pose from camera data */
      // Correct pose estimate with vision measurements
      var visionEst = vision.getEstimatedGlobalPose();
      visionEst.ifPresent(
              est -> {
                  var estPose = est.estimatedPose.toPose2d();
                  // Change our trust in the measurement based on the tags we can see
                  var estStdDevs = vision.getEstimationStdDevs(estPose);

                  m_robotContainer.drivetrain.addVisionMeasurement(
                          est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
              });
      m_robotContainer.logPosition();

      m_field.setRobotPose(m_robotContainer.drivetrain.getState().Pose);
        //m_odometry.getPoseMeters());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }
/*slsd */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
