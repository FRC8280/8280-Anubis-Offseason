// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

//import edu.wpi.first.net.PortForwarder;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
//import org.photonvision.targeting.PhotonTrackedTarget;
//import org.photonvision.targeting.TargetCorner;
//import org.photonvision.targeting.PhotonTrackedTarget;
//import org.photonvision.utils.PacketUtils;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private final boolean UseLimelight = false;

  /*Photon Vision Initialization */
  PhotonCamera camera = new PhotonCamera("Arducam");
  public double TargetYaw = Constants.VisionData.NoTarget;

  public static final double targetWidth =
          Units.inchesToMeters(41.30) - Units.inchesToMeters(6.70); // meters

  public static final double targetHeight =
          Units.inchesToMeters(98.19) - Units.inchesToMeters(81.19); // meters

  public static final double kFarTgtXPos = Units.feetToMeters(54);
  public static final double kFarTgtYPos =
          Units.feetToMeters(27 / 2) - Units.inchesToMeters(43.75) - Units.inchesToMeters(48.0 / 2.0);
  public static final double kFarTgtZPos =
          (Units.inchesToMeters(98.19) - targetHeight) / 2 + targetHeight;

  public static final Pose3d kFarTargetPose =
          new Pose3d(
                  new Translation3d(kFarTgtXPos, kFarTgtYPos, kFarTgtZPos),
                  new Rotation3d(0.0, 0.0, Units.degreesToRadians(180)));
  /* End Photon Vision Initialization */

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

  @Override
  public void robotInit() {
    PortForwarder.add(5800, "photonvision.local", 5800);

    m_robotContainer = new RobotContainer();
    m_robotContainer.drivetrain.getDaqThread().setThreadPriority(99);
    m_robotContainer.FixArm();
  }
  @Override
  public void robotPeriodic() {

    //see this link - https://www.chiefdelphi.com/t/apriltag-odometry-with-photonvision-help/451360/24
    
    CommandScheduler.getInstance().run();
    
    var result = camera.getLatestResult();
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
            TargetYaw = target.getYaw();

            double range =
                PhotonUtils.calculateDistanceToTargetMeters(
                        Constants.VisionData.CAMERA_HEIGHT_METERS,
                        Constants.VisionData.TARGET_HEIGHT_METERS,
                        Constants.VisionData.CAMERA_PITCH_RADIANS,
                        Units.degreesToRadians(target.getPitch()));
            m_robotContainer.SetShooterRange(range);
          }
          else
          { 
             TargetYaw = Constants.VisionData.NoTarget;
             m_robotContainer.SetShooterRange(Constants.VisionData.NoTarget);
          }
      } 
      else
        TargetYaw = Constants.VisionData.NoTarget;
      
    
    /**
     * This example of adding Limelight is very simple and may not be sufficient for on-field use.
     * Users typically need to provide a standard deviation that scales with the distance to target
     * and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible, though exact implementation
     * of how to use vision should be tuned per-robot and to the team's specification.
     */
    if (UseLimelight) {
      var lastResult = LimelightHelpers.getLatestResults("limelight").targetingResults;

      Pose2d llPose = lastResult.getBotPose2d_wpiBlue();

      if (lastResult.valid) {
        m_robotContainer.drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp());
      }
    }

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
