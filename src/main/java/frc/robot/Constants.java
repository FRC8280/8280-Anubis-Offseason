package frc.robot;
//import com.revrobotics.CANSparkMax.IdleMode;
//import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.lib.PIDGains;


import edu.wpi.first.math.Matrix;

public final class Constants {

    public static class Vision {

        public static double CAMERA_HEIGHT_METERS = Units.inchesToMeters(11.5);
        public static double TARGET_HEIGHT_METERS = Units.inchesToMeters(51.875);
        public static double CAMERA_PITCH_RADIANS = Units.degreesToRadians(45);

        public static int NoTarget = -5000;

        public static final String kCameraName = "Arducam";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(Units.inchesToMeters(13), Units.inchesToMeters(2.5), Units.inchesToMeters(11.5)),
                new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-45), Units.degreesToRadians(0)));  //Todo: Add my camera position. 

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFields.kDefaultField.loadAprilTagLayoutField();

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

    public static final double stickDeadband = 0.1;//0.1;
    public static double gyroOffset = 0;

    public static final Constraints kThetaControllerConstraints = null;

    public static class Intake {
        // Motors
        public static final int kIntakeMotorId = 12; //todo: change to ours
        public static final int kPivotMotorId = 13; //todo: change to ours
      
        // Absolute encoder offset
        public static final double k_pivotEncoderOffset = 0.166842; // Straight up, sketchy to reset to "up"
    
        
        // Pivot set point encoder values =
        public static final double k_pivotAngleGround = -43;//-55;
        public static final double k_pivotAngleSource = /*-0.30;*/ -25.047;
        public static final double k_pivotAngleAmp = k_pivotAngleSource;
        public static final double k_pivotAngleStow = 0;
        public static final double k_pivotAmp = -24.047;
        public static final double k_NoteInIntakeDistance = 260;

    
        // Intake speeds
        public static final double k_intakeSpeed = 0.9;
        public static final double k_ejectSpeed = -1;
        public static final double k_ejectSpeedNudge = -0.20;
        public static final double k_feedShooterSpeed = -0.5;

        /*New commit from REV */
        public static final double kArmGearRatio = 1.0 / (100); 
        public static final double kPositionFactor = kArmGearRatio * 2.0 * Math.PI; //multiply SM value by this number and get arm position in radians
        public static final double kVelocityFactor = kArmGearRatio * 2.0 * Math.PI / 60.0;
        public static final double kArmFreeSpeed = 5676.0 * kVelocityFactor;
        public static final ArmFeedforward kArmFeedforward = new ArmFeedforward(0.0, 0.4, 12.0/kArmFreeSpeed, 0.0);
        public static final PIDGains kArmPositionGains = new PIDGains(0.6, 0.0, 0.0);
        
        public static final TrapezoidProfile.Constraints kArmMotionConstraint = new TrapezoidProfile.Constraints(2.0, 2.0);
        public static final double TrapezoidProfileMaxVel = 2.0;
        public static final double TrapezoidProfileMaxAcc = 2.0;

        public static final double kHomePosition = 0.0;
        public static final double kScoringPosition = 3.78;
        public static final double kIntakePosition = 1.82;
        public static final double kFeederPosition = 3.564;
        public static final double kHighPosition = 3.594; //3.3746;

        public static final double kSoftLimitReverse = 0; //-140;
        public static final double kSoftLimitForward = 4.77;//4.6 or 143.878

        public static final double kArmZeroCosineOffset = - Math.PI / 6; //radians to add to converted arm position to get real-world arm position (starts at ~30deg angle)
      }
      

    public static final class OIConstants {
        public static final double kArmManualDeadband = 0.05;
        public static final double kArmManualScale = 0.5;
    }

    public static final class Climber{
        public static final int leftClimberID = 30;
        public static final int rightClimberID = 11;
        public static final double maxClimberHeight = 230;
        public static final double minClimberHeight = 5;

        public static final double ampScoreHeight = 191;
        public static final double cruitseHeight = 57;
        public static final double climberPower = 0.9;
    }

    public static final class Shooter {
        public static final double kAutoShotDuration = 0.75;
        public static final int kTopCanId = 16;
        public static final int kBottomCanId = 17;
        public static final int kIndexCanId = 14;
        public static final int kPivotShooterCanId = 15;
        public static final int kAmpArmId = 18;
    
        public static final int kCurrentLimit = 80;
        public static final int kIndexCurrentLimit = 35;
        public static final int kSPivotCurrentLimit = 40;
    
        public static final double kTopPower = 0.7;
        public static final double kBottomPower = 0.8;
        public static final double kIndexPower = 0.9;
        public static final double kIndexPowerSlow = 0.4;
        public static final double kInputPower = 0.3;

        public static final double k_ShooterRPM = 4000;
        public static final double k_ShooterMAXRPM = 5000;
        public static final double k_ShooterTimer = 500;

        public static final double k_PivotMargin = 0.468;//2;
        public static final double k_PivotZero = 0.468;//2;
        public static final double k_PivotMiddle = 0.468;//24.05;
        public static final double k_PivotNinety = 0.468;//34;
        public static final double k_SubWoofer = 0.203;//16;
        public static final double k_SubWooferAuton = 0.203;//13.1;
        public static final double k_WingShot = 0.68;//28.4;
        public static final double k_StageShot = 0.468;//20.5;
        public static final double k_FeedShot = 0.468;//20.5;
        public static final double k_ShooterPivotLoaded = 0.55;//0.468;
        public static final double k_ShooterPivotIncrement = 0.02;
        
        public static final double k_ArmIntakeValue = 0.25;
        public static final double kNoteDetectionDistance = 145;//110;
        public static final double kNoteDetectionDistanceAdjusted = 110;
        public static final double kNoteTooCloseShooters = 65;//80.0;
        public static final double kPreShotLimit = 0.17;
        
        public static final double k_RPMShooterTimeLimit = 2.5;
        //Tune this base distance value
        public static final double kBaseVisionDistance = 0.61;
        public static final double kAutoElevationConstant = 12.0741;
        public static final double kBaseShooterElevation = k_SubWoofer;

        //Amp shot data
        public static final double k_AmpShot = 0.36;// 0.32;
        public static final double k_AmpTopRPM =  1800;//900;
        public static final double k_AmpBottomRPM = 1200;//1200;;
        public static final double k_AmpArmScore = 1.25;  //Todo increase this value
        public static final double k_AmpArmScorAbs = 0.535;  //Todo increase this value
        public static final double k_AmpArmHalfWayPoint = 6;
        public static final double k_AmpArmHalfWayPointAbs = 0.83;// 0.723; //absolute encoder value
        public static final double k_AmpArmIntakePosition = 8;
        public static final double k_AmpArmIntakePositionAbs = 0.797; //absolute encoder value

        public static final double k_FeedTopRPM = 3000;
        public static final double k_FeedBottomRPM = 5000;

        //public static final double kArmStowed = [idk find the value later]
      }
}
