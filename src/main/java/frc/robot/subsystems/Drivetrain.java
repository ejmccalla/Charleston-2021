package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.HARDWARE;
import frc.robot.Constants.DRIVETRAIN;
import frc.robot.lib.drivers.TalonSRX;
import frc.robot.lib.drivers.VictorSPX;
// import frc.robot.lib.controllers.LimelightVision;
// import frc.robot.lib.controllers.LimelightVision.SharedState;
import com.analog.adis16470.frc.ADIS16470_IMU;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;

/**
* The Drivetrain class is designed to use the command-based programming model
* and extends the SubsystemBase class.
* @see {@link edu.wpi.first.wpilibj2.command.SubsystemBase}
*/
public class Drivetrain extends SubsystemBase {

    // Constants
    private final double ENCODER_MULTIPLIER = ( 1.0 / DRIVETRAIN.ENCODER_GEARING ) * ( 1.0 / DRIVETRAIN.ENCODER_PPR );

    // Hardware
    private final WPI_TalonSRX mLeftMaster;
    private final WPI_VictorSPX mLeftFollower_1;
    private final WPI_VictorSPX mLeftFollower_2;
    private final WPI_TalonSRX mRightMaster;
    private final WPI_VictorSPX mRightFollower_1;
    private final WPI_VictorSPX mRightFollower_2;
    private final DoubleSolenoid mShifter;
    private final ADIS16470_IMU mIMU;

    // Drive conrol (both open and closed loop)
    public DifferentialDrive mDifferentialDrive;
    private SpeedControllerGroup mLeftSpeedControllerGroup;
    private SpeedControllerGroup mRightSpeedControllerGroup;

    // Track the robots whereabouts
    private final DifferentialDriveOdometry mOdometry;

    // Limelight Controller for closed-loop control
    // private Notifier mLimelightVisionControllerThread;  // Threading interface only
    // private LimelightVision mLimelightVisionController;
    // private SharedState mLimelightVisionControllerSharedState;

    // State variables
    private boolean mIsReversedDirection;
    private boolean mIsHighGear;
    private boolean mIsBrakeMode;

    // Ramsete controller and PID controlers for closed-loop control during trajectory following
    private final RamseteController mRamseteConroller = new RamseteController();
    private final PIDController mLeftPIDController = new PIDController( 10.0, 0, 0 );
    private final PIDController mRightPIDController = new PIDController( 10.0, 0, 0 );

    // Trajectory generation and following
    private final SimpleMotorFeedforward mFeedforward = new SimpleMotorFeedforward( DRIVETRAIN.kS, DRIVETRAIN.kV, DRIVETRAIN.kA );
    private final DifferentialDriveKinematics mDriveKinematics = new DifferentialDriveKinematics( DRIVETRAIN.kTRACK_WIDTH_METERS );
    private final DifferentialDriveVoltageConstraint mDriveConstraints = new DifferentialDriveVoltageConstraint( mFeedforward, mDriveKinematics, DRIVETRAIN.MAX_VOLTAGE );
    public final TrajectoryConfig mTrajectoryConfig = new TrajectoryConfig( DRIVETRAIN.MAX_SPEED_METERS_PER_SECOND, DRIVETRAIN.MAX_ACCELERATIOIN_METERS_PER_SECOND_SECOND ).setKinematics( mDriveKinematics ).addConstraint( mDriveConstraints );
    private Trajectory mTrajectoryToFollow;
    private DifferentialDriveWheelSpeeds mPreviousWheelSpeeds;

    // Coprocessor
    private NetworkTableEntry mWaypointsXEntry = NetworkTableInstance.getDefault().getEntry("/Coprocessor/Waypoints_x_in");
    private NetworkTableEntry mWaypointsYEntry = NetworkTableInstance.getDefault().getEntry("/Coprocessor/Waypoints_y_in");
    private NetworkTableEntry mPathValidEntry = NetworkTableInstance.getDefault().getEntry("/Coprocessor/Path_Valid");
    private double[] mDefaultWaypoints = new double[0];
    public double[] mWaypointsX_in = mDefaultWaypoints;
    public double[] mWaypointsY_in = mDefaultWaypoints;
    public boolean mPathValid = false;


    //-----------------------------------------------------------------------------------------------------------------
    /*                                                PUBLIC METHODS                                                 */
    //-----------------------------------------------------------------------------------------------------------------


    /**
    * This method will set determine the direction of the drive motors.
    *
    * @param wantsReversed boolean True if the driver wants the direction reversed, false otherwise
    */ 
    public void SetReversedDirection ( boolean wantsReversedDirection ) {
        if ( wantsReversedDirection != mIsReversedDirection ) {
            mIsReversedDirection = wantsReversedDirection;
            mLeftMaster.setInverted( !mIsReversedDirection );
            mLeftFollower_1.setInverted( !mIsReversedDirection );
            mLeftFollower_2.setInverted( !mIsReversedDirection );
            mRightMaster.setInverted( !mIsReversedDirection) ;
            mRightFollower_1.setInverted( !mIsReversedDirection );
            mRightFollower_2.setInverted( !mIsReversedDirection );
        }
    }

    /**
    * This method will set the gearing of the transmission.
    *
    * @param wantsHighGear boolean True if the driver wants high gear, false for low gear
    */ 
    public void SetHighGear ( boolean wantsHighGear ) {
        if ( wantsHighGear && !mIsHighGear ) {
            mIsHighGear = wantsHighGear;
            mShifter.set( DoubleSolenoid.Value.kForward );

        } else if ( !wantsHighGear && mIsHighGear ) {
            mIsHighGear = wantsHighGear;
            mShifter.set( DoubleSolenoid.Value.kReverse );

        }
    }

    /**
    * This method will set the neutral mode of the motor controllers.
    *
    * @param wantsBrake boolean True if the driver wants brake, false for coast
    */ 
    public void SetBrakeMode ( boolean wantsBrake ) {
        if ( wantsBrake && !mIsBrakeMode ) {
            mIsBrakeMode = wantsBrake;
            mLeftMaster.setNeutralMode( NeutralMode.Brake );
            mLeftFollower_1.setNeutralMode( NeutralMode.Brake );
            mLeftFollower_2.setNeutralMode( NeutralMode.Brake );
            mRightMaster.setNeutralMode( NeutralMode.Brake );
            mRightFollower_1.setNeutralMode( NeutralMode.Brake );
            mRightFollower_2.setNeutralMode( NeutralMode.Brake );

        } else if ( !wantsBrake && mIsBrakeMode ) {
            mIsBrakeMode = wantsBrake;
            mLeftMaster.setNeutralMode( NeutralMode.Coast );
            mLeftFollower_1.setNeutralMode( NeutralMode.Coast );
            mLeftFollower_2.setNeutralMode( NeutralMode.Coast );
            mRightMaster.setNeutralMode( NeutralMode.Coast );
            mRightFollower_1.setNeutralMode( NeutralMode.Coast );
            mRightFollower_2.setNeutralMode( NeutralMode.Coast );

        }
    }

    /**
    * This method will return the state of the reversed mode 
    *
    * @return boolean True if the direction is reversed, false otherwise
    */
    public boolean IsReversedDirection () {
        return mIsReversedDirection;
    }

    /**
    * This method will return the state of the shifting transmission 
    *
    * @return boolean True if the transmission is in high-gear, false for low-gear
    */
    public boolean IsHighGear () {
        return mIsHighGear;
    }

    /**
    * This method will return the state of motor controllers neutral mode 
    *
    * @return boolean True for brake, false for coast
    */
    public boolean IsBrakeMode () {
        return mIsBrakeMode;
    }

    // /**
    // * This method will set the Limelight vision controller to perform the turn-to-target command.
    // */
    // public void StartTurnToTarget () {
    //     mLimelightVisionController.TurnToTarget();
    // }

    // /**
    // * This method will set the Limelight vision controller to perform the drive-to-target command.
    // */
    // public void StartDriveToTarget ( double targetDistance ) {
    //     mLimelightVisionController.DriveToTarget( targetDistance );
    // }

    // /**
    // * This method will set the Limelight vision controller to idle.
    // */
    // public void EndLimelightCommand () {
    //     mLimelightVisionController.Idle();
    // }

    // /**
    // * This method will set the Limelight vision controller's output to the curvature drives turning input and set the
    // * quickturn flag in order to get the robot to turn towards the target (taking into account the reversed direction
    // * state).
    // */
    // public void SetLimelightVisionControllerOutput ( boolean quickTurn ) {
    //     if ( mIsReversedDirection ) {
    //         mDifferentialDrive.arcadeDrive( mLimelightVisionControllerSharedState.outputDistance,
    //                                         -mLimelightVisionControllerSharedState.outputTurn );
    //     } else {
    //         mDifferentialDrive.arcadeDrive( -mLimelightVisionControllerSharedState.outputDistance,
    //                                         mLimelightVisionControllerSharedState.outputTurn );

    //     }
    // }

    // /**
    // * This method will set the Limelight vision controller's output to the curvature drives turning input and set and
    // * use input as the throttle.
    // */
    // public void SetLimelightVisionControllerOutput ( double throttle, boolean quickTurn ) {
    //     if ( mIsReversedDirection ) {
    //         //mDifferentialDrive.curvatureDrive( throttle, -mLimelightVisionControllerSharedState.outputTurn, quickTurn );
    //         mDifferentialDrive.arcadeDrive( throttle, -mLimelightVisionControllerSharedState.outputTurn );
    //     } else {
    //         //mDifferentialDrive.curvatureDrive( throttle, mLimelightVisionControllerSharedState.outputTurn, quickTurn );
    //         mDifferentialDrive.arcadeDrive( throttle, mLimelightVisionControllerSharedState.outputTurn );
    //     }
    // }

    /**
    * This method will set the output based on the driver inputs and the reversed direction state.
    */
    public void SetOpenLoopOutput ( double throttle, double turn, boolean quickTurn ) {
        if ( mIsReversedDirection ) {
            mDifferentialDrive.curvatureDrive( throttle, -turn, quickTurn );
        } else {
            mDifferentialDrive.curvatureDrive( throttle, turn, quickTurn );
        }
    }

    /**
    * This method will set the output based on a motor voltage.
    */
    public void SetOpenLoopOutput( double leftVolts, double rightVolts ) {
        mLeftSpeedControllerGroup.setVoltage( leftVolts );
        mRightSpeedControllerGroup.setVoltage( -rightVolts );
        mDifferentialDrive.feed();
      }


    /**
    * This method will calibrate the IMU.
    */
    public void CalibrateIMU() {
        mIMU.calibrate();
    }

    /**
    * This method will set the IMU calibration time to 8 seconds.
    */
    public void ConfigureIMU() {
        mIMU.configCalTime(ADIS16470_IMU.ADIS16470CalibrationTime._8s);
    }

    /**
    * This method will reset the IMU.
    */
    public void ResetIMU() {
        mIMU.reset();
    }





    
    /**
    * This method will initialize the drivetrain for trajectory following.
    */ 
    public void InitializeTrajectoryFollowing (Trajectory trajectory) {
        mTrajectoryToFollow = trajectory;
        var initialState = mTrajectoryToFollow.sample( 0 );
        mPreviousWheelSpeeds = mDriveKinematics.toWheelSpeeds( new ChassisSpeeds( initialState.velocityMetersPerSecond, 0, initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond ) );
        mLeftPIDController.reset();
        mRightPIDController.reset();
        resetOdometry( mTrajectoryToFollow.getInitialPose() );
    }

    /**
    * This method will updated the drivetrain trajectory following outputs.
    */ 
    public void UpdateTrajectoryFollowing (double currentTime, double dt) {
        
        var currentState = mTrajectoryToFollow.sample( currentTime );
        var targetWheelSpeeds = mDriveKinematics.toWheelSpeeds( mRamseteConroller.calculate( getPose(), currentState ) );
        var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
        var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

        double leftFeedforward = mFeedforward.calculate( leftSpeedSetpoint, (leftSpeedSetpoint - mPreviousWheelSpeeds.leftMetersPerSecond) / dt);
        double rightFeedforward = mFeedforward.calculate( rightSpeedSetpoint, (rightSpeedSetpoint - mPreviousWheelSpeeds.rightMetersPerSecond) / dt);
        double leftOutput = leftFeedforward + mLeftPIDController.calculate( getWheelSpeeds().leftMetersPerSecond, leftSpeedSetpoint);
        double rightOutput = rightFeedforward + mRightPIDController.calculate( getWheelSpeeds().rightMetersPerSecond, rightSpeedSetpoint);

        SetOpenLoopOutput( leftOutput, rightOutput );
        mPreviousWheelSpeeds = targetWheelSpeeds;
        
    }

    /**
    * This method will return the left-side sensor position in rotations.
    *
    * @return double
    */
    public double GetLeftPositionRotations() {
        return mLeftMaster.getSelectedSensorPosition() * ENCODER_MULTIPLIER;
    }

    /**
    * This method will return the right-side sensor position in rotations.
    *
    * @return double
    */
    public double GetRightPositionRotations() {
        return mRightMaster.getSelectedSensorPosition() * ENCODER_MULTIPLIER;
    }

    /**
    * This method will return the left-side sensor velocity in rotations per second.
    *
    * @return double
    */
    public double GetLeftVelocityRotationsPerSecond() {
        return mLeftMaster.getSelectedSensorVelocity() * ENCODER_MULTIPLIER * 10; // native units per 100ms
    }

    /**
    * This method will return the right-side sensor velocity in rotations per second.
    *
    * @return double
    */
    public double GetRightVelocityRotationsPerSecond() {
        return mRightMaster.getSelectedSensorVelocity() * ENCODER_MULTIPLIER * 10; // native units per 100ms
    }


    //-----------------------------------------------------------------------------------------------------------------
    /*                                                PRIVATE METHODS                                                */
    //-----------------------------------------------------------------------------------------------------------------


    /**
    * This method will return the left-side sensor position in meters.
    *
    * @return double
    */
    private double GetLeftPositionMeters() {
        return mLeftMaster.getSelectedSensorPosition() * ENCODER_MULTIPLIER * DRIVETRAIN.WHEEL_DIAMETER_METERS * Math.PI;
    }

    /**
    * This method will return the right-side sensor position in meters.
    *
    * @return double
    */
    private double GetRightPositionMeters() {
        return -mRightMaster.getSelectedSensorPosition() * ENCODER_MULTIPLIER * DRIVETRAIN.WHEEL_DIAMETER_METERS * Math.PI;
    }

    /**
    * This method will return the left-side sensor velocity in meters per second.
    *
    * @return double
    */
    private double GetLeftVelocityMetersPerSecond() {
        return mLeftMaster.getSelectedSensorVelocity() * ENCODER_MULTIPLIER * 10 * DRIVETRAIN.WHEEL_DIAMETER_METERS * Math.PI; // native units per 100ms
    }

    /**
    * This method will return the right-side sensor velocity in meters per second.
    *
    * @return double
    */
    private double GetRightVelocityMetersPerSecond() {
        return -mRightMaster.getSelectedSensorVelocity() * ENCODER_MULTIPLIER * 10 * DRIVETRAIN.WHEEL_DIAMETER_METERS * Math.PI; // native units per 100ms
    }

    public double GetGyroAngleInRadians() {
        return Math.toRadians( mIMU.getAngle() );
    }

    private DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds( GetLeftVelocityMetersPerSecond(), GetRightVelocityMetersPerSecond() );
    }

    private Pose2d getPose() {
       return mOdometry.getPoseMeters();
    }

    private double getHeading() {
        return mIMU.getRotation2d().getDegrees();
    }

    private double getTurnRate() {
        return -mIMU.getRate();
    }

    private void zeroHeading() {
        mIMU.reset();
    }

    private double getAverageEncoderDistance() {
        double leftPositionInMeters = mLeftMaster.getSelectedSensorPosition() * ENCODER_MULTIPLIER * DRIVETRAIN.WHEEL_DIAMETER_METERS * Math.PI;
        double rightPositionInMeters = mLeftMaster.getSelectedSensorPosition() * ENCODER_MULTIPLIER * DRIVETRAIN.WHEEL_DIAMETER_METERS * Math.PI;
        return ( leftPositionInMeters + rightPositionInMeters ) / 2.0;
     }

     private void resetOdometry( Pose2d pose ) {
        resetEncoders();
        mOdometry.resetPosition( pose, mIMU.getRotation2d() );
    }

    private void resetEncoders() {
        mLeftMaster.setSelectedSensorPosition( 0.0 );
        mRightMaster.setSelectedSensorPosition( 0.0 );
    }


    /**
    * This method will initialize the Drivetrain subsystem.
    */
    private void Initialize () {
        ResetMotorControllers();
        ResetSensors();
        ResetState();
    }

    /**
    * This method will reset senor and vision controller information.
    */ 
    private void ResetSensors () {
        mLeftMaster.setSelectedSensorPosition( 0.0 );
        mRightMaster.setSelectedSensorPosition( 0.0 );
        mIMU.reset();
    }

    /**
    * This method will reset senor and vision controller information.
    */ 
    private void ResetMotorControllers () {
        TalonSRX.ConfigureTalonSRX( mLeftMaster );
        TalonSRX.CTREMagEncoderConfig( mLeftMaster );
        // mLeftMaster.setSensorPhase();
        // mLeftMaster.configSelectedFeedbackCoefficient(coefficient)
        VictorSPX.ConfigureVictorSPX( mLeftFollower_1 );
        VictorSPX.ConfigureVictorSPX( mLeftFollower_2 );
        TalonSRX.ConfigureTalonSRX( mRightMaster );
        TalonSRX.CTREMagEncoderConfig( mRightMaster );
        // mRightMaster.setSensorPhase();
        // mRightMaster.configSelectedFeedbackCoefficient(coefficient)
        VictorSPX.ConfigureVictorSPX( mRightFollower_1 );
        VictorSPX.ConfigureVictorSPX( mRightFollower_2 );
    }

    /**
    * This method will reset all of the internal states.
    */ 
    private void ResetState () {
        mIsHighGear = false;
        SetHighGear( true );
        mIsBrakeMode = false;
        SetBrakeMode( true );
        mIsReversedDirection = true;
        SetReversedDirection( false );
    }


    //-----------------------------------------------------------------------------------------------------------------
    /*                                        CLASS CONSTRUCTOR AND OVERRIDES                                        */
    //-----------------------------------------------------------------------------------------------------------------


    /**
    * The constructor for the Drivetrain class.
    *
    * @param leftMaster WPI_TalonSRX A Talon SRX motor controller object
    * @param leftFollower_1 WPI_VictorSPX A Talon SRX motor controller object
    * @param leftFollower_2 WPI_VictorSPX A Talon SRX motor controller object
    * @param rightMaster WPI_TalonSRX A Talon SRX motor controller object
    * @param rightFollower_1 WPI_VictorSPX A Talon SRX motor controller object
    * @param rightFollower_2 WPI_VictorSPX A Talon SRX motor controller object
    * @param differentialDrive DifferentialDrive A differential drive object
    * @param limelightVision LimelightVision A limelight vision controller object
    * @param shifter DoubleSolenoid A double solenoid object for shifting the transmission
    * @param imu ADIS16470_IMU An Analog Devices 16470 IMU object
    */  
    // public Drivetrain ( WPI_TalonSRX leftMaster, WPI_VictorSPX leftFollower_1, WPI_VictorSPX leftFollower_2,
    //                     WPI_TalonSRX rightMaster, WPI_VictorSPX rightFollower_1, WPI_VictorSPX rightFollower_2,
    //                     LimelightVision limelightVision, DoubleSolenoid shifter, ADIS16470_IMU imu ) {
    public Drivetrain ( WPI_TalonSRX leftMaster, WPI_VictorSPX leftFollower_1, WPI_VictorSPX leftFollower_2,WPI_TalonSRX rightMaster, WPI_VictorSPX rightFollower_1, WPI_VictorSPX rightFollower_2, DoubleSolenoid shifter, ADIS16470_IMU imu ) {
        mLeftMaster = leftMaster;
        mLeftFollower_1 = leftFollower_1; 
        mLeftFollower_2 = leftFollower_2;
        mRightMaster = rightMaster; 
        mRightFollower_1 = rightFollower_1;
        mRightFollower_2 = rightFollower_2;
        mShifter = shifter;
        mIMU = imu;
        mLeftSpeedControllerGroup = new SpeedControllerGroup(leftMaster, leftFollower_1, leftFollower_2);
        mRightSpeedControllerGroup = new SpeedControllerGroup(rightMaster, rightFollower_1, rightFollower_2);
        mDifferentialDrive = new DifferentialDrive( mLeftSpeedControllerGroup, mRightSpeedControllerGroup );
        mOdometry = new DifferentialDriveOdometry( mIMU.getRotation2d() );
        // mLimelightVisionController = limelightVision;
        // mLimelightVisionControllerSharedState = mLimelightVisionController.GetSharedState();
        // if ( DRIVETRAIN.VISION_THREADED ) {
        //     mLimelightVisionControllerThread = new Notifier ( mLimelightVisionController.mThread ); 
        //     mLimelightVisionControllerThread.startPeriodic( 0.01 );
        // }
        Initialize();
    }

    /**
    * This mehtod will create a new Drivetrain object.  The purpose of doing the constructor this way is to allow for
    * unit testing.
    */  
    public static Drivetrain Create () {
        WPI_TalonSRX leftMaster = new WPI_TalonSRX( DRIVETRAIN.LEFT_MASTER_ID );
        WPI_VictorSPX leftFollower_1 = new WPI_VictorSPX( DRIVETRAIN.LEFT_FOLLOWER_1_ID );
        WPI_VictorSPX leftFollower_2 = new WPI_VictorSPX( DRIVETRAIN.LEFT_FOLLOWER_2_ID );
        WPI_TalonSRX rightMaster = new WPI_TalonSRX( DRIVETRAIN.RIGHT_MASTER_ID );
        WPI_VictorSPX rightFollower_1 = new WPI_VictorSPX( DRIVETRAIN.RIGHT_FOLLOWER_1_ID );
        WPI_VictorSPX rightFollower_2 = new WPI_VictorSPX( DRIVETRAIN.RIGHT_FOLLOWER_2_ID );
        DoubleSolenoid shifter = new DoubleSolenoid( HARDWARE.PCM_ID, DRIVETRAIN.HIGH_GEAR_SOLENOID_ID, 
                                                     DRIVETRAIN.LOW_GEAR_SOLENOID_ID );
        // LimelightVision limelightVision = LimelightVision.Create( DRIVETRAIN.VISION_SEARCH_TIMEOUT_S,
        //                                                           DRIVETRAIN.VISION_SEEK_TIMEOUT_S,
        //                                                           DRIVETRAIN.VISION_SEEK_RETRY_LIMIT,
        //                                                           DRIVETRAIN.VISION_TURN_PID_P,
        //                                                           DRIVETRAIN.VISION_TURN_PID_I,
        //                                                           DRIVETRAIN.VISION_TURN_PID_D,
        //                                                           DRIVETRAIN.VISION_TURN_PID_F,
        //                                                           DRIVETRAIN.VISION_DISTANCE_PID_P,
        //                                                           DRIVETRAIN.VISION_DISTANCE_PID_I,
        //                                                           DRIVETRAIN.VISION_DISTANCE_PID_D,
        //                                                           DRIVETRAIN.VISION_DISTANCE_PID_F,
        //                                                           DRIVETRAIN.VISION_ON_TARGET_TURN_THRESHOLD_DEG,
        //                                                           DRIVETRAIN.VISION_ON_TARGET_DISTANCE_THRESHOLD_FT,
        //                                                           DRIVETRAIN.VISION_DISTANCE_ESTIMATOR,
        //                                                           DRIVETRAIN.VISION_TARGET_WIDTH_FT,
        //                                                           DRIVETRAIN.VISION_FOCAL_LENGTH_FT,
        //                                                           DRIVETRAIN.VISION_FLOOR_TO_TARGET_FT,
        //                                                           DRIVETRAIN.VISION_FLOOR_TO_LIMELIGHT_FT,
        //                                                           DRIVETRAIN.VISION_LIMELIGHT_MOUNT_ANGLE_DEG );
        ADIS16470_IMU imu = new ADIS16470_IMU(); 
        // return new Drivetrain( leftMaster, leftFollower_1, leftFollower_2, rightMaster, rightFollower_1,
        //                        rightFollower_2, limelightVision, shifter, imu );
        return new Drivetrain( leftMaster, leftFollower_1, leftFollower_2, rightMaster, rightFollower_1, rightFollower_2, shifter, imu );
    }

    /**
    * The subsystem periodic method gets called by the CommandScheduler at the
    * very beginning of each robot loop. All sensor readings should be updated
    in this method.
    * @see {@link edu.wpi.first.wpilibj2.command.CommandScheduler#run}
    */ 
    @Override
    public void periodic () {
        // if ( !DRIVETRAIN.VISION_THREADED ) {
        //     mLimelightVisionController.RunUpdate();
        // }

        // The odometry class requires the gyro angle to be counter-clockwise increasing
        mOdometry.update( Rotation2d.fromDegrees( mIMU.getAngle()), GetLeftPositionMeters(), GetRightPositionMeters() );

        // Get the coprocessor path
        mWaypointsX_in = mWaypointsXEntry.getDoubleArray( mDefaultWaypoints );
        mWaypointsX_in = mWaypointsYEntry.getDoubleArray( mDefaultWaypoints );
        mPathValid = mPathValidEntry.getBoolean( false );
    }

}
