package frc.robot;

import edu.wpi.first.wpilibj.SolenoidBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import frc.robot.Constants.HARDWARE;
import frc.robot.Constants.DRIVER;
import frc.robot.Constants.PRESSURE_SENSOR;
import frc.robot.lib.drivers.PressureSensor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.DrivetrainFollowTrajectory;
import frc.robot.commands.CISVisionDriveToTarget;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import java.util.List;
// import edu.wpi.first.wpilibj.util.Units;

/**
* The RobotContainer class contains the subsystems, button/joystick bindings
* and logging.
*/
public class RobotContainer {

    // Hardware
    private final Joystick mDriverJoystickThrottle;
    private final JoystickButton mDriverJoystickThrottleButton;
    private final Joystick mDriverJoystickTurn;
    private final JoystickButton mDriverJoystickTurnButton;
    private final JoystickButton mDriverButtonBoard_2;
    private final JoystickButton mDriverButtonBoard_3;
    private final PressureSensor mPressureSensor;
    private final PowerDistributionPanel mPDP;
    
    // Subsystems
    // private Drivetrain mDrivetrain = Drivetrain.Create();
    public Drivetrain mDrivetrain = Drivetrain.Create();
    
    // Autonomous chooser
    private SendableChooser<Command> mAutoChooser = new SendableChooser<>();

    // Auto trajectories - the drivetrain object needs to be created before building trajectories
    private final Trajectory mAuto1Trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(1, 0),
                new Translation2d(2, 1)
        ),
        new Pose2d(3, 2, new Rotation2d( Math.toRadians(0.0) ) ),
        mDrivetrain.mTrajectoryConfig );
    private final Trajectory mAuto2Trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(1, 0),
                new Translation2d(2, -1)
        ),
        new Pose2d(3, -2, new Rotation2d( Math.toRadians(0.0) ) ),
        mDrivetrain.mTrajectoryConfig );


    // exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     new Pose2d(0, 0, new Rotation2d( Units.degreesToRadians(-45.0) )),
    //         List.of(
    //             new Translation2d(Units.inchesToMeters(54-48), Units.inchesToMeters(36-78)),
    //             new Translation2d(Units.inchesToMeters(54-48), Units.inchesToMeters(36-108)),
    //             new Translation2d(Units.inchesToMeters(80-48), Units.inchesToMeters(36-144)),
    //             new Translation2d(Units.inchesToMeters(115-48), Units.inchesToMeters(36-167))
    //         ),
    //     new Pose2d(Units.inchesToMeters(145-48), Units.inchesToMeters(36-167), new Rotation2d( Units.degreesToRadians(0.0) )),
    //     mDrivetrain.mTrajectoryConfig);



    //-----------------------------------------------------------------------------------------------------------------
    /*                                                PUBLIC METHODS                                                 */
    //-----------------------------------------------------------------------------------------------------------------


    /**
    * This method will get the autonomous command to run from the autonomous
    * chooser.
    #
    * @return Command The autonomous command to run
    */     
    public Command GetAutonomousCommand () {
        return mAutoChooser.getSelected();
    }


    //-----------------------------------------------------------------------------------------------------------------
    /*                                                PRIVATE METHODS                                                */
    //-----------------------------------------------------------------------------------------------------------------



    /**
    * This method will configure the joysticks and buttons. This means commands
    * and their behaviours will be assigned to the driver/operators controls.
    */
    private void ConfigureButtonBindings () {
        mDriverJoystickThrottleButton.whenPressed( new InstantCommand( () -> mDrivetrain.SetHighGear( !mDrivetrain.IsHighGear() ), mDrivetrain ) );
        // mDriverButtonBoard_2.whileHeld( new DriveToTarget( mDrivetrain, 3.5 ) );
        mDriverButtonBoard_2.whenPressed( new CISVisionDriveToTarget( mDrivetrain ) );        
        mDriverJoystickTurnButton.whenPressed( new InstantCommand( () -> mDrivetrain.SetReversedDirection( !mDrivetrain.IsReversedDirection() ), mDrivetrain ) );
        // mDriverButtonBoard_3.whileHeld( new TurnToTarget( mDrivetrain, mDriverJoystickThrottle ) );
    }

    /**
    * This method will intialize the RobotContainer class by setting the local
    * state variables, configuring the buttons and joysticks, setting the
    * default subsystem commands, setting up the autonomous chooser, and
    * clearing faults in the PCM and PDP.
    */
    private void Initialize () {
        ConfigureButtonBindings();
        mDrivetrain.setDefaultCommand( new TeleopDrive( mDrivetrain, mDriverJoystickThrottle, mDriverJoystickTurn ) );
        mAutoChooser.setDefaultOption( "Auto 1", new DrivetrainFollowTrajectory( mDrivetrain, mAuto1Trajectory ) );
        mAutoChooser.addOption( "Auto 2", new DrivetrainFollowTrajectory( mDrivetrain, mAuto2Trajectory ) );
        SmartDashboard.putData( "Auto Chooser", mAutoChooser );
        mPDP.clearStickyFaults();
        SolenoidBase.clearAllPCMStickyFaults( HARDWARE.PCM_ID );        
    }


    //-----------------------------------------------------------------------------------------------------------------
    /*                                        CLASS CONSTRUCTOR AND OVERRIDES                                        */
    //-----------------------------------------------------------------------------------------------------------------


    /**
    * This is the robot container class consructor. It is setup for injecting
    * the dependencies in order to allow for mocking those dependencies during
    * unit-testing.
    *
    * @param driverJoystickThrottle Joystick Driver joystick throttle
    * @param driverJoystickThrottleButton JoystickButton Driver joystick throttle button
    * @param driverJoystickTurn Joystick Driver joystick turn
    * @param driverJoystickTurnButton JoystickButton Driver joystick turn button
    * @param driverButtonBoard_2 JoystickButton Driver button board 2
    * @param driverButtonBoard_3 JoystickButton Driver button board 3
    * @param pressureSensor PressureSensor Analog pressure sensor
    * @param powerDistributionPanel PowerDistributionPanel power distribution panel
    */
    public RobotContainer ( Joystick driverJoystickThrottle, JoystickButton driverJoystickThrottleButton, Joystick driverJoystickTurn, JoystickButton driverJoystickTurnButton, JoystickButton driverButtonBoard_2, JoystickButton driverButtonBoard_3, PressureSensor pressureSensor, PowerDistributionPanel powerDistributionPanel ) {
        mDriverJoystickThrottle = driverJoystickThrottle;
        mDriverJoystickThrottleButton = driverJoystickThrottleButton;
        mDriverJoystickTurn = driverJoystickTurn;
        mDriverJoystickTurnButton = driverJoystickTurnButton;
        mDriverButtonBoard_2 = driverButtonBoard_2;
        mDriverButtonBoard_3 = driverButtonBoard_3;
        mPressureSensor = pressureSensor;
        mPDP = powerDistributionPanel;
        Initialize();
    }

    /**
    * This is methods calls the robot container consructor and creates the
    * robot container object.
    * @see {@link frc.robot.lib.drivers.Photoeye}
    * @see {@link frc.robot.lib.drivers.PressureSensor}
    */   
    public static RobotContainer Create () {
        Joystick driverJoystickThrottle = new Joystick( DRIVER.JOYSTICK_THROTTLE );
        JoystickButton driverJoystickThrottleButton = new JoystickButton( driverJoystickThrottle, 1 );
        Joystick driverJoystickTurn = new Joystick( DRIVER.JOYSTICK_TURN );
        JoystickButton driverJoystickTurnButton = new JoystickButton( driverJoystickTurn, 1 );
        final Joystick driverButtonBoard = new Joystick( DRIVER.DRIVER_BUTTON_BOARD );
        JoystickButton driverButtonBoard_2 = new JoystickButton( driverButtonBoard, 2 );
        JoystickButton driverButtonBoard_3 = new JoystickButton( driverButtonBoard, 3 );
        PressureSensor pressureSensor = new PressureSensor( PRESSURE_SENSOR.ANALOG_CHANNEL, PRESSURE_SENSOR.VOLTS_AT_ZERO_PRESSURE, PRESSURE_SENSOR.PSI_PER_VOLT );
        PowerDistributionPanel powerDistributionPanel = new PowerDistributionPanel( HARDWARE.PDP_ID );

        return new RobotContainer( driverJoystickThrottle, driverJoystickThrottleButton, driverJoystickTurn, driverJoystickTurnButton, driverButtonBoard_2, driverButtonBoard_3, pressureSensor, powerDistributionPanel );
    }

}
