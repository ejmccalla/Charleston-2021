package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer.MatchState_t;
// import edu.wpi.first.wpilibj.DriverStation;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
// import edu.wpi.first.wpilibj.CAN;
// import java.time.LocalDateTime;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.ArrayList;

/**
* The Robot class contains the RobotContainer and data logger objects and
* simply calls the command-based scheduler to run the robot commands from
* the given robot states. Also, the data logger is used to log data during
* while the robot is in the various states.
*/
public class Robot extends TimedRobot {
    
    private static final Logger mLogger = LogManager.getLogger( Robot.class );
    
    private RobotContainer mRobotContainer;
    private Command mAutonomousCommand;
    // private CAN mCanbusLogger = new CAN(0);  // DeviceID = 0
    // private LocalDateTime mLocalDateTime;
    // private byte[] mCanbusLoggerData = {0, 0, 0, 0, 0, 0, 0, 0};

    double priorAutospeed = 0;
    double[] numberArray = new double[10];
    ArrayList<Double> entries = new ArrayList<Double>();
    NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
    NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
    NetworkTableEntry rotateEntry = NetworkTableInstance.getDefault().getEntry("/robot/rotate");


    /**
    * This method is called when the robot is first powered up. This method
    * gets called only once.
    */ 
    @Override
    public void robotInit () {
        mLogger.info("<=========== ROBOT INIT ===========>");
        mRobotContainer = RobotContainer.Create();
        mRobotContainer.SetMatchState( MatchState_t.robotInit );
        mRobotContainer.LogRobotDataHeader( mLogger );
        mRobotContainer.LogRobotDataToRoboRio( mLogger );
        mRobotContainer.UpdateSmartDashboard();


        // Set the update rate instead of using flush because of a ntcore bug
        // -> probably don't want to do this on a robot in competition
        NetworkTableInstance.getDefault().setUpdateRate(0.010);
    }

    /**
    * This method is called when the robot is disabled. This method gets called
    * only once.
    */     
    @Override
    public void disabledInit () {
        mLogger.info( "<=========== DISABLED INIT ===========>" );
        mRobotContainer.SetMatchState( MatchState_t.disabledInit );
        mRobotContainer.LogRobotDataToRoboRio( mLogger );
        mRobotContainer.UpdateSmartDashboard();
        // mCanbusLogger.writePacket( mCanbusLoggerData, 0 );  // API class: 0, API index: 0
    }

    /**
    * This method is called when the robot is entering autonomous. This method
    * gets called only once.
    */     
    @Override
    public void autonomousInit () {
        // mLogger.info( "<=========== AUTONOMOUS INIT ===========>" );
        // mRobotContainer.SetMatchState( MatchState_t.autonomousInit );
        // mAutonomousCommand = mRobotContainer.GetAutonomousCommand();
        // if (mAutonomousCommand != null) {
        //     mAutonomousCommand.schedule();
        //     mLogger.info( "Starting autonomous command {}",
        //         mAutonomousCommand.getName() );
        // }
        // CommandScheduler.getInstance().run();
        // mRobotContainer.LogRobotDataToRoboRio( mLogger );
        // mRobotContainer.UpdateSmartDashboard();         
    }

    /**
    * This method is called when the robot is entering teleop. This method gets
    * called only once.
    */     
    @Override
    public void teleopInit () {
        mLogger.error( "<=========== TELEOP INIT ===========>" );
        mRobotContainer.SetMatchState( MatchState_t.teleopInit );
        if (mAutonomousCommand != null) {
            mAutonomousCommand.cancel();
        }
        CommandScheduler.getInstance().run();
        mRobotContainer.LogRobotDataToRoboRio( mLogger );
        mRobotContainer.UpdateSmartDashboard();
       
        // mLocalDateTime = LocalDateTime.now();
        // mCanbusLoggerData[0] = (byte) ( mLocalDateTime.getYear() - 2000 );
        // mCanbusLoggerData[1] = (byte) mLocalDateTime.getMonthValue();
        // mCanbusLoggerData[2] = (byte) mLocalDateTime.getDayOfMonth();
        // mCanbusLoggerData[3] = (byte) mLocalDateTime.getHour();
        // mCanbusLoggerData[4] = (byte) mLocalDateTime.getMinute();
        // mCanbusLoggerData[5] = (byte) mLocalDateTime.getSecond();
        // mCanbusLogger.writePacket(mCanbusLoggerData, 1);  // API class: 0, API index: 1
    }

    /**
    * This method is called whenever the robot is in a periodic mode. This
    * means that both autonomous periodic and teleop periodic will run prior
    * to this code.  The disabled periodic will also run prior to this method.
    */     
    @Override
    public void robotPeriodic () {
    }

    /**
    * This method is called periodically whenever the robot is disabled.
    */     
    @Override
    public void disabledPeriodic () {
        if ( mRobotContainer.GetMatchState() != MatchState_t.disabledPeriodic ) {
            mLogger.info("<=========== DISABLED PERIODIC ===========>");
            mRobotContainer.SetMatchState( MatchState_t.disabledPeriodic );
            mRobotContainer.LogRobotDataToRoboRio( mLogger );
            mRobotContainer.UpdateSmartDashboard();
        }
    }

    /**
    * This method is called periodically whenever the robot is in autonomous mode.
    */     
    @Override
    public void autonomousPeriodic () {
        // if ( mRobotContainer.GetMatchState() != MatchState_t.autonomousPeriodic ) {
        //     mLogger.info("<=========== AUTONOMOUS PERIODIC ===========>");
        //     mRobotContainer.SetMatchState( MatchState_t.autonomousPeriodic );
        // }
        // CommandScheduler.getInstance().run();
        // mRobotContainer.LogRobotDataToRoboRio( mLogger );
        // mRobotContainer.UpdateSmartDashboard();
    
        double now = Timer.getFPGATimestamp();
        double leftPosition = mRobotContainer.mDrivetrain.GetLoggingData().mLeftEncoderPosition;
        double leftRate = mRobotContainer.mDrivetrain.GetLoggingData().mLeftEncoderVelocity;
        double rightPosition = mRobotContainer.mDrivetrain.GetLoggingData().mRightEncoderPosition;
        double rightRate = mRobotContainer.mDrivetrain.GetLoggingData().mRightEncoderVelocity;
        double gyroAngleRadians = Math.toRadians(mRobotContainer.mDrivetrain.GetLoggingData().mGyroAngle_deg);
        double battery = RobotController.getBatteryVoltage();
        double motorVolts = battery * Math.abs(priorAutospeed);
        double leftMotorVolts = motorVolts;
        double rightMotorVolts = motorVolts;
    
        // Retrieve the commanded speed from NetworkTables
        double autospeed = autoSpeedEntry.getDouble(0);
        priorAutospeed = autospeed;
    
        // command motors to do things
        mRobotContainer.mDrivetrain.SetOpenLoopOutput(autospeed, 0.0, false);
    
        numberArray[0] = now;
        numberArray[1] = battery;
        numberArray[2] = autospeed;
        numberArray[3] = leftMotorVolts;
        numberArray[4] = rightMotorVolts;
        numberArray[5] = leftPosition;
        numberArray[6] = rightPosition;
        numberArray[7] = leftRate;
        numberArray[8] = rightRate;
        numberArray[9] = gyroAngleRadians;
    
        // Add data to a string that is uploaded to NT
        for (double num : numberArray) {
          entries.add(num);
        }
        //counter++;

    
    
    
    }

    /**
    * This method is called periodically whenever the robot is in teleop mode.
    */   
    @Override
    public void teleopPeriodic () {
        if ( mRobotContainer.GetMatchState() != MatchState_t.teleopPeriodic ) {
            mLogger.info("<=========== TELEOP PERIODIC ===========>");
            mRobotContainer.SetMatchState( MatchState_t.teleopPeriodic );
        }
        CommandScheduler.getInstance().run();
        mRobotContainer.LogRobotDataToRoboRio( mLogger );
        mRobotContainer.UpdateSmartDashboard();
        // mCanbusLogger.writeRTRFrame(2, 16);                 // API Class: 1, API index: 0
    }


    /**
    * This method is called when the robot is entering test mode. This method
    * gets called only once.
    */       
    @Override
    public void testInit () {
        mLogger.info( "<=========== TEST INIT ===========>" );
        CommandScheduler.getInstance().enable();            // Re-enable the command scheduler
        //mRobotContainer.LogRobotDataToRoboRio( mLogger );
        mRobotContainer.UpdateSmartDashboard();
    }    

    /**
    * This method is called periodically whenever the robot is in test mode.
    */
    @Override
    public void testPeriodic () {
        CommandScheduler.getInstance().run();
        mRobotContainer.UpdateSmartDashboard();
    }

}
