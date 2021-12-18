package frc.robot;

// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import frc.robot.RobotContainer.MatchState_t;
// import org.apache.logging.log4j.LogManager;
// import org.apache.logging.log4j.Logger;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.CAN;
// import java.time.LocalDateTime;


/**
* The Robot class contains the RobotContainer and data logger objects and
* simply calls the command-based scheduler to run the robot commands from
* the given robot states. Also, the data logger is used to log data during
* while the robot is in the various states.
*/
public class Robot extends TimedRobot {

    // private static final Logger mLogger = LogManager.getLogger( Robot.class );
    private static final RobotContainer mRobotContainer = RobotContainer.Create();
    private Command mAutonomousCommand;
    // private CAN mCanbusLogger = new CAN(0);  // DeviceID = 0
    // private LocalDateTime mLocalDateTime;
    // private byte[] mCanbusLoggerData = {0, 0, 0, 0, 0, 0, 0, 0};

    /**
    * This method is called when the robot is first powered up. This method
    * gets called only once.
    */ 
    @Override
    public void robotInit () {
        //mLogger.info("<=========== ROBOT INIT ===========>");
        mRobotContainer.SetMatchState( MatchState_t.robotInit );
        //mRobotContainer.LogRobotDataHeader( mLogger );
        mRobotContainer.UpdateSmartDashboard();
        LiveWindow.disableAllTelemetry();   // Force users to enable the telemetry that matters
    }

    /**
    * This method is called when the robot is disabled. This method gets called
    * only once.
    */     
    @Override
    public void disabledInit () {
        //mLogger.info( "<=========== DISABLED INIT ===========>" );
        mRobotContainer.SetMatchState( MatchState_t.disabledInit );
        // mCanbusLogger.writePacket( mCanbusLoggerData, 0 );  // API class: 0, API index: 0
    }

    /**
    * This method is called when the robot is entering autonomous. This method
    * gets called only once.
    */     
    @Override
    public void autonomousInit () {
        //mLogger.info( "<=========== AUTONOMOUS INIT ===========>" );
        mRobotContainer.SetMatchState( MatchState_t.autonomousInit );
        mRobotContainer.mDrivetrain.CalibrateIMU();
        mAutonomousCommand = mRobotContainer.GetAutonomousCommand();
        if (mAutonomousCommand != null) {
            mAutonomousCommand.schedule();
            // mLogger.info( "Starting autonomous command {}", mAutonomousCommand.getName() );
            }
    }

    /**
    * This method is called periodically whenever the robot is in autonomous mode.
    */     
    @Override
    public void autonomousPeriodic () {
        if ( mRobotContainer.GetMatchState() != MatchState_t.autonomousPeriodic ) {
            // mLogger.info("<=========== AUTONOMOUS PERIODIC ===========>");
            mRobotContainer.SetMatchState( MatchState_t.autonomousPeriodic );
        }
        CommandScheduler.getInstance().run();
    }


    /**
    * This method is called when the robot is entering teleop. This method gets
    * called only once.
    */     
    @Override
    public void teleopInit () {
        // mLogger.info( "<=========== TELEOP INIT ===========>" );
        mRobotContainer.SetMatchState( MatchState_t.teleopInit );
        if (mAutonomousCommand != null) {
            mAutonomousCommand.cancel();
        }
        CommandScheduler.getInstance().run();
       
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
    * to this code. The disabled periodic will also run prior to this method.
    */     
    @Override
    public void robotPeriodic () {
        //mRobotContainer.LogRobotDataToRoboRio( mLogger );
        mRobotContainer.UpdateSmartDashboard();        
    }

    /**
    * This method is called periodically whenever the robot is disabled.
    */     
    @Override
    public void disabledPeriodic () {
        if ( mRobotContainer.GetMatchState() != MatchState_t.disabledPeriodic ) {
            // mLogger.info("<=========== DISABLED PERIODIC ===========>");
            mRobotContainer.SetMatchState( MatchState_t.disabledPeriodic );
        }
    }

    /**
    * This method is called periodically whenever the robot is in teleop mode.
    */   
    @Override
    public void teleopPeriodic () {
        if ( mRobotContainer.GetMatchState() != MatchState_t.teleopPeriodic ) {
            // mLogger.info("<=========== TELEOP PERIODIC ===========>");
            mRobotContainer.SetMatchState( MatchState_t.teleopPeriodic );
        }
        CommandScheduler.getInstance().run();
        // mCanbusLogger.writeRTRFrame(2, 16);                 // API Class: 1, API index: 0
    }


    /**
    * This method is called when the robot is entering test mode. This method
    * gets called only once.
    */       
    @Override
    public void testInit () {
        // mLogger.info( "<=========== TEST INIT ===========>" );
        CommandScheduler.getInstance().enable();            // Re-enable the command scheduler
        
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
