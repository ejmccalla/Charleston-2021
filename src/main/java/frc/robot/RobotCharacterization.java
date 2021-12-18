package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;

public class RobotCharacterization extends TimedRobot {

    private RobotContainer mRobotContainer;
    private boolean m_runCal = false;
    private boolean m_configCal = false;
    private boolean m_reset = false;    
    String data = "";
    int counter = 0;
    double startTime = 0;
    double priorAutospeed = 0;
    double[] numberArray = new double[10];
    ArrayList<Double> entries = new ArrayList<Double>();
    NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
    NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
    NetworkTableEntry rotateEntry = NetworkTableInstance.getDefault().getEntry("/robot/rotate");

    public RobotCharacterization() {
        super(.010);
        LiveWindow.disableAllTelemetry();
        SmartDashboard.putBoolean("IMU Config Cal", false); // Specific for the ADIS16470
        SmartDashboard.putBoolean("IMU Reset", false);      // Specific for the ADIS16470
        SmartDashboard.putBoolean("IMU Run Cal", false);    // Specific for the ADIS16470
    }

    @Override
    public void robotInit () {
        mRobotContainer = RobotContainer.Create();
        NetworkTableInstance.getDefault().setUpdateRate(0.010);
    }

    @Override
    public void disabledInit () {
        double elapsedTime = Timer.getFPGATimestamp() - startTime;
        System.out.println("Robot disabled");
        mRobotContainer.mDrivetrain.SetOpenLoopOutput(0.0, 0.0, false);
        data = entries.toString();
        data = data.substring(1, data.length() - 1) + ", ";
        telemetryEntry.setString(data);
        entries.clear();
        System.out.println("Robot disabled");
        System.out.println("Collected : " + counter + " in " + elapsedTime + " seconds");
        data = "";
    }
  
    @Override
    public void autonomousInit () {
        startTime = Timer.getFPGATimestamp();
        counter = 0;
    }
 
    @Override
    public void teleopInit () {}

    @Override
    public void robotPeriodic () {}

    @Override
    public void disabledPeriodic () {
        m_configCal = SmartDashboard.getBoolean("IMU Config Cal", false);
        m_reset = SmartDashboard.getBoolean("IMU Reset", false);
        m_runCal = SmartDashboard.getBoolean("IMU Run Cal", false);
        if (m_configCal) {
            mRobotContainer.mDrivetrain.ConfigureIMU();
            m_configCal = SmartDashboard.putBoolean("IMU Config Cal", false);
        }
        if (m_reset) {
            mRobotContainer.mDrivetrain.ResetIMU();
            m_reset = SmartDashboard.putBoolean("IMU Reset", false);
        }
        if (m_runCal) {
            mRobotContainer.mDrivetrain.CalibrateIMU();
            m_runCal = SmartDashboard.putBoolean("IMU Run Cal", false);
        }
    }

    @Override
    public void autonomousPeriodic () {
        double now = Timer.getFPGATimestamp();
        double leftPosition = mRobotContainer.mDrivetrain.GetLeftPositionRotations();
        double leftRate = mRobotContainer.mDrivetrain.GetLeftVelocityRotationsPerSecond();
        double rightPosition = mRobotContainer.mDrivetrain.GetRightPositionRotations();
        double rightRate = mRobotContainer.mDrivetrain.GetRightVelocityRotationsPerSecond();
        double gyroAngleRadians = mRobotContainer.mDrivetrain.GetGyroAngleInRadians();
        double battery = RobotController.getBatteryVoltage();
        double motorVolts = battery * Math.abs(priorAutospeed);
        double leftMotorVolts = motorVolts;
        double rightMotorVolts = motorVolts;
        double autospeed = autoSpeedEntry.getDouble(0);
        priorAutospeed = autospeed;
        mRobotContainer.mDrivetrain.mDifferentialDrive.tankDrive(
            (rotateEntry.getBoolean(false) ? -1 : 1) * autospeed, autospeed,
            false);
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
        for (double num : numberArray) {
          entries.add(num);
        }
        counter++;
    }

    @Override
    public void teleopPeriodic () {}

    @Override
    public void testInit () {}    

    @Override
    public void testPeriodic () {}

}
