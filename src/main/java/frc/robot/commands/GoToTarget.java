package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import java.util.List;
import java.util.ArrayList;
import java.util.Arrays;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.util.Units;
//import org.apache.logging.log4j.Logger;

public class GoToTarget extends CommandBase {

    private final Drivetrain m_drivetrain;
    private final Timer m_timer = new Timer();
    private final PIDController m_leftController = new PIDController(10.0, 0, 0);
    private final PIDController m_rightController = new PIDController(10.0, 0, 0);
    private Trajectory m_trajectory;
    private DifferentialDriveWheelSpeeds m_prevSpeeds;
    private double m_prevTime;
    //private Logger mLogger;

    public GoToTarget( Drivetrain drivetrain ) {
        m_drivetrain = drivetrain;
        //mLogger = logger;
        addRequirements(m_drivetrain);
        this.setName( "GoToTarget" );
    }

    @Override
    public void initialize() {
        double[] defaultValue = new double[0];
        double[] waypoints_x_in = NetworkTableInstance.getDefault().getEntry("/CoProcessor/Waypoints_x_in").getDoubleArray(defaultValue);
        double[] waypoints_y_in = NetworkTableInstance.getDefault().getEntry("/CoProcessor/Waypoints_y_in").getDoubleArray(defaultValue);
        
        // Need at least 4 waypoints...start, 2 interior, stop 
        if (waypoints_x_in.length < 4) {
            m_trajectory = new Trajectory(Arrays.asList(new Trajectory.State()));
            System.out.println("Not enough trajectory points");
        
        } else {
            System.out.println("Start building trajectory ");
            List<Translation2d> inner_pts = new ArrayList<Translation2d>();
            for (int i = 1 ; i != waypoints_x_in.length-1 ; i++) {
                inner_pts.add(new Translation2d( Units.inchesToMeters(waypoints_x_in[i] ), 
                                                 Units.inchesToMeters(waypoints_y_in[i] ) ) );
            }

            m_trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d( 0.0, 0.0, new Rotation2d(Units.degreesToRadians(-45.0)) ),
                inner_pts,
                new Pose2d(Units.inchesToMeters(waypoints_x_in[waypoints_x_in.length-1]),
                           Units.inchesToMeters(waypoints_y_in[waypoints_y_in.length-1]),
                           new Rotation2d(Units.degreesToRadians(0.0)) ),
                m_drivetrain.mTrajectoryConfig );

                System.out.println("Done building trajectory ");
                System.out.println("Trajectory time : " + m_trajectory.getTotalTimeSeconds());

        }

        m_prevTime = -1;
        var initialState = m_trajectory.sample( 0 );
        m_prevSpeeds = m_drivetrain.mDriveKinematics.toWheelSpeeds( new ChassisSpeeds( initialState.velocityMetersPerSecond, 0, initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond ) );
        m_timer.reset();
        m_timer.start();
        m_leftController.reset();
        m_rightController.reset();
        m_drivetrain.resetOdometry( m_trajectory.getInitialPose() );
        // mLogger.info( "Time,Traj_X,Traj_y,Traj_Theta,Odometry_X,Odometry_Y,Odometry_Theta,Measured_Left_Wheel_Speed,Target_Left_Wheel_Speed,Left_FF,Left_Output,Measured_Right_Wheel_Speed,Target_Right_Wheel_Speed,Right_FF,Right_Output" );
        // mLogger.info("{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
        //     m_timer.get(),
        //     initialState.poseMeters.getX(),
        //     initialState.poseMeters.getY(),
        //     initialState.poseMeters.getRotation().getDegrees(),
        //     m_drivetrain.getPose().getX(),
        //     m_drivetrain.getPose().getY(),
        //     m_drivetrain.getPose().getRotation().getDegrees(),
        //     m_drivetrain.getWheelSpeeds().leftMetersPerSecond,
        //     0.0,
        //     0.0,
        //     0.0,
        //     m_drivetrain.getWheelSpeeds().rightMetersPerSecond,
        //     0.0,
        //     0.0,
        //     0.0
        // );
    }

    @Override
    public void execute() {
        double curTime = m_timer.get();
        double dt = curTime - m_prevTime;

        if (m_prevTime < 0) {
          m_drivetrain.SetOpenLoopOutput(0.0, 0.0);
          m_prevTime = curTime;
          return;
        }

        var currentState = m_trajectory.sample( curTime );
        var targetWheelSpeeds = m_drivetrain.mDriveKinematics.toWheelSpeeds( m_drivetrain.mRamseteConroller.calculate( m_drivetrain.getPose(), currentState ) );
        var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
        var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

        double leftFeedforward = m_drivetrain.mFeedforward.calculate( leftSpeedSetpoint, (leftSpeedSetpoint - m_prevSpeeds.leftMetersPerSecond) / dt);
        double rightFeedforward = m_drivetrain.mFeedforward.calculate( rightSpeedSetpoint, (rightSpeedSetpoint - m_prevSpeeds.rightMetersPerSecond) / dt);
        double leftOutput = leftFeedforward + m_leftController.calculate( m_drivetrain.getWheelSpeeds().leftMetersPerSecond, leftSpeedSetpoint);
        double rightOutput = rightFeedforward + m_rightController.calculate( m_drivetrain.getWheelSpeeds().rightMetersPerSecond, rightSpeedSetpoint);

        m_drivetrain.SetOpenLoopOutput( leftOutput, rightOutput );
        m_prevSpeeds = targetWheelSpeeds;
        m_prevTime = curTime;

        // mLogger.info("{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
        //     curTime,
        //     currentState.poseMeters.getX(),
        //     currentState.poseMeters.getY(),
        //     currentState.poseMeters.getRotation().getDegrees(),
        //     m_drivetrain.getPose().getX(),
        //     m_drivetrain.getPose().getY(),
        //     m_drivetrain.getPose().getRotation().getDegrees(),
        //     m_drivetrain.getWheelSpeeds().leftMetersPerSecond,
        //     leftSpeedSetpoint,
        //     leftFeedforward,
        //     leftOutput,
        //     m_drivetrain.getWheelSpeeds().rightMetersPerSecond,
        //     rightSpeedSetpoint,
        //     rightFeedforward,
        //     rightOutput
        // );
    }

    @Override
    public void end( boolean interrupted ) {
        m_timer.stop();

        if (interrupted) {
            m_drivetrain.SetOpenLoopOutput( 0.0, 0.0 );
        }
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed( m_trajectory.getTotalTimeSeconds() );
    }
}
