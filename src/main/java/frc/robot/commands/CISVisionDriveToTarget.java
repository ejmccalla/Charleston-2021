package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import java.util.List;
import java.util.ArrayList;
import java.util.Arrays;
import edu.wpi.first.wpilibj.util.Units;

public class CISVisionDriveToTarget extends CommandBase {
    @SuppressWarnings( { "PMD.UnusedPrivateField", "PMD.SingularField" } )
    private Trajectory mTrajectory;
    private final Drivetrain mDrivetrain;
    private final Timer mTimer = new Timer();
    private double mPreviousTime;


    @Override
    public void initialize() {
        int numPoints = mDrivetrain.mWaypointsX_in.length;
        Pose2d startingPose = new Pose2d( 0.0, 0.0, new Rotation2d( mDrivetrain.GetGyroAngleInRadians() ) );
        List<Translation2d> translationPoints = new ArrayList<Translation2d>();
        Pose2d endingPose;

        /**
        *   The waypoints being sent out by the coprocessor are all relative to the current
        *   pose of the robot. When building the trajectories, the first waypoint will be
        *   the current pose of the robot and doesn't need to be sent over with the list of
        *   waypoints. Since the trajectory is being built using hermite cubic splines, we need to
        *   handle the cases below for the number of waypoints:
        *
        *   1 waypoint:
        *       Use waypoint as the pose-defined endpoint and linearly interpolate the
        *       interior translations using the assumed starting waypoint 0,0.
        *   
        *   2 waypoints:
        *       Use the second waypoint as the pose-defined endpoint insert another waypoint
        *       using the two given waypoints.
        *   
        *   3 waypoints:
        *       There are enough points, no need to add any.
        *
        *   The end heading is currently being set to the vector direction of the final 2
        *   waypoints. This may cause some excessive curvature earlier in the path following.
        */
        if ( numPoints < 1 ) {
            mTrajectory = new Trajectory(Arrays.asList(new Trajectory.State()));
            //System.out.println("Not enough trajectory points");
        } else { 
            if ( numPoints == 1 ) {
                double oneThirdX = mDrivetrain.mWaypointsX_in[0] / 3.0;
                double oneThirdY = mDrivetrain.mWaypointsY_in[0] / 3.0;
                translationPoints.add( new Translation2d( Units.inchesToMeters( oneThirdX ),
                                                          Units.inchesToMeters( oneThirdY ) ) );
                translationPoints.add( new Translation2d( Units.inchesToMeters( 2.0 * oneThirdX ),
                                                          Units.inchesToMeters( 2.0 * oneThirdY ) ) );
                // Set the end heading as the direction of the vector starting at 0,0 and ending at the waypoint
                endingPose = new Pose2d( Units.inchesToMeters( mDrivetrain.mWaypointsX_in[0] ),
                                         Units.inchesToMeters( mDrivetrain.mWaypointsY_in[0] ),
                                         new Rotation2d( mDrivetrain.mWaypointsX_in[0], mDrivetrain.mWaypointsY_in[0] ) );                 
            } else if ( numPoints == 2 ) {
                translationPoints.add( new Translation2d( Units.inchesToMeters( mDrivetrain.mWaypointsX_in[0] ),
                                                          Units.inchesToMeters( mDrivetrain.mWaypointsY_in[0] ) ) );
                translationPoints.add( new Translation2d( Units.inchesToMeters( ( mDrivetrain.mWaypointsX_in[1] - mDrivetrain.mWaypointsX_in[0] ) / 2.0 ),
                                                          Units.inchesToMeters( ( mDrivetrain.mWaypointsY_in[1] - mDrivetrain.mWaypointsY_in[0] ) / 2.0 ) ) );
                // Set the end heading as the direction of the vector starting at the second to last waypoint
                // and ending at the last waypoint
                endingPose = new Pose2d( Units.inchesToMeters( mDrivetrain.mWaypointsX_in[1] ),
                                         Units.inchesToMeters( mDrivetrain.mWaypointsY_in[1] ),
                                         new Rotation2d( mDrivetrain.mWaypointsX_in[1] - mDrivetrain.mWaypointsX_in[0],
                                                         mDrivetrain.mWaypointsY_in[1] - mDrivetrain.mWaypointsY_in[0] ) );                 
            } else {
                for (int i = 0 ; i != numPoints - 1 ; i++) {
                    translationPoints.add(new Translation2d( Units.inchesToMeters(mDrivetrain.mWaypointsX_in[i] ), 
                                                             Units.inchesToMeters(mDrivetrain.mWaypointsY_in[i] ) ) );
                }
                // Set the end heading as the direction of the vector starting at the second to last waypoint
                // and ending at the last waypoint
                endingPose = new Pose2d( Units.inchesToMeters( mDrivetrain.mWaypointsX_in[numPoints-1] ),
                                         Units.inchesToMeters( mDrivetrain.mWaypointsY_in[numPoints-1] ),
                                         new Rotation2d( mDrivetrain.mWaypointsX_in[numPoints-1] - mDrivetrain.mWaypointsX_in[numPoints-2],
                                                         mDrivetrain.mWaypointsY_in[numPoints-1] - mDrivetrain.mWaypointsY_in[numPoints-2] ) );                 
            }

            mTrajectory = TrajectoryGenerator.generateTrajectory( startingPose, translationPoints, endingPose, mDrivetrain.mTrajectoryConfig );
        }

        // For debug...
        mTrajectory = new Trajectory(Arrays.asList(new Trajectory.State()));

        mDrivetrain.InitializeTrajectoryFollowing( mTrajectory );
        mPreviousTime = -1;
        mTimer.reset();
        mTimer.start();
    }

    @Override
    public void execute() {
        double currentTime = mTimer.get();
        double dt = currentTime - mPreviousTime;

        // The this method is called immediately after the intialize, set the output to 0 and
        // updated the previous time
        if (mPreviousTime < 0) {
            mDrivetrain.SetOpenLoopOutput( 0.0, 0.0 );
            mPreviousTime = currentTime;
          return;
        }

        // Let the drivetrain handle tracking the trajectory based on the current time and the 
        // delta time from the last update
        mDrivetrain.UpdateTrajectoryFollowing( currentTime, dt );
        mPreviousTime = currentTime;
    }

    @Override
    public void end( boolean interrupted ) {
        mTimer.stop();

        if ( interrupted ) {
            mDrivetrain.SetOpenLoopOutput( 0.0, 0.0 );
        }
    }

    @Override
    public boolean isFinished() {
        return mTimer.hasElapsed( mTrajectory.getTotalTimeSeconds() );
    }

    public CISVisionDriveToTarget( Drivetrain drivetrain ) {
        mDrivetrain = drivetrain;
        addRequirements(mDrivetrain);
        this.setName( "CISVisionDriveToTarget" );
    }

}
