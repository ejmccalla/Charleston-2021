package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

public class DrivetrainFollowTrajectory extends CommandBase {
    @SuppressWarnings( { "PMD.UnusedPrivateField", "PMD.SingularField" } )
    private final Trajectory mTrajectory;
    private final Drivetrain mDrivetrain;
    private final Timer mTimer = new Timer();
    private double mPreviousTime;


    @Override
    public void initialize() {
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

    public DrivetrainFollowTrajectory ( Drivetrain drivetrain, Trajectory trajectory ) {
        mDrivetrain = drivetrain;
        mTrajectory = trajectory;
        addRequirements(mDrivetrain);
        this.setName( "DrivetrainFollowTrajectory" );
    }

}
