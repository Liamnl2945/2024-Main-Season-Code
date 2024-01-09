package frc.robot.auto.actions;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;

import frc.robot.subsystems.Drive;
import lib.swerve.ChassisSpeeds;

public class SwerveTrajectoryAction implements Action {

    private Drive mDrive = Drive.getInstance();

    private final Trajectory mTrajectory;
    private final Rotation2d mHeading;

    public SwerveTrajectoryAction(Trajectory trajectory, Rotation2d heading){
        mTrajectory = trajectory;
        mHeading = heading;
    }

    @Override
    public void start() {
        if (mDrive.readyForAuto()) {
            System.out.println("Starting trajectory! (length=" + mTrajectory.getTotalTimeSeconds() + " seconds)");
            mDrive.setTrajectory(mTrajectory, mHeading);
        } else {
            System.out.println("Odometry reset failed!!! Not starting trajectory!!!");
        }

    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        if (mDrive.isDoneWithTrajectory()) {
            mDrive.stopModules();
            return true;
        } 
        return false;
    }

    @Override
    public void done() {}
}
