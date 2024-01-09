package frc.robot.auto.modes;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;

public class DoNothingMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("doing nothing");
    }

    @Override
    public Pose2d getStartingPose() {
        return new Pose2d();
    }
}
