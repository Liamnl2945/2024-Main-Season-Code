package frc.robot.auto.actions;

import frc.robot.subsystems.EndEffector;

public class WaitForAquisition implements Action {
    EndEffector effector;

    public WaitForAquisition() {
        effector = EndEffector.getInstance();
    }

    @Override
    public boolean isFinished() {
        return effector.hasGamePiece();
    }

    @Override
    public void start() {
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {

    }

}