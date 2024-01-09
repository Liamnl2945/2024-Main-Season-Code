// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.net.NetworkInterface;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Enumeration;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeExecutor;
import frc.robot.auto.AutoModeSelector;
import frc.robot.controls.ControlBoard;
import frc.robot.controls.CustomXboxController.Button;
import frc.robot.controls.CustomXboxController.Side;
import frc.robot.loops.CrashTracker;
import frc.robot.loops.Looper;
import frc.robot.shuffleboard.ShuffleBoardInteractions;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Superstructure;
import lib.logger.LoggingSystem;
import lib.sim.PhysicsSim;
import lib.swerve.ChassisSpeeds;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {



  // util instances
	private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
	private final ControlBoard mControlBoard = ControlBoard.getInstance();
	private final ShuffleBoardInteractions mShuffleboard = ShuffleBoardInteractions.getInstance();
	private final LoggingSystem mLogger = LoggingSystem.getInstance();
  private Command m_autonomousCommand;

  // subsystem instances
	private final Superstructure mSuperstructure = Superstructure.getInstance();
	private final Drive mDrive = Drive.getInstance();

  private SubsystemManager m_robotContainer;


  // instantiate enabled and disabled loopers
  private final Looper mEnabledLooper = new Looper();
	private final Looper mDisabledLooper = new Looper();
	private final Looper mLoggingLooper = new Looper(0.002);


  // auto instances
	private AutoModeExecutor mAutoModeExecutor;
	public final static AutoModeSelector mAutoModeSelector = new AutoModeSelector();

  public static boolean is_red_alliance = false;
	public static boolean flip_trajectories = false;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

   public Robot() {
		CrashTracker.logRobotConstruction();
	}

 @Override
	public void robotInit() {
		try {
			CrashTracker.logRobotInit();

			LiveWindow.disableAllTelemetry();

			mSubsystemManager.setSubsystems(
					mDrive,
					mSuperstructure
			);

			mSubsystemManager.registerEnabledLoops(mEnabledLooper);
			mSubsystemManager.registerDisabledLoops(mDisabledLooper);

			mLoggingLooper.register(mLogger.Loop());

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    mShuffleboard.update();
		mSubsystemManager.outputToSmartDashboard();
		mEnabledLooper.outputToSmartDashboard();
  
  }

  /** This function is called once each time the robot enters Disabled mode. */


  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
   // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out
			try {
        if (is_red_alliance) {
          mDrive.zeroGyro(mDrive.getHeading().getDegrees() + 180.0);
          flip_trajectories = false;
        }
        mDisabledLooper.stop();
        mEnabledLooper.start();
        mLoggingLooper.start();
        mSuperstructure.stop();
  
        mDrive.setNeutralBrake(true);
  
      } catch (Throwable t) {
        CrashTracker.logThrowableCrash(t);
        throw t;
      }
    }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    try {

			/* Drive */
			if (mControlBoard.zeroGyro()) {
				mDrive.zeroGyro();
				mDrive.resetModulesToAbsolute();
			}

			if (mControlBoard.driver.getController().getAButtonPressed()) {
				System.out.println("Autobalance Started");
				mSuperstructure.autoBalance();
			} else if (mControlBoard.driver.getButton(Button.Y)) {
				mDrive.orientModules(List.of(
						Rotation2d.fromDegrees(45),
						Rotation2d.fromDegrees(-45),
						Rotation2d.fromDegrees(-45),
						Rotation2d.fromDegrees(45)));
			} else if (!mControlBoard.driver.getController().getAButton()) {
				mDrive.feedTeleopSetpoint(ChassisSpeeds.fromFieldRelativeSpeeds(
						mControlBoard.getSwerveTranslation().x(),
						mControlBoard.getSwerveTranslation().y(),
						mControlBoard.getSwerveRotation(),
						mDrive.getHeading()));
			}

			if (mControlBoard.driver.getController().getXButton()) {
				mDrive.setHeadingControlTarget(270.0);
			} else if (mControlBoard.driver.getController().getBButton()) {
				mDrive.setHeadingControlTarget(90.0);
			}

			/* SUPERSTRUCTURE */

			if (mControlBoard.driver.getTrigger(Side.RIGHT)) {
				mSuperstructure.setEndEffectorForwards();
			} else if ((mControlBoard.driver.getTrigger(Side.LEFT))) {
				mSuperstructure.setEndEffectorReverse();
			} else {
				mSuperstructure.setEndEffectorIdle();
			}

			if (mControlBoard.driver.getController().getPOV() == 0) {
			//	mWrist.setWantJog(1.0);
			} else if (mControlBoard.driver.getController().getPOV() == 180) {
				//mWrist.setWantJog(-1.0);
			}

			if (mControlBoard.driver.getController().getLeftBumper()) {
				mSuperstructure.stowState();
				mDrive.setKinematicLimits(Constants.SwerveConstants.kUncappedLimits);
			} else if (mControlBoard.driver.getController().getRightBumper()) {
				mSuperstructure.chooseScoreState();
			} else if (mControlBoard.operator.getController().getRightBumper()) {
				mSuperstructure.groundIntakeState();
			} else if (mControlBoard.operator.getButton(Button.A)) {
				mSuperstructure.chooseShelfIntake();
			} else if (mControlBoard.operator.getButton(Button.Y)) {
				mSuperstructure.slideIntakeState();
			}else if (mControlBoard.operator.getButton(Button.X)) {
				mSuperstructure.scoreStandbyState();
			} else if (mControlBoard.operator.getButton(Button.B)) {
				mSuperstructure.yoshiState();
			} else if (mControlBoard.operator.getController().getLeftStickButtonPressed()) {
				mSuperstructure.climbFloatState();
			} else if (mControlBoard.operator.getController().getRightStickButtonPressed()) {
				mSuperstructure.climbScrapeState();
			} else if (mControlBoard.operator.getTrigger(Side.LEFT)) {
				mSuperstructure.climbCurlState();
			}

			if (mControlBoard.driver.getController().getLeftStickButton()) {
				mDrive.setKinematicLimits(Constants.SwerveConstants.kScoringLimits);
			} else if (mControlBoard.driver.getController().getLeftStickButtonReleased()) {
				mDrive.setKinematicLimits(Constants.SwerveConstants.kUncappedLimits);
			}

			if (mControlBoard.driver.getController().getRightStickButton()) {
				mDrive.setKinematicLimits(Constants.SwerveConstants.kLoadingStationLimits);
			} else if (mControlBoard.driver.getController().getRightStickButtonReleased()) {
				mDrive.setKinematicLimits(Constants.SwerveConstants.kUncappedLimits);
			}

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
  }

  @Override
	public void disabledInit() {
		try {

			CrashTracker.logDisabledInit();
			mEnabledLooper.stop();
			mLoggingLooper.stop();
			mDisabledLooper.start();
			

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}

		if (mAutoModeExecutor != null) {
			mAutoModeExecutor.stop();
		}

		// Reset all auto mode state.
		mAutoModeSelector.reset();
		mAutoModeSelector.updateModeCreator(false);
		mAutoModeExecutor = new AutoModeExecutor();

	}

  @Override
	public void disabledPeriodic() {
		try {

			mDrive.resetModulesToAbsolute();

			boolean alliance_changed = false;
			if (DriverStation.isDSAttached()) {
				if (edu.wpi.first.wpilibj.DriverStation.getAlliance() == Alliance.Red) {
					if (!is_red_alliance) {
						alliance_changed = true;
					} else {
						alliance_changed = false;
					}
					is_red_alliance = true;
				} else if (edu.wpi.first.wpilibj.DriverStation.getAlliance() == Alliance.Blue) {
					if (is_red_alliance) {
						alliance_changed = true;
					} else {
						alliance_changed = false;
					}
					is_red_alliance = false;
				}
			} else {
				alliance_changed = true;
			}
			flip_trajectories = is_red_alliance;

			mAutoModeSelector.updateModeCreator(alliance_changed);
			Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
			if (autoMode.isPresent()) {
				mAutoModeExecutor.setAutoMode(autoMode.get());
			}

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}


  @Override
	public void testInit() {
		try {
			mDisabledLooper.stop();
			mEnabledLooper.stop();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */

  

  /** This function is called periodically whilst in simulation. */
  @Override
	public void simulationPeriodic() {
		PhysicsSim.getInstance().run();
	}
}
