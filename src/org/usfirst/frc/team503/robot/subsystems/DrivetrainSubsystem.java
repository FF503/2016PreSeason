package org.usfirst.frc.team503.robot.subsystems;


import java.util.Set;

import org.usfirst.frc.team503.robot.Robot;
import org.usfirst.frc.team503.robot.RobotHardwareProgrammingBot.Constants;
import org.usfirst.frc.team503.robot.RobotState;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import util.AdaptivePurePursuitController;
import util.DriveSignal;
import util.Kinematics;
import util.NavXGyro;
import util.Path;
import util.RigidTransform2d;
import util.Rotation2d;
import util.SynchronousPID;

/**
 * The robot's drivetrain, which implements the Superstructure abstract class.
 * The drivetrain has several states and builds on the abstract class by
 * offering additional control methods, including control by path and velocity.
 * 
 * @see Subsystem.java
 */
public class DrivetrainSubsystem extends Subsystem {
		    
    protected static final int kVelocityControlSlot = 0;
    protected static final int kBaseLockControlSlot = 1;

    private static DrivetrainSubsystem instance_ = new DrivetrainSubsystem();
    private double mLastHeadingErrorDegrees;

    public static DrivetrainSubsystem getInstance() {
        return instance_;
    }

    // The robot drivetrain's various states
    public enum DriveControlState {
        OPEN_LOOP, BASE_LOCKED, VELOCITY_SETPOINT, VELOCITY_HEADING_CONTROL, PATH_FOLLOWING_CONTROL
    }

    private final CANTalon leftMaster_, leftSlave_, rightMaster_, rightSlave_;
    private boolean isHighGear_ = false;
    private boolean isBrakeMode_ = true;
    private NavXGyro gyro;

    private DriveControlState driveControlState_;
    private VelocityHeadingSetpoint velocityHeadingSetpoint_;
    private AdaptivePurePursuitController pathFollowingController_;
    private SynchronousPID velocityHeadingPid_;



    // The constructor instantiates all of the drivetrain components when the
    // robot powers up
    private DrivetrainSubsystem() {
        leftMaster_ = Robot.bot.getCANTalonObj(0);
        leftSlave_ = Robot.bot.getCANTalonObj(2);
        rightMaster_ = Robot.bot.getCANTalonObj(1);
        rightSlave_ = Robot.bot.getCANTalonObj(3);
        setHighGear(true);
/*        lineSensor1_ = new DigitalInput(Constants.kLineSensor1DIO);
        lineSensor2_ = new DigitalInput(Constants.kLineSensor2DIO);
        lineSensorCounter1_ = new Counter(lineSensor1_);
        lineSensorCounter2_ = new Counter(lineSensor2_);*/

        // Get status at 100Hz
        leftMaster_.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);
        rightMaster_.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);

        // Start in open loop mode
        leftMaster_.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        leftMaster_.set(0);
        leftSlave_.changeControlMode(CANTalon.TalonControlMode.Follower);
        leftSlave_.set(Constants.kLeftDriveMasterId);
        rightMaster_.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        rightMaster_.set(0);
        rightSlave_.changeControlMode(CANTalon.TalonControlMode.Follower);
        rightSlave_.set(Constants.kRightDriveMasterId);
        setBrakeMode(false);

        // Set up the encoders
        leftMaster_.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        if (leftMaster_.isSensorPresent(CANTalon.FeedbackDevice.QuadEncoder) 
        		!= CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
            DriverStation.reportError("Could not detect left drive encoder!", false);
        }
        leftMaster_.reverseSensor(false);
        leftMaster_.reverseOutput(true);
        leftSlave_.reverseOutput(true);
        rightMaster_.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        if (rightMaster_.isSensorPresent(
                CANTalon.FeedbackDevice.QuadEncoder) != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
            DriverStation.reportError("Could not detect right drive encoder!", false);
        }
        rightMaster_.reverseSensor(false);
        rightMaster_.reverseOutput(false);
        rightSlave_.reverseOutput(false);

        // Load velocity control gains
        leftMaster_.setPID(Constants.kDriveVelocityKp, Constants.kDriveVelocityKi, Constants.kDriveVelocityKd,
                Constants.kDriveVelocityKf, Constants.kDriveVelocityIZone, Constants.kDriveVelocityRampRate,
                kVelocityControlSlot);
        rightMaster_.setPID(Constants.kDriveVelocityKp, Constants.kDriveVelocityKi, Constants.kDriveVelocityKd,
                Constants.kDriveVelocityKf, Constants.kDriveVelocityIZone, Constants.kDriveVelocityRampRate,
                kVelocityControlSlot);
        // Load base lock control gains
        leftMaster_.setPID(Constants.kDriveBaseLockKp, Constants.kDriveBaseLockKi, Constants.kDriveBaseLockKd,
                Constants.kDriveBaseLockKf, Constants.kDriveBaseLockIZone, Constants.kDriveBaseLockRampRate,
                kBaseLockControlSlot);
        rightMaster_.setPID(Constants.kDriveBaseLockKp, Constants.kDriveBaseLockKi, Constants.kDriveBaseLockKd,
                Constants.kDriveBaseLockKf, Constants.kDriveBaseLockIZone, Constants.kDriveBaseLockRampRate,
                kBaseLockControlSlot);

        velocityHeadingPid_ = new SynchronousPID(Constants.kDriveHeadingVelocityKp, Constants.kDriveHeadingVelocityKi,
                Constants.kDriveHeadingVelocityKd);
        velocityHeadingPid_.setOutputRange(-30, 30);

        setOpenLoop(DriveSignal.NEUTRAL);
        gyro = new NavXGyro();
    }

    protected synchronized void setLeftRightPower(double left, double right) {
        leftMaster_.set(-left);
        rightMaster_.set(right);
    }

    public synchronized void setOpenLoop(DriveSignal signal) {
        if (driveControlState_ != DriveControlState.OPEN_LOOP) {
            leftMaster_.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
            rightMaster_.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
            driveControlState_ = DriveControlState.OPEN_LOOP;
        }
        setLeftRightPower(signal.leftMotor, signal.rightMotor);
    }

    public synchronized void setBaseLockOn() {
        if (driveControlState_ != DriveControlState.BASE_LOCKED) {
            leftMaster_.setProfile(kBaseLockControlSlot);
            leftMaster_.changeControlMode(CANTalon.TalonControlMode.Position);
            leftMaster_.setAllowableClosedLoopErr(Constants.kDriveBaseLockAllowableError);
            leftMaster_.set(leftMaster_.getPosition());
            rightMaster_.setProfile(kBaseLockControlSlot);
            rightMaster_.changeControlMode(CANTalon.TalonControlMode.Position);
            rightMaster_.setAllowableClosedLoopErr(Constants.kDriveBaseLockAllowableError);
            rightMaster_.set(rightMaster_.getPosition());
            driveControlState_ = DriveControlState.BASE_LOCKED;
            setBrakeMode(true);
        }
        setHighGear(false);
    }

    public synchronized void setVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        configureTalonsForSpeedControl();
        driveControlState_ = DriveControlState.VELOCITY_SETPOINT;
        updateVelocitySetpoint(left_inches_per_sec, right_inches_per_sec);
    }

    public synchronized void setVelocityHeadingSetpoint(double forward_inches_per_sec, Rotation2d headingSetpoint) {
        if (driveControlState_ != DriveControlState.VELOCITY_HEADING_CONTROL) {
            configureTalonsForSpeedControl();
            driveControlState_ = DriveControlState.VELOCITY_HEADING_CONTROL;
            velocityHeadingPid_.reset();
        }
        velocityHeadingSetpoint_ = new VelocityHeadingSetpoint(forward_inches_per_sec, forward_inches_per_sec,
                headingSetpoint);
        updateVelocityHeadingSetpoint();
    }

    /**
     * The robot follows a set path, which is defined by Waypoint objects.
     * 
     * @param Path
     *            to follow
     * @param reversed
     * @see com.team254.lib.util/Path.java
     */
    public synchronized void followPath(Path path, boolean reversed) {
        if (driveControlState_ != DriveControlState.PATH_FOLLOWING_CONTROL) {
            configureTalonsForSpeedControl();
            driveControlState_ = DriveControlState.PATH_FOLLOWING_CONTROL;
            velocityHeadingPid_.reset();
        }
        pathFollowingController_ = new AdaptivePurePursuitController(Constants.kPathFollowingLookahead,
                Constants.kPathFollowingMaxAccel, Constants.kLooperDt, path, reversed, 0.25);
        updatePathFollower();
    }

    /**
     * @return Returns if the robot mode is Path Following Control and the set
     *         path is complete.
     */
    public synchronized boolean isFinishedPath() {
        return (driveControlState_ == DriveControlState.PATH_FOLLOWING_CONTROL && pathFollowingController_.isDone())
                || driveControlState_ != DriveControlState.PATH_FOLLOWING_CONTROL;
    }

    /**
     * Path Markers are an optional functionality that name the various
     * Waypoints in a Path with a String. This can make defining set locations
     * much easier.
     * 
     * @return Set of Strings with Path Markers that the robot has crossed.
     */
    public synchronized Set<String> getPathMarkersCrossed() {
        if (driveControlState_ != DriveControlState.PATH_FOLLOWING_CONTROL) {
            return null;
        } else {
            return pathFollowingController_.getMarkersCrossed();
        }
    }

    public double getLeftDistanceInches() {
        return rotationsToInches(leftMaster_.getPosition());
    }

    public double getRightDistanceInches() {
        return rotationsToInches(rightMaster_.getPosition());
    }
    
    public double getCurrentDistance(){
    	return (getLeftDistanceInches() + getRightDistanceInches())/2.0;
    }

    public double getLeftVelocityInchesPerSec() {
        return rpmToInchesPerSecond(leftMaster_.getSpeed());
    }

    public double getRightVelocityInchesPerSec() {
        return rpmToInchesPerSecond(rightMaster_.getSpeed());
    }

    public synchronized Rotation2d getGyroAngle() {
        return Rotation2d.fromDegrees(gyro.getAngle());
    }

    public boolean isHighGear() {
        return isHighGear_;
    }

    public void setHighGear(boolean high_gear) {
        isHighGear_ = high_gear;
        //shifter_.set(!high_gear);
    }

    public synchronized void resetEncoders() {
        leftMaster_.setPosition(0);
        rightMaster_.setPosition(0);

        leftMaster_.setEncPosition(0);
        rightMaster_.setEncPosition(0);
    }

    public synchronized DriveControlState getControlState() {
        return driveControlState_;
    }

    public synchronized VelocityHeadingSetpoint getVelocityHeadingSetpoint() {
        return velocityHeadingSetpoint_;
    }

    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("left_distance", getLeftDistanceInches());
        SmartDashboard.putNumber("right_distance", getRightDistanceInches());
        SmartDashboard.putNumber("left_velocity", getLeftVelocityInchesPerSec());
        SmartDashboard.putNumber("right_velocity", getRightVelocityInchesPerSec());
        SmartDashboard.putNumber("left_error", leftMaster_.getClosedLoopError());
        SmartDashboard.putNumber("right_error", leftMaster_.getClosedLoopError());
        SmartDashboard.putNumber("gyro_angle", gyro.getAngle());
//        SmartDashboard.putNumber("gyro_center", gyro.getCenter());
        SmartDashboard.putNumber("heading_error", mLastHeadingErrorDegrees);
        SmartDashboard.putNumber("left_motor_power", leftMaster_.get());
        SmartDashboard.putNumber("right_motor_power", rightMaster_.get());
    }

    public synchronized void zeroSensors() {
        resetEncoders();
        gyro.reset();
    }

    private void configureTalonsForSpeedControl() {
        if (driveControlState_ != DriveControlState.VELOCITY_HEADING_CONTROL
                && driveControlState_ != DriveControlState.VELOCITY_SETPOINT
                && driveControlState_ != DriveControlState.PATH_FOLLOWING_CONTROL) {
            leftMaster_.changeControlMode(CANTalon.TalonControlMode.Speed);
            leftMaster_.setProfile(kVelocityControlSlot);
            leftMaster_.setAllowableClosedLoopErr(Constants.kDriveVelocityAllowableError);
            rightMaster_.changeControlMode(CANTalon.TalonControlMode.Speed);
            rightMaster_.setProfile(kVelocityControlSlot);
            rightMaster_.setAllowableClosedLoopErr(Constants.kDriveVelocityAllowableError);
            setHighGear(true);
            setBrakeMode(true);
        }
    }

    private synchronized void updateVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        if (driveControlState_ == DriveControlState.VELOCITY_HEADING_CONTROL
                || driveControlState_ == DriveControlState.VELOCITY_SETPOINT
                || driveControlState_ == DriveControlState.PATH_FOLLOWING_CONTROL) {
            leftMaster_.set(inchesPerSecondToRpm(left_inches_per_sec));
            rightMaster_.set(inchesPerSecondToRpm(right_inches_per_sec));
            SmartDashboard.putString("In setpoint","Updating velocity setpoint.");
        } else {
            System.out.println("Hit a bad velocity control state");
            leftMaster_.set(0);
            rightMaster_.set(0);
        }
    }

    public void updateVelocityHeadingSetpoint() {
        Rotation2d actualGyroAngle = getGyroAngle();

        mLastHeadingErrorDegrees = velocityHeadingSetpoint_.getHeading().rotateBy(actualGyroAngle.inverse())
                .getDegrees();

        double deltaSpeed = velocityHeadingPid_.calculate(mLastHeadingErrorDegrees);
        updateVelocitySetpoint(velocityHeadingSetpoint_.getLeftSpeed() + deltaSpeed / 2,
                velocityHeadingSetpoint_.getRightSpeed() - deltaSpeed / 2);
    }

    public void updatePathFollower() {
        RigidTransform2d robot_pose = RobotState.getInstance().getLatestFieldToVehicle().getValue();
        RigidTransform2d.Delta command = pathFollowingController_.update(robot_pose, Timer.getFPGATimestamp());
        Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);

        // Scale the command to respect the max velocity limits
        double max_vel = 0.0;
        max_vel = Math.max(max_vel, Math.abs(setpoint.left));
        max_vel = Math.max(max_vel, Math.abs(setpoint.right));
        if (max_vel > Constants.kPathFollowingMaxVel) {
            double scaling = Constants.kPathFollowingMaxVel / max_vel;
            setpoint = new Kinematics.DriveVelocity(setpoint.left * scaling, setpoint.right * scaling);
        }
        updateVelocitySetpoint(setpoint.left, setpoint.right);
    }

    private static double rotationsToInches(double rotations) {
        return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    private static double inchesToRotations(double inches) {
        return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double inchesPerSecondToRpm(double inches_per_second) {
        return inchesToRotations(inches_per_second) * 60;
    }

    public void setBrakeMode(boolean on) {
        if (isBrakeMode_ != on) {
            leftMaster_.enableBrakeMode(on);
            leftSlave_.enableBrakeMode(on);
            rightMaster_.enableBrakeMode(on);
            rightSlave_.enableBrakeMode(on);
            isBrakeMode_ = on;
        }
    }

    /**
     * VelocityHeadingSetpoints are used to calculate the robot's path given the
     * speed of the robot in each wheel and the polar coordinates. Especially
     * useful if the robot is negotiating a turn and to forecast the robot's
     * location.
     */
    public static class VelocityHeadingSetpoint {
        private final double leftSpeed_;
        private final double rightSpeed_;
        private final Rotation2d headingSetpoint_;

        // Constructor for straight line motion
        public VelocityHeadingSetpoint(double leftSpeed, double rightSpeed, Rotation2d headingSetpoint) {
            leftSpeed_ = leftSpeed;
            rightSpeed_ = rightSpeed;
            headingSetpoint_ = headingSetpoint;
        }

        public double getLeftSpeed() {
            return leftSpeed_;
        }

        public double getRightSpeed() {
            return rightSpeed_;
        }

        public Rotation2d getHeading() {
            return headingSetpoint_;
        }
    }

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
	public void arcadeDrive(double moveValue, double rotateValue) {
        double leftMotorSpeed;
        double rightMotorSpeed;
        
        moveValue = limit(moveValue,-1,1);
        rotateValue = limit(rotateValue,-1,1);

        if (moveValue > 0.0) {
            if (rotateValue > 0.0) {
                leftMotorSpeed = moveValue - rotateValue;
                rightMotorSpeed = Math.max(moveValue, rotateValue);
            } else {
                leftMotorSpeed = Math.max(moveValue, -rotateValue);
                rightMotorSpeed = moveValue + rotateValue;
            }
        } else {
            if (rotateValue > 0.0) {
                leftMotorSpeed = -Math.max(-moveValue, rotateValue);
                rightMotorSpeed = moveValue + rotateValue;
            } else {
                leftMotorSpeed = moveValue - rotateValue;
                rightMotorSpeed = -Math.max(-moveValue, -rotateValue);
            }
        }
        setLeftRightPower(leftMotorSpeed, rightMotorSpeed);
    }
	
    public void tankDrive(double leftValue, double rightValue) {
        leftValue = limit(leftValue,-1,1);
        rightValue = limit(rightValue,-1,1);
        setLeftRightPower(leftValue, rightValue);
    }
    public static double limit(double num,double min,double max) {
        if (num > max) {
            num= max;
        }
        else if (num < min) {
            num= min;
        }
        	return num;
    }
    
}
