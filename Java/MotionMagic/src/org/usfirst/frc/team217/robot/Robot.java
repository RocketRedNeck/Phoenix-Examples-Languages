/**
 * Example demonstrating the motion magic control mode.
 * Tested with Logitech F710 USB Gamepad inserted into Driver Station.
 * 
 * Be sure to select the correct feedback sensor using configSelectedFeedbackSensor() below.
 *
 * After deploying/debugging this to your RIO, first use the left Y-stick 
 * to throttle the Talon manually.  This will confirm your hardware setup/sensors
 * and will allow you to take initial measurements.
 * 
 * Be sure to confirm that when the Talon is driving forward (green) the 
 * position sensor is moving in a positive direction.  If this is not the 
 * cause, flip the boolean input to the setSensorPhase() call below.
 *
 * Once you've ensured your feedback device is in-phase with the motor,
 * and followed the walk-through in the Talon SRX Software Reference Manual,
 * use button1 to motion-magic servo to target position specified by the gamepad stick.
 */
package org.usfirst.frc.team217.robot;

import edu.wpi.first.wpilibj.IterativeRobot; 
import edu.wpi.first.wpilibj.Joystick;

import java.util.concurrent.TimeUnit;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.*;

public class Robot extends IterativeRobot {
	TalonSRX _talon = new TalonSRX(4);
	Joystick _joy = new Joystick(0);
	StringBuilder _sb = new StringBuilder();
	
	public static double elevatorMotorKf = 0.03868035368; // 0.111544836;
	public static double elevatorMotorKp = 0.9654738498; //0.005028509634 * 2 * 2 * 2 * 2 * 2 * 2 * 2 * 1.5 ; //0.1778688524;		
	public static double elevatorMotorKi = 0.004;
	public static double elevatorMotorKd = 9.654738498; //5.336065573;
	public static int    elevatorMotorIZone = 50; //300;	
	//Magic Motion Constants for the Elevator Subsystem
	public final static boolean ELEVATOR_MOTOR_SENSOR_PHASE = false;

	public static final double ELEVATOR_SPROCKET_DIAMETER_INCHES  = 1.4323944878270580219199538703526;
	public static final double ELEVATOR_SPROCKET_CIRCUMFERENCE_INCHES = (ELEVATOR_SPROCKET_DIAMETER_INCHES*Math.PI);

	public final static FeedbackDevice ELEVATOR_MOTOR_FEEDBACK_DEVICE = FeedbackDevice.QuadEncoder;
	public final static int ELEVATOR_MOTOR_NATIVE_TICKS_PER_REV = 8192;
	public final static double ELEVATOR_MOTOR_FULL_THROTTLE_AVERAGE_SPEED_NATIVE_TICKS = 26447.5; //9171.2;	// per 100 ms, average of 10 samples
	
	public final static int ELEVATOR_MOTOR_MOTION_CRUISE_SPEED_NATIVE_TICKS = (int)(0.80 * 
                                                                              ELEVATOR_MOTOR_FULL_THROTTLE_AVERAGE_SPEED_NATIVE_TICKS);

	public final static int ELEVATOR_MOTOR_MOTION_ACCELERATION_NATIVE_TICKS = ELEVATOR_MOTOR_MOTION_CRUISE_SPEED_NATIVE_TICKS;
	
	//Deadband defines when motion can start (i.e., minimum input required)
	public final static double ELEVATOR_MOTOR_NEUTRAL_DEADBAND  = 0.000; //ADJUST
	
	public static final double ELEVATOR_INCHES_PER_NATIVE_TICKS = ELEVATOR_SPROCKET_CIRCUMFERENCE_INCHES / ELEVATOR_MOTOR_NATIVE_TICKS_PER_REV;

	public static final double ELEVATOR_POSITION_TOLERANCE_INCH = 0.25;	/// TODO: What is possible and what do we want?
	public static final int ELEVATOR_POSITION_TOLERANCE_NATIVE_TICKS = (int) (ELEVATOR_POSITION_TOLERANCE_INCH / ELEVATOR_INCHES_PER_NATIVE_TICKS); //50;	/// TODO: Convert from inches or meter tolerance
	
	public void robotInit() {

		/* first choose the sensor */
		_talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		_talon.setSensorPhase(false);
		_talon.setInverted(true);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

		/* set the peak and nominal outputs */
		_talon.configNominalOutputForward(0, Constants.kTimeoutMs);
		_talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_talon.configPeakOutputForward(1, Constants.kTimeoutMs);
		_talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		/* set closed loop gains in slot0 - see documentation */
		_talon.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		
		_talon.config_kF(0, elevatorMotorKf, Constants.kTimeoutMs);
		_talon.config_kP(0, elevatorMotorKp, Constants.kTimeoutMs);
		_talon.config_kI(0, elevatorMotorKi, Constants.kTimeoutMs);
		_talon.config_kD(0, elevatorMotorKd, Constants.kTimeoutMs);
		_talon.config_IntegralZone(0, elevatorMotorIZone, Constants.kTimeoutMs);
		
		/* set acceleration and vcruise velocity - see documentation */
		_talon.configMotionCruiseVelocity(ELEVATOR_MOTOR_MOTION_CRUISE_SPEED_NATIVE_TICKS, 
				Constants.kTimeoutMs);
		_talon.configMotionAcceleration(ELEVATOR_MOTOR_MOTION_ACCELERATION_NATIVE_TICKS, 
				Constants.kTimeoutMs);
		
		/* zero the sensor */
		_talon.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
	}

	// Re-zero the sensor each enable
	public void teleopInit()
	{
		_talon.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
	}
	
	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		/* get gamepad axis - forward stick is positive */
		double leftYstick = -1.0 * _joy.getY();
		leftYstick *= Math.abs(leftYstick * leftYstick);
		
		/* calculate the percent motor output */
		double motorOutput = _talon.getMotorOutputPercent();
		/* prepare line to print */
		_sb.append("\tOut%:");
		_sb.append(motorOutput);
		_sb.append("\tVel:");
		_sb.append(_talon.getSelectedSensorVelocity(Constants.kPIDLoopIdx));

		if (_joy.getRawButton(1)) {
			/* Motion Magic - 8192 ticks/rev * 1 Rotations in either direction */
			double targetPos = leftYstick * 8192 * 10.0;
			_talon.set(ControlMode.MotionMagic, targetPos);

			/* append more signals to print when in speed mode. */
			_sb.append("\terr:");
			_sb.append(_talon.getClosedLoopError(Constants.kPIDLoopIdx));
			_sb.append("\ttrg:");
			_sb.append(targetPos);
		} else {
			/* Percent voltage mode */
			_talon.set(ControlMode.PercentOutput, leftYstick);
		}
		/* instrumentation */
		Instrum.Process(_talon, _sb);
		try {
			TimeUnit.MILLISECONDS.sleep(10);
		} catch (Exception e) {
		}
	}
}
