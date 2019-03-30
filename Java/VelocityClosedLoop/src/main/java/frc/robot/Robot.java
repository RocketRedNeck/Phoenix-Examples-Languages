/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products
 * as well as the FRC roboRIO when in use in FRC Competition.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */

/**
 * Description:
 * The VelocityClosedLoop example demonstrates the velocity closed-loop servo.
 * Tested with Logitech F350 USB Gamepad inserted into Driver Station]
 * 
 * Be sure to select the correct feedback sensor using configSelectedFeedbackSensor() below.
 * Use Percent Output Mode (Holding A and using Left Joystick) to confirm talon is driving 
 * forward (Green LED on Talon/Victor) when the postion sensor is moving in the postive 
 * direction. If this is not the case, flip the boolean input in setSensorPhase().
 * 
 * Controls:
 * Button 1: When held, start and run Velocity Closed Loop on Talon/Victor
 * Left Joystick Y-Axis:
 * 	+ Percent Output: Throttle Talon forward and reverse, use to confirm hardware setup
 * 	+ Velocity Closed Loop: Servo Talon forward and reverse [-500, 500] RPM
 * 
 * Gains for Velocity Closed Loop may need to be adjusted in Constants.java
 * 
 * Supported Version:
 * - Talon SRX: 4.00
 * - Victor SPX: 4.00
 * - Pigeon IMU: 4.00
 * - CANifier: 4.00
 */
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.*;

public class Robot extends TimedRobot {
    /* Hardware */
	TalonSRX _talon = new TalonSRX(6);
	TalonSRX _talon1 = new TalonSRX(5);
	TalonSRX _talon2 = new TalonSRX(4);
    Joystick _joy = new Joystick(0);
    
    /* String for output */
    StringBuilder _sb = new StringBuilder();
    
    /* Loop tracker for prints */
	int _loops = 0;

	public void robotInit() {
		SmartDashboard.putNumber("Target RPM",500.0);
		_talon1.follow(_talon);
		_talon2.follow(_talon);
		
        /* Factory Default all hardware to prevent unexpected behaviour */
		_talon.configFactoryDefault();
		_talon1.configFactoryDefault();
		_talon2.configFactoryDefault();


		/* Config sensor used for Primary PID [Velocity] */
        _talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,
                                            Constants.kPIDLoopIdx, 
                                            Constants.kTimeoutMs);

		/**
		 * Configure Talon SRX Output and Sesnor direction accordingly
		 * Invert Motor to have green LEDs when driving Talon Forward / Requesting Postiive Output
		 * Phase sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		_talon.setSensorPhase(true);  // When controller goes green encoder shows +motion
		_talon.setInverted(true);	   // When +1 goes into controller it goes green	
		_talon1.setInverted(true);	   // When +1 goes into controller it goes green	
		_talon2.setInverted(true);	   // When +1 goes into controller it goes green	

		/* Set relevant frame periods to be at least as fast as periodic rate */
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);

		_talon.configClosedloopRamp(0.0);

		/* Config the peak and nominal outputs */
		_talon.configNominalOutputForward(0, Constants.kTimeoutMs);
		_talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_talon.configPeakOutputForward(1, Constants.kTimeoutMs);
		_talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		/* Config the Velocity closed loop gains in slot0 */
		_talon.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kF, Constants.kTimeoutMs);
		_talon.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kP, Constants.kTimeoutMs);
		_talon.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
		_talon.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kD, Constants.kTimeoutMs);
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		double targetRPM = SmartDashboard.getNumber("Target RPM",500.0);
		double stick = -1.0*_joy.getRawAxis(1); //(_joy.getRawButton(2)?-1.0:1.0)

		if (!_joy.getRawButton(1) && !_joy.getRawButton(2))
		{
			_talon.set(ControlMode.PercentOutput,0.0);
			int spd = _talon.getSelectedSensorVelocity(Constants.kPIDLoopIdx);
			SmartDashboard.putNumber("spd",spd);
			int pos = _talon.getSelectedSensorPosition(Constants.kPIDLoopIdx);
			SmartDashboard.putNumber("pos",pos);
			return;
			// _sb.append("\tspd:");
			// _sb.append(spd);
			// _sb.append("\tpos:");
			// _sb.append(pos);
		}
		else
		{
			/* Get Talon/Victor's current output percentage */
			double motorOutput = _talon.getMotorOutputPercent();
			double motorCurrent = _talon.getOutputCurrent();
			
			/* Prepare line to print */
			_sb.append("\tCommand%:");
			_sb.append(stick);
			_sb.append("\tOut%:");
			_sb.append(motorOutput);
			_sb.append("\tOut AMPS:");
			_sb.append(motorCurrent);
			_sb.append("\tspd:");
			int spd = _talon.getSelectedSensorVelocity(Constants.kPIDLoopIdx);
			SmartDashboard.putNumber("spd",spd);
			int pos = _talon.getSelectedSensorPosition(Constants.kPIDLoopIdx);
			SmartDashboard.putNumber("pos",pos);
			_sb.append(spd);


			/** 
			 * When button 1 is held, start and run Velocity Closed loop.
			 * Velocity Closed Loop is controlled by joystick position x500 RPM, [-500, 500] RPM
			 */
			if (_joy.getRawButton(1)) {
				/* Velocity Closed Loop */

				/**
				 * Convert 500 RPM to units / 100ms.
				 * 4096 Units/Rev * 500 RPM / 600 100ms/min in either direction:
				 * velocity setpoint is in units/100ms
				 */
				double targetVelocity_UnitsPer100ms =  (_joy.getRawButton(2)?-1.0:1.0) * targetRPM * Constants.kTicksPerRev / 600;
				/* 500 RPM in either direction */
				_talon.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);

				/* Append more signals to print when in speed mode. */
				_sb.append("\terr:");
				int err = _talon.getClosedLoopError(Constants.kPIDLoopIdx);
				SmartDashboard.putNumber("err",err);
				_sb.append(err);
				_sb.append("\ttrg:");
				_sb.append(targetVelocity_UnitsPer100ms);
			} else {
				/* Percent Output */

				_talon.set(ControlMode.PercentOutput, stick);
			}
		}

        /* Print built string every 10 loops */
		if (++_loops >= 5) {
			_loops = 0;
			System.out.println(_sb.toString());
        }
        /* Reset built string */
		_sb.setLength(0);
	}
}
