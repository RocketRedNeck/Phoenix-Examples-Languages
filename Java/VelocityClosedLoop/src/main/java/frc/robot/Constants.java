/**
 * Simple class containing constants used throughout project
 */
package frc.robot;

class Constants {

	public static final int kTicksPerRev = 8192;
	/**
	 * Which PID slot to pull gains from. Starting 2018, you can choose from
	 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
	 * configuration.
	 */
	public static final int kSlotIdx = 0;

	/**
	 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;

	/**
	 * Set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
    public static final int kTimeoutMs = 30;

	/**
	 * PID Gains may have to be adjusted based on the responsiveness of control loop.
     * kF: 1023 represents output value to Talon at 100%, 7200 represents Velocity units at 100% output
     * 
	 * 	                                    			 */
	public final static Gains kGains_Velocit = new Gains( 0.14014/2, 		//P
														  0.001, 		//I
														  10.0*0.14014/2, 			//D
														  1023.0/20000.0,//F
														  400,  		//Iz
														  1.00);		// peak output
}
