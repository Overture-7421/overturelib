package com.overture.lib.motorcontrollers.overtalonfx;

import com.ctre.phoenix6.hardware.core.CoreTalonFX;

/**
 * The OverTalonFX class is a wrapper for the TalonFX class from the CTRE
 * Phoenix library.
 */
public class OverTalonFX extends CoreTalonFX {

	/**
	 * Constructor for OverTalonFX
	 * 
	 * @param deviceNumber The device number of the TalonFX.
	 */
	public OverTalonFX(int deviceNumber) {
		super(deviceNumber);
	}

}
