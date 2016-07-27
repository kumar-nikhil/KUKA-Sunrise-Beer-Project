package com.kuka.generated.ioAccess;

import javax.inject.Inject;
import javax.inject.Singleton;

import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.ioModel.AbstractIOGroup;
import com.kuka.roboticsAPI.ioModel.IOTypes;

/**
 * Automatically generated class to abstract I/O access to I/O group <b>External</b>.<br>
 * <i>Please, do not modify!</i>
 * <p>
 * <b>I/O group description:</b><br>
 * ./.
 */
@Singleton
public class ExternalIOGroup extends AbstractIOGroup
{
	/**
	 * Constructor to create an instance of class 'External'.<br>
	 * <i>This constructor is automatically generated. Please, do not modify!</i>
	 *
	 * @param controller
	 *            the controller, which has access to the I/O group 'External'
	 */
	@Inject
	public ExternalIOGroup(Controller controller)
	{
		super(controller, "External");

		addInput("IN1", IOTypes.BOOLEAN, 1);
		addInput("IN2", IOTypes.BOOLEAN, 1);
		addInput("IN3", IOTypes.BOOLEAN, 1);
		addInput("IN4", IOTypes.BOOLEAN, 1);
		addInput("IN5", IOTypes.BOOLEAN, 1);
		addDigitalOutput("OUT1", IOTypes.BOOLEAN, 1);
		addDigitalOutput("OUT2", IOTypes.BOOLEAN, 1);
		addDigitalOutput("OUT3", IOTypes.BOOLEAN, 1);
		addDigitalOutput("OUT4", IOTypes.BOOLEAN, 1);
		addDigitalOutput("OUT5", IOTypes.BOOLEAN, 1);
		addDigitalOutput("GripperClose", IOTypes.BOOLEAN, 1);
		addDigitalOutput("GripperOpen", IOTypes.BOOLEAN, 1);
	}

	/**
	 * Gets the value of the <b>digital input '<i>IN1</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'IN1'
	 */
	public boolean getIN1()
	{
		return getBooleanIOValue("IN1", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>IN2</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'IN2'
	 */
	public boolean getIN2()
	{
		return getBooleanIOValue("IN2", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>IN3</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'IN3'
	 */
	public boolean getIN3()
	{
		return getBooleanIOValue("IN3", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>IN4</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'IN4'
	 */
	public boolean getIN4()
	{
		return getBooleanIOValue("IN4", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>IN5</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'IN5'
	 */
	public boolean getIN5()
	{
		return getBooleanIOValue("IN5", false);
	}

	/**
	 * Gets the value of the <b>digital output '<i>OUT1</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'OUT1'
	 */
	public boolean getOUT1()
	{
		return getBooleanIOValue("OUT1", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>OUT1</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'OUT1'
	 */
	public void setOUT1(java.lang.Boolean value)
	{
		setDigitalOutput("OUT1", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>OUT2</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'OUT2'
	 */
	public boolean getOUT2()
	{
		return getBooleanIOValue("OUT2", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>OUT2</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'OUT2'
	 */
	public void setOUT2(java.lang.Boolean value)
	{
		setDigitalOutput("OUT2", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>OUT3</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'OUT3'
	 */
	public boolean getOUT3()
	{
		return getBooleanIOValue("OUT3", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>OUT3</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'OUT3'
	 */
	public void setOUT3(java.lang.Boolean value)
	{
		setDigitalOutput("OUT3", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>OUT4</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'OUT4'
	 */
	public boolean getOUT4()
	{
		return getBooleanIOValue("OUT4", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>OUT4</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'OUT4'
	 */
	public void setOUT4(java.lang.Boolean value)
	{
		setDigitalOutput("OUT4", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>OUT5</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'OUT5'
	 */
	public boolean getOUT5()
	{
		return getBooleanIOValue("OUT5", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>OUT5</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'OUT5'
	 */
	public void setOUT5(java.lang.Boolean value)
	{
		setDigitalOutput("OUT5", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>GripperClose</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'GripperClose'
	 */
	public boolean getGripperClose()
	{
		return getBooleanIOValue("GripperClose", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>GripperClose</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'GripperClose'
	 */
	public void setGripperClose(java.lang.Boolean value)
	{
		setDigitalOutput("GripperClose", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>GripperOpen</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'GripperOpen'
	 */
	public boolean getGripperOpen()
	{
		return getBooleanIOValue("GripperOpen", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>GripperOpen</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'GripperOpen'
	 */
	public void setGripperOpen(java.lang.Boolean value)
	{
		setDigitalOutput("GripperOpen", value);
	}

}
