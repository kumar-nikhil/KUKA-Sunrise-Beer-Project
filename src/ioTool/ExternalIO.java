package ioTool;

import java.util.concurrent.TimeUnit;

import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.ExternalIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.conditionModel.BooleanIOCondition;
import com.kuka.roboticsAPI.conditionModel.ICondition;
import com.kuka.roboticsAPI.controllerModel.Controller;

public class ExternalIO extends ExternalIOGroup {
	
	private RoboticsAPIApplication _app;
	
	// constructor
	public ExternalIO(Controller controller) {
		super(controller);
	}
	
	public ExternalIO(Controller controller, RoboticsAPIApplication _app) {
		super(controller);
		this._app = _app;
	}
	// endof constructor

	public void gripperOpen() {
//		setGripperOpen(true);
		setOUT1(true);
		ThreadUtil.milliSleep(50);
//		setGripperOpen(false);
		setOUT1(false);
	}
	
	public void gripperClose() {
//		setGripperClose(true);
		setOUT2(true);
		ThreadUtil.milliSleep(50);
//		setGripperClose(false);
		setOUT2(false);
	}
	
	public int buttonWait() {

		// with sensor input
		BooleanIOCondition in1 = new BooleanIOCondition(getInput("IN1"), true);
		BooleanIOCondition in2 = new BooleanIOCondition(getInput("IN2"), true);
		BooleanIOCondition in3 = new BooleanIOCondition(getInput("IN3"), true);
		BooleanIOCondition in4 = new BooleanIOCondition(getInput("IN4"), true);
		BooleanIOCondition in5 = new BooleanIOCondition(getInput("IN5"), true);
		ICondition compC = in1.or(in2).or(in3).or(in4).or(in5);
				
		boolean ret = _app.getObserverManager().waitFor(compC, -1, TimeUnit.MILLISECONDS);
		int val = 10;
		
		if (ret) {
			if ( _app.getObserverManager().evaluate(in1) ) {
				ThreadUtil.milliSleep(300);
				if ( _app.getObserverManager().evaluate(in1) ) {
					_app.getLogger().info("Long IN1 HIGH");
					val = 5;
				} else {
					_app.getLogger().info("IN1 HIGH");
					val = 0;
				}
			} else if ( _app.getObserverManager().evaluate(in2) ) {
				ThreadUtil.milliSleep(300);
				if ( _app.getObserverManager().evaluate(in2) ) {
					_app.getLogger().info("Long IN2 HIGH");
					val = 6;
				} else {
					_app.getLogger().info("IN2 HIGH");
					val = 1;
				}
			} else if ( _app.getObserverManager().evaluate(in3) ) {
				ThreadUtil.milliSleep(300);
				if ( _app.getObserverManager().evaluate(in3) ) {
					_app.getLogger().info("Long IN3 HIGH");
					val = 7;
				} else {
					_app.getLogger().info("IN3 HIGH");
					val = 2;
				}
			} else if ( _app.getObserverManager().evaluate(in4) ) {
				ThreadUtil.milliSleep(300);
				if ( _app.getObserverManager().evaluate(in4) ) {
					_app.getLogger().info("Long IN4 HIGH");
					val = 8;
				} else {
					_app.getLogger().info("IN4 HIGH");
					val = 3;
				}
			} else if ( _app.getObserverManager().evaluate(in5) ) {
				ThreadUtil.milliSleep(300);
				if ( _app.getObserverManager().evaluate(in5) ) {
					_app.getLogger().info("Long IN5 HIGH");
					val = 9;
				} else {
					_app.getLogger().info("IN5 HIGH");
					val = 4;
				}
			}
		} else {
			val = 10;
		}
		return val;
	}
	
	public int buttonEvaluate() {
		// with sensor input
		BooleanIOCondition in1 = new BooleanIOCondition(getInput("IN1"), true);
		BooleanIOCondition in2 = new BooleanIOCondition(getInput("IN2"), true);
		BooleanIOCondition in3 = new BooleanIOCondition(getInput("IN3"), true);
		BooleanIOCondition in4 = new BooleanIOCondition(getInput("IN4"), true);
		BooleanIOCondition in5 = new BooleanIOCondition(getInput("IN5"), true);
		ICondition compC = in1.or(in2).or(in3).or(in4).or(in5);
		
		boolean ret = _app.getObserverManager().evaluate(compC);
		int val = 10;

		if (ret) {
			if (_app.getObserverManager().evaluate(in1)) {
					_app.getLogger().info("IN1 HIGH");
					val = 0;
			} else if (_app.getObserverManager().evaluate(in2)) {
					_app.getLogger().info("IN2 HIGH");
					val = 1;
			} else if (_app.getObserverManager().evaluate(in3)) {
					_app.getLogger().info("IN3 HIGH");
					val = 2;
			} else if (_app.getObserverManager().evaluate(in4)) {
					_app.getLogger().info("IN4 HIGH");
					val = 3;
			} else if (_app.getObserverManager().evaluate(in5)) {
				_app.getLogger().info("IN5 HIGH");
				val = 4;
			}
		} else {
			val = 10;
		}
		return val;
	}
	
}
