package ioTool;

import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.ExternalIOGroup;
import com.kuka.roboticsAPI.controllerModel.Controller;

public class ExGripper extends ExternalIOGroup {

	public ExGripper(Controller controller) {
		super(controller);
	}
	
	public void gripperOpen() {
		setOUT02(true);
		ThreadUtil.milliSleep(500);
		setOUT02(false);
	}
	
	public void gripperClose() {
		setOUT01(true);
		ThreadUtil.milliSleep(500);
		setOUT01(false);
	}

}
