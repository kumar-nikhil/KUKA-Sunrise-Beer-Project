package ioTool;

import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.ExternalIOGroup;
import com.kuka.roboticsAPI.controllerModel.Controller;

public class ExGripper extends ExternalIOGroup {

	public ExGripper(Controller controller) {
		super(controller);
	}
	
	public void gripperOpen() {
		setOUT01(true);
		ThreadUtil.milliSleep(200);
		setOUT01(false);
	}
	
	public void gripperClose() {
		setOUT02(true);
		ThreadUtil.milliSleep(200);
		setOUT02(false);
	}

}
