package background;


import ioTool.ExternalIO;

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPIBackgroundTask;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyAlignment;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyLED;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyLEDSize;

/**
 * Implementation of a background task.
 * <p>
 * The background task provides a {@link RoboticsAPIBackgroundTask#initialize()} 
 * and a {@link RoboticsAPIBackgroundTask#run()} method, which will be called 
 * successively in the task lifecycle.<br>
 * The task will terminate automatically after the <code>run</code> method has 
 * finished or after stopping the task.
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the 
 * {@link RoboticsAPITask#dispose()} method.</b>
 */
public class NC_ExIoBar extends RoboticsAPIBackgroundTask {
	private Controller _cabinet;
	// ExternalIO
	private ExternalIO exIO;
	boolean _pushFlag;
	

	
	public void initialize() {
		_cabinet = getController("KUKA_Sunrise_Cabinet_1");
		// ExternalIO
		exIO = new ExternalIO(_cabinet);
		_pushFlag = false;
	}

	public void run() {
		
		exIoKeyBar();
		while(true) {
			ThreadUtil.milliSleep(1000*60*60);
		}
		
	}
	
	// User Key Bar
	public void exIoKeyBar() {
		IUserKeyListener ledOutListener = new IUserKeyListener() {
			
			@Override
			public void onKeyEvent(IUserKey key, UserKeyEvent event) {
				if (event == UserKeyEvent.KeyDown && _pushFlag == false) {
					key.setText(UserKeyAlignment.TopMiddle, "Open");
					key.setLED(UserKeyAlignment.Middle, UserKeyLED.Red, UserKeyLEDSize.Normal);
					_pushFlag = true;
					
//					_exIo.open();
//					_exIo.setGripperOpen(true);
					exIO.gripperOpen();
//					ThreadUtil.milliSleep(100);
//					_exIo.setGripperOpen(false);
					
				} else if (event == UserKeyEvent.KeyDown && _pushFlag == true) {
					key.setText(UserKeyAlignment.TopMiddle, "Close");
					key.setLED(UserKeyAlignment.Middle, UserKeyLED.Green, UserKeyLEDSize.Normal);
					_pushFlag = false;
					
//					_exIo.close();
//					_exIo.setGripperClose(true);
					exIO.gripperClose();
//					ThreadUtil.milliSleep(100);
//					_exIo.setGripperClose(false);
					
				}
			}
		};
		
		IUserKeyBar exIoBar = this.getApplicationUI().createUserKeyBar("BG ExIO");
		IUserKey ledOut = exIoBar.addUserKey(0, ledOutListener, true);
		ledOut.setText(UserKeyAlignment.TopMiddle, "Open");
		ledOut.setLED(UserKeyAlignment.Middle, UserKeyLED.Red, UserKeyLEDSize.Normal);
		
		
		exIoBar.publish();
	}	// endof user key bar
}