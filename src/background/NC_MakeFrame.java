package background;


import java.util.ArrayList;

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPIBackgroundTask;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.persistenceModel.IPersistenceEngine;
import com.kuka.roboticsAPI.persistenceModel.XmlApplicationDataSource;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyAlignment;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyLED;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyLEDSize;

/**
 * @author Seulki-Kim , Jun 20, 2016 , KUKA Robotics Korea<p>
 * @modified Seulki-Kim , Jun 20, 2016 , KUKA Robotics Korea<p>
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
 * @see UseRoboticsAPIContext
 * 
 */
public class NC_MakeFrame extends RoboticsAPIBackgroundTask {
	@SuppressWarnings("unused")
	private Controller		cabinet;
	ArrayList<ObjectFrame>	frames;

	public void initialize() {
		cabinet = getController("KUKA_Sunrise_Cabinet_1");
	}

	public void run() {
		exIoKeyBar();
		while(true) {
			ThreadUtil.milliSleep(1000*60*60*24);			
		}
	}
	

	// User Key Bar
	public void exIoKeyBar() {
		IUserKeyListener keyListener = new IUserKeyListener() {
			
			@Override
			public void onKeyEvent(IUserKey key, UserKeyEvent event) {
				if ( event == UserKeyEvent.KeyDown && key.getSlot() == 0 ) {
					key.setLED(UserKeyAlignment.Middle, UserKeyLED.Red, UserKeyLEDSize.Normal);
					int cnt = checkFrames();
					ThreadUtil.milliSleep(400);
					key.setLED(UserKeyAlignment.Middle, UserKeyLED.Green, UserKeyLEDSize.Normal);
					String name = String.format("P_gen_%03d", cnt);
					makeFrame(name);
					ThreadUtil.milliSleep(400);
					key.setLED(UserKeyAlignment.Middle, UserKeyLED.Grey, UserKeyLEDSize.Normal);
				}
			}
		};
		
		IUserKeyBar frameBar = this.getApplicationUI().createUserKeyBar("Frame");
		IUserKey base = frameBar.addUserKey(0, keyListener, true);
		base.setText(UserKeyAlignment.TopMiddle, "Create");
		base.setLED(UserKeyAlignment.Middle, UserKeyLED.Grey, UserKeyLEDSize.Normal);
		
		frameBar.publish();
	}	// endof user key bar
	
	//
	// methods for make Frame
	//
	public int checkFrames() {
		IPersistenceEngine engine = this.getContext().getEngine(IPersistenceEngine.class);
		XmlApplicationDataSource defaultDataSource = (XmlApplicationDataSource) engine.getDefaultDataSource();
		frames = new ArrayList<ObjectFrame>();
		frames.addAll(defaultDataSource.loadAllFrames());
		int cnt = 1;
		for ( ObjectFrame f : frames ) {
			if ( f.isChildOf(World.Current.getRootFrame()) && f.getName().contains("P_gen_")) {
				cnt++;
			}
		}
		return cnt;
	}
	
	public void makeFrame(String name) {
		IPersistenceEngine engine = this.getContext().getEngine(IPersistenceEngine.class);
		XmlApplicationDataSource defaultDataSource = (XmlApplicationDataSource) engine.getDefaultDataSource();
		ObjectFrame newFrame = null;
		
		newFrame = defaultDataSource.addFrame(name, World.Current.getRootFrame());
		defaultDataSource.saveFile(false);
		newFrame.setAdditionalParameter("UseAsBaseForJogging", false);
	}
	
	
}
