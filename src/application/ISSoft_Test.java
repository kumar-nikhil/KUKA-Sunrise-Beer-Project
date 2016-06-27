package application;


import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

/**
 * @author Seulki-Kim , Jun 27, 2016 , KUKA Robotics Korea<p>
 * @modified Seulki-Kim , Jun 27, 2016 , KUKA Robotics Korea<p>
 * Implementation of a robot application.
 * <p>
 * The application provides a {@link RoboticsAPITask#initialize()} and a 
 * {@link RoboticsAPITask#run()} method, which will be called successively in 
 * the application lifecycle. The application will terminate automatically after 
 * the {@link RoboticsAPITask#run()} method has finished or after stopping the 
 * task. The {@link RoboticsAPITask#dispose()} method will be called, even if an 
 * exception is thrown during initialization or run. 
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the 
 * {@link RoboticsAPITask#dispose()} method.</b> 
 * 
 * @see UseRoboticsAPIContext
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class ISSoft_Test extends RoboticsAPIApplication {
	private Controller	cabinet;
	private LBR			lbr;
	// Tool
	private Tool		tool;
	private ObjectFrame	tcp;
	// flag
	private boolean			loopFlag;
	// Frames
	private JointPosition	home;
	private ObjectFrame		product01, product02, product03;
	private ObjectFrame		p1, p2, p3;
	// CycleTimer
	

	@Override
	public void initialize() {
		cabinet = getController("KUKA_Sunrise_Cabinet_1");
		lbr = (LBR) getDevice(cabinet, "LBR_iiwa_14_R820_1");
		tool = getApplicationData().createFromTemplate("Tool");
		tcp = tool.getFrame("TCP");
		tool.attachTo(lbr.getFlange());
		
		loopFlag = true;
		// frames
		initFrames(null);
		
	}

	private void initFrames(ObjectFrame product) {
		home = new JointPosition(0, Math.toRadians(30), 0, -Math.toRadians(60), 0, Math.toRadians(90), 0);
		product01 = getApplicationData().getFrame("/Product01");
		product02 = getApplicationData().getFrame("/Product02");
		product03 = getApplicationData().getFrame("/Product03");
		//
		if ( product != null ) {
			p1 = product.getChild("P1");
			p2 = product.getChild("P2");
			p3 = product.getChild("P3");			
		}
	}

	@Override
	public void run() {

		while (loopFlag) {
			getLogger().info("Starting the application");

			tcp.move(ptp(home).setJointVelocityRel(1.0));
			
//			triAxialOscillationTest();

			int key = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION,
					"Select what to do", "Product 01", "Product 02", "Product 03", "END");
			
			switch (key) {
			case 0: // Product 01
				getLogger().info("Product 01 selected");
				work(product01);
				break;
			case 1:	// Product 02
				getLogger().info("Product 02 selected");
				work(product02);
				break;
			case 2: // Product 03
				getLogger().info("Product 03 selected");
				work(product03);
				break;
			case 3: // END
				getLogger().info("Ending the application");
				loopFlag = false;
				break;
			}	// end of switch-case
		} // end of while()

		tcp.move(ptp(home).setJointVelocityRel(1.0));
		
		getLogger().info("Ending the application");
		
	}

	private void work(ObjectFrame product) {
		// TODO Auto-generated method stub
		
	}

	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		ISSoft_Test app = new ISSoft_Test();
		app.runApplication();
	}
}
