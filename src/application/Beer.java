package application;


import java.util.ArrayList;
import java.util.List;

import ioTool.ExGripper;

import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.conditionModel.ICallbackAction;
import com.kuka.roboticsAPI.conditionModel.ITriggerAction;
import com.kuka.roboticsAPI.conditionModel.JointTorqueCondition;
import com.kuka.roboticsAPI.conditionModel.MotionPathCondition;
import com.kuka.roboticsAPI.conditionModel.ReferenceType;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.executionModel.IFiredTriggerInfo;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.Workpiece;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

/**
 * @author Seulki-Kim , Sep 7, 2016 , KUKA Robotics Korea<p>
 * @modified Seulki-Kim , Sep 8, 2016 , KUKA Robotics Korea<p>
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
public class Beer extends RoboticsAPIApplication {
	private static final Exception	Exception	= null;
	private Controller			cabinet;
	private LBR					lbr;
	private Tool				tool;
	private ObjectFrame			tcpGrip, tcpTip;
	private Workpiece			bottle, glass, fluid;
	// IO
	private MediaFlangeIOGroup	mfIo;
	private ExGripper			exIO;
	private boolean				loopFlag;
	// Frames
	private JointPosition		home;
	private ObjectFrame			beerBase, glassBase, openerBase, pourBase;
	private ObjectFrame			glassDetect, glassLean, pouring, tempHome;
	private List<ObjectFrame>	pouringSPL;
	// Process data
	private double				aov;
	

	// gripper Open
	MotionPathCondition gOpenC = new MotionPathCondition(ReferenceType.DEST, 0, -300);
	ITriggerAction gOpenAction = new ICallbackAction() {
		@Override
		public void onTriggerFired(IFiredTriggerInfo triggerInformation) {
			exIO.gripperOpen();				
		}
	};
	
	@Override
	public void initialize() {
		cabinet = getController("KUKA_Sunrise_Cabinet_1");
		lbr = getContext().getDeviceFromType(LBR.class);
		// tool
		tool = getApplicationData().createFromTemplate("Tool");
		tcpGrip = tool.getFrame("TCP_Grip");
		tcpTip = tool.getFrame("TCP_Tip");
		// wp
		bottle = getApplicationData().createFromTemplate("Bottle");
		glass = getApplicationData().createFromTemplate("Glass");
		fluid = getApplicationData().createFromTemplate("Fluid");
		// io
		mfIo = new MediaFlangeIOGroup(cabinet);
		exIO = new ExGripper(cabinet);
		// flag
		loopFlag = true;
		// Force process data
		processDataUpdate();
		
		initFrames();
		
		tool.attachTo(lbr.getFlange());
		
		getApplicationControl().clipApplicationOverride(aov);
	}

	private void processDataUpdate() {
//		forceE_1 = getApplicationData().getProcessData("forceE_1").getValue();
//		getApplicationData().getProcessData("forceE_1").setDefaultValue(forceE_1);
		
		aov = getApplicationData().getProcessData("aovClip").getValue();
		getApplicationData().getProcessData("aovClip").setDefaultValue(aov);
	}

	private void initFrames() {
		home = new JointPosition(0, Math.toRadians(30), 0, -Math.toRadians(60), 0, Math.toRadians(90), 0);
		beerBase = getApplicationData().getFrame("/BeerWorld/BeerBase");
		glassBase = getApplicationData().getFrame("/BeerWorld/GlassBase");
		glassDetect = glassBase.getChild("detect");
		openerBase = getApplicationData().getFrame("/BeerWorld/OpenerBase");
		pourBase = getApplicationData().getFrame("/BeerWorld/PourBase");
//		glassLean = pourBase.getChild("GlassJig").getChild("leanPosition");
		glassLean = getApplicationData().getFrame("/BeerWorld/PourBase/GlassJig/leanPosition");
		pouring = pourBase.getChild("Pouring");
		tempHome = getApplicationData().getFrame("/BeerWorld/tempHome");
		
		pouringSPL = new ArrayList<ObjectFrame>();
		pouringSPL.addAll(pouring.getChildren());
	}

	@Override
	public void run() {

		while (loopFlag) {
			
			lbr.move(ptp(home).setJointVelocityRel(1.0));
			
			int ret = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "To do?", "Start", "END");
			switch (ret) {
			case 0:
				getLogger().info("Starting application");
				beerApp();
				break;
			case 1:
				getLogger().info("Ending the application");
				loopFlag = false;
				break;
			}	// end of switch
						
		}	// end of while

		lbr.move(ptp(home).setJointVelocityRel(1.0));
		
	}

	private void beerApp() {
		
		try {
			// glass preparation
			getGlass();
			putGlass();
			
			// bottle preparation
			getBottle();
			openBottle();
			
			// pouring
			pourBeer();
			
			// trashing bottle
			trashBottle();
			
			// serving glass
			serveGlass();
			
		} catch (Exception e) {
			e.printStackTrace();
			getLogger().error("!!!!! [ Application failed ] !!!!!");
		}
		
		
		
	}


	private void getGlass() throws Exception {
		getLogger().info("Getting a glass");
		// move in (glassBase)
		Frame detectAir = glassDetect.copyWithRedundancy();
		detectAir.transform(World.Current.getRootFrame(), Transformation.ofTranslation(0, 0, 350));
		Frame detectApr = glassDetect.copyWithRedundancy();
		detectApr.transform(glassBase, Transformation.ofTranslation(0, 0, -30));
		
		tcpTip.moveAsync(ptp(detectAir).setJointVelocityRel(1.0).setBlendingRel(0.2)
				.triggerWhen(gOpenC, gOpenAction) );
		tcpTip.move(lin(detectApr).setCartVelocity(1000));
		
		// detect glass position
		double j1t = lbr.getExternalTorque().getSingleTorqueValue(JointEnum.J1);
		getLogger().info(String.format("current Torque on J1 : %.2f", j1t) );
		JointTorqueCondition j1tc = new JointTorqueCondition(JointEnum.J1, j1t-1.5, j1t+1.5);
		CartesianImpedanceControlMode detectCICM = new CartesianImpedanceControlMode();
		detectCICM.parametrize(CartDOF.Z).setStiffness(200);
		
		IMotionContainer mc = tcpTip.move(linRel(0, 0, 500, glassBase).setCartVelocity(30).setMode(detectCICM).breakWhen(j1tc));
		
		Frame target = null;
		// move & grasp
		if ( mc.hasFired(j1tc) ) {
			getLogger().error("Glass detected");
			double detectedZOffset = lbr.getCurrentCartesianPosition(tcpTip, glassBase).getZ();
			target = glassBase.copyWithRedundancy();
			
			target.transform(glassBase, Transformation.ofTranslation(0, 0, detectedZOffset+37));
			Frame targetGripApr = target.copyWithRedundancy();
			targetGripApr.transform(glassBase, Transformation.ofDeg(0, 0, 0, 0, -20, 0));
			Frame targetAir = target.copyWithRedundancy();
			targetAir.transform(glassBase, Transformation.ofDeg(0, 0, -70, 0, -20, 0));
			
			tcpTip.move(ptp(lbr.getCurrentCartesianPosition(tcpTip)));
			tcpTip.moveAsync(linRel(0, 0, -50).setCartVelocity(1000).setBlendingRel(0.1));
			tcpGrip.moveAsync(lin(targetAir).setJointVelocityRel(1.0).setBlendingRel(0.1));
			tcpGrip.moveAsync(lin(targetGripApr).setCartVelocity(600).setBlendingRel(0.1));
			tcpGrip.move(lin(target).setCartVelocity(300));
			
			exIO.gripperClose();
			glass.attachTo(tcpGrip);
		} else {
			getLogger().error("!!!!! [ No glasses ] !!!!!");
			throw Exception;
		}
		
		// move out
		Frame targetAir = target.copyWithRedundancy();
		targetAir.transform(World.Current.getRootFrame(), Transformation.ofTranslation(0,0,200));
		Frame glassJigAir = pourBase.copyWithRedundancy();
		glassJigAir.transform(World.Current.getRootFrame(), Transformation.ofTranslation(0,0,200));
		
		tcpGrip.moveAsync(lin(targetAir).setCartVelocity(1000));
		tcpGrip.moveAsync(ptp(glassJigAir).setJointVelocityRel(1.0).setBlendingRel(0.1));

	}

	private void putGlass() throws Exception {
		getLogger().info("Placing a glass");
		// move in (pourGlass) 
		Frame air = glassLean.copyWithRedundancy();
		air.transform(World.Current.getRootFrame(), Transformation.ofTranslation(0,0,50));
		tcpGrip.moveAsync(lin(air).setJointVelocityRel(1.0).setBlendingRel(0.1));
		tcpGrip.move(lin(glassLean).setCartVelocity(500));
		
		// detect floor (z)
		double j2t = lbr.getExternalTorque().getSingleTorqueValue(JointEnum.J2);
		getLogger().info(String.format("current Torque on J2 : %.2f", j2t) );
		JointTorqueCondition j2tc = new JointTorqueCondition(JointEnum.J2, j2t-3.0, j2t+3.0);
		CartesianImpedanceControlMode dFloorCICM = new CartesianImpedanceControlMode();
		dFloorCICM.parametrize(CartDOF.Z).setStiffness(200);
		dFloorCICM.setReferenceSystem(World.Current.getRootFrame());
		
		IMotionContainer mc = tcpGrip.move(linRel(0, 0, -80, World.Current.getRootFrame()).setCartVelocity(200).setMode(dFloorCICM).breakWhen(j2tc));
		if ( mc.hasFired(j2tc) ) {
			
		} else {
			getLogger().error("!!!!! [ Floor detection failed ] !!!!!");
			throw Exception;
		}
		
		
		// leaning & detect hole (y)
		
		// release
		
		// move out

//		throw Exception;
	}

	private void getBottle() throws Exception {
		getLogger().info("Getting a bottle");
		// move in (bootleBase)
		
		// detect bottle position
		
		// move & grasp
		
		// move out

//		throw Exception;
	}

	private void openBottle() throws Exception {
		getLogger().info("Opening a bottle cap");
		// move in (openerBase) 
		
		// detect contact
		
		// opening motion
		
		// move out

//		throw Exception;
	}

	private void pourBeer() throws Exception {
		getLogger().info("Pouring beer");
		// move in (pouring)
		
		// pouring motion
		
		// shaking motion
		
		// pouring bottom-up
		
		// move out

//		throw Exception;
	}

	private void trashBottle() throws Exception {
		getLogger().info("Trashing bottle");
		// move in (trashBeer)
		
		// release
		
		// move out

//		throw Exception;
	}

	private void serveGlass() throws Exception {
		getLogger().info("Serving glass");
		// move in (pourGlass) 
		
		// grasp
		
		// move out
		
		// move to serve position & wait
		
		// move out (home)

//		throw Exception;
	}
	
}
