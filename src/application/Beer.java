package application;


import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

import ioTool.ExGripper;

import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.conditionModel.ICallbackAction;
import com.kuka.roboticsAPI.conditionModel.ICondition;
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
import com.kuka.roboticsAPI.geometricModel.math.CoordinateAxis;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.Spline;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianSineImpedanceControlMode;
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
	private ObjectFrame			tcpGrip, tcpTip, tcpBottleCap;
	private Workpiece			bottle, glass, fluid;
	// IO
	private MediaFlangeIOGroup	mfIo;
	private ExGripper			exIO;
	private boolean				loopFlag;
	// Frames
	private JointPosition		home;
	private ObjectFrame			beerBase, glassBase, openerBase, pourBase;
	private ObjectFrame			glassDetect, glassLean, pouring, tempHome;
	private List<ObjectFrame>	beers;
	private List<ObjectFrame>	pouringSPL;
	private List<ObjectFrame>	bottomUpSPL;
	private List<ObjectFrame>	moveOutSPL;
	// Process data
	private double				aov;
	

	// gripper Open
	MotionPathCondition gOpenC = new MotionPathCondition(ReferenceType.DEST, 0, -300);
	ITriggerAction gOpenAction = new ICallbackAction() {
		@Override
		public void onTriggerFired(IFiredTriggerInfo triggerInformation) {
			exIO.gripperOpen();
			lbr.detachAll();
			tool.attachTo(lbr.getFlange());
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
		tcpBottleCap = tool.getFrame("TCP_BottleCap");
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
		
		beers = new ArrayList<ObjectFrame>();
		beers.addAll(beerBase.getChildren());
		pouringSPL = new ArrayList<ObjectFrame>();
		pouringSPL.addAll(pouring.getChildren());
		bottomUpSPL = new ArrayList<ObjectFrame>();
		bottomUpSPL.addAll(getApplicationData().getFrame("/BeerWorld/PourBase/BottomUp").getChildren());
		moveOutSPL = new ArrayList<ObjectFrame>();
		moveOutSPL.addAll(getApplicationData().getFrame("/BeerWorld/PourBase/MoveOut").getChildren());
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
//			getGlass();
//			putGlass();
			// finished
			
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
			getLogger().info("Glass detected");
			double detectedZOffset = lbr.getCurrentCartesianPosition(tcpTip, glassBase).getZ();
			getLogger().info("detectedZOffset from Base : " + (detectedZOffset+64) );
			target = glassBase.copyWithRedundancy();
			target.transform(glassBase, Transformation.ofTranslation(0, 0, (detectedZOffset+64) ));
			
			Frame targetGripApr = target.copyWithRedundancy();
			targetGripApr.transform(target, Transformation.ofDeg(-30, 0, 0, 0, -20, 0));
			Frame targetAir02 = target.copyWithRedundancy();
			targetAir02.transform(target, Transformation.ofDeg(-30, 0, -70, 0, -20, 0));
			Frame targetAir01 = target.copyWithRedundancy();
			targetAir01.transform(target, Transformation.ofDeg(0, 0, -70, 0, -20, 0));
			
			tcpTip.move(ptp(lbr.getCurrentCartesianPosition(tcpTip)));
			tcpTip.moveAsync(linRel(50, 0, -50, glassBase).setCartVelocity(1000).setBlendingRel(0.1));
			tcpGrip.moveAsync(lin(targetAir01).setJointVelocityRel(1.0).setBlendingRel(0.1));
			tcpGrip.moveAsync(lin(targetAir02).setJointVelocityRel(1.0).setBlendingRel(0.1));
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
		tcpGrip.move(lin(air).setJointVelocityRel(1.0));

		CartesianImpedanceControlMode dFloorCICM = new CartesianImpedanceControlMode();
		dFloorCICM.parametrize(CartDOF.Z).setStiffness(200);
		dFloorCICM.setReferenceSystem(World.Current.getRootFrame());
		
		tcpGrip.move(lin(glassLean).setCartVelocity(50).setMode(dFloorCICM));
		tcpGrip.move(ptp(lbr.getCurrentCartesianPosition(tcpGrip)));
		/*
		// detect floor (z)
		double j2t = lbr.getExternalTorque().getSingleTorqueValue(JointEnum.J2);
		getLogger().info(String.format("current Torque on J2 : %.2f", j2t) );
		JointTorqueCondition j2tc = new JointTorqueCondition(JointEnum.J2, j2t-2.0, j2t+2.0);
		CartesianImpedanceControlMode dFloorCICM = new CartesianImpedanceControlMode();
		dFloorCICM.parametrize(CartDOF.Z).setStiffness(200);
		dFloorCICM.setReferenceSystem(World.Current.getRootFrame());
		
		IMotionContainer mc = tcpGrip.move(linRel(0, 0, -80, World.Current.getRootFrame()).setCartVelocity(50).setMode(dFloorCICM).breakWhen(j2tc));
		if ( mc.hasFired(j2tc) ) {
			getLogger().info("Floor detected");
			
		} else {
			getLogger().error("!!!!! [ Floor detection failed ] !!!!!");
			throw Exception;
		}
		*/
		
		// leaning & detect hole (y)
		
		// release
		exIO.gripperOpen();
		lbr.detachAll();
		tool.attachTo(lbr.getFlange());
		
		ThreadUtil.milliSleep(500);
		// move out
		tcpTip.moveAsync(linRel(0, 0, -50, pourBase).setCartVelocity(600).setBlendingRel(0.1));
		tcpTip.moveAsync(linRel(50, 0, -50, pourBase).setCartVelocity(1000).setBlendingRel(0.1));
		tcpGrip.move(ptp(tempHome).setJointVelocityRel(1.0));

//		throw Exception;
	}

	private void getBottle() throws Exception {
		getLogger().info("Getting a bottle");
		// move in (bootleBase)
		Frame beerAir = beerBase.copyWithRedundancy();
		beerAir.transform(World.Current.getRootFrame(), Transformation.ofTranslation(0, 0, 300));
		
		tcpGrip.moveAsync(ptp(beerAir).setJointVelocityRel(1.0).setBlendingRel(0.2)
				.triggerWhen(gOpenC, gOpenAction) );
		
		// move & grasp & evaluate Load
		for (int i = 0; i < beers.size(); i++) {
			Frame target = beers.get(i).copyWithRedundancy();
			Frame targetAir = target.copyWithRedundancy();
			targetAir.transform(World.Current.getRootFrame(), Transformation.ofTranslation(0, 0, 200));
			
			tcpGrip.moveAsync(ptp(targetAir).setJointVelocityRel(1.0).setBlendingRel(0.2)
					.triggerWhen(gOpenC, gOpenAction) );
			tcpGrip.move(lin(target).setCartVelocity(600));

			exIO.gripperClose();
			
			tcpGrip.move(lin(targetAir).setCartVelocity(1000));
			
			int key = evaluateLoad();
			
			switch (key) {
			case 0: // Nothing
				getLogger().info("Try next position");
				break;
			case 1:	// Empty bottle
				getLogger().info("The bottle is empty...");
				trashBottle();
				break;
			case 2:	// Full bottle
				getLogger().info("Good to go to open the bottle");
				i = beers.size();
				break;
			case 3:
			case 4:
			case 10:
				getLogger().error("!!!!! Undesired object is gripped !!!!!");
				tcpGrip.move(lin(target).setCartVelocity(600));
				
				exIO.gripperOpen();
				lbr.detachAll();
				tool.attachTo(lbr.getFlange());
				
				tcpGrip.move(lin(targetAir).setCartVelocity(1000));
				
				throw Exception;
			}	// end of sw-case
		}	// end of for
		
		// move out

//		throw Exception;
	}

	private int evaluateLoad() {
		int ret = 0;
		ThreadUtil.milliSleep(500);
		
		double bottleMass = bottle.getLoadData().getMass();
		double fluidMass = fluid.getLoadData().getMass();
		double glassMass = glass.getLoadData().getMass();
		getLogger().info(String.format("Mass - [Bottle : %.03f], [Fluid : %.03f], [Glass : %.03f]",
				bottleMass, fluidMass, glassMass));
		
		double zForce = lbr.getExternalForceTorque(tcpGrip, World.Current.getRootFrame()).getForce().getZ();
		getLogger().info("Rated force is : " + Math.abs(zForce) + " N");
		double load = zForce / 9.8;
		getLogger().info("Rated load is : " + Math.abs(load) + " kg");
		
		if ( Math.abs(load) <= 0.1 ) {	// 0.0 +- 0.1
			getLogger().info("Load evaluation : Nothing");
			ret = 0;
		} else if ( ( Math.abs(load) - Math.abs(bottleMass) ) <= 0.1 ) {	// 0.273 +- 0.1
			getLogger().info("Load evaluation : Empty bottle");
			bottle.attachTo(tcpGrip);
			ret = 1;
		} else if ( ( Math.abs(load) - Math.abs(bottleMass + fluidMass) ) <= 0.15) {	// 0.599 +- 0.1
			getLogger().info("Load evaluation : Full bottle");
			bottle.attachTo(tcpGrip);
			fluid.attachTo(tcpGrip);
			ret = 2;
		} else if ( ( Math.abs(load) - Math.abs(glassMass) ) <= 0.1) {	//	0.450 +- 0.1
			getLogger().info("Load evaluation : Empty glass");
			glass.attachTo(tcpGrip);
			ret = 3;
		} else if ( ( Math.abs(load) - Math.abs(glassMass + fluidMass) ) <= 0.1) {	// 0.776 +- 0.1
			getLogger().info("Load evaluation : Full glass");
			glass.attachTo(tcpGrip);
			fluid.attachTo(tcpGrip);
			ret = 4;
		} else {
			getLogger().info(String.format("Load evaluation : Unknown object, Mass = %.03f", load));
			ret = 10;
		}
		
		return ret;
	}

	private void openBottle() throws Exception {
		getLogger().info("Opening a bottle cap");
		Frame openerAir = openerBase.copyWithRedundancy();
		openerAir.transform(openerBase, Transformation.ofTranslation(10, 10, 0));
		// move in (openerBase) 
		tcpGrip.moveAsync(ptp(openerBase.getChildren().get(0)).setJointVelocityRel(1.0).setBlendingRel(0.1));
		tcpGrip.moveAsync(ptp(openerBase.getChildren().get(1)).setJointVelocityRel(1.0).setBlendingRel(0.1));
		tcpGrip.moveAsync(ptp(openerBase.getChildren().get(2)).setJointVelocityRel(1.0).setBlendingRel(0.1));
		tcpGrip.move(ptp(openerAir).setJointVelocityRel(0.15));
		
		// contact
		CartesianImpedanceControlMode contactCICM = new CartesianImpedanceControlMode();
		contactCICM.parametrize(CartDOF.X, CartDOF.Y).setStiffness(200);
		contactCICM.setReferenceSystem(openerBase);
		
		double fX = lbr.getExternalForceTorque(tcpGrip, openerBase).getForce().getX();
		double fY = lbr.getExternalForceTorque(tcpGrip, openerBase).getForce().getY();
		getLogger().info(String.format("Current Force X : %.03f,  Y : %.03f", fX, fY));
		ForceCondition detectX = ForceCondition.createNormalForceCondition(tcpGrip, openerBase, CoordinateAxis.X, 5.0);
		ForceCondition detectY = ForceCondition.createNormalForceCondition(tcpGrip, openerBase, CoordinateAxis.Y, 5.0);
		tcpGrip.move(linRel(-60, 0, 0, openerBase).setCartVelocity(20).setMode(contactCICM).breakWhen(detectX));
		getLogger().info("X directional contact made");
		ThreadUtil.milliSleep(500);
		tcpGrip.move(linRel(0, -60, 0, openerBase).setCartVelocity(20).setMode(contactCICM).breakWhen(detectX));
		getLogger().info("Y directional contact made");
		ThreadUtil.milliSleep(500);
		
		// opening motion
		lbr.setESMState("2");
		getLogger().info("ESM state chaanged into 2");
		CartesianImpedanceControlMode openCICM = new CartesianImpedanceControlMode();
		openCICM.parametrize(CartDOF.X, CartDOF.Y).setStiffness(200);
		openCICM.setReferenceSystem(openerBase);

		openCICM.setAdditionalControlForce(-10, -10, 0, 0, 0, 0);
		tcpBottleCap.move(linRel(0, 0, 0, Math.toRadians(30), 0, 0).setMode(openCICM));
		
		// move out
		tcpGrip.moveAsync(linRel(0, 0, -50, World.Current.getRootFrame()).setCartVelocity(100).setBlendingRel(0.1));
		tcpGrip.move(linRel(0, -50, 0, World.Current.getRootFrame()).setCartVelocity(100));

		lbr.setESMState("1");
		getLogger().info("ESM state chaanged into 1");
		
//		throw Exception;
	}

	private void pourBeer() throws Exception {
		getLogger().info("Pouring beer");
		// move in (pouring)
		Frame pouringAir = pouring.copyWithRedundancy();
		pouringAir.transform(World.Current.getRootFrame(), Transformation.ofTranslation(0, 30, 30));
		tcpTip.moveAsync(lin(pouringAir).setJointVelocityRel(0.3).setBlendingRel(0.2));
		tcpTip.move(lin(pouring).setJointVelocityRel(0.3));
		
		// pouring motion
		Spline pouringSpl = new Spline(
				spl(pouringSPL.get(0)),
				spl(pouringSPL.get(1)),
				spl(pouringSPL.get(2)).setOrientationVelocity(0.15),
				spl(pouringSPL.get(3)).setOrientationVelocity(0.15),
				spl(pouringSPL.get(4)),
				spl(pouringSPL.get(5)),
				spl(pouringSPL.get(6))
				).setOrientationVelocity(0.3).setJointVelocityRel(0.2);
		tcpTip.move(pouringSpl);
		ThreadUtil.milliSleep(500);
		
		// shaking motion
		getLogger().info("Shaking beer");
		CartesianSineImpedanceControlMode shakingCSICM = new CartesianSineImpedanceControlMode();
		shakingCSICM.parametrize(CartDOF.C).setStiffness(300).setAmplitude(5.0).setFrequency(0.5);
		shakingCSICM.parametrize(CartDOF.B).setStiffness(300).setAmplitude(5.0).setFrequency(0.5).setPhaseDeg(90);
		shakingCSICM.setReferenceSystem(World.Current.getRootFrame());
		
		tcpTip.move(positionHold(shakingCSICM, 6, TimeUnit.SECONDS));
		
		// pouring bottom-up
		getLogger().info("Finishing pouring");
		Spline buSpl = new Spline(
				spl(bottomUpSPL.get(0)),
				spl(bottomUpSPL.get(1)).setOrientationVelocity(0.1)
				).setOrientationVelocity(0.3).setJointVelocityRel(0.5);
		tcpTip.move(buSpl);
		ThreadUtil.milliSleep(500);

		CartesianSineImpedanceControlMode shakingCSICM2 = new CartesianSineImpedanceControlMode();
		shakingCSICM2.parametrize(CartDOF.Z).setStiffness(300).setAmplitude(5.0).setFrequency(1);
		shakingCSICM2.setReferenceSystem(World.Current.getRootFrame());
		
		tcpTip.move(positionHold(shakingCSICM2, 3, TimeUnit.SECONDS));
		
		
		// move out
		getLogger().info("Moving out");
		Spline moSPL = new Spline(
				spl(moveOutSPL.get(0)),
				spl(moveOutSPL.get(1))
				).setJointVelocityRel(1.0);
		tcpTip.move(moSPL);
		ThreadUtil.milliSleep(500);
		

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
