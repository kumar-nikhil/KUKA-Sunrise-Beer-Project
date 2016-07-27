package application;


import java.util.concurrent.TimeUnit;

import ioTool.ExternalIO;

import additionalFunction.CycleTimer;
import additionalFunction.ForceTorqueDataSender;
import additionalFunction.ForceTorqueDataSender.Type;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.conditionModel.ICallbackAction;
import com.kuka.roboticsAPI.conditionModel.ITriggerAction;
import com.kuka.roboticsAPI.conditionModel.MotionPathCondition;
import com.kuka.roboticsAPI.conditionModel.ReferenceType;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.executionModel.IFiredTriggerInfo;
import com.kuka.roboticsAPI.geometricModel.AbstractFrame;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.geometricModel.math.CoordinateAxis;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.Spline;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianSineImpedanceControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

/**
 * @author Seulki-Kim , Jun 20, 2016 , KUKA Robotics Korea<p>
 * @modified Seulki-Kim , Jul 27, 2016 , KUKA Robotics Korea<p>
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
@SuppressWarnings("unused")
public class Kefico extends RoboticsAPIApplication {
	private Controller		cabinet;
	private LBR				lbr;
	// tool
	private Tool			tool;
	private ObjectFrame		tcp;
	// IO
	private ExternalIO		exIO;
	// flag
	private boolean			loopFlag;
	// Frames
	private JointPosition	home;
	private ObjectFrame		partBase, productBase;
	private Frame			partBase_Air, productBase_Air;
	private Frame			partBase_Con, productBase_Con;
	// CycleTimer
	private CycleTimer		totalCT, pickCT, insertCT;
	private CycleTimer		ejectCT, placeCT;
	// FT
	private ForceTorqueDataSender	ftdSenderInsert, ftdSenderEject;
	final boolean			forceSend	= false;

	@Override
	public void initialize() {
		cabinet = getController("KUKA_Sunrise_Cabinet_1");
		lbr = (LBR) getDevice(cabinet, "LBR_iiwa_14_R820_1");
		tool = getApplicationData().createFromTemplate("Tool");
		tcp = tool.getFrame("TCP");

		exIO = new ExternalIO(cabinet, this);
		
		loopFlag = true;
		// frames
		initFrames();
		// CT
		totalCT = new CycleTimer("Total", getLogger());
		pickCT = new CycleTimer("Pick", getLogger());
		insertCT = new CycleTimer("Insert", getLogger());
		placeCT = new CycleTimer("Place", getLogger());
		ejectCT = new CycleTimer("Eject", getLogger());

		tool.attachTo(lbr.getFlange());
//		exIO.gripperClose();
	}

	private void initFrames() {
		home = new JointPosition(0, Math.toRadians(30), 0, -Math.toRadians(60), 0, Math.toRadians(90), 0);
		partBase = getApplicationData().getFrame("/partBase");
		partBase_Air = partBase.copyWithRedundancy();
		partBase_Air.transform(Transformation.ofTranslation(0, 0, -100));
		partBase_Con = partBase.copyWithRedundancy();
		partBase_Con.transform(Transformation.ofTranslation(0, 0, -10));
		
		productBase = getApplicationData().getFrame("/productBase");
		productBase_Air = productBase.copyWithRedundancy();
		productBase_Air.transform(Transformation.ofTranslation(0, 0, -100));
		productBase_Con = productBase.copyWithRedundancy();
		productBase_Con.transform(Transformation.ofTranslation(0, 0, -40));
		
		
	}
	
	@Override
	public void run() {

		while (loopFlag) {
			getLogger().info("Starting the application");

			tcp.move(ptp(home).setJointVelocityRel(1.0));
			
//			triAxialOscillationTest();

			int key = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION,
					"Select what to do", "Insert", "Eject", "END");
			
			switch (key) {
			case 0: // Insert
				getLogger().info("Insert selected");
				workInsert();
				break;
			case 1:	// Eject
				getLogger().info("Eject selected");
				workEject();
				break;
			case 2: // END
				getLogger().info("Ending the application");
				loopFlag = false;
				break;
			}	// end of switch-case
		} // end of while()

		tcp.move(ptp(home).setJointVelocityRel(1.0));
		
		getLogger().info("Ending the application");
	}

	private void workEject() {
		totalCT.start();

		// move in
		MotionPathCondition gOpenC = new MotionPathCondition(ReferenceType.DEST, 0, -100);
		ITriggerAction gOpenAction = new ICallbackAction() {
			@Override
			public void onTriggerFired(IFiredTriggerInfo triggerInformation) {
				exIO.gripperOpen();				
			}
		};
		getLogger().info("Starting PickPart");
		tcp.moveAsync(ptp(productBase_Air).setJointVelocityRel(1.0).setBlendingRel(1.0).triggerWhen(gOpenC, gOpenAction));
		tcp.moveAsync(lin(productBase_Con).setJointVelocityRel(1.0).setBlendingRel(0.5));
		tcp.move(lin(productBase).setCartVelocity(500));
		// pick
		exIO.gripperClose();
		
		// eject
		ejectCT.start();
		eject();
		ejectCT.end();
		
		placeCT.start();
		place();
		placeCT.end();
		
		totalCT.end();
	}

	private void workInsert() {
		totalCT.start();
//		exIO.gripperOpen();
		
		// part picking
		pickCT.start();
		pickPart();
		pickCT.end();
		// move out
		getLogger().info("Moving...");
		tcp.moveAsync(lin(partBase_Con).setJointVelocityRel(1.0).setBlendingRel(0.5));
		tcp.moveAsync(lin(partBase_Air).setJointVelocityRel(1.0).setBlendingRel(1.0));
		tcp.moveAsync(ptp(productBase_Air).setJointVelocityRel(1.0).setBlendingRel(1.0));
		tcp.move(lin(productBase_Con).setJointVelocityRel(1.0));
//		tcp.move(spline(
//				lin(partBase_Con),
//				lin(partBase_Air),
//				spl(productBase_Air),
//				lin(productBase_Con)
//				).setJointVelocityRel(1.0) );
		
		//
		insertCT.start();
		insert();
		insertCT.end();
		// move out
		getLogger().info("Moving...");
		tcp.moveAsync(lin(productBase_Con).setJointVelocityRel(1.0).setBlendingRel(0.5));
		tcp.moveAsync(lin(productBase_Air).setJointVelocityRel(1.0).setBlendingRel(0.5));
		

		totalCT.end();
	}

	private void pickPart() {
		// move in
		MotionPathCondition gOpenC = new MotionPathCondition(ReferenceType.DEST, 0, -100);
		ITriggerAction gOpenAction = new ICallbackAction() {
			@Override
			public void onTriggerFired(IFiredTriggerInfo triggerInformation) {
				exIO.gripperOpen();				
			}
		};
		getLogger().info("Starting PickPart");
		tcp.moveAsync(ptp(partBase_Air).setJointVelocityRel(1.0).setBlendingRel(1.0).triggerWhen(gOpenC, gOpenAction));
		tcp.moveAsync(lin(partBase_Con).setJointVelocityRel(1.0).setBlendingRel(0.5));
		tcp.move(lin(partBase).setCartVelocity(500));
		// pick
		exIO.gripperClose();
	}

	@SuppressWarnings("deprecation")
	private void insert() {		
		/*
		 * Insert motion 확인 가능한 느린 영상 촬영 (POV 30%)
		 */
		
		getLogger().info("Starting insertion with CICM");
		
		// Condition & CICM
		ForceCondition fC = ForceCondition.createNormalForceCondition(tcp, CoordinateAxis.Z, 4.0);
		CartesianImpedanceControlMode contactCICM = new CartesianImpedanceControlMode();
		contactCICM.parametrize(CartDOF.Z).setStiffness(1500);
		contactCICM.parametrize(CartDOF.X, CartDOF.Y).setStiffness(800).setDamping(0.3);
		CartesianSineImpedanceControlMode insertCSICM = new CartesianSineImpedanceControlMode();
		insertCSICM.parametrize(CartDOF.Z).setStiffness(1000);
		insertCSICM.parametrize(CartDOF.X, CartDOF.Y).setStiffness(300).setDamping(0.3);
		insertCSICM.parametrize(CartDOF.A).setStiffness(100).setAmplitude(10.0).setFrequency(1.5);
		
		if ( forceSend ) {
			ftdSenderInsert = new ForceTorqueDataSender(lbr, tcp, "172.31.1.101", 30000, -1, 5,
					Type.FORCETORQUE_XYZABC, "Insert");
			ftdSenderInsert.start();
		}
		// insert
		getLogger().info("Moving until contact occurs");
		IMotionContainer mc = tcp.move(lin(productBase).setCartVelocity(200).setMode(contactCICM).breakWhen(fC));
		if ( mc.hasFired(fC) ) {
			getLogger().info("Contact made!, trying insertion");
			insertCSICM.setAdditionalControlForce(0, 0, 25, 0, 0, 0);
			tcp.move(lin(productBase).setCartVelocity(100).setMode(insertCSICM));
			insertCSICM.setAdditionalControlForceToDefaultValue();
			// evaluate
			if ( ! evaluate(productBase) ) {	// fail
				getLogger().info("Distance or Force not in range, re-trying with 40N");
				insertCSICM.setAdditionalControlForce(0, 0, 40, 0, 0, 0);
				tcp.move(positionHold(insertCSICM, 1000, TimeUnit.MILLISECONDS));
				insertCSICM.setAdditionalControlForceToDefaultValue();
			}	// end of if
		} else {
			getLogger().info("Error on approaching");
		}	// end of if-else
		
		tcp.move(ptp(lbr.getCurrentCartesianPosition(tcp)));
		getLogger().info("Insert finished");
		exIO.gripperOpen();
		
		if ( forceSend ) {
			ftdSenderInsert.interrupt();
		}
		
	}
	
	private void eject() {
		// Condition & CICM
		CartesianSineImpedanceControlMode ejectCSICM = new CartesianSineImpedanceControlMode();
		ejectCSICM.parametrize(CartDOF.Z).setStiffness(1000);
		ejectCSICM.parametrize(CartDOF.X, CartDOF.Y).setStiffness(300).setDamping(0.3);
		ejectCSICM.parametrize(CartDOF.A).setStiffness(100).setAmplitude(10.0).setFrequency(1.5);

		if ( forceSend ) {
			ftdSenderEject = new ForceTorqueDataSender(lbr, tcp, "172.31.1.101", 30000, -1, 5,
					Type.FORCETORQUE_XYZABC, "Eject");
			ftdSenderEject.start();
		}
		// eject
		getLogger().info("Starting ejection with CICM");
//		ejectCSICM.setAdditionalControlForce(0, 0, -10, 0, 0, 0);
		tcp.move(lin(productBase_Air).setCartVelocity(300).setMode(ejectCSICM));
//		ejectCSICM.setAdditionalControlForceToDefaultValue();

		tcp.moveAsync(ptp(productBase_Air).setJointVelocityRel(1.0).setBlendingRel(0.5));
		
		getLogger().info("Ejection finished");
		
		if ( forceSend ) {
			ftdSenderEject.interrupt();
		}
	}	

	private void place() {
		getLogger().info("Starting place part");
		tcp.moveAsync(ptp(partBase_Air).setJointVelocityRel(1.0).setBlendingRel(1.0));
		tcp.moveAsync(lin(partBase_Con).setJointVelocityRel(1.0).setBlendingRel(0.5));
		tcp.move(lin(partBase).setCartVelocity(200));
		
		exIO.gripperOpen();
		// move out
		getLogger().info("Moving...");
		tcp.moveAsync(lin(partBase_Con).setJointVelocityRel(1.0).setBlendingRel(0.5));
		tcp.move(lin(partBase_Air).setJointVelocityRel(1.0).setBlendingRel(1.0));
	}
	
	private boolean evaluate(AbstractFrame target) {
		double dist = lbr.getCurrentCartesianPosition(tcp, productBase).getZ();
		getLogger().info( String.format("Distance to the destination : %03f", dist));
		double force = Math.abs( lbr.getExternalForceTorque(tcp).getForce().getZ() );
		getLogger().info( String.format("Z directional Force : %03f", force));
		boolean ret = false;
		if ( dist >= 0.0 && force < 30 ) {
			ret = true;
		} else {
			ret = false;
		}
		return ret;
	}
	
	
	
	private void triAxialOscillationTest() {
		CartesianSineImpedanceControlMode triMode = new CartesianSineImpedanceControlMode();
		triMode.parametrize(CartDOF.X).setStiffness(5000);
		triMode.parametrize(CartDOF.X).setAmplitude(100.0);
		triMode.parametrize(CartDOF.X).setFrequency(0.9);
		//
		triMode.parametrize(CartDOF.Y).setStiffness(5000);
		triMode.parametrize(CartDOF.Y).setAmplitude(100.0);
		triMode.parametrize(CartDOF.Y).setFrequency(1.0);
		//
		triMode.parametrize(CartDOF.Z).setStiffness(5000);
		triMode.parametrize(CartDOF.Z).setAmplitude(100.0);
		triMode.parametrize(CartDOF.Z).setFrequency(1.1);
		
		lbr.move(positionHold(triMode, 10, TimeUnit.SECONDS));
	}

	
	
	
}
