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
import com.kuka.roboticsAPI.motionModel.OrientationReferenceSystem;
import com.kuka.roboticsAPI.motionModel.Spline;
import com.kuka.roboticsAPI.motionModel.SplineJP;
import com.kuka.roboticsAPI.motionModel.SplineOrientationType;
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
	// switch enum
	private enum Con		{ Electric, Oil_Big, Oil_Small };
	// Frames
	private JointPosition	home;
	private ObjectFrame		tempAirAfterElectric, tempAirAfterOil;
	private ObjectFrame		jigCon_Electric, jigCon_Oil_Big, jigCon_Oil_Small;
	private ObjectFrame		jigCon_Electric_aprGrip, jigCon_Electric_aprAsy;
	private ObjectFrame		jigCon_Oil_Big_aprGrip, jigCon_Oil_Big_aprAsy;
	private ObjectFrame		jigCon_Oil_Small_aprGrip, jigCon_Oil_Small_aprAsy;
	
	private ObjectFrame		insertCon_Electric, insertCon_Oil_Big, insertCon_Oil_Small;
	private ObjectFrame		insertCon_Electric_aprGrip, insertCon_Electric_aprAsy;
	private ObjectFrame		insertCon_Oil_Big_aprGrip, insertCon_Oil_Big_aprAsy;
	private ObjectFrame		insertCon_Oil_Small_aprGrip, insertCon_Oil_Small_aprAsy;

	private Frame			pick, pick_aprGrip, pick_aprAsy;
	private Frame			place, place_aprAsy, place_aprGrip;
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
		home = new JointPosition(0, Math.toRadians(30), 0, -Math.toRadians(60), 0, Math.toRadians(90), Math.toRadians(115));
		tempAirAfterElectric = getApplicationData().getFrame("/jigBase/tempAir_AfterElectric");
		tempAirAfterOil = getApplicationData().getFrame("/jigBase/tempAir_AfterOil");
		jigCon_Electric = getApplicationData().getFrame("/jigBase/jigCon_Electric");
		jigCon_Electric_aprAsy = getApplicationData().getFrame("/jigBase/jigCon_Electric/aprAsy");
		jigCon_Electric_aprGrip = getApplicationData().getFrame("/jigBase/jigCon_Electric/aprGrip");
		jigCon_Oil_Big = getApplicationData().getFrame("/jigBase/jigCon_Oil_Big");
		jigCon_Oil_Big_aprAsy = getApplicationData().getFrame("/jigBase/jigCon_Oil_Big/aprAsy");
		jigCon_Oil_Big_aprGrip = getApplicationData().getFrame("/jigBase/jigCon_Oil_Big/aprGrip");
		jigCon_Oil_Small = getApplicationData().getFrame("/jigBase/jigCon_Oil_Small");
		jigCon_Oil_Small_aprAsy = getApplicationData().getFrame("/jigBase/jigCon_Oil_Small/aprAsy");
		jigCon_Oil_Small_aprGrip = getApplicationData().getFrame("/jigBase/jigCon_Oil_Small/aprGrip");
		
		insertCon_Electric = getApplicationData().getFrame("/jigBase/insertCon_Electric");
		insertCon_Electric_aprAsy = getApplicationData().getFrame("/jigBase/insertCon_Electric/aprAsy");
		insertCon_Electric_aprGrip = getApplicationData().getFrame("/jigBase/insertCon_Electric/aprGrip");
		insertCon_Oil_Big = getApplicationData().getFrame("/jigBase/insertCon_Oil_Big");
		insertCon_Oil_Big_aprAsy = getApplicationData().getFrame("/jigBase/insertCon_Oil_Big/aprAsy");
		insertCon_Oil_Big_aprGrip = getApplicationData().getFrame("/jigBase/insertCon_Oil_Big/aprGrip");
		insertCon_Oil_Small = getApplicationData().getFrame("/jigBase/insertCon_Oil_Small");
		insertCon_Oil_Small_aprAsy = getApplicationData().getFrame("/jigBase/insertCon_Oil_Small/aprAsy");
		insertCon_Oil_Small_aprGrip = getApplicationData().getFrame("/jigBase/insertCon_Oil_Small/aprGrip");
		
		
	}
	
	@Override
	public void run() {

		while (loopFlag) {
			getLogger().info("Starting the application");

			tcp.move(ptp(home).setJointVelocityRel(1.0));
			
//			triAxialOscillationTest();

			int key = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION,
					"Select what to do", "Work", "END");
			
			switch (key) {
			case 0:
				getLogger().info("Work selected");
				// Insert
				getLogger().info("Starting Insertion");
//				workInsert(Con.Oil_Big);
//				workInsert(Con.Oil_Small);
				workInsert(Con.Electric);
				
				tcp.move(ptp(home).setJointVelocityRel(1.0));
				// Eject
//				getLogger().info("Starting Ejection");
//				workEject(Con.Oil_Big);
//				workEject(Con.Oil_Small);
//				workEject(Con.Electric);
				break;
			case 1: // END
				getLogger().info("Ending the application");
				loopFlag = false;
				break;
			}	// end of switch-case
		} // end of while()

		tcp.move(ptp(home).setJointVelocityRel(1.0));
		
		getLogger().info("Ending the application");
	}

	private void workEject(Con type) {
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
		tcp.moveAsync(ptp(place_aprAsy).setJointVelocityRel(1.0).setBlendingRel(1.0).triggerWhen(gOpenC, gOpenAction));
		tcp.moveAsync(lin(place_aprGrip).setJointVelocityRel(1.0).setBlendingRel(0.5));
		tcp.move(lin(place).setCartVelocity(500));
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

	private void workInsert(Con type) {
		totalCT.start();
//		exIO.gripperOpen();
		
		switch (type) {
		case Electric:
			getLogger().info("Electric connector");
			pick = jigCon_Electric.copyWithRedundancy();
			pick_aprAsy = jigCon_Electric_aprAsy.copyWithRedundancy();
			pick_aprGrip = jigCon_Electric_aprGrip.copyWithRedundancy();
			place = insertCon_Electric.copyWithRedundancy();
			place_aprAsy = insertCon_Electric_aprAsy.copyWithRedundancy();
			place_aprGrip = insertCon_Electric_aprGrip.copyWithRedundancy();
			break;
		case Oil_Big:
			getLogger().info("Oil_Big connector");
			pick = jigCon_Oil_Big.copyWithRedundancy();
			pick_aprAsy = jigCon_Oil_Big_aprAsy.copyWithRedundancy();
			pick_aprGrip = jigCon_Oil_Big_aprGrip.copyWithRedundancy();
			place = insertCon_Oil_Big.copyWithRedundancy();
			place_aprAsy = insertCon_Oil_Big_aprAsy.copyWithRedundancy();
			place_aprGrip = insertCon_Oil_Big_aprGrip.copyWithRedundancy();
			break;
		case Oil_Small:
			getLogger().info("Oil_Small connector");
			pick = jigCon_Oil_Small.copyWithRedundancy();
			pick_aprAsy = jigCon_Oil_Small_aprAsy.copyWithRedundancy();
			pick_aprGrip = jigCon_Oil_Small_aprGrip.copyWithRedundancy();
			place = insertCon_Oil_Small.copyWithRedundancy();
			place_aprAsy = insertCon_Oil_Small_aprAsy.copyWithRedundancy();
			place_aprGrip = insertCon_Oil_Small_aprGrip.copyWithRedundancy();
			break;
		}
		
		// part picking
		pickCT.start();
		pickPart(type);
		pickCT.end();
		// move out
		getLogger().info("Moving...");
		CartesianImpedanceControlMode gripCICM = new CartesianImpedanceControlMode();
		gripCICM.parametrize(CartDOF.Y).setStiffness(1500);
		gripCICM.parametrize(CartDOF.X, CartDOF.Z).setStiffness(800).setDamping(0.3);
		
		tcp.move(lin(pick_aprAsy).setCartVelocity(500).setMode(gripCICM));
		
		moveJig_To_Insert(type);
		
		//
		insertCT.start();
		insert(type);
		insertCT.end();
		// move out
		getLogger().info("Moving...");
		
		switch (type) {
		case Electric:
			Frame approach = pick_aprGrip.copyWithRedundancy();
			approach.transform(Transformation.ofTranslation(0, 0, -100));
			
			tcp.move(lin(place_aprGrip).setCartVelocity(500));
			tcp.moveAsync(lin(approach).setCartVelocity(500).setBlendingRel(0.2));
			tcp.move(ptp(tempAirAfterElectric).setJointVelocityRel(1.0));
			break;
		case Oil_Big:
			tcp.moveAsync(lin(place_aprGrip).setJointVelocityRel(1.0).setBlendingRel(0.5));
			moveInsert_To_Jig(type);
			break;
		case Oil_Small:
			tcp.moveAsync(lin(place_aprGrip).setJointVelocityRel(1.0).setBlendingRel(0.5));
			moveInsert_To_Jig(type);
			break;
		}
		

		totalCT.end();
	}
	
	private void moveInsert_To_Jig(Con type) {
		SplineJP spl = null;
		switch (type) {
		case Electric:
			break;
		case Oil_Big:
			spl = new SplineJP(
					ptp(getApplicationData().getFrame("/jigBase/SPLJigToInsert_Oil_Big/P4")),
					ptp(getApplicationData().getFrame("/jigBase/SPLJigToInsert_Oil_Big/P3")),
					ptp(getApplicationData().getFrame("/jigBase/SPLJigToInsert_Oil_Big/P2")),
					ptp(getApplicationData().getFrame("/jigBase/SPLJigToInsert_Oil_Big/P1")),
					ptp(tempAirAfterOil)
					/*.setOrientationType(SplineOrientationType.OriJoint)*/ );
			break;
		case Oil_Small:
			spl = new SplineJP(
					ptp(getApplicationData().getFrame("/jigBase/SPLJigToInsert_Oil_Small/P4")),
					ptp(getApplicationData().getFrame("/jigBase/SPLJigToInsert_Oil_Small/P3")),
					ptp(getApplicationData().getFrame("/jigBase/SPLJigToInsert_Oil_Small/P2")),
					ptp(getApplicationData().getFrame("/jigBase/SPLJigToInsert_Oil_Small/P1")),
					ptp(tempAirAfterOil)
					/*.setOrientationType(SplineOrientationType.OriJoint)*/ );
			break;
		}
		tcp.moveAsync(spl.setJointVelocityRel(1.0).setBlendingRel(0.5));
	}

	private void moveJig_To_Insert(Con type) {
		SplineJP spl = null;
		switch (type) {
		case Electric:
			spl = new SplineJP(
					ptp(getApplicationData().getFrame("/jigBase/SPLJigToInsert_Electric/P1")),
					ptp(getApplicationData().getFrame("/jigBase/SPLJigToInsert_Electric/P2")),
					ptp(getApplicationData().getFrame("/jigBase/SPLJigToInsert_Electric/P3")),
					ptp(getApplicationData().getFrame("/jigBase/SPLJigToInsert_Electric/P4")),
					ptp(place_aprAsy)
					/*.setOrientationType(SplineOrientationType.OriJoint)*/ );
			break;
		case Oil_Big:
			spl = new SplineJP(
					ptp(getApplicationData().getFrame("/jigBase/SPLJigToInsert_Oil_Big/P1")),
					ptp(getApplicationData().getFrame("/jigBase/SPLJigToInsert_Oil_Big/P2")),
					ptp(getApplicationData().getFrame("/jigBase/SPLJigToInsert_Oil_Big/P3")),
					ptp(getApplicationData().getFrame("/jigBase/SPLJigToInsert_Oil_Big/P4")),
					ptp(place_aprAsy)
					/*.setOrientationType(SplineOrientationType.OriJoint)*/ );
			break;
		case Oil_Small:
			spl = new SplineJP(
					ptp(getApplicationData().getFrame("/jigBase/SPLJigToInsert_Oil_Small/P1")),
					ptp(getApplicationData().getFrame("/jigBase/SPLJigToInsert_Oil_Small/P2")),
					ptp(getApplicationData().getFrame("/jigBase/SPLJigToInsert_Oil_Small/P3")),
					ptp(getApplicationData().getFrame("/jigBase/SPLJigToInsert_Oil_Small/P4")),
					ptp(place_aprAsy)
					/*.setOrientationType(SplineOrientationType.OriJoint)*/ );
			break;
		}
		tcp.move(spl.setJointVelocityRel(1.0));
	}

	private void pickPart(Con type) {
		// move in
		MotionPathCondition gOpenC = new MotionPathCondition(ReferenceType.DEST, 0, -300);
		ITriggerAction gOpenAction = new ICallbackAction() {
			@Override
			public void onTriggerFired(IFiredTriggerInfo triggerInformation) {
				exIO.gripperOpen();				
			}
		};
		Frame approach = pick_aprGrip.copyWithRedundancy();
		
		getLogger().info("Starting PickPart");
		if ( type == Con.Electric ) {
			approach.transform(Transformation.ofTranslation(0, 0, -60));
			tcp.moveAsync(ptp(approach).setJointVelocityRel(1.0).setBlendingRel(1.0).triggerWhen(gOpenC, gOpenAction));
			tcp.move(lin(pick_aprGrip).setCartVelocity(500));
			tcp.move(lin(pick).setCartVelocity(500));
		} else {
			approach.transform(Transformation.ofTranslation(0, 0, -40));
			tcp.moveAsync(ptp(approach).setJointVelocityRel(1.0).setBlendingRel(1.0).triggerWhen(gOpenC, gOpenAction));
			tcp.moveAsync(lin(pick_aprGrip).setJointVelocityRel(1.0).setBlendingRel(0.5));
			tcp.move(lin(pick).setCartVelocity(500));			
		}
		// pick
		CartesianImpedanceControlMode gripCICM = new CartesianImpedanceControlMode();
		gripCICM.parametrize(CartDOF.Y).setStiffness(1500);
		gripCICM.parametrize(CartDOF.X, CartDOF.Z).setStiffness(800).setDamping(0.3);
		tcp.moveAsync(positionHold(gripCICM, 300, TimeUnit.MILLISECONDS));
		exIO.gripperClose();
	}

	@SuppressWarnings("deprecation")
	private void insert(Con type) {
		getLogger().info("Starting insertion with CICM");
		
		// Condition & CICM
		ForceCondition fC = null;
		CartesianSineImpedanceControlMode insertCSICM = new CartesianSineImpedanceControlMode();
		if ( type == Con.Electric ) {
			fC = ForceCondition.createNormalForceCondition(tcp, CoordinateAxis.Y, 10.0);
			insertCSICM.parametrize(CartDOF.Y).setStiffness(1000);
			insertCSICM.parametrize(CartDOF.ROT).setStiffness(200).setDamping(0.3);
			insertCSICM.parametrize(CartDOF.X).setStiffness(500).setAmplitude(3.0).setFrequency(3);
			insertCSICM.parametrize(CartDOF.Z).setStiffness(500).setAmplitude(3.0).setFrequency(3);
		} else {
			fC = ForceCondition.createNormalForceCondition(tcp, CoordinateAxis.Y, 4.0);
			insertCSICM.parametrize(CartDOF.Y).setStiffness(1000);
			insertCSICM.parametrize(CartDOF.X, CartDOF.Z).setStiffness(300).setDamping(0.3);
			insertCSICM.parametrize(CartDOF.A).setStiffness(100).setAmplitude(10.0).setFrequency(1.5);	
		}
		
		CartesianImpedanceControlMode contactCICM = new CartesianImpedanceControlMode();
		contactCICM.parametrize(CartDOF.Y).setStiffness(1500);
		contactCICM.parametrize(CartDOF.X, CartDOF.Z).setStiffness(800).setDamping(0.3);
		
		
		if ( forceSend ) {
			ftdSenderInsert = new ForceTorqueDataSender(lbr, tcp, "172.31.1.101", 30000, -1, 5,
					Type.FORCETORQUE_XYZABC, "Insert");
			ftdSenderInsert.start();
		}
		// insert
		getLogger().info("Moving until contact occurs");
		IMotionContainer mc = tcp.move(lin(place).setCartVelocity(200).setMode(contactCICM).breakWhen(fC));
		if ( mc.hasFired(fC) ) {
			getLogger().info("Contact made!, trying insertion");
			insertCSICM.setAdditionalControlForce(0, 5, 0, 0, 0, 0);		// 25
			tcp.move(lin(place).setCartVelocity(100).setMode(insertCSICM));
			insertCSICM.setAdditionalControlForceToDefaultValue();
			// evaluate
			if ( ! evaluate(place) ) {	// fail
				getLogger().info("Distance or Force not in range, re-trying with 40N");
				insertCSICM.setAdditionalControlForce(0, 10, 0, 0, 0, 0);	//40
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
		tcp.move(lin(place_aprAsy).setCartVelocity(300).setMode(ejectCSICM));
//		ejectCSICM.setAdditionalControlForceToDefaultValue();

		tcp.moveAsync(ptp(place_aprAsy).setJointVelocityRel(1.0).setBlendingRel(0.5));
		
		getLogger().info("Ejection finished");
		
		if ( forceSend ) {
			ftdSenderEject.interrupt();
		}
	}	

	private void place() {
		getLogger().info("Starting place part");
		tcp.moveAsync(ptp(pick_aprAsy).setJointVelocityRel(1.0).setBlendingRel(1.0));
		tcp.moveAsync(lin(pick_aprGrip).setJointVelocityRel(1.0).setBlendingRel(0.5));
		tcp.move(lin(pick).setCartVelocity(200));
		
		exIO.gripperOpen();
		// move out
		getLogger().info("Moving...");
		tcp.moveAsync(lin(pick_aprGrip).setJointVelocityRel(1.0).setBlendingRel(0.5));
		tcp.move(lin(pick_aprAsy).setJointVelocityRel(1.0).setBlendingRel(1.0));
	}
	
	private boolean evaluate(AbstractFrame target) {
		double dist = lbr.getCurrentCartesianPosition(tcp, place).getZ();
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
