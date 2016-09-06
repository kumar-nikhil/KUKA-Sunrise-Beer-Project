package application;


import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
import ioTool.ExternalIO;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

import additionalFunction.CycleTimer;
import additionalFunction.ForceTorqueDataSender;
import additionalFunction.ForceTorqueDataSender.Type;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.applicationModel.tasks.UseRoboticsAPIContext;
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
import com.kuka.roboticsAPI.geometricModel.AbstractFrame;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.geometricModel.math.CoordinateAxis;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.SplineJP;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianSineImpedanceControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import com.kuka.task.RoboticsAPITask;

/**
 * @author Seulki-Kim , Jun 20, 2016 , KUKA Robotics Korea<p>
 * @modified Seulki-Kim , Sep 5, 2016 , KUKA Robotics Korea<p>
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
	private Controller	cabinet;
	private LBR			lbr;
	// tool
	private Tool		tool;
	private ObjectFrame	tcp;
	// IO
	private ExternalIO	exIO;
	// flag
	private boolean		loopFlag;

	// switch enum
	private enum Con {
		Electric, Oil_Big, Oil_Small
	};
	
	private enum Work {
		Insert, Eject
	};

	private Work			workType;

	// Frames
	private JointPosition	home;
	private ObjectFrame		tempAirAfterElectric, tempAirAfterOil;
	private ObjectFrame		tempAirAfterElectric_iTj, tempAirAfterOil_iTj;
	private ObjectFrame		jigCon_Electric, jigCon_Oil_Big, jigCon_Oil_Small;
	private ObjectFrame		jigCon_Electric_aprGrip, jigCon_Electric_aprAsy;
	private ObjectFrame		jigCon_Oil_Big_aprGrip, jigCon_Oil_Big_aprAsy;
	private ObjectFrame		jigCon_Oil_Small_aprGrip, jigCon_Oil_Small_aprAsy;

	private ObjectFrame		insertCon_Electric, insertCon_Oil_Big, insertCon_Oil_Small;
	private ObjectFrame		insertCon_Electric_aprGrip, insertCon_Electric_aprAsy;
	private ObjectFrame		insertCon_Oil_Big_aprGrip, insertCon_Oil_Big_aprAsy;
	private ObjectFrame		insertCon_Oil_Small_aprGrip, insertCon_Oil_Small_aprAsy;

	private List<ObjectFrame>	jTi_Electric, jTi_Oil_Big, jTi_Oil_Small;
	private List<ObjectFrame>	j_ElectricTI_Oil_Small, homeToJig_OilBig;

	private Frame				pick, pick_aprGrip, pick_aprAsy;
	private Frame				place, place_aprAsy, place_aprGrip;
	// CycleTimer
	private CycleTimer			processCT, pickCT, insertCT;
	private CycleTimer			ejectCT, placeCT;
	private CycleTimer			TotalCT;
	// FT
	private ForceTorqueDataSender	ftdSenderInsert, ftdSenderEject;
	final boolean					forceSend	= false;
	// Force process data
	private double					forceE_1, forceE_2, forceOB_1, forceOB_2, forceOS_1, forceOS_2;
	
	
	@Override
	public void initialize() {
		cabinet = getController("KUKA_Sunrise_Cabinet_1");
		lbr = (LBR) getDevice(cabinet, "LBR_iiwa_14_R820_1");
		tool = getApplicationData().createFromTemplate("Tool");
		tcp = tool.getFrame("TCP");

		exIO = new ExternalIO(cabinet, this);
		
		loopFlag = true;
		workType = Work.Insert;
		// frames
		initFrames();
		// CT
		TotalCT = new CycleTimer("Total", getLogger());
		processCT = new CycleTimer("1 Process", getLogger());
		pickCT = new CycleTimer("Pick", getLogger());
		insertCT = new CycleTimer("Insert", getLogger());
		placeCT = new CycleTimer("Place", getLogger());
		ejectCT = new CycleTimer("Eject", getLogger());
		// Force process data
		processDataUpdate();
		
		tool.attachTo(lbr.getFlange());
//		exIO.gripperClose();
	}

	private void processDataUpdate() {
		forceE_1 = getApplicationData().getProcessData("forceE_1").getValue();
		getApplicationData().getProcessData("forceE_1").setDefaultValue(forceE_1);
		forceE_2 = getApplicationData().getProcessData("forceE_2").getValue();
		getApplicationData().getProcessData("forceE_2").setDefaultValue(forceE_2);
		forceOB_1 = getApplicationData().getProcessData("forceOB_1").getValue();
		getApplicationData().getProcessData("forceOB_1").setDefaultValue(forceOB_1);
		forceOB_2 = getApplicationData().getProcessData("forceOB_2").getValue();
		getApplicationData().getProcessData("forceOB_2").setDefaultValue(forceOB_2);
		forceOS_1 = getApplicationData().getProcessData("forceOS_1").getValue();
		getApplicationData().getProcessData("forceOS_1").setDefaultValue(forceOS_1);
		forceOS_2 = getApplicationData().getProcessData("forceOS_2").getValue();
		getApplicationData().getProcessData("forceOS_2").setDefaultValue(forceOS_2);
	}

	private void initFrames() {
		home = new JointPosition(0, Math.toRadians(30), 0, -Math.toRadians(60), 0, Math.toRadians(90), Math.toRadians(25));
		tempAirAfterElectric = getApplicationData().getFrame("/jigBase/tempAir_AfterElectric");
		tempAirAfterOil = getApplicationData().getFrame("/jigBase/tempAir_AfterOil");
		tempAirAfterElectric_iTj = getApplicationData().getFrame("/jigBase/tempAir_AfterElectric/insertToJig");
		tempAirAfterOil_iTj = getApplicationData().getFrame("/jigBase/tempAir_AfterOil/insertToJig");
		
		jigCon_Electric = getApplicationData().getFrame("/jigBase/jigCon/jigCon_Electric");
		jigCon_Electric_aprAsy = getApplicationData().getFrame("/jigBase/jigCon/jigCon_Electric/aprAsy");
		jigCon_Electric_aprGrip = getApplicationData().getFrame("/jigBase/jigCon/jigCon_Electric/aprGrip");
		jigCon_Oil_Big = getApplicationData().getFrame("/jigBase/jigCon/jigCon_Oil_Big");
		jigCon_Oil_Big_aprAsy = getApplicationData().getFrame("/jigBase/jigCon/jigCon_Oil_Big/aprAsy");
		jigCon_Oil_Big_aprGrip = getApplicationData().getFrame("/jigBase/jigCon/jigCon_Oil_Big/aprGrip");
		jigCon_Oil_Small = getApplicationData().getFrame("/jigBase/jigCon/jigCon_Oil_Small");
		jigCon_Oil_Small_aprAsy = getApplicationData().getFrame("/jigBase/jigCon/jigCon_Oil_Small/aprAsy");
		jigCon_Oil_Small_aprGrip = getApplicationData().getFrame("/jigBase/jigCon/jigCon_Oil_Small/aprGrip");
		
		insertCon_Electric = getApplicationData().getFrame("/jigBase/insertCon/insertCon_Electric");
		insertCon_Electric_aprAsy = getApplicationData().getFrame("/jigBase/insertCon/insertCon_Electric/aprAsy");
		insertCon_Electric_aprGrip = getApplicationData().getFrame("/jigBase/insertCon/insertCon_Electric/aprGrip");
		insertCon_Oil_Big = getApplicationData().getFrame("/jigBase/insertCon/insertCon_Oil_Big");
		insertCon_Oil_Big_aprAsy = getApplicationData().getFrame("/jigBase/insertCon/insertCon_Oil_Big/aprAsy");
		insertCon_Oil_Big_aprGrip = getApplicationData().getFrame("/jigBase/insertCon/insertCon_Oil_Big/aprGrip");
		insertCon_Oil_Small = getApplicationData().getFrame("/jigBase/insertCon/insertCon_Oil_Small");
		insertCon_Oil_Small_aprAsy = getApplicationData().getFrame("/jigBase/insertCon/insertCon_Oil_Small/aprAsy");
		insertCon_Oil_Small_aprGrip = getApplicationData().getFrame("/jigBase/insertCon/insertCon_Oil_Small/aprGrip");
		
		jTi_Electric = new ArrayList<ObjectFrame>();
		jTi_Electric.addAll(getApplicationData().getFrame("/jigBase/SPL/SPLJigToInsert_Electric").getChildren());
		jTi_Oil_Big = new ArrayList<ObjectFrame>();
		jTi_Oil_Big.addAll(getApplicationData().getFrame("/jigBase/SPL/SPLJigToInsert_Oil_Big").getChildren());
		jTi_Oil_Small = new ArrayList<ObjectFrame>();
		jTi_Oil_Small.addAll(getApplicationData().getFrame("/jigBase/SPL/SPLJigToInsert_Oil_Small").getChildren());
		j_ElectricTI_Oil_Small = new ArrayList<ObjectFrame>();
		j_ElectricTI_Oil_Small.addAll(getApplicationData().getFrame("/jigBase/SPL/SPLJig_ElectricToInsert_Oil_Small").getChildren());
		homeToJig_OilBig = new ArrayList<ObjectFrame>();
		homeToJig_OilBig.addAll(getApplicationData().getFrame("/jigBase/SPL/SPLHomeToOilBigJig").getChildren());
		
	}
	
	@Override
	public void run() {

		while (loopFlag) {
			getLogger().info("Starting the application");

			tcp.move(ptp(home).setJointVelocityRel(1.0));

			int key = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION,
					"Select what to do", "Insert", "Eject", "END");
			
			
			switch (key) {
			case 0:
				TotalCT.start();
				getLogger().info("Insert selected");
				workType = Work.Insert;
				// Insert
				getLogger().info("Starting Insertion");
				workInsert(Con.Oil_Big);
				workInsert(Con.Oil_Small);
				workInsert(Con.Electric);
				TotalCT.end();
				break;
			case 1:
				TotalCT.start();
				getLogger().info("Eject selected");
				workType = Work.Eject;
				// Eject
				getLogger().info("Starting Ejection");
				workEject(Con.Electric);
				workEject(Con.Oil_Small);
				workEject(Con.Oil_Big);
				TotalCT.end();
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

	private void workEject(Con type) {
		processCT.start();

		switch (type) {
		case Electric:
			getLogger().info("Electric connector");
			pick = insertCon_Electric.copyWithRedundancy();
			pick_aprAsy = insertCon_Electric_aprAsy.copyWithRedundancy();
			pick_aprGrip = insertCon_Electric_aprGrip.copyWithRedundancy();
			place = jigCon_Electric.copyWithRedundancy();
			place_aprAsy = jigCon_Electric_aprAsy.copyWithRedundancy();
			place_aprGrip = jigCon_Electric_aprGrip.copyWithRedundancy();
			break;
		case Oil_Big:
			getLogger().info("Oil_Big connector");
			pick = insertCon_Oil_Big.copyWithRedundancy();
			pick_aprAsy = insertCon_Oil_Big_aprAsy.copyWithRedundancy();
			pick_aprGrip = insertCon_Oil_Big_aprGrip.copyWithRedundancy();
			place = jigCon_Oil_Big.copyWithRedundancy();
			place_aprAsy = jigCon_Oil_Big_aprAsy.copyWithRedundancy();
			place_aprGrip = jigCon_Oil_Big_aprGrip.copyWithRedundancy();
			break;
		case Oil_Small:
			getLogger().info("Oil_Small connector");
			pick = insertCon_Oil_Small.copyWithRedundancy();
			pick_aprAsy = insertCon_Oil_Small_aprAsy.copyWithRedundancy();
			pick_aprGrip = insertCon_Oil_Small_aprGrip.copyWithRedundancy();
			place = jigCon_Oil_Small.copyWithRedundancy();
			place_aprAsy = jigCon_Oil_Small_aprAsy.copyWithRedundancy();
			place_aprGrip = jigCon_Oil_Small_aprGrip.copyWithRedundancy();
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
		tcp.move(ptp(lbr.getCurrentCartesianPosition(tcp)).setJointVelocityRel(1.0));
		
		moveInsert_To_Jig(type);
		
		// eject
		ejectCT.start();
		insert_iTj(type);
		ejectCT.end();

		// move out
		getLogger().info("Moving...");
		
		Frame approach = place_aprGrip.copyWithRedundancy();
		switch (type) {
		case Electric:			
			tcp.moveAsync(lin(place_aprGrip).setJointVelocityRel(1.0).setBlendingRel(0.1));

			SplineJP spl = new SplineJP(
					ptp(j_ElectricTI_Oil_Small.get(0)),
					ptp(j_ElectricTI_Oil_Small.get(1)),
					ptp(j_ElectricTI_Oil_Small.get(2)),
					ptp(j_ElectricTI_Oil_Small.get(3)),
					ptp(tempAirAfterElectric_iTj)
					/*.setOrientationType(SplineOrientationType.OriJoint)*/ );
			tcp.moveAsync(spl.setJointVelocityRel(1.0).setBlendingRel(0.2));
			
			break;
		case Oil_Big:
			approach.transform(World.Current.getRootFrame(), Transformation.ofTranslation(0, 0, 250));
			tcp.moveAsync(lin(place_aprGrip).setJointVelocityRel(1.0).setBlendingRel(0.5));
			tcp.moveAsync(lin(approach).setCartVelocity(500).setBlendingRel(0.2));
			
//			tcp.moveAsync(ptp(getApplicationData().getFrame("/jigBase/TempAir_btwBigSmall")).setJointVelocityRel(1.0).setBlendingRel(0.5));
//			tcp.moveAsync(ptp(tempAirAfterOil_iTj).setJointVelocityRel(1.0).setBlendingRel(0.5));
			break;
		case Oil_Small:
			tcp.moveAsync(lin(place_aprGrip).setJointVelocityRel(1.0).setBlendingRel(0.5));
			tcp.moveAsync(ptp(tempAirAfterOil_iTj).setJointVelocityRel(1.0).setBlendingRel(0.5));
			break;
		}
		
		processCT.end();
	}

	private void workInsert(Con type) {
		processCT.start();
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
		gripCICM.parametrize(CartDOF.Y).setStiffness(2500);
		gripCICM.parametrize(CartDOF.X, CartDOF.Z).setStiffness(800).setDamping(0.3);
		
		tcp.move(lin(pick_aprAsy).setCartVelocity(500).setMode(gripCICM));
		tcp.move(ptp(lbr.getCurrentCartesianPosition(tcp)).setJointVelocityRel(1.0));
		
		moveJig_To_Insert(type);
		
		//
		insertCT.start();
		insert(type);
		insertCT.end();
		// move out
		getLogger().info("Moving...");
		
		Frame approach = place_aprGrip.copyWithRedundancy();
		switch (type) {
		case Electric:
			approach.transform(World.Current.getRootFrame(), Transformation.ofTranslation(0, 0, 100));
			tcp.moveAsync(lin(place_aprGrip).setJointVelocityRel(1.0).setBlendingRel(0.1));
			tcp.move(lin(approach).setJointVelocityRel(1.0));
//			tcp.move(ptp(tempAirAfterElectric).setJointVelocityRel(1.0));
			break;
		case Oil_Big:
//			approach.transform(World.Current.getRootFrame(), Transformation.ofTranslation(0, -100, 100));
			tcp.moveAsync(lin(place_aprGrip).setJointVelocityRel(1.0).setBlendingRel(0.1));
			tcp.move(ptp(getApplicationData().getFrame("/jigBase/TempAir_btwBigSmall")).setJointVelocityRel(1.0));
			break;
		case Oil_Small:
			tcp.moveAsync(lin(place_aprGrip).setJointVelocityRel(1.0).setBlendingRel(0.1));
//			approach.transform(Transformation.ofTranslation(0, 0, -90));
//			tcp.moveAsync(lin(approach).setCartVelocity(500).setBlendingRel(0.2));
//			approach.transform(Transformation.ofTranslation(250, -250, -90));
//			tcp.moveAsync(lin(approach).setCartVelocity(500).setBlendingRel(0.2));
//			tcp.move(ptp(tempAirAfterOil).setJointVelocityRel(1.0));

			SplineJP spl = new SplineJP(
					ptp(j_ElectricTI_Oil_Small.get(3)),
					ptp(j_ElectricTI_Oil_Small.get(2)),
					ptp(j_ElectricTI_Oil_Small.get(1)),
					ptp(j_ElectricTI_Oil_Small.get(0))
//					ptp(tempAirAfterElectric_iTj)
					/*.setOrientationType(SplineOrientationType.OriJoint)*/ );
			tcp.moveAsync(spl.setJointVelocityRel(1.0).setBlendingRel(0.2));
			
//			tcp.move( new SplineJP(
//					ptp(jTi_Oil_Small.get(3)),
//					ptp(jTi_Oil_Small.get(2)),
//					ptp(jTi_Oil_Small.get(1)),
//					ptp(jTi_Oil_Small.get(0)),
//					ptp(tempAirAfterOil) )
//			.setJointVelocityRel(1.0));
			break;
		}
		

		processCT.end();
	}
	
	private void moveInsert_To_Jig(Con type) {
		SplineJP spl = null;
		switch (type) {
		case Electric:
			spl = new SplineJP(
					ptp(jTi_Electric.get(3)),
					ptp(jTi_Electric.get(2)),
					ptp(jTi_Electric.get(1)),
					ptp(jTi_Electric.get(0)),
					ptp(place_aprAsy)
					/*.setOrientationType(SplineOrientationType.OriJoint)*/ );
			break;
		case Oil_Big:
			spl = new SplineJP(
					ptp(jTi_Oil_Big.get(3)),
					ptp(jTi_Oil_Big.get(2)),
					ptp(jTi_Oil_Big.get(1)),
					ptp(jTi_Oil_Big.get(0)),
					ptp(place_aprAsy)
					/*.setOrientationType(SplineOrientationType.OriJoint)*/ );
			break;
		case Oil_Small:
			spl = new SplineJP(
					ptp(jTi_Oil_Small.get(3)),
					ptp(jTi_Oil_Small.get(2)),
					ptp(jTi_Oil_Small.get(1)),
					ptp(jTi_Oil_Small.get(0)),
					ptp(place_aprAsy)
					/*.setOrientationType(SplineOrientationType.OriJoint)*/ );
			break;
		}
		tcp.moveAsync(spl.setJointVelocityRel(1.0).setBlendingRel(0.05));
	}

	private void moveJig_To_Insert(Con type) {
		SplineJP spl = null;
		switch (type) {
		case Electric:
			spl = new SplineJP(
					ptp(jTi_Electric.get(0)),
					ptp(jTi_Electric.get(1)),
					ptp(jTi_Electric.get(2)),
					ptp(jTi_Electric.get(3)),
					ptp(place_aprAsy)
					/*.setOrientationType(SplineOrientationType.OriJoint)*/ );
			break;
		case Oil_Big:
			spl = new SplineJP(
					ptp(jTi_Oil_Big.get(0)),
					ptp(jTi_Oil_Big.get(1)),
					ptp(jTi_Oil_Big.get(2)),
					ptp(jTi_Oil_Big.get(3)),
					ptp(place_aprAsy)
					/*.setOrientationType(SplineOrientationType.OriJoint)*/ );
			break;
		case Oil_Small:
			spl = new SplineJP(
					ptp(jTi_Oil_Small.get(0)),
					ptp(jTi_Oil_Small.get(1)),
					ptp(jTi_Oil_Small.get(2)),
					ptp(jTi_Oil_Small.get(3)),
					ptp(place_aprAsy)
					/*.setOrientationType(SplineOrientationType.OriJoint)*/ );
			break;
		}
		tcp.moveAsync(spl.setJointVelocityRel(1.0).setBlendingRel(0.05));
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
		if ( workType == Work.Insert ) {
			if (type == Con.Electric) {
				approach.transform(World.Current.getRootFrame(), Transformation.ofTranslation(0, 0, 150));
				tcp.moveAsync(ptp(approach).setJointVelocityRel(1.0).setBlendingRel(0.5)
						.triggerWhen(gOpenC, gOpenAction));
				tcp.moveAsync(lin(pick_aprGrip).setJointVelocityRel(1.0).setBlendingRel(0.1));
				tcp.move(lin(pick).setCartVelocity(200));
			} else if ( type == Con.Oil_Big ){
				approach.transform(World.Current.getRootFrame(), Transformation.ofTranslation(0, 0, 150));
				tcp.moveAsync(ptp(approach).setJointVelocityRel(1.0).setBlendingRel(0.5)
						.triggerWhen(gOpenC, gOpenAction));
				tcp.moveAsync(lin(pick_aprGrip).setJointVelocityRel(1.0).setBlendingRel(0.1));
				tcp.move(lin(pick).setCartVelocity(200));
			} else if ( type == Con.Oil_Small ){
				tcp.moveAsync(lin(pick_aprGrip).setJointVelocityRel(1.0).setBlendingRel(0.1)
						.triggerWhen(gOpenC, gOpenAction));
				tcp.move(lin(pick).setCartVelocity(200));
			}
		} else if ( workType == Work.Eject ) {
			switch (type) {
			case Electric:
//				tcp.moveAsync(ptp(jTi_Electric.get(4)).setJointVelocityRel(1.0).setBlendingRel(0.5)
//						.triggerWhen(gOpenC, gOpenAction));
//				tcp.moveAsync(ptp(jTi_Electric.get(5)).setJointVelocityRel(1.0).setBlendingRel(0.5));
				tcp.moveAsync(lin(pick_aprGrip).setJointVelocityRel(1.0).setBlendingRel(0.1)
						.triggerWhen(gOpenC, gOpenAction));
				tcp.move(lin(pick).setCartVelocity(200));
				break;
			case Oil_Big:
				tcp.moveAsync(ptp(jTi_Oil_Big.get(4)).setJointVelocityRel(1.0).setBlendingRel(0.5)
						.triggerWhen(gOpenC, gOpenAction));
				tcp.moveAsync(ptp(jTi_Oil_Big.get(5)).setJointVelocityRel(1.0).setBlendingRel(0.5));
				tcp.moveAsync(lin(pick_aprGrip).setJointVelocityRel(1.0).setBlendingRel(0.1));
				tcp.move(lin(pick).setCartVelocity(200));
				break;
			case Oil_Small:
//				tcp.moveAsync(ptp(jTi_Oil_Small.get(4)).setJointVelocityRel(1.0).setBlendingRel(0.5)
//						.triggerWhen(gOpenC, gOpenAction));
//				tcp.moveAsync(ptp(jTi_Oil_Small.get(5)).setJointVelocityRel(1.0).setBlendingRel(0.5));
				tcp.moveAsync(lin(pick_aprGrip).setJointVelocityRel(1.0).setBlendingRel(0.1)
						.triggerWhen(gOpenC, gOpenAction));
				tcp.move(lin(pick).setCartVelocity(200));
				break;
			}
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
		ICondition fC = null;
		double[] force = new double[2];
		CartesianSineImpedanceControlMode insertCSICM = new CartesianSineImpedanceControlMode();
		if ( type == Con.Electric ) {
//			fC = ForceCondition.createNormalForceCondition(tcp, CoordinateAxis.Y, 10.0);
			double tq1 = lbr.getExternalTorque().getSingleTorqueValue(JointEnum.J1);
			fC = new JointTorqueCondition(JointEnum.J1, tq1-3.0, tq1+3.0);
			force[0] = -forceE_1;
			force[1] = -forceE_2;
			insertCSICM.parametrize(CartDOF.Y).setStiffness(2000);
//			insertCSICM.parametrize(CartDOF.ROT).setStiffness(200).setDamping(0.3);
			insertCSICM.parametrize(CartDOF.X).setStiffness(1000).setAmplitude(1.0).setFrequency(3);
			insertCSICM.parametrize(CartDOF.Z).setStiffness(1000).setAmplitude(1.0).setFrequency(3);
		} else if ( type == Con.Oil_Big ) {
			fC = ForceCondition.createNormalForceCondition(tcp, CoordinateAxis.Y, 5.0);
			force[0] = forceOB_1;
			force[1] = forceOB_2;
			insertCSICM.parametrize(CartDOF.Y).setStiffness(2000);
			insertCSICM.parametrize(CartDOF.X, CartDOF.Z).setStiffness(1500).setDamping(0.3);
//			insertCSICM.parametrize(CartDOF.A).setStiffness(200).setAmplitude(5.0).setFrequency(1.5);	
		} else if ( type == Con.Oil_Small ) {
			fC = ForceCondition.createNormalForceCondition(tcp, CoordinateAxis.Y, 5.0);
			force[0] = forceOS_1;
			force[1] = forceOS_2;
			insertCSICM.parametrize(CartDOF.Y).setStiffness(2000);
			insertCSICM.parametrize(CartDOF.X, CartDOF.Z).setStiffness(1500).setDamping(0.3);
		}
		
		CartesianImpedanceControlMode contactCICM = new CartesianImpedanceControlMode();
		contactCICM.parametrize(CartDOF.Y).setStiffness(1000);
		contactCICM.parametrize(CartDOF.X, CartDOF.Z).setStiffness(3000).setDamping(0.3);
		
		
		if ( forceSend ) {
			ftdSenderInsert = new ForceTorqueDataSender(lbr, tcp, "172.31.1.101", 30000, -1, 5,
					Type.FORCETORQUE_XYZABC, "Insert");
			ftdSenderInsert.start();
		}
		// insert
		getLogger().info("Moving until contact occurs");
		IMotionContainer mc = tcp.move(lin(place).setCartVelocity(200).setMode(contactCICM).breakWhen(fC));
		if ( mc.hasFired(fC) ) {
			getLogger().info(String.format("Contact made!, trying insertion with %.02f N", force[0]));
			insertCSICM.setAdditionalControlForce(0, force[0], 0, 0, 0, 0);
			tcp.move(lin(place).setCartVelocity(100).setMode(insertCSICM));
			insertCSICM.setAdditionalControlForceToDefaultValue();
			// evaluate
			if ( ! evaluate(place) ) {	// fail
				getLogger().info(String.format("Distance or Force not in range, re-trying with %.02f N", force[1]));
				insertCSICM.setAdditionalControlForce(0, force[1], 0, 0, 0, 0);
//				tcp.move(positionHold(insertCSICM, 1000, TimeUnit.MILLISECONDS));
				tcp.move(lin(place).setCartVelocity(100).setMode(insertCSICM));
				insertCSICM.setAdditionalControlForceToDefaultValue();
			}	// end of if
		} else {
			getLogger().info("Error on approaching");
		}	// end of if-else

		evaluate(place);
		tcp.move(ptp(lbr.getCurrentCartesianPosition(tcp)));
		getLogger().info("Insert finished");
		exIO.gripperOpen();
		
		if ( forceSend ) {
			ftdSenderInsert.interrupt();
		}
		
	}

	@SuppressWarnings("deprecation")
	private void insert_iTj(Con type) {
		getLogger().info("Starting insertion with CICM");
		
		// Condition & CICM
		ICondition fC = null;
		double[] force = new double[2];
		force[0] = 0; force[1] = 0;
		CartesianSineImpedanceControlMode insertCSICM = new CartesianSineImpedanceControlMode();
		if ( type == Con.Electric ) {
			fC = ForceCondition.createNormalForceCondition(tcp, CoordinateAxis.Y, 5.0);
//			double tq1 = lbr.getExternalTorque().getSingleTorqueValue(JointEnum.J1);
//			fC = new JointTorqueCondition(JointEnum.J1, tq1-4.0, tq1+4.0);
			force[0] = -forceE_1;
			force[1] = -forceE_2;
			insertCSICM.parametrize(CartDOF.Y).setStiffness(2000);
//			insertCSICM.parametrize(CartDOF.ROT).setStiffness(200).setDamping(0.3);
			insertCSICM.parametrize(CartDOF.X).setStiffness(1000).setAmplitude(1.0).setFrequency(3);
			insertCSICM.parametrize(CartDOF.Z).setStiffness(1000).setAmplitude(1.0).setFrequency(3);
		} else if ( type == Con.Oil_Big ) {
			fC = ForceCondition.createNormalForceCondition(tcp, CoordinateAxis.Y, 5.0);
			force[0] = forceOB_1;
			force[1] = forceOB_2;
			insertCSICM.parametrize(CartDOF.Y).setStiffness(2000);
			insertCSICM.parametrize(CartDOF.X, CartDOF.Z).setStiffness(1500).setDamping(0.3);
//			insertCSICM.parametrize(CartDOF.A).setStiffness(200).setAmplitude(5.0).setFrequency(1.5);	
		} else if ( type == Con.Oil_Small ) {
			fC = ForceCondition.createNormalForceCondition(tcp, CoordinateAxis.Y, 5.0);
			force[0] = forceOS_1;
			force[1] = forceOS_2;
			insertCSICM.parametrize(CartDOF.Y).setStiffness(2000);
			insertCSICM.parametrize(CartDOF.X, CartDOF.Z).setStiffness(1500).setDamping(0.3);
		}
		
		CartesianImpedanceControlMode contactCICM = new CartesianImpedanceControlMode();
		contactCICM.parametrize(CartDOF.Y).setStiffness(1000);
		contactCICM.parametrize(CartDOF.X, CartDOF.Z).setStiffness(3000).setDamping(0.3);
		
		
		if ( forceSend ) {
			ftdSenderInsert = new ForceTorqueDataSender(lbr, tcp, "172.31.1.101", 30000, -1, 5,
					Type.FORCETORQUE_XYZABC, "Insert");
			ftdSenderInsert.start();
		}
		// insert
		getLogger().info("Moving until contact occurs");
		IMotionContainer mc = tcp.move(lin(place).setCartVelocity(200).setMode(contactCICM).breakWhen(fC));
		if ( mc.hasFired(fC) ) {
			getLogger().info(String.format("Contact made!, trying insertion with %.02f N", force[0]));
			insertCSICM.setAdditionalControlForce(0, force[0], 0, 0, 0, 0);
			tcp.move(lin(place).setCartVelocity(100).setMode(insertCSICM));
			insertCSICM.setAdditionalControlForceToDefaultValue();
			// evaluate
			if ( ! evaluate(place) ) {	// fail
				getLogger().info(String.format("Distance or Force not in range, re-trying with %.02f N", force[1]));
				insertCSICM.setAdditionalControlForce(0, force[1], 0, 0, 0, 0);
//				tcp.move(positionHold(insertCSICM, 1000, TimeUnit.MILLISECONDS));
				tcp.move(lin(place).setCartVelocity(100).setMode(insertCSICM));
				insertCSICM.setAdditionalControlForceToDefaultValue();
			}	// end of if
		} else {
			getLogger().info("Error on approaching");
		}	// end of if-else
		
		evaluate(place);
		tcp.move(ptp(lbr.getCurrentCartesianPosition(tcp)));
		getLogger().info("Insert finished");
		exIO.gripperOpen();
		
		if ( forceSend ) {
			ftdSenderInsert.interrupt();
		}
		
	}
	
	
	private boolean evaluate(AbstractFrame target) {
		double dist = lbr.getCurrentCartesianPosition(tcp, place).getZ();
		getLogger().info( String.format("Distance to the destination : %03f", dist));
		double force = Math.abs( lbr.getExternalForceTorque(tcp).getForce().getY() );
		getLogger().info( String.format("Y directional Force : %03f", force));
		boolean ret = false;
		if ( Math.abs(dist) < 0.4 /*&& force < 30*/ ) {	// success
			ret = true;
		} else {							// fail
			ret = false;
		}
		return ret;
	}
		
}
