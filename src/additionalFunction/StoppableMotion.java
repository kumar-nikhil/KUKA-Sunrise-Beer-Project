package additionalFunction;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.linRel;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.util.concurrent.TimeUnit;

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.conditionModel.ICondition;
import com.kuka.roboticsAPI.conditionModel.JointTorqueCondition;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.AbstractFrame;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.task.ITaskLogger;

/**
 * @author Seulki-Kim , 2015. 7. 23. , KUKA Robotics Korea<p>
 * StoppableMotion
 *
 */
public class StoppableMotion {
	
	private LBR lbr;
	private static ITaskLogger logger;

	// collision
	private JointTorqueCondition[] cj = new JointTorqueCondition[7];
	private ICondition collisionC;
	private double collisionThreshold;
	
	// frames & position
	private JointPosition currentJPosition;
	private Frame currentPosition;
	
	// constructor
	/**
	 * @param lbr	-	LBR<br>
	 * <b>threshold</b> 10Nm
	 */
	public StoppableMotion(LBR lbr) {
		this.lbr = lbr;
		logger = null;
		
		cj[0] = new JointTorqueCondition(JointEnum.J1, -10, 10);
		cj[1] = new JointTorqueCondition(JointEnum.J2, -10, 10);
		cj[2] = new JointTorqueCondition(JointEnum.J3, -10, 10);
		cj[3] = new JointTorqueCondition(JointEnum.J4, -10, 10);
		cj[4] = new JointTorqueCondition(JointEnum.J5, -10, 10);
		cj[5] = new JointTorqueCondition(JointEnum.J6, -10, 10);
		cj[6] = new JointTorqueCondition(JointEnum.J7, -10, 10);
		collisionC = cj[0].or(cj[1]).or(cj[2]).or(cj[3]).or(cj[4]).or(cj[5]).or(cj[6]);
	}
	/**
	 * @param lbr	-	LBR<br>
	 * @param logger	-	ITaskLogger<br>
	 * <b>threshold</b> 10Nm
	 */
	public StoppableMotion(LBR lbr, ITaskLogger logger) {
		this.lbr = lbr;
		if (logger != null) {
			StoppableMotion.logger = logger;
		}
		
		cj[0] = new JointTorqueCondition(JointEnum.J1, -10, 10);
		cj[1] = new JointTorqueCondition(JointEnum.J2, -10, 10);
		cj[2] = new JointTorqueCondition(JointEnum.J3, -10, 10);
		cj[3] = new JointTorqueCondition(JointEnum.J4, -10, 10);
		cj[4] = new JointTorqueCondition(JointEnum.J5, -10, 10);
		cj[5] = new JointTorqueCondition(JointEnum.J6, -10, 10);
		cj[6] = new JointTorqueCondition(JointEnum.J7, -10, 10);
		collisionC = cj[0].or(cj[1]).or(cj[2]).or(cj[3]).or(cj[4]).or(cj[5]).or(cj[6]);
	}
	
	/**
	 * @param lbr	-	LBR
	 * @param threshold	-	JointTorqueCondition minTorque & maxTorque <i>(-threshold ~ threshold)</i>
	 */
	public StoppableMotion(LBR lbr, double threshold) {
		this.lbr = lbr;
		this.collisionThreshold = threshold;
		logger = null;
		
		cj[0] = new JointTorqueCondition(JointEnum.J1, -collisionThreshold, collisionThreshold);
		cj[1] = new JointTorqueCondition(JointEnum.J2, -collisionThreshold, collisionThreshold);
		cj[2] = new JointTorqueCondition(JointEnum.J3, -collisionThreshold, collisionThreshold);
		cj[3] = new JointTorqueCondition(JointEnum.J4, -collisionThreshold, collisionThreshold);
		cj[4] = new JointTorqueCondition(JointEnum.J5, -collisionThreshold, collisionThreshold);
		cj[5] = new JointTorqueCondition(JointEnum.J6, -collisionThreshold, collisionThreshold);
		cj[6] = new JointTorqueCondition(JointEnum.J7, -collisionThreshold, collisionThreshold);
		collisionC = cj[0].or(cj[1]).or(cj[2]).or(cj[3]).or(cj[4]).or(cj[5]).or(cj[6]);
	}
	/**
	 * @param lbr	-	LBR
	 * @param threshold	-	JointTorqueCondition minTorque & maxTorque <i>(-threshold ~ threshold)</i>
	 * @param logger	-	ITaskLogger
	 */
	public StoppableMotion(LBR lbr, double threshold, ITaskLogger logger) {
		this.lbr = lbr;
		this.collisionThreshold = threshold;
		if (logger != null) {
			StoppableMotion.logger = logger;
		}
		
		cj[0] = new JointTorqueCondition(JointEnum.J1, -collisionThreshold, collisionThreshold);
		cj[1] = new JointTorqueCondition(JointEnum.J2, -collisionThreshold, collisionThreshold);
		cj[2] = new JointTorqueCondition(JointEnum.J3, -collisionThreshold, collisionThreshold);
		cj[3] = new JointTorqueCondition(JointEnum.J4, -collisionThreshold, collisionThreshold);
		cj[4] = new JointTorqueCondition(JointEnum.J5, -collisionThreshold, collisionThreshold);
		cj[5] = new JointTorqueCondition(JointEnum.J6, -collisionThreshold, collisionThreshold);
		cj[6] = new JointTorqueCondition(JointEnum.J7, -collisionThreshold, collisionThreshold);
		collisionC = cj[0].or(cj[1]).or(cj[2]).or(cj[3]).or(cj[4]).or(cj[5]).or(cj[6]);
	}
	// end of constructor

	
	//
	// method
	//

	/**
	 *<code> public <b>stoppableHomePosition</b>()</code><p>
	 * Stoppable ptp motion to Home position; JP(0, 30, 0, -60, 0, 90, 0) in degrees.
	 * <br> with default JointVelococity(0.4)
	 * @return <b>JointPosition</b> -	Joint Position when collision is occurred.
	 */
	public JointPosition stoppableHomePosition() {

		IMotionContainer mc = lbr.move(ptp(new JointPosition(0, Math.toRadians(30), 0, -Math.toRadians(60), 0, Math.toRadians(90), 0))
				.setJointVelocityRel(0.4)
				.breakWhen(collisionC));
		if ( mc.hasFired(collisionC) ) {
			currentJPosition = lbr.getCurrentJointPosition();
			whenCollision();
			stoppableHomePosition();
		}
		return currentJPosition;
	}
	
	/**
	 *<code> public <b>stoppableHomePosition</b>()</code><p>
	 * Stoppable ptp motion to Home position; JP(0, 30, 0, -60, 0, 90, 0) in degrees.
	 * <br> with default JointVelococity(0.4)
	 * @param vel					-	Joint velocity
	 * @return <b>JointPosition</b> -	Joint Position when collision is occurred.
	 */
	public JointPosition stoppableHomePosition(double vel) {

		IMotionContainer mc = lbr.move(ptp(new JointPosition(0, Math.toRadians(30), 0, -Math.toRadians(60), 0, Math.toRadians(90), 0))
				.setJointVelocityRel(vel)
				.breakWhen(collisionC));
		if ( mc.hasFired(collisionC) ) {
			currentJPosition = lbr.getCurrentJointPosition();
			whenCollision();
			stoppableHomePosition(vel);
		}
		return currentJPosition;
	}

	/**
	 *<code> public <b>stoppableJointPosition</b>(JointPosition jp, double vel)</code><p>
	 * Stoppable ptp motion to joint position
	 * @param jp	-	JointPosition
	 * @param vel	-	Joint velocity
	 * @return <b>JointPosition</b> -	Joint Position when collision is occurred.
	 */
	public JointPosition stoppableJointPosition(JointPosition jp, double vel) {

		IMotionContainer mc = lbr.move(ptp(jp)
				.setJointVelocityRel(vel)
				.breakWhen(collisionC));
		if ( mc.hasFired(collisionC) ) {
			currentJPosition = lbr.getCurrentJointPosition();
			whenCollision();
			stoppableJointPosition(jp, vel);
		}
		return currentJPosition;
	}
	
	/**
	 *<code> public <b>stoppableLINMotion</b>(ObjectFrame tcp, AbstractFrame dest)</code><p>
	 * Stoppable lin motion <code><b>tcp</b>.move(lin(<b>dest</b>))</code> with default CartVelocity(100)
	 * @param tcp -	tcp to make a moove
	 * @param dest -	destination frame<br>
	 * @return <b>Frame</b> -	Current cartesian position of <code><b>tcp</b></code>
	 * relative to the root frame when collision is occurred.
	 */
	public Frame stoppableLINMotion(ObjectFrame tcp, AbstractFrame dest) {

		IMotionContainer mc = tcp.move(lin(dest)
				.setCartVelocity(100)
				.breakWhen(collisionC));
		if ( mc.hasFired(collisionC) ) {
			currentPosition = lbr.getCurrentCartesianPosition(tcp, World.Current.getRootFrame());
			whenCollision();
			stoppableLINMotion(tcp, dest);
		}
		return currentPosition;
	}

	/**
	 *<code> public <b>stoppableLINMotion</b>(ObjectFrame tcp, AbstractFrame dest, double vel)</code><p>
	 * Stoppable lin motion <code><b>tcp</b>.move(lin(<b>dest</b>).setCartVelocity(<b>vel</b>))</code>
	 * @param tcp -	tcp to make a moove
	 * @param dest -	destination frame
	 * @param vel -	cartesianVelocity
	 * @return <b>Frame</b> -	Current cartesian position of <code><b>tcp</b></code>
	 * relative to the root frame when collision is occurred.
	 */
	public Frame stoppableLINMotion(ObjectFrame tcp, AbstractFrame dest, double vel) {

		IMotionContainer mc = tcp.move(lin(dest)
				.setCartVelocity(vel)
				.breakWhen(collisionC));
		if ( mc.hasFired(collisionC) ) {
			currentPosition = lbr.getCurrentCartesianPosition(tcp, World.Current.getRootFrame());
			whenCollision();
			stoppableLINMotion(tcp, dest, vel);
		}
		return currentPosition;
	}

	/**
	 *<code> public <b>stoppablePTPMotion</b>(ObjectFrame tcp, AbstractFrame dest)</code><p>
	 * Stoppable ptp motion <code><b>tcp</b>.move(ptp(<b>dest</b>))</code> with default JointVelococity(0.4)
	 * @param tcp -	tcp to make a moove
	 * @param dest -	destination frame<br>
	 * @return <code><b>Frame</b></code> -	Current cartesian position of <code><b>tcp</b></code>
	 * relative to the root frame when collision is occurred.
	 */
	public Frame stoppablePTPMotion(ObjectFrame tcp, AbstractFrame dest) {

		IMotionContainer mc = tcp.move(ptp(dest)
				.setJointVelocityRel(0.4)
				.breakWhen(collisionC));
		if ( mc.hasFired(collisionC) ) {
			currentPosition = lbr.getCurrentCartesianPosition(tcp, World.Current.getRootFrame());
			whenCollision();
			stoppablePTPMotion(tcp, dest);
		}
		return currentPosition;
	}

	/**
	 *<code> public <b>stoppablePTPMotion</b>(ObjectFrame tcp, AbstractFrame dest, double vel)</code><p>
	 * Stoppable ptp motion <code><b>tcp</b>.move(ptp(<b>dest</b>).setCartVelocity(<b>vel</b>))</code>
	 * @param tcp -	tcp to make a moove
	 * @param dest -	destination frame
	 * @param vel -	jointVelocity [0...1]
	 * @return <b>Frame</b>	-	Current cartesian position of <code><b>tcp</b></code>
	 * relative to the root frame when collision is occurred.
	 */
	public Frame stoppablePTPMotion(ObjectFrame tcp, AbstractFrame dest, double vel) {

		IMotionContainer mc = tcp.move(ptp(dest)
				.setJointVelocityRel(vel)
				.breakWhen(collisionC));
		if ( mc.hasFired(collisionC) ) {
			currentPosition = lbr.getCurrentCartesianPosition(tcp, World.Current.getRootFrame());
			whenCollision();
			stoppablePTPMotion(tcp, dest, vel);
		}
		return currentPosition;
	}


	private void whenCollision() {
		if ( logger != null ) {
			logger.info("stopping robot. turn 7th axis to resume");
		}
		ThreadUtil.milliSleep(200);
		JointImpedanceControlMode resumeCIC = new JointImpedanceControlMode(7);
		double etJ7 = lbr.getExternalTorque().getSingleTorqueValue(JointEnum.J7);
		JointTorqueCondition resume = new JointTorqueCondition(JointEnum.J7, (etJ7 - 1.5), (etJ7 + 1.5) );
		lbr.move(positionHold(resumeCIC, -1, TimeUnit.MILLISECONDS).breakWhen(resume));
		if ( logger != null ) {
			logger.info("resuming robot");
		}
		ThreadUtil.milliSleep(200);
		lbr.move(linRel(0, 0, -30, World.Current.getRootFrame()).setCartVelocity(70));
		lbr.move(linRel(0, 0, 30, World.Current.getRootFrame()).setCartVelocity(70));
		ThreadUtil.milliSleep(300);
	}
	
	//
	// end of method
	//
	
}
