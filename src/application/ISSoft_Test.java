package application;


import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

import additionalFunction.CycleTimer;
import additionalFunction.TCPServer;
import additionalFunction.WriteFrameToAPIdataXML;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.applicationModel.tasks.UseRoboticsAPIContext;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.task.RoboticsAPITask;

/**
 * @author Seulki-Kim , Jun 27, 2016 , KUKA Robotics Korea<p>
 * @modified Seulki-Kim , Jun 30, 2016 , KUKA Robotics Korea<p>
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
	private Controller			cabinet;
	private LBR					lbr;
	// Tool
	private Tool				tool;
	private ObjectFrame			tcp;
	// flag
	private boolean				loopFlag;
	// Frames
	private JointPosition		home;
	private ObjectFrame			base;
	// CycleTimer
	private CycleTimer			workTimer;
	private TCPServer			server;
	// Process Data
	private double				linVel;
	private double				jointVel;
	// tbd
	private IMotionContainer	tbd;
	private mode				modeFlag;
	private enum mode 			{ tbd, manual };
	
	

	@Override
	public void initialize() {
		cabinet = getController("KUKA_Sunrise_Cabinet_1");
		lbr = (LBR) getDevice(cabinet, "LBR_iiwa_14_R820_1");
		tool = getApplicationData().createFromTemplate("Tool");
		tcp = tool.getFrame("TCP");
		tool.attachTo(lbr.getFlange());
		
		loopFlag = true;
		modeFlag = mode.manual;
		// frames
		initFrames();
		// process data
		processData();
		// CT
		workTimer = new CycleTimer("WorkTimer", getLogger());
		// TCP Server
		server = new TCPServer(getLogger());
		
	}

	private void processData() {
		linVel = getApplicationData().getProcessData("linVel").getValue();
		getApplicationData().getProcessData("linVel").setDefaultValue(linVel);
		jointVel = getApplicationData().getProcessData("jointVel").getValue();
		getApplicationData().getProcessData("jointVel").setDefaultValue(jointVel);
	}

	private void initFrames() {
		home = new JointPosition(0, Math.toRadians(30), 0, -Math.toRadians(60), 0, Math.toRadians(90), 0);
		base = getApplicationData().getFrame("/Base");
	}

	@Override
	public void run() throws Exception {
		server.startComm();

		getLogger().info("Starting the application");
		tcp.move(ptp(home).setJointVelocityRel(jointVel));

//		server.send("Ready");
		while (loopFlag) {
			String received = server.receiveWait();
			if ( modeFlag == mode.tbd ) {
				commandInterpreterOnTbdMode(received);
			} else if ( modeFlag == mode.manual ){
				commandInterpreter(received);				
			}
			
		} // end of while()

		tcp.move(ptp(home).setJointVelocityRel(jointVel));
		
//		server.endComm();
		
		getLogger().info("Ending the application");
		
	}

	
	
	//
	// command Interpreter
	//
	
	private boolean commandInterpreterOnTbdMode(String message) {
		boolean ret = true;
		String[] command = message.split(" ");

		if ( command[0].matches("set") ) {
			ret = commandSet(command);
			respondCommand_ok_fail( ret, command );
		} else if ( command[0].matches("get") ) {
			ret = commandGet_respond(command);
		} else {
			getLogger().error("Illegal command!!");
			getLogger().error("Robot is in TBD mode, only [set/get] command is available");
			return false;
		}
		
		return ret;
	}

	public boolean commandInterpreter(String message) {
		boolean ret = false;
		String[] command = message.split(" ");
		
		if ( command[0].matches("moverel") ) {
			ret = commandMoverel(command);
			respondCommand_ok_fail( ret, command );
		} else if ( command[0].matches("move") ) {
			ret = commandMove(command);
			respondCommand_ok_fail( ret, command );
		} else if ( command[0].matches("set") ) {
			ret = commandSet(command);
			respondCommand_ok_fail( ret, command );
		} else if ( command[0].matches("get") ) {
			ret = commandGet_respond(command);
			
		} else {
			getLogger().error("Illegal command!!");
			showCommands();
			ret =  false;
		}
		
		return ret;
	}
	

	private void respondCommand_ok_fail(boolean ret, String[] command) {
		String ok_fail = ret ? "ok" : "fail"; 
		server.send(command[0] + " " + command[1] + " " + ok_fail);
	}
	
	private boolean commandGet_respond(String[] command) {
		boolean ret = true;
		
		if ( command[1].matches("status") ) {
			if ( lbr.isReadyToMove() && modeFlag == mode.manual ) {
				server.send(command[0] + " " + command[1] + " " + "ready");
			} else {
				server.send(command[0] + " " + command[1] + " " + "busy");
			}
		} else if ( command[1].matches("position") ) {
			Frame currentPosition = lbr.getCurrentCartesianPosition(tcp, World.Current.getRootFrame());
			server.send(command[0] + " " + command[1] + " " + frameToString(currentPosition) );
			getLogger().info( String.format("get position : [ " + currentPosition.toStringInWorld() + " ]") );
		} else if ( command[1].matches("mode") ) {
			server.send(command[0] + " " + command[1] + " " + modeFlag.toString());
			getLogger().info( String.format("get mode : [ " + modeFlag.toString() + " ]") );
		} else {
			getLogger().error("Illegal command!!");
			showCommands();
			return false;
		}
		
		return ret;
	}

	
	
	private boolean commandSet(String[] command) {
		boolean ret = true;
		
		try {
			if ( command[1].matches("speed") ) {
				jointVel = Double.parseDouble( command[2] );
				getLogger().info( String.format("Joint Velocity is set to : [ %.03f ]", jointVel) );
			} else if ( command[1].matches("point") ) {
				WriteFrameToAPIdataXML wfXML = new WriteFrameToAPIdataXML(this, base, getLogger());

				int arg = Integer.parseInt( command[2] );
				Frame currentPosition = lbr.getCurrentCartesianPosition(tcp, World.Current.getRootFrame());
				wfXML.deleteFrame( String.format("P%d", arg) );
				wfXML.writeFrame(currentPosition, String.format("P%d", arg) );
				initFrames();
				
				getLogger().info( String.format("P%d is set to : [ " + frameToString(currentPosition) + " ]", arg) );				
			} else if ( command[1].matches("mode") ) {
				String arg = command[2];
				if ( arg.matches("tbd") ) {
					if ( modeFlag == mode.manual ) {
						CartesianImpedanceControlMode cicm = new CartesianImpedanceControlMode();
						cicm.parametrize(CartDOF.TRANSL).setStiffness(0);
						cicm.parametrize(CartDOF.ROT).setStiffness(0);
						tbd = lbr.moveAsync(positionHold(cicm, -1, TimeUnit.SECONDS));
						modeFlag = mode.tbd;						
					} else if ( modeFlag == mode.tbd ) {
						getLogger().error("mode is already set to [ tbd ]");
					}
				} else if ( arg.matches("manual") ) {
					if ( modeFlag == mode.tbd ) {
						tbd.cancel();
						tcp.move(ptp(lbr.getCurrentCartesianPosition(tcp)));
						modeFlag = mode.manual;						
					} else if ( modeFlag == mode.manual ) {
						getLogger().error("mode is already set to [ manual ]");
					}
				} else {
					getLogger().error("Illegal command!!");
					showCommands();
					return false;
				}
				getLogger().info("mode is set to : [ " + arg + " ]");
			} else {
				getLogger().error("Illegal command!!");
				showCommands();
				return false;
			}
		} catch (Exception e) {
			e.printStackTrace();
			ret = false;
		}
		
		return ret;
	}

	private boolean commandMove(String[] command) {
		boolean ret = false;
		double arg = Double.parseDouble( command[2] );

		// set Frame
		Frame currentPosition = lbr.getCurrentCartesianPosition(tcp, World.Current.getRootFrame());
		Frame newPosition = currentPosition.copyWithRedundancy();

		if ( command[1].matches("x") ) {
			newPosition.setX(arg);
		} else if ( command[1].matches("y") ) {
			newPosition.setY(arg);
		} else if ( command[1].matches("z") ) {
			newPosition.setZ(arg);
		} else if ( command[1].matches("a") ) {
			newPosition.setAlphaRad( Math.toRadians(arg) );
		} else if ( command[1].matches("b") ) {
			newPosition.setBetaRad( Math.toRadians(arg) );
		} else if ( command[1].matches("c") ) {
			newPosition.setGammaRad( Math.toRadians(arg) );
		} else if ( command[1].matches("p") ) {
			Double[] args = new Double[6];
			for ( int i=0; i<6; i++ ) {
				args[i] = Double.parseDouble( command[i+2] );
			}
			
			newPosition = new Frame(World.Current.getRootFrame(),
					args[0], args[1], args[2],
					Math.toRadians(args[3]), Math.toRadians(args[4]), Math.toRadians(args[5]));
		} else if ( command[1].matches("point") ) {
			int argI = (int) arg;
			newPosition = base.getChild(String.format("P%d", argI)).copyWithRedundancy();
			getLogger().info("selected point : [" + base.getChild(String.format("P%d", argI)).getName() + "]");
		} else {
			getLogger().error("Illegal command!!");
			showCommands();
			return false;
		}
		
		// motion
		try {
			getLogger().info("moving to frame : [ " + frameToString(newPosition) + " ]");
			tcp.move(ptp(newPosition).setJointVelocityRel(jointVel));
		} catch (Exception e) {
			e.printStackTrace();
			ret = false;
		}
		
		return ret;
	}
	
	private boolean commandMoverel(String[] command) {
		boolean ret = false;
		double arg = Double.parseDouble( command[2] );

		// set Frame
		Frame currentPosition = lbr.getCurrentCartesianPosition(tcp, World.Current.getRootFrame());
		Frame newPosition = currentPosition.copyWithRedundancy();

		if ( command[1].matches("x") ) {
//			newPosition.transform(World.Current.getRootFrame(), Transformation.ofDeg(arg, 0, 0, 0, 0, 0));
			newPosition.setX(newPosition.getX()+arg);
		} else if ( command[1].matches("y") ) {
//			newPosition.transform(World.Current.getRootFrame(), Transformation.ofDeg(0, arg, 0, 0, 0, 0));
			newPosition.setY(newPosition.getY()+arg);
		} else if ( command[1].matches("z") ) {
//			newPosition.transform(World.Current.getRootFrame(), Transformation.ofDeg(0, 0, arg, 0, 0, 0));
			newPosition.setZ(newPosition.getZ()+arg);
		} else if ( command[1].matches("a") ) {
//			newPosition.transform(World.Current.getRootFrame(), Transformation.ofDeg(0, 0, 0, arg, 0, 0));
			newPosition.setAlphaRad( newPosition.getAlphaRad() + Math.toRadians(arg) );
		} else if ( command[1].matches("b") ) {
//			newPosition.transform(World.Current.getRootFrame(), Transformation.ofDeg(0, 0, 0, 0, arg, 0));
			newPosition.setBetaRad( newPosition.getBetaRad() + Math.toRadians(arg) );
		} else if ( command[1].matches("c") ) {
//			newPosition.transform(World.Current.getRootFrame(), Transformation.ofDeg(0, 0, 0, 0, 0, arg));
			newPosition.setGammaRad( newPosition.getGammaRad() + Math.toRadians(arg) );
		} else {
			getLogger().error("Illegal command!!");
			showCommands();
			return false;
		}

		// motion
		try {
			getLogger().info("moving to frame : [ " + frameToString(newPosition) + " ]");
			tcp.move(ptp(newPosition).setJointVelocityRel(jointVel));
		} catch (Exception e) {
			e.printStackTrace();
			ret = false;
		}

		return ret;
	}
	
	public void showCommands() {
		getLogger().info("moverel x/y/z/a/b/c 0.0");
		getLogger().info("move x/y/z/a/b/c 0.0");
		getLogger().info("move p 0.0 0.0 0.0 0.0 0.0 0.0");
		getLogger().info("move point 0");
		getLogger().info("set speed 0.0");
		getLogger().info("set point 0");
		getLogger().info("set mode tbd/manual");
		getLogger().info("get status/position/mode");
	}
	
	private String frameToString(Frame frame) {
		String data = String.format("%.03f %.03f %.03f %.03f %.03f %.03f",
				frame.getX(), frame.getY(), frame.getZ(),
				Math.toDegrees(frame.getAlphaRad()),
				Math.toDegrees(frame.getBetaRad()),
				Math.toDegrees(frame.getGammaRad()) );
		return data;
	}
	
	@Override
	public void dispose() {
		try {
			server.endComm();
		} catch (Exception e) {
			e.printStackTrace();
		}
		super.dispose();
	}
	
}
