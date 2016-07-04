package application;


import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

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
import com.kuka.roboticsAPI.motionModel.IMotion;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.LIN;
import com.kuka.roboticsAPI.motionModel.Motion;
import com.kuka.roboticsAPI.motionModel.RelativeLIN;
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
	// Frames
	private JointPosition		home;
	private ObjectFrame			base;
	private TCPServer			server;
	// Process Data
	private double				linVel;
	private double				jointVel;
	// tbd
	private IMotionContainer	tbd;
	private mode				modeFlag;
	private enum mode 			{ tbd, manual };
	// return type
	private enum retType		{ ok, fail, wrong };
	// TCP command
	private String				received;
	

	@Override
	public void initialize() {
		cabinet = getController("KUKA_Sunrise_Cabinet_1");
		lbr = (LBR) getDevice(cabinet, "LBR_iiwa_14_R820_1");
		tool = getApplicationData().createFromTemplate("Tool");
		tcp = tool.getFrame("TCP");
		tool.attachTo(lbr.getFlange());
		
		modeFlag = mode.manual;
		// frames
		initFrames();
		// process data
		processData();
		// TCP Server
		server = new TCPServer(getLogger());
		
	}

	private void processData() {
		linVel = getApplicationData().getProcessData("linVel").getValue();
		getApplicationData().getProcessData("linVel").setDefaultValue(linVel);
		jointVel = getApplicationData().getProcessData("jointVel").getValue();
		getApplicationData().getProcessData("jointVel").setDefaultValue(jointVel);
		getLogger().info(String.format("Process Data is set to : [linVel = %.03f] / [jointVel = %.03f]", linVel, jointVel));
	}

	private void initFrames() {
		home = new JointPosition(0, Math.toRadians(30), 0, -Math.toRadians(60), 0, Math.toRadians(90), 0);
		base = getApplicationData().getFrame("/Base");
	}

	@Override
	public void run() {

		getLogger().info("Starting the application");
		tcp.move(ptp(home).setJointVelocityRel(jointVel));

		server.startComm();
		
//		server.send("Ready");
		while ( server.checkComm() ) {
			try {
				received = server.receiveWait();
				if ( modeFlag == mode.tbd ) {
					commandInterpreterOnTbdMode(received);
				} else if ( modeFlag == mode.manual ){
					commandInterpreter(received);				
				}
			} catch (Exception e) {
//				e.printStackTrace();
				break;
			}	// end of try-catch
			
		} // end of while()

		tcp.move(ptp(home).setJointVelocityRel(jointVel));
		
//		server.endComm();
		
		getLogger().info("Ending the application");
		
	}

	
	
	//
	// command Interpreter
	//
	
	private retType commandInterpreterOnTbdMode(String message) {
		retType ret = retType.ok;
		String[] command = message.split(" ");

		if ( command[0].matches("set") ) {
			ret = commandSet(command);
			respondCommand_ok_fail( ret, command );
		} else if ( command[0].matches("get") ) {
			ret = commandGet_respond(command);
		} else {
			getLogger().error("Wrong command!! [ " + received + " ]");
			getLogger().error("Robot is in TBD mode, only [set/get] command is available");
			server.send("Wrong command [ " + received + " ]");
			return retType.wrong;
		}
		
		return ret;
	}

	public retType commandInterpreter(String message) {
		retType ret = retType.fail;
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
			getLogger().error("Wrong command!! [ " + received + " ]");
			server.send("Wrong command!! [ " + received + " ]");
			showCommands();
			ret =  retType.wrong;
		}
		
		return ret;
	}
	

	private void respondCommand_ok_fail(retType ret, String[] command) {
		if ( ret == retType.ok ) {
			server.send(command[0] + " " + command[1] + " " + "ok");
		} else if ( ret == retType.fail ) {
			server.send(command[0] + " " + command[1] + " " + "fail");
		}
	}
	
	private retType commandGet_respond(String[] command) {
		retType ret = retType.ok;
		
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
			getLogger().error("Wrong command!! [ " + received + " ]");
			server.send("Wrong command!! [ " + received + " ]");
			showCommands();
			return retType.wrong;
		}
		
		return ret;
	}

	
	
	private retType commandSet(String[] command) {
		retType ret = retType.ok;
		
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
					getLogger().error("Wrong command!! [ " + received + " ]");
					server.send("Wrong command!! [ " + received + " ]");
					showCommands();
					return retType.wrong;
				}
				getLogger().info("mode is set to : [ " + arg + " ]");
			} else {
				getLogger().error("Wrong command!! [ " + received + " ]");
				server.send("Wrong command!! [ " + received + " ]");
				showCommands();
				return retType.wrong;
			}
		} catch (Exception e) {
			e.printStackTrace();
			ret = retType.fail;
		}
		
		return ret;
	}

	private retType commandMove(String[] command) {
		retType ret = retType.ok;
		double arg;
		try {
			arg = Double.parseDouble( command[2] );
		} catch (Exception e1) {
			e1.printStackTrace();
			getLogger().error("Wrong command!! [ " + received + " ]");
			server.send("Wrong command!! [ " + received + " ]");
			showCommands();
			return retType.wrong;
		}

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
			getLogger().error("Wrong command!! [ " + received + " ]");
			server.send("Wrong command!! [ " + received + " ]");
			showCommands();
			return retType.wrong;
		}
		
		// motion
		try {
			getLogger().info("moving to frame : [ " + frameToString(newPosition) + " ]");
			tcp.move(ptp(newPosition).setJointVelocityRel(jointVel));
		} catch (Exception e) {
			e.printStackTrace();
			ret = retType.fail;
		}
		
		return ret;
	}
	
	private retType commandMoverel(String[] command) {
		retType ret = retType.ok;
		double arg;
		try {
			arg = Double.parseDouble( command[2] );
		} catch (Exception e1) {
			e1.printStackTrace();
			getLogger().error("Wrong command!! [ " + received + " ]");
			server.send("Wrong command!! [ " + received + " ]");
			showCommands();
			return retType.wrong;
		}

		// set Frame
		Frame currentPosition = lbr.getCurrentCartesianPosition(tcp, World.Current.getRootFrame());
		Frame newPosition = currentPosition.copyWithRedundancy();
		Motion<RelativeLIN> motion;

		if ( command[1].matches("x") ) {
			motion = linRel(arg, 0, 0, 0, 0, 0, base).setJointVelocityRel(jointVel);
//			newPosition.transform(base, Transformation.ofDeg(arg, 0, 0, 0, 0, 0));
//			newPosition.setX(newPosition.getX()+arg);
		} else if ( command[1].matches("y") ) {
			motion = linRel(0, arg, 0, 0, 0, 0, base).setJointVelocityRel(jointVel);
//			newPosition.transform(base, Transformation.ofDeg(0, arg, 0, 0, 0, 0));
//			newPosition.setY(newPosition.getY()+arg);
		} else if ( command[1].matches("z") ) {
			motion = linRel(0, 0, arg, 0, 0, 0, base).setJointVelocityRel(jointVel);
//			newPosition.transform(base, Transformation.ofDeg(0, 0, arg, 0, 0, 0));
//			newPosition.setZ(newPosition.getZ()+arg);
		} else if ( command[1].matches("a") ) {
			motion = linRel(0, 0, 0, Math.toRadians(arg), 0, 0, base).setJointVelocityRel(jointVel);
//			newPosition.transform(base, Transformation.ofDeg(0, 0, 0, arg, 0, 0));
//			newPosition.setAlphaRad( newPosition.getAlphaRad() + Math.toRadians(arg) );
		} else if ( command[1].matches("b") ) {
			motion = linRel(0, 0, 0, 0, Math.toRadians(arg), 0, base).setJointVelocityRel(jointVel);
//			newPosition.transform(base, Transformation.ofDeg(0, 0, 0, 0, arg, 0));
//			newPosition.setBetaRad( newPosition.getBetaRad() + Math.toRadians(arg) );
		} else if ( command[1].matches("c") ) {
			motion = linRel(0, 0, 0, 0, 0, Math.toRadians(arg), base).setJointVelocityRel(jointVel);
//			newPosition.transform(base, Transformation.ofDeg(0, 0, 0, 0, 0, arg));
//			newPosition.setGammaRad( newPosition.getGammaRad() + Math.toRadians(arg) );
		} else {
			getLogger().error("Wrong command!! [ " + received + " ]");
			server.send("Wrong command!! [ " + received + " ]");
			showCommands();
			return retType.wrong;
		}

		// motion
		try {
			getLogger().info("moving to frame : [ " + frameToString(newPosition) + " ]");
//			tcp.move(ptp(newPosition).setJointVelocityRel(jointVel));
			tcp.move(motion);
		} catch (Exception e) {
			e.printStackTrace();
			ret = retType.fail;
		}

		return ret;
	}
	
	public void showCommands() {
		getLogger().info("¦£-----------Command list---------------¦¤");
		getLogger().info("| moverel x/y/z/a/b/c 0.0 \t\t|");
		getLogger().info("| move x/y/z/a/b/c 0.0 \t\t|");
		getLogger().info("| move p 0.0 0.0 0.0 0.0 0.0 0.0 \t|");
		getLogger().info("| move point 0 \t\t\t|");
		getLogger().info("| set speed 0.0 \t\t\t|");
		getLogger().info("| set point 0 \t\t\t|");
		getLogger().info("| set mode tbd/manual \t\t|");
		getLogger().info("| get status/position/mode \t|");
		getLogger().info("¦¦------------------------------------------¦¥");
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
//			e.printStackTrace();
		}
		super.dispose();
	}
	
}
