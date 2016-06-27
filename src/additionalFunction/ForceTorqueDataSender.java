package additionalFunction;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.Timer;
import java.util.TimerTask;

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;

/**
 * @author Seulki-Kim , 2016. 4. 22. , KUKA Robotics Korea<p>
 * @modified Seulki-Kim , 2016. 5. 4. , KUKA Robotics Korea<p>
 * <pre>Usage example<br><code>
 * // Initialize
 * ForceTorqueDataSender FTrecorder = new ForceTorqueDataSender(
 * 	lbr, tcp, "172.31.1.101", 30000, -1, ForceTorqueDataSender.Type.FORCETORQUE_XYZABC,
 * 	"title" );
 *  // starting
 * FTrecorder.start();
 *  // motion
 * <i>motion to monitor F/T</i>
 *  // finishing
 * FTrecorder.interrupt();
 * </code></pre>
 */
public class ForceTorqueDataSender extends Thread {
	private LBR				lbr;
	private ObjectFrame		tcp;

	private long			time;
	private long			period;
	private double			tData;

	private DatagramSocket	dSock;
	private InetAddress		server;
	private DatagramPacket	outPacket;
	private int				udpPort;

	private String			title;
	private String			_data;
	private int				packetCnt;
	private Type			type;

	private ForceSensorData	fsD;
	private Vector			fd;
	private Vector			td;
	private double[]		vals;
	private String			category;
	private double			startTime;

	/**<pre>
	 * <b>FORCETORQUE_XYZABC</b>	: Force XYZ [N] and Torque ABC [Nm]
	 * <b>JOINT_TORQUE</b>		: Joint torque J1~J7 [Nm]
	 * <b>BOTH</b>			: Both. ForceTorque first and then Joint Torque
	 </pre> */
	public enum Type {
		FORCETORQUE_XYZABC, JOINT_TORQUE, BOTH
	};

	/**<pre>
	 * @param lbr		LBR object
	 * @param tcp		TCP object - measure frame to calculate F/T
	 * @param udpIPAdrs	IP address of UDP Server(extern pc) e.g. "172.31.1.140"...
	 * @param udpPort		Port - 30000 to 30010 are allowed
	 * @param time		auto finishing time in [ms]. use <b>-1</b> if you want to explicitly finish the class with interrupt()
	 * @param period		fixed-rate period - check every "period"[ms]
	 * @param type		Force&Torque / Joint Torque / Both
	 </pre> */
	public ForceTorqueDataSender(LBR lbr, ObjectFrame tcp, String udpIPAdrs, int udpPort, long time, long period, Type type) {
		try {
			this.lbr = lbr;
			this.tcp = tcp;
			this.time = time<=0 ? (1000*60*60*24) : time;
			this.period = period;
			this.tData = 0;
			this._data = "";
			this.packetCnt = 0;
			this.udpPort = udpPort;
			this.type = type;
			this.title = null;
			
			server = InetAddress.getByName(udpIPAdrs);
			dSock = new DatagramSocket();
		} catch (UnknownHostException e) {
			e.printStackTrace();
		} catch (SocketException e) {
			e.printStackTrace();
		}
		
	}
	/** <pre>
	 * @param lbr		LBR object
	 * @param tcp		TCP object - measure frame to calculate F/T
	 * @param udpIPAdrs	IP address of UDP Server(extern pc) e.g. "172.31.1.140"...
	 * @param udpPort		Port - 30000 to 30010 are allowed
	 * @param time		auto finishing time in [ms]. use <b>-1</b> if you want to explicitly finish the class with interrupt()
	 * @param period		fixed-rate period - check every "period"[ms]
	 * @param type		Force&Torque / Joint Torque / Both
	 * @param title		title of a Data group. this will be the first packet to be sent
	 </pre> */
	public ForceTorqueDataSender(LBR lbr, ObjectFrame tcp, String udpIPAdrs, int udpPort, long time, long period, Type type, String title) {
		this(lbr, tcp, udpIPAdrs, udpPort, time, period, type);
		this.title = String.format(title + "\n");
	}
	
	/**
	 * @return	data from the start till now
	 */
	public String getData() {
		return _data;
	}
	
	/**
	 * @return	elapsed time [ms] from the start
	 */
	public double getTimeStamp() {
		return tData;
	}
	
	public void initialize() {
		category = null;
		if ( type == Type.FORCETORQUE_XYZABC ) {
			category = String.format("msec,X,Y,Z,TX,TY,TZ\n");
		} else if ( type == Type.JOINT_TORQUE ) {
			category = String.format("msec,J1,J2,J3,J4,J5,J6,J7\n");
		} else if ( type == Type.BOTH ) {
			category = String.format("msec,X,Y,Z,TX,TY,TZ, ,J1,J2,J3,J4,J5,J6,J7\n");				
		}

		startTime = System.nanoTime();
		// title
		try {
			if ( ! title.isEmpty() ) {
				outPacket = new DatagramPacket(title.getBytes(), title.getBytes().length, server, udpPort);
				dSock.send(outPacket);
			}
			// category
			byte[] sendT = category.getBytes();
			outPacket = new DatagramPacket(sendT, sendT.length, server, udpPort);
			dSock.send(outPacket);
		} catch (IOException e) {
			e.printStackTrace();
		}
		
	}
	
	@Override
	public void run() {
		initialize();
		
		// UDP cleint
		try {
			ScheduledJob job = new ScheduledJob();
			Timer jobScheduler = new Timer();
			jobScheduler.scheduleAtFixedRate(job, 0, period);

			while ( tData <= time ) {
//				loop();
//				keeping this run() alive while timer is working
//				timer is to be set '0' to get out of this while loop when this.interrupt() is called
			}
			// finish timer task
			jobScheduler.cancel();
			

			if ( time == 0 ) {
				System.out.println("Motion finished. Ending Data sending");
				System.out.println("Time taken : " + tData);
			} else {
				System.out.println("Data sending finished before motion is finished. ");
				
			}
			System.out.println("packets sent : " + packetCnt);
			
			String endS = String.format("END\n");
			byte[] end = endS.getBytes();
			outPacket = new DatagramPacket(end, end.length, server, udpPort);
			for ( int i=0; i<2; i++ ) {
				dSock.send(outPacket);
				ThreadUtil.milliSleep(20);
			}
			
		} catch (IOException e) {
			e.printStackTrace();
		}	// end of try-catch for UDP


	}	// end of run()
	
	
	class ScheduledJob extends TimerTask {
		@Override
		public void run() {
			try {
				loop();
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
		
	}
	
	public void loop() throws IOException {
		String ftData = null, jtData = null, data = null;
//		tData = ((double)lbr.getExternalForceTorque(tcp).getTimestamp() - startTime)/1000000d;
		tData = ((double)System.nanoTime() - startTime)/1000000d;

		if ( type == Type.FORCETORQUE_XYZABC || type == Type.BOTH ) {
			fsD = lbr.getExternalForceTorque(tcp);
			fd = fsD.getForce();
			td = fsD.getTorque();
			ftData = String.format("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
					fd.getX(), fd.getY(), fd.getZ(), td.getX(), td.getY(), td.getZ() );
		}
		if ( type == Type.JOINT_TORQUE || type == Type.BOTH  ) {
			vals = lbr.getExternalTorque().getTorqueValues();
			jtData = String.format("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
					vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], vals[6]);
		}

		switch (type) {
		case FORCETORQUE_XYZABC:
			data = String.format("%.1f," + ftData + "\n", tData);
			break;
		case JOINT_TORQUE:
			data = String.format("%.1f," + jtData + "\n", tData);
			break;
		case BOTH:
			data = String.format("%.1f," + ftData + ",," + jtData + "\n", tData);
			break;
		}
		_data += data;

		byte[] send = data.getBytes();
		outPacket = new DatagramPacket(send, send.length, server, udpPort);
		dSock.send(outPacket);
		packetCnt++;

//		Thread.sleep(0);

	}

	@Override
	public void interrupt() {
//		super.interrupt();
		try {
			time = 0;
			System.out.println("Waiting for the thread to be finished...");
			this.join(500);
			dSock.close();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}	// end of interrupt()
	
}
