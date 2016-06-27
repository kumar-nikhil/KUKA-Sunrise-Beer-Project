package additionalFunction;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.CharBuffer;

import com.kuka.task.ITaskLogger;


public class TCPServer {
    private Thread rT;
    private ServerSocket server_socket;
    private Socket socket;
    private static BufferedReader in;
    private static PrintWriter out;
	boolean end_flag = false;
    static int port;
    String threadedMessage;
    public String getThreadedMessage() {
		return threadedMessage;
	}


	// loger
    private static ITaskLogger logger;
    
    
    public TCPServer ( ITaskLogger t ) {
    	port = 30000;
    	logger = t;
    }

    
	/**
	 * @param message : message to send to client
	 */
	public void send(String message) {
        try {
			logger.info("TCP Server [SEND] : \"" + message + "\"");
			out.println(message);   // sending message via outputstream
		} catch (Exception e) {
			e.printStackTrace();
		}
		
	}
	
	/**
	 * use this method once, and then use <b>getThreadedMessage()</b> method to get messages continuously.
	 */
	public void receiveWithThread() {
		threadedMessage = "";
		
		ReceiveRunnable rR = new ReceiveRunnable();
		rT = new Thread(rR);
		rT.start();
		
	}
	
	public String receiveFile() {
		String content = "";
		
    	try {
    		// clear buffered stacks
    		while( socket.getInputStream().available() != 0 ) {
				socket.getInputStream().read();
			}
    		
    		logger.info("Waitiing for a message from Client...");
    		
    		String message;
    		do {
    			// receiving message via inputstream
    			content += in.readLine();
			} while (content.contains("END"));
    		
		} catch (IOException e) {
			e.printStackTrace();
		}
    	
		return content;
	}
	
	public String receiveWait() {
    	try {
    		// clear buffered stacks
    		while( socket.getInputStream().available() != 0 ) {
				socket.getInputStream().read();
			}
    		
    		logger.info("Waitiing for a message from Client...");
    		
    		String message;
			if ( (message = in.readLine()) != null ) { // receiving message via inputstream
//    		message = in.readLine();
				logger.info("TCP Server [RECEIVE] : \"" + message + "\"");
				return message;
			}
//			}
		} catch (IOException e) {
			e.printStackTrace();
		}
    	
		return null;
	}
    
	public void startComm() {
    	logger.info("Starting TCP communication");
    	 try{
         	server_socket = null;
 			int maxretry = 10;
 			
 			while ( maxretry-- > 0 ) {
 				try {
 					server_socket = new ServerSocket(port);
 					break;	// try succeed
 				} catch (Exception e) {
// 					e.printStackTrace();
 					if ( port > 30010 ) {
 						logger.info("All ports are being used");
 						e.printStackTrace();
 					}
 					logger.info("port " + port + " is in use");
 					port++;
 				} finally {	}
 			}	// end of while

 			logger.info("Server is ready on port [" + server_socket.getLocalPort() + "]" );
 			logger.info("TCP\tC : Wating for connection...");

 			socket = server_socket.accept();
 			logger.info("got a connection from [" + socket.getInetAddress() + "]\n");
 			out = new PrintWriter(new BufferedWriter(new OutputStreamWriter(socket.getOutputStream())), true);
 			in = new BufferedReader(new InputStreamReader(socket.getInputStream()));


         } catch (IOException e) {
             logger.info("TCP Server : Error" + e);
         }
    	 
	}
	
	public void endComm() {
        try {
        	logger.info("Ending TCP communication");
			socket.close();
			server_socket.close();
			end_flag = true;
			try {
				rT.interrupt();
			} catch (Exception e) {
			}				
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	public boolean checkComm() {
		return socket.isConnected();
	}

    // socket thread
    public class ReceiveRunnable implements Runnable {
    	public String message;

        public String getMessage() {
			return message;
		}

		public void run() {
        	while ( !Thread.currentThread().isInterrupted() && !end_flag ) {
    			try {
					if ( (message = in.readLine()) != null ) { // receiving message via inputstream
						threadedMessage = message;
					}
				} catch (IOException e) {
//					e.printStackTrace();
				}
        	}
        }	// end of run()

    }
}
