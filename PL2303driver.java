/*
 *  author atuschen75 at gmail dot com
 	modified by swift1911
 */

package com.query.driver;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.IntBuffer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;

import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.usb.UsbConstants;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbEndpoint;
import android.hardware.usb.UsbInterface;
import android.hardware.usb.UsbManager;
import android.hardware.usb.UsbRequest;
import android.util.Log;

/*
 * PL2303 USB Serial Converter Driver for
 */

public class PL2303driver implements Runnable {
	
	// ApplicationContext necessary because of UsbManager and PermissionIntent
	private Context mAppContext; 
	
	// All USB Classes
	private UsbManager mUsbManager;
	private UsbDevice mDevice;
	private UsbDeviceConnection mConnection;
	private UsbInterface mUsbIntf;
	private UsbEndpoint mEp0;
	private UsbEndpoint mEp1;
	private UsbEndpoint mEp2;
	private UsbRequest mUsbRequest;
	
	// ArrayList of all PL2303 converters connected
	private ArrayList<UsbDevice> mPL2303ArrayList = new ArrayList<UsbDevice>();
	
	// Serial Port settings like Baudrate, see setup() 
	private byte[] mPortSetting = new byte[7]; 
	
	// Status of RTC/CTS FlowControl
	private FlowControl mFlow = FlowControl.OFF;
	
	// Status of DTR/RTS Lines
	private int mControlLines = 0;
	
	// Status of DSR/CTS/DCD/RI Lines
	private byte mStatusLines = 0;
	
	// Type 0 = PL2303, Type 1 = PL2303-HX
	private int mPL2303type = 0; 			

	// Callback Class interface
	private PL2303callback mPL2303Callback; 	
	
	// Control variable for Monitoring thread
	private boolean mStopMonitoringThread = false;
	
	private boolean mConnected = false;
	
	public static enum BaudRate {
		B0, 
		B75,
		B150,
		B300,
		B600,
		B1200,
		B1800,
		B2400,
		B4800,
		B9600,
		B19200,
		B38400,
		B57600,
		B115200,
		B230400,
		B460800,
		B614400,
		B921600,
		B1228800,
		B2457600,
		B3000000,
		B6000000
	};

	public static enum DataBits {
		D5,
		D6,
		D7,
		D8
	};

	public static enum StopBits {
		S1,
		S2
	};

	public static enum Parity {
		NONE,
		ODD,
		EVEN
	};
	
	public static enum FlowControl {
		OFF,
		RTSCTS,
		DTRDSR,	// not yet implemented
		XONXOFF	// not yet implemented
	};

	// USB control commands
	private static final int USB_TIMEOUT = 100; // Timeout 100ms for USB transfers
	private static final int SET_LINE_REQUEST_TYPE = 0x21;
	private static final int SET_LINE_REQUEST = 0x20;
	private static final int BREAK_REQUEST_TYPE = 0x21;
	private static final int BREAK_REQUEST = 0x23;	
	private static final int BREAK_OFF = 0x0000;
	private static final int GET_LINE_REQUEST_TYPE = 0xa1;
	private static final int GET_LINE_REQUEST = 0x21;
	private static final int VENDOR_WRITE_REQUEST_TYPE = 0x40;
	private static final int VENDOR_WRITE_REQUEST = 0x01;
	private static final int VENDOR_READ_REQUEST_TYPE = 0xc0;
	private static final int VENDOR_READ_REQUEST = 0x01;
	private static final int SET_CONTROL_REQUEST_TYPE = 0x21;
	private static final int SET_CONTROL_REQUEST = 0x22;
	
	// RS232 Line constants
	private static final int FLOWCONTROL_TIMEOUT = 500; // Timeout 500ms for Flowcontrol
	private static final int CONTROL_DTR = 0x01;
	private static final int CONTROL_RTS = 0x02;
	private static final int UART_DCD = 0x01;
	private static final int UART_DSR = 0x02;
	private static final int UART_RING = 0x08;
	private static final int UART_CTS = 0x80;
	
	// XON/XOFF FlowControl
	private static final int XON = 0x11;
	private static final int XOFF = 0x13;

	// Tag for Log.d function
	private static final String TAG = "pl2303";
	
	// Action for PendingIntent
	private static final String ACTION_USB_PERMISSION 	=   "com.android.hardware.USB_PERMISSION";
	private static final String ACTION_USB_DEVICE_DETACHED 	=   "android.hardware.usb.action.USB_DEVICE_DETACHED";
	
	/**
	 * BroadcastReceiver for permission to use USB-Device
	 * Called by the System after the user has denied/granted access to a USB-Device
	 */
	private final BroadcastReceiver mUsbPermissionReceiver = new BroadcastReceiver() {
		public void onReceive(Context context, Intent intent) {
			String action = intent.getAction();
			if (ACTION_USB_PERMISSION.equals(action)) {
				synchronized (this) {
					UsbDevice device = (UsbDevice)intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
					if ((intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)) && (device != null)) {
						
						Log.d(TAG, "Permission granted for device " + device.getDeviceName());
						
						if (mInitializePL2303(device)){
							
							Log.d(TAG, "Device successfully initialized");
							mPL2303Callback.onInitSuccess(device.getDeviceName());
							
						} else {
							
							Log.d(TAG, "Device initialization failed");
							mPL2303Callback.onInitFailed("Device initialization failed");
							close();
						
						}
						
					} else {
						
						mDevice = null;
						Log.d(TAG, "Permission denied for device " + device.getDeviceName());
						mPL2303Callback.onInitFailed("Permission denied");
						
					}
				}
			}
			
		}
	};

	
	/**
	 * BroadcastReceiver for event USB-Device detached
	 * Called by the System if a USB-Device is detached
	 */
	private final BroadcastReceiver mUsbDeviceDetachedReceiver = new BroadcastReceiver() {
		public void onReceive(Context context, Intent intent) {
			String action = intent.getAction();
				if (mConnected && (ACTION_USB_DEVICE_DETACHED.equals(action))) {
					synchronized (this) {
						UsbDevice device = (UsbDevice)intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
						if (device.getDeviceName().equals(mDevice.getDeviceName())) {
							Log.d(TAG, "Device detached");
							mStopMonitoringThread=true;
							mUsbRequest.cancel();
							mConnection.releaseInterface(mUsbIntf);
							mConnection.close();
							/*mConnection = null;
							mDevice = null;
							mEp0 = null;
							mEp1 = null;
							mEp2 = null;*/ 
							mConnected = false;
							mPL2303Callback.onDeviceDetached(device.getDeviceName());
						}
					}
				}
		}
	};
	
	/**
	 * Constructor
	 * @param context Application Context
	 * @param PL2303callback Object which implements the callback methods
	 */
	public PL2303driver(Context context, PL2303callback callback) {
		Log.d(TAG, "PL2303 driver starting");
		mAppContext = context;
		mPL2303Callback = callback;
		mUsbManager = (UsbManager) mAppContext.getSystemService(Context.USB_SERVICE);

		// Register BroadcastReceiver for Permission Intent
		IntentFilter filterPermission = new IntentFilter(ACTION_USB_PERMISSION);
		mAppContext.registerReceiver(mUsbPermissionReceiver, filterPermission);
		
		// Register BroadcastReceiver for Detached Intent
		IntentFilter filterDetached = new IntentFilter(ACTION_USB_DEVICE_DETACHED);
		mAppContext.registerReceiver(mUsbDeviceDetachedReceiver, filterDetached);
	}
	
	/**
	 * Get a list of pl2303 devices currently connected. Must be called before open().
	 * @return ArrayList<UsbDevice> 
	 */
	public ArrayList<UsbDevice> getDeviceList() {
		mPL2303ArrayList.clear();
		
		// Get the USB device list (of all devices)
		HashMap<String, UsbDevice> deviceList = mUsbManager.getDeviceList();
		Log.d(TAG, deviceList.size()+" USB device(s) found");
		
		// Scan the devices and copy all PL2303-Adaptors into the pl2303ArrayList
		Iterator<UsbDevice> deviceIterator = deviceList.values().iterator();
		while(deviceIterator.hasNext()){
			UsbDevice device = deviceIterator.next();
			if ((device.getProductId()==0x2303) && (device.getVendorId()==0x067b)) {
			mPL2303ArrayList.add(device); }
		}
		Log.d(TAG, mPL2303ArrayList.size()+" PL2303 device(s) found");
 
		return mPL2303ArrayList;
	}
	
	/**
	 * Open USB-Connection to a device (after getDeviceList() has been called)
	 * @param UsbDevice
	 * @throws PL2303Exception if device is not PL2303 or was not in the original getDeviceList()
	 */
	public void open(UsbDevice device) throws PL2303Exception {
		if (mPL2303ArrayList.isEmpty()) throw new PL2303Exception ("No devices connected, or getDeviceList() was not called");
		if (!mPL2303ArrayList.contains(device)) throw new PL2303Exception("Device not in original list");
		if ((device.getProductId()!=0x2303) && (device.getVendorId()!=0x067b)) throw new PL2303Exception("Not a compatible PL2303-device");
		
		PendingIntent mPermissionIntent;
		mPermissionIntent = PendingIntent.getBroadcast(mAppContext, 0, new Intent(ACTION_USB_PERMISSION), PendingIntent.FLAG_ONE_SHOT);

		// Request the permission to use the device from the user
		mUsbManager.requestPermission(device, mPermissionIntent);
		Log.d(TAG, "Requesting permission to use " + device.getDeviceName());
	}
		
	
	/**
	 * Vendor specific USB read request
	 * @param value
	 * @param index
	 * @param buffer
	 * @param length
	 * @throws PL2303Exception
	 */
	private void mVendorRead(int value, int index, byte[] buffer, int length) throws PL2303Exception {
		int ret = mConnection.controlTransfer(VENDOR_READ_REQUEST_TYPE, VENDOR_READ_REQUEST, value, index, buffer, length, USB_TIMEOUT);
		if (ret < length) throw new PL2303Exception("Vendor read request failed! Value: 0x"+ String.format("%04X", value) + " Index: " + index + "Length: " + length +" Return: " + ret);
	}
	
	
	/**
	 * Vendor specific USB write request 
	 * @param value
	 * @param index
	 * @param buffer
	 * @param length
	 * @throws PL2303Exception
	 */
	private void mVendorWrite(int value, int index, byte[] buffer, int length) throws PL2303Exception {
		int ret = mConnection.controlTransfer(VENDOR_WRITE_REQUEST_TYPE, VENDOR_WRITE_REQUEST, value, index, buffer, length, USB_TIMEOUT);
		if (ret < length) throw new PL2303Exception("Vendor write request failed! Value: 0x"+ String.format("%04X", value) + " Index: " + index + "Length: " + length +" Return: " + ret);
	}
	
	
	/**
	 * initialize the PL2303 converter
	 * @return true on success
	 */
	private boolean mInitializePL2303(UsbDevice device) {
		mDevice = device;
		Log.d(TAG, "Device Name: " + mDevice.getDeviceName());
		Log.d(TAG, "VendorID: 0x"+ String.format("%04X", mDevice.getVendorId()));
		Log.d(TAG, "ProductID: 0x"+ String.format("%04X", mDevice.getProductId()));
		
		mUsbIntf = mDevice.getInterface(0);
		if (mUsbIntf == null) {
			Log.e(TAG, "Failed to get USB interface!");
			return false;
		}
		
		// endpoint addr 0x81 = input interrupt
		mEp0 = mUsbIntf.getEndpoint(0); 
		if ((mEp0.getType() != UsbConstants.USB_ENDPOINT_XFER_INT) || (mEp0.getDirection() != UsbConstants.USB_DIR_IN)) {
			Log.e(TAG, "Failed to get USB endpoint 0 (control)!");
			return false;
		}
		
		// endpoint addr 0x82 = output bulk
		mEp1 = mUsbIntf.getEndpoint(1); 
		if ((mEp1.getType() != UsbConstants.USB_ENDPOINT_XFER_BULK) || (mEp1.getDirection() != UsbConstants.USB_DIR_OUT)) {
			Log.e(TAG, "Failed to get USB endpoint 1 (output)!");
			return false;
		}
		
		// endpoint addr 0x83 = input bulk
		mEp2 = mUsbIntf.getEndpoint(2); 
		if ((mEp2.getType() != UsbConstants.USB_ENDPOINT_XFER_BULK) || (mEp2.getDirection() != UsbConstants.USB_DIR_IN)) {
			Log.e(TAG, "Failed to get USB endpoint 2 (input)!");
			return false;
		}
		
		UsbDeviceConnection connection = mUsbManager.openDevice(mDevice);
		if (connection == null) {
			Log.e(TAG, "Failed to get USB device connection!");
			return false;
		}
		
		if (!connection.claimInterface(mUsbIntf, true)) {
			Log.e(TAG, "Failed to claim exclusive interface access!");
			return false;
		}
		
		mConnection = connection;

		if (mConnection.getRawDescriptors()[7] == 64) mPL2303type = 1; //Type 1 = PL2303HX
		Log.d(TAG, "PL2303 type " +mPL2303type+ " detected");		
		
		// Initialization of PL2303 according to linux pl2303.c driver
		byte[] buffer = new byte[1];
		
		try {
			mVendorRead(0x8484, 0, buffer, 1);
			mVendorWrite(0x0404, 0, null, 0);
			mVendorRead(0x8484, 0, buffer, 1);
			mVendorRead(0x8383, 0, buffer, 1);
			mVendorRead(0x8484, 0, buffer, 1);
			mVendorWrite(0x0404, 1, null, 0);
			mVendorRead(0x8484, 0, buffer, 1);
			mVendorRead(0x8383, 0, buffer, 1);
			mVendorWrite(0, 1, null, 0);
			mVendorWrite(1, 0, null, 0);
			
			if (mPL2303type == 1) mVendorWrite(2, 0x44, null, 0);
			else mVendorWrite(2, 0x24, null, 0);
			
		} catch (PL2303Exception e) {
			Log.e(TAG, "Failed to initialize PL2303: ", e);
			e.printStackTrace();
			return false;
		}
		
		// Start control thread for status lines DSR,CTS,DCD and RI
		mStopMonitoringThread = false;
		Thread t = new Thread(this);
		t.start();
		mConnected = true;
		return true;
	}

	/**
	 *  Close the connection
	 */
	public void close() {
		if (mConnected) {
			// drop DTR/RTS
			try {
				setDTR(false);
			} catch (PL2303Exception e) {
				Log.e(TAG,"Error on close: ",e);
			}
			try {
				setRTS(false);
			} catch (PL2303Exception e) {
				Log.e(TAG,"Error on close: ",e);
			}
			mStopMonitoringThread=true;
			mUsbRequest.cancel();
			mConnection.releaseInterface(mUsbIntf);
			mConnection.close();
			/*mConnection = null;
			mDevice = null;
			mEp0 = null;
			mEp1 = null;
			mEp2 = null; */
			mConnected = false;
			Log.d(TAG, "Device closed");
			
			// Wait a moment 
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
			}
		}
	}

	/**
	 * Are we connected to a pl2303 converter?
	 * This is only the USB connection, not the serial connection.
	 * 
	 * @return true on connection
	 */
	public boolean isConnected() {
		if (mConnected) return true;
		else return false;
	}

	/**
	 * Get current Baudrate
	 * @return int
	 */
	public int getBaudRate() {
		ByteBuffer bb = ByteBuffer.wrap(mPortSetting);
		bb.order(ByteOrder.LITTLE_ENDIAN);
		IntBuffer ib = bb.asIntBuffer();
		return ib.get(0);
	}
	
	/**
	 * Get current Stopbit setting
	 * @return string
	 */
	public String getStopbits() {
		String sb_read="";
		switch (mPortSetting[4]) {
		case 0: sb_read="1"; break; 
		case 1: sb_read="1.5"; break;
		case 2: sb_read="2"; break;
		}
		return sb_read;
	}
	
	/**
	 * Get current Parity setting
	 * @return string
	 */
	public String getParity() {
		String p_read="";
		switch (mPortSetting[5]) {
		case 0: p_read="NONE"; break;
		case 1: p_read="ODD"; break;
		case 2: p_read="EVEN"; break;
		}
		return p_read;
	}

	/**
	 * Get current Databit setting
	 * @return int
	 */
	public int getDatabits() {
		return mPortSetting[6];
	}
	
	/**
	 * Get current FlowControl setting
	 * @return string
	 */
	public String getFlowControl() {
		String fc_read="";
		switch (mFlow) {
		case OFF: fc_read="OFF"; break;
		case RTSCTS: fc_read="RTSCTS"; break;
		case DTRDSR: break;
		case XONXOFF: break;
		}
		return fc_read;
	}
	
	/**
	 * Setup basic communication parameters according to linux pl2303.c driver 
	 * @param Enum BaudRate 
	 * @param Enum DataBits
	 * @param Enum StopBits
	 * @param Enum Parity
	 * @param Enum FlowControl
	 * @throws IOException if connection is closed or any other USB IO error
	 * @throws IndexOutOfBoundsException if settings not supported
	 */
	public void setup(BaudRate R, DataBits D, StopBits S, Parity P, FlowControl F) throws PL2303Exception {
		
		if (!mConnected) throw new PL2303Exception("Connection closed");

		// Get current settings
		int ret = mConnection.controlTransfer(GET_LINE_REQUEST_TYPE, GET_LINE_REQUEST, 0, 0, mPortSetting, 7, USB_TIMEOUT);
		if (ret < 7) throw new PL2303Exception("Could not get serial port settings");
		
		//mConnection.controlTransfer(VENDOR_WRITE_REQUEST_TYPE, VENDOR_WRITE_REQUEST, 0, 1, null, 0, 100);
		Log.d(TAG, "Current serial configuration: Baudrate " + getBaudRate() + ", Stopbits "+ getStopbits() +", Parity "+ getParity() +", Databits "+ getDatabits());

		// Setup Baudrate
		int baud;
		switch (R) {
		case B0: baud = 0; break;
		case B75: baud = 75; break;
		case B150: baud = 150; break;
		case B300: baud = 300; break;
		case B600: baud = 600; break;
		case B1200: baud = 1200; break;
		case B1800: baud = 1800; break;
		case B2400: baud = 2400; break;
		case B4800: baud = 4800; break;
		case B9600: baud = 9600; break;
		case B19200: baud = 19200; break;
		case B38400: baud = 38400; break;
		case B57600: baud = 57600; break;
		case B115200: baud = 115200; break;
		case B230400: baud = 230400; break;
		case B460800: baud = 460800; break;
		case B614400: baud = 614400; break;
		case B921600: baud = 921600; break;
		case B1228800: baud = 1228800; break;
		// The following rates only for HX-Type of PL2303
		case B2457600: baud = 2457600; break;
		case B3000000: baud = 3000000; break;
		case B6000000: baud = 6000000; break;
		default: throw new PL2303Exception("Baudrate not supported");
		}

		if  ((baud > 1228800) && (mPL2303type == 0)) throw new PL2303Exception("Baudrate not supported by non HX-type of PL2303"); // Only PL2303HX supports the higher baudrates   

		mPortSetting[0]=(byte) (baud & 0xff);
		mPortSetting[1]=(byte) ((baud >> 8) & 0xff);
		mPortSetting[2]=(byte) ((baud >> 16) & 0xff);
		mPortSetting[3]=(byte) ((baud >> 24) & 0xff);

		// Setup Stopbits
		// TODO: add 1.5 Stopbits
		switch (S) {
		case S1: mPortSetting[4] = 0; break;
		case S2: mPortSetting[4] = 2; break;
		default: throw new PL2303Exception("Stopbit setting not supported"); 
		}

		// Setup Parity
		// TODO: add Mark/Space parity
		switch (P) {
		case NONE: mPortSetting[5] = 0; break;
		case ODD: mPortSetting[5] = 1; break;
		case EVEN: mPortSetting[5] = 2; break;
		default: throw new PL2303Exception("Parity setting not supported"); 
		}

		// Setup Databits
		switch (D) {
		case D5: mPortSetting[6] = 5; break;
		case D6: mPortSetting[6] = 6; break;
		case D7: mPortSetting[6] = 7; break;
		case D8: mPortSetting[6] = 8; break;
		default: throw new PL2303Exception("Databit setting not supported");
		}

		// Set new configuration on PL2303
		ret = mConnection.controlTransfer(SET_LINE_REQUEST_TYPE, SET_LINE_REQUEST, 0, 0, mPortSetting, 7, USB_TIMEOUT);
		if (ret < 7) throw new PL2303Exception("Could not set serial port settings");
		
		Log.d(TAG, "New serial configuration: Baudrate " + getBaudRate() + ", Stopbits "+ getStopbits() +", Parity "+ getParity() +", Databits "+ getDatabits());				

		// Disable BreakControl
		ret = mConnection.controlTransfer(BREAK_REQUEST_TYPE, BREAK_REQUEST, BREAK_OFF, 0, null, 0, USB_TIMEOUT);
		if (ret < 0) throw new PL2303Exception("Could not disable BreakControl");

		// Enable/Disable FlowControl
		switch (F) {
		case OFF:
			mVendorWrite( 0x0, 0x0, null, 0);
			setRTS(false);
			setDTR(false);
			mFlow = F;
			Log.d(TAG, "FlowControl disabled");
			break;
			
		case RTSCTS: 
			if (mPL2303type == 1) mVendorWrite(0x0, 0x61, null, 0);
			else mVendorWrite(0x0, 0x41, null, 0);
			setDTR(true);
			setRTS(true);
			mFlow = F;
			Log.d(TAG, "RTS/CTS FlowControl enabled");
			break;
		
		case DTRDSR: break;
		case XONXOFF: break;
		}
		
	}

	/**
	 * Switch DTR on or off
	 * @param state
	 */
	public void setDTR(boolean state) throws PL2303Exception {
		if ((state) && !((mControlLines & CONTROL_DTR)==CONTROL_DTR)) mControlLines = mControlLines + CONTROL_DTR;
		if (!(state) && ((mControlLines & CONTROL_DTR)==CONTROL_DTR)) mControlLines = mControlLines - CONTROL_DTR;
		
		int ret = mConnection.controlTransfer(SET_CONTROL_REQUEST_TYPE, SET_CONTROL_REQUEST, mControlLines , 0, null, 0, USB_TIMEOUT);
		if (ret < 0) throw new PL2303Exception("Failed to set DTR to " + state);	
		else Log.d(TAG, "DTR set to " + state);
	}
	
	/**
	 * Switch RTS on or off
	 * @param state
	 */
	public void setRTS(boolean state) throws PL2303Exception {
		if ((state) && !((mControlLines & CONTROL_RTS)==CONTROL_RTS)) mControlLines = mControlLines + CONTROL_RTS;
		if (!(state) && ((mControlLines & CONTROL_RTS)==CONTROL_RTS)) mControlLines = mControlLines - CONTROL_RTS;
		
		int ret = mConnection.controlTransfer(SET_CONTROL_REQUEST_TYPE, SET_CONTROL_REQUEST, mControlLines , 0, null, 0, USB_TIMEOUT);
		if (ret < 0) throw new PL2303Exception("Failed to set RTS to " + state); 
		else Log.d(TAG, "RTS set to " + state);
	}
	
	
	/** 
	 * Get the InputStream of the connection. 
	 * This returns the raw bytestream. 
	 * The stream is not buffered. 
	 * To get a character-stream wrap it in some reader i.e. InputStreamReader.
	 * For buffering wrap it in something like BufferedInputStreamReader.   
	 * Supported functions:
	 * 
	 * read() - blocking read (only one byte) 
	 * read(byte[], int, int) - non-blocking read (any size)
	 * 
	 * @return InputStream if connected else null
	 */
	public InputStream getInputStream() {
		if (mConnected) {
			InputStream in = new InputStream() {
				
				/* BulkTransfer always tries to read up to MaxPacketSize, even if length-parameter is lower!
				 * so we need at least a buffer of MaxPacketSize and an index (readPos).
				 */
				
				int PacketSize = mEp2.getMaxPacketSize();
				byte [] readBuffer = new byte[PacketSize];
				int bytesRead=0;
				int readPos=1;
				
				// Blocking read (Timeout set to 0)
				@Override
				public int read() throws IOException{
					synchronized (this) {
						int retVal= -1;
						if (!mConnected) throw new IOException("Connection closed");
						
						// If FlowControl: Check DSR before read
						if ((mFlow==FlowControl.RTSCTS) && ((mStatusLines & UART_DSR) != UART_DSR)) throw new IOException ("DSR down");
						
						if (readPos>bytesRead-1) {
							bytesRead = mConnection.bulkTransfer(mEp2, readBuffer, PacketSize, 0);
							if (bytesRead > 0)readPos=0;
						}
						
						if (bytesRead > 0) {
							retVal = readBuffer[readPos];
							readPos++;
						}
						
						return retVal;
					}
				}

				// Non-blocking read (Timeout set to USB_TIMEOUT)
				@Override
				public int read(byte[] buffer, int offset, int length) throws IOException, IndexOutOfBoundsException {
					synchronized (this) {
						
						int totalBytesRead = 0;
						
						
						if ((offset < 0) || (length < 0) || ((offset + length) > buffer.length)) throw new IndexOutOfBoundsException();
						if (!mConnected) throw new IOException("Connection closed");
						
												
						while (totalBytesRead < length) {
							// If FlowControl: Check DSR before read
							if ((mFlow == FlowControl.RTSCTS) && ((mStatusLines & UART_DSR) != UART_DSR)) throw new IOException ("DSR down");
							
							if (readPos>bytesRead-1) {
								bytesRead = mConnection.bulkTransfer(mEp2, readBuffer, PacketSize, USB_TIMEOUT);
								if (bytesRead > 0)readPos=0;
							}
							
							if (bytesRead > 0) {
								System.arraycopy(readBuffer, readPos, buffer, offset, bytesRead-readPos);
								offset = offset+bytesRead-readPos;
								totalBytesRead = totalBytesRead + bytesRead - readPos;
								readPos = bytesRead-readPos;
							} else break;
							
						}
						
						return totalBytesRead;	
					}
				}
			};
			return in;
		} else return null;
	}

	/** 
	 * Get the OutputStream of the connection. 
	 * This returns the raw bytestream.
	 * The stream is not buffered.
	 * To get a character-stream wrap it in some writer i.e. PrintWriter.
	 * Supported functions:
	 * 
	 * public void write(int) - blocking write (only one byte) 
	 * public void write(byte[], int, int) - non-blocking write (any size)
	 * 
	 * @return OutputStream if connected else null
	 */
	public OutputStream getOutputStream() {
		if (mConnection != null) {
			OutputStream out = new OutputStream() {
				
				// Blocking write (Timeout set to 0)
				@Override 
				public void write(int oneByte) throws IOException{
					synchronized (this) {
						if (mConnected) throw new IOException("Connection closed");
						
						// If FlowControl: Check DSR / CTS before write
						if (mFlow==FlowControl.RTSCTS) {
							if ((mStatusLines & UART_DSR) != UART_DSR) throw new IOException ("DSR down");

							// Wait FLOWCONTROL_TIMEOUT miliseconds until CTS is up, then check again
							if ((mStatusLines & UART_CTS) != UART_CTS) {
								 try {
									Thread.sleep(FLOWCONTROL_TIMEOUT);
								} catch (InterruptedException e) {
								}
								if ((mStatusLines & UART_CTS) != UART_CTS) throw new IOException ("CTS down"); 
							}
							 
						}
						
						byte [] writeBuffer = new byte[1];
						
						int bytesWritten = mConnection.bulkTransfer(mEp1, writeBuffer, 1, 0);
						Log.d(TAG,"Bytes written: " + bytesWritten);
						if (bytesWritten < 1 ) throw new IOException ("BulkWrite failed - written: "+bytesWritten); 
					}
				}

				// Non-blocking write (Timeout set to USB_TIMEOUT)
				@Override
				public void write (byte[] buffer, int offset, int count) throws IOException, IndexOutOfBoundsException {
					synchronized (this) {
						int PacketSize = mEp1.getMaxPacketSize();
						byte [] writeBuffer = new byte[PacketSize];
						
						
						if ((offset < 0) || (count < 0) || ((offset + count) > buffer.length)) throw new IndexOutOfBoundsException();
						if (!mConnected) throw new IOException("Connection closed");
						
						// Max Packet Size 64 bytes! Split larger write-requests in multiple bulk-transfers
						int numTransfers = count / PacketSize;
						if (count % PacketSize > 0) numTransfers++;
						
						for (int i=0;i<numTransfers;i++) {

							// If FlowControl: Check DSR /CTS before write
							if (mFlow==FlowControl.RTSCTS) {
								if ((mStatusLines & UART_DSR) != UART_DSR) throw new IOException ("DSR down");

								// Wait FLOWCONTROL_TIMEOUT miliseconds until CTS is up, then check again
								if ((mStatusLines & UART_CTS) != UART_CTS ) {
									 try {
										Thread.sleep(FLOWCONTROL_TIMEOUT);
									} catch (InterruptedException e) {
									}
									if ((mStatusLines & UART_CTS) != UART_CTS ) throw new IOException ("CTS down");
								}
								
							}
							
							if(i != 0) offset += PacketSize;
							
							// If this is the last part of multiple transfers correct the PacketSize (might be smaller than maxPacketSize) 
							if (i == numTransfers - 1) PacketSize = count - ((numTransfers-1) * PacketSize); 
							
							System.arraycopy(buffer, offset, writeBuffer, 0, PacketSize);
							int bytesWritten = mConnection.bulkTransfer(mEp1, writeBuffer, PacketSize, USB_TIMEOUT);
							Log.d(TAG,"Bytes written: " + bytesWritten);
							if (bytesWritten != PacketSize) throw new IOException ("BulkWrite failed - count: " + PacketSize + " written: "+bytesWritten);
								
						}
					}
				}
			};
			return out;
		} else return null;
	}

	/**
	 * Runnable for detection of DSR, CTS , DCD and RI
	 * Calls the appropriate Callback-function on status change
	 * UsbRequest on Endpoint zero returns 10 bytes. Byte 9 contains the line status. 
	 * This thread is automatically started at initialization and terminated by close(). 
	 */
	@Override
	public void run() {
		ByteBuffer readBuffer = ByteBuffer.allocate(mEp0.getMaxPacketSize());
		mUsbRequest = new UsbRequest();
		
		// Although documentation says that UsbRequest doesn't work on Endpoint 0 it actually works  
		mUsbRequest.initialize(mConnection, mEp0);
		
		Log.d(TAG, "Status line monitoring thread started");
		
		while (!mStopMonitoringThread) {
			mUsbRequest.queue(readBuffer, mEp0.getMaxPacketSize());
			UsbRequest retRequest = mConnection.requestWait();
			
			// The request returns when any line status has changed
			if  ((!mStopMonitoringThread) && (retRequest != null) && (retRequest.getEndpoint()==mEp0)) {
				
				if ((readBuffer.get(8) & UART_DSR) != (mStatusLines & UART_DSR)) {
					Log.d(TAG,"Change on DSR detected: "+(readBuffer.get(8) & UART_DSR));
					if ((readBuffer.get(8) & UART_DSR) == UART_DSR) mPL2303Callback.onDSR(true);
					else mPL2303Callback.onDSR(false);
				}
				
				if ((readBuffer.get(8) & UART_CTS) != (mStatusLines & UART_CTS)) {
					Log.d(TAG,"Change on CTS detected: "+(readBuffer.get(8) & UART_CTS));
					if ((readBuffer.get(8) & UART_CTS) == UART_CTS) mPL2303Callback.onCTS(true);
					else mPL2303Callback.onCTS(false);
				}
				
				if ((readBuffer.get(8) & UART_DCD) != (mStatusLines & UART_DCD)) {
					Log.d(TAG,"Change on DCD detected: "+(readBuffer.get(8) & UART_DCD));
					if ((readBuffer.get(8) & UART_DCD) == UART_DCD) mPL2303Callback.onDCD(true);
					else mPL2303Callback.onDCD(false);
				}
				
				if ((readBuffer.get(8) & UART_RING) != (mStatusLines & UART_RING)) {
					Log.d(TAG,"Change on RI detected: "+(readBuffer.get(8) & UART_RING));
					if ((readBuffer.get(8) & UART_RING) == UART_RING) mPL2303Callback.onRI(true);
					else mPL2303Callback.onRI(false);
				}
				
				// Save status
				mStatusLines = readBuffer.get(8);
			}
			
		}
		Log.d(TAG, "Status line monitoring thread stopped");
	} 
	
}
