package main

import (
	"fmt"
	"time"

	"github.com/aler9/gomavlib"
	"github.com/aler9/gomavlib/dialects/ardupilotmega"
)

func main() {
	// Proxy (or man-in-the-middle ;>) between my rpi3 connected to a Pixhawk, and QGroundControl running on my local
	// desktop, and display the message details.
	node, err := gomavlib.NewNode(gomavlib.NodeConf{
		Endpoints: []gomavlib.EndpointConf{
			gomavlib.EndpointUdpClient{":14550"},
			gomavlib.EndpointUdpClient{"10.1.1.165:14550"},
		},
		Dialect:     ardupilotmega.Dialect,
		OutVersion:  gomavlib.V2, // change to V1 if you're unable to write to the target
		OutSystemId: 128,
	})
	if err != nil {
		panic(err)
	}
	defer node.Close()

	fmt.Println("MAVLink decoder running...")

	for evt := range node.Events() {
		if _, ok := evt.(*gomavlib.EventChannelOpen); ok {
			fmt.Printf("EventChannelOpen received\n\n")
		}
		if _, ok := evt.(*gomavlib.EventChannelClose); ok {
			fmt.Printf("EventChannelClose received\n\n")
		}
		if _, ok := evt.(*gomavlib.EventParseError); ok {
			fmt.Printf("EventParseError received\n\n")
		}
		if _, ok := evt.(*gomavlib.EventStreamRequested); ok {
			fmt.Printf("EventStreamRequested received\n\n")
		}

		if frm, ok := evt.(*gomavlib.EventFrame); ok {
			switch frm.SystemId() {
			case 1:
				// This is a message from the drone
				switch msg := frm.Message().(type) {
				case *ardupilotmega.MessageHeartbeat: // Id 0
					fmt.Printf("Heartbeat from drone at %v\n", time.Now().Format(time.RFC1123))
					fmt.Printf("MAVLink version: %s\n", frm.Frame.GetVersion())
					fmt.Printf("Drone status: %v\n", msg.SystemStatus)

				case *ardupilotmega.MessageSysStatus: // Id 1
					fmt.Println("System status message from drone")
					fmt.Printf("Mainloop time used %d%%, batt voltage: %d, batt current: %d, batt remain: %d\n",
						msg.Load/10, msg.VoltageBattery, msg.CurrentBattery, msg.BatteryRemaining)
					fmt.Printf("Comm drop rate: %d, comm errors: %d\n", msg.DropRateComm, msg.ErrorsComm)

				case *ardupilotmega.MessageSystemTime: // Id 2
					fmt.Println("System time message received from drone")
					sinceBoot, err := time.ParseDuration(fmt.Sprintf("%dms", msg.TimeBootMs))
					if err != nil {
						panic(err)
					}
					fmt.Printf("Time since drone boot: %s\n", sinceBoot)

				case *ardupilotmega.MessageParamValue: // Id 22
					fmt.Printf("Param message from drone with parameter %s (index %d) value %v\n",
						msg.ParamId, msg.ParamIndex, msg.ParamValue)

				case *ardupilotmega.MessageGpsRawInt: // Id 24
					sinceBoot, err := time.ParseDuration(fmt.Sprintf("%dus", msg.TimeUsec))
					if err != nil {
						panic(err)
					}
					fmt.Printf("Raw GPS values message sent from drone at: %v since boot\n", sinceBoot)
					fmt.Printf("GPS fix type: %v, Lat: %v, Lon: %v, Alt: %v\n", msg.FixType, msg.Lat, msg.Lon, msg.Alt)
					fmt.Printf("HDOP: %v, HDVP: %v, Ground speed: %v, Cog: %v\n", msg.Eph, msg.Epv, msg.Vel, msg.Cog)
					fmt.Printf("# visible sats: %v, AltEllipsoid: %v, Pos uncert: %v, Alt uncert: %v, Spd uncert: %v\n",
						msg.SatellitesVisible, msg.AltEllipsoid, msg.HAcc, msg.VAcc, msg.VelAcc)
					fmt.Printf("Head uncert: %v, Yaw from north: %v\n", msg.HdgAcc, msg.Yaw)

				case *ardupilotmega.MessageRawImu: // Id 27
					sinceBoot, err := time.ParseDuration(fmt.Sprintf("%dus", msg.TimeUsec))
					if err != nil {
						panic(err)
					}
					fmt.Printf("Raw IMU values message sent from drone at: %v since boot\n", sinceBoot)
					fmt.Printf("Accel - x: %d, y: %d, z: %d\n", msg.Xacc, msg.Yacc, msg.Zacc)
					fmt.Printf("Angular speed around - x: %d, y: %d, z: %d\n", msg.Xgyro, msg.Ygyro, msg.Zgyro)
					fmt.Printf("Magnetic field - x: %d, y: %d, z: %d\n", msg.Xmag, msg.Ymag, msg.Zmag)
					fmt.Printf("IMU # %d, Temp: %d\n", msg.Id, msg.Temperature)

				case *ardupilotmega.MessageScaledPressure: // Id 29
					sinceBoot, err := time.ParseDuration(fmt.Sprintf("%dms", msg.TimeBootMs))
					if err != nil {
						panic(err)
					}
					fmt.Printf("Scaled pressure values message sent from drone at: %s after boot\n", sinceBoot)
					// TODO: Find out how to interpret the "Temperature" value.  It's reporting "4012" just now, on a day
					//       here in Aust that's probably only about 20 degress C at most.
					fmt.Printf("Pressure - Abs: %0.2f, Diff: %0.2f, Temp: %d\n", msg.PressAbs, msg.PressDiff, msg.Temperature)

				case *ardupilotmega.MessageAttitude: // Id 30
					sinceBoot, err := time.ParseDuration(fmt.Sprintf("%dms", msg.TimeBootMs))
					if err != nil {
						panic(err)
					}
					fmt.Printf("Attitude message sent from drone at: %s after boot\n", sinceBoot)
					fmt.Printf("Angles - Roll: %0.2f, Pitch: %0.2f, Yaw: %0.2f\n", msg.Roll, msg.Pitch, msg.Yaw)
					fmt.Printf("Angular speed - Roll: %0.2f, Pitch: %0.2f, Yaw: %0.2f\n", msg.Rollspeed,
						msg.Pitchspeed, msg.Yawspeed)

				case *ardupilotmega.MessageGlobalPositionInt: // Id 33
					sinceBoot, err := time.ParseDuration(fmt.Sprintf("%dms", msg.TimeBootMs))
					if err != nil {
						panic(err)
					}
					fmt.Printf("Filtered GPS Postion message sent from drone at: %s after boot\n", sinceBoot)
					// TODO: Find out what the units of measurement/display are for these values
					fmt.Printf("Lat: %d, Lon: %d, Alt (MSL): %d, Alt above gnd: %d\n", msg.Lat, msg.Lon,
						msg.Alt, msg.RelativeAlt)
					fmt.Printf("Ground speed - x: %d, y: %d, z: %d\n", msg.Vx, msg.Vy, msg.Vz)
					fmt.Printf("Vehicle heading: %0.2f°\n", float32(msg.Hdg)/100.0)

				case *ardupilotmega.MessageRcChannelsScaled: // Id 34
					sinceBoot, err := time.ParseDuration(fmt.Sprintf("%dms", msg.TimeBootMs))
					if err != nil {
						panic(err)
					}
					fmt.Printf("Scaled RC Channel values message sent from drone at: %s after boot\n", sinceBoot)
					fmt.Printf("Servo output port: %d, Receive signal str: %d\n", msg.Port, msg.Rssi)
					fmt.Printf("RC channel values - 1: %d, 2: %d, 3: %d, 4: %d\n", msg.Chan1Scaled,
						msg.Chan2Scaled, msg.Chan3Scaled, msg.Chan4Scaled)
					fmt.Printf("RC channel values - 5: %d, 6: %d, 7: %d, 8: %d\n", msg.Chan5Scaled,
						msg.Chan6Scaled, msg.Chan7Scaled, msg.Chan8Scaled)

				case *ardupilotmega.MessageRcChannelsRaw: // Id 35
					sinceBoot, err := time.ParseDuration(fmt.Sprintf("%dms", msg.TimeBootMs))
					if err != nil {
						panic(err)
					}
					fmt.Printf("RC Channels Raw message sent from drone at: %s after boot\n", sinceBoot)
					fmt.Printf("Servo output port: %d, Receive signal str: %d\n", msg.Port, msg.Rssi)
					fmt.Printf("RC channel values - 1: %d, 2: %d, 3: %d, 4: %d\n", msg.Chan1Raw, msg.Chan2Raw,
						msg.Chan3Raw, msg.Chan4Raw)
					fmt.Printf("RC channel values - 5: %d, 6: %d, 7: %d, 8: %d\n", msg.Chan5Raw, msg.Chan6Raw,
						msg.Chan7Raw, msg.Chan8Raw)

				case *ardupilotmega.MessageServoOutputRaw: // Id 36
					sinceBoot, err := time.ParseDuration(fmt.Sprintf("%dus", msg.TimeUsec))
					if err != nil {
						panic(err)
					}
					fmt.Printf("Raw Servo Output message sent from drone at: %s after boot\n", sinceBoot)
					fmt.Printf("Servo output port: %d\n", msg.Port)
					fmt.Printf("Servo output values - 1: %d, 2: %d, 3: %d, 4: %d\n", msg.Servo1Raw, msg.Servo2Raw,
						msg.Servo3Raw, msg.Servo4Raw)
					fmt.Printf("Servo output values - 5: %d, 6: %d, 7: %d, 8: %d\n", msg.Servo5Raw, msg.Servo6Raw,
						msg.Servo7Raw, msg.Servo8Raw)
					fmt.Printf("Servo output values - 9: %d, 10: %d, 11: %d, 12: %d\n", msg.Servo9Raw,
						msg.Servo10Raw, msg.Servo11Raw, msg.Servo12Raw)
					fmt.Printf("Servo output values - 13: %d, 14: %d, 15: %d, 16: %d\n", msg.Servo13Raw,
						msg.Servo14Raw, msg.Servo15Raw, msg.Servo16Raw)

				case *ardupilotmega.MessageMissionCurrent: // Id 42
					fmt.Printf("Drone message status current mission is # %d\n", msg.Seq)

				case *ardupilotmega.MessageMissionCount: // Id 44
					fmt.Printf("Mission count message from drone at %v\n", time.Now().Format(time.RFC1123))
					fmt.Printf("Target system: %d, component: %d, # missions: %d, mission type: %s\n",
						msg.TargetSystem, msg.TargetComponent, msg.Count, msg.MissionType)

				case *ardupilotmega.MessageNavControllerOutput: // Id 62
					fmt.Printf("Navigation state controller output message received from drone at: %s\n",
						time.Now().Format(time.RFC1123))
					fmt.Printf("Current desired values - Roll: %0.2f, Pitch: %0.2f, Bearing: %d\n",
						msg.NavRoll, msg.NavPitch, msg.NavBearing)
					fmt.Printf("Bearing to waypoing: %d, Dist to waypoint: %d\n", msg.TargetBearing, msg.WpDist)
					fmt.Printf("Current errors - Alt: %0.2f, Airspeed: %0.2f, X-Y crosstrack: %0.2f\n",
						msg.AltError, msg.AspdError, msg.XtrackError)

				case *ardupilotmega.MessageRcChannels: // Id 65
					sinceBoot, err := time.ParseDuration(fmt.Sprintf("%dms", msg.TimeBootMs))
					if err != nil {
						panic(err)
					}
					fmt.Printf("RC channels message sent from drone at: %s after boot\n", sinceBoot)
					fmt.Printf("Channel count: %d, Receive signal str: %d\n", msg.Chancount, msg.Rssi)
					fmt.Printf("RC Channel values - 1: %d, 2: %d, 3: %d, 4: %d\n", msg.Chan1Raw, msg.Chan2Raw,
						msg.Chan3Raw, msg.Chan4Raw)
					fmt.Printf("RC Channel values - 5: %d, 6: %d, 7: %d, 8: %d\n", msg.Chan5Raw, msg.Chan6Raw,
						msg.Chan7Raw, msg.Chan8Raw)
					fmt.Printf("RC Channel values - 9: %d, 10: %d, 11: %d, 12: %d\n", msg.Chan9Raw,
						msg.Chan10Raw, msg.Chan11Raw, msg.Chan12Raw)
					fmt.Printf("RC Channel values - 13: %d, 14: %d, 15: %d, 16: %d\n", msg.Chan13Raw,
						msg.Chan14Raw, msg.Chan15Raw, msg.Chan16Raw)
					fmt.Printf("RC Channel values - 17: %d, 18: %d\n", msg.Chan17Raw,
						msg.Chan18Raw)

				case *ardupilotmega.MessageMissionItemInt: // Id 73
					fmt.Printf("Mission item message received from drone at: %s\n",
						time.Now().Format(time.RFC1123))
					fmt.Printf("Target; System: %d, component: %d\n", msg.TargetSystem, msg.TargetComponent)
					fmt.Printf("Waypoint ID: %d, frame type: %d, sched action: %d, current? %d, autocont: %d\n",
						msg.Seq, msg.Frame, msg.Command, msg.Current, msg.Autocontinue)
					fmt.Printf("Command params: 1: %0.2f, 2: %0.2f, 3: %0.2f, 4: %0.2f\n", msg.Param1,
						msg.Param2, msg.Param3, msg.Param4)
					fmt.Printf("Command params: 5: %d, 6: %d, 7: %0.2f\n", msg.X, msg.Y, msg.Z)
					fmt.Printf("Mission type: %d\n", msg.MissionType)

				case *ardupilotmega.MessageVfrHud: // Id 74
					fmt.Printf("HUD Metrics message received from drone at: %s\n",
						time.Now().Format(time.RFC1123))
					fmt.Printf("Vehicle speed: %0.2f, Ground speed: %0.2f, Throttle setting: %d%%\n",
						msg.Airspeed, msg.Groundspeed, msg.Throttle)
					fmt.Printf("Compass heading: %d, Alt: %0.2f, Climb rate: %0.2f\n", msg.Heading, msg.Alt,
						msg.Climb)

				case *ardupilotmega.MessageCommandAck: // Id 77
					fmt.Printf("Command Ack message from drone at %v\n", time.Now().Format(time.RFC1123))
					fmt.Printf("Command ID: %s, Result: %s, Progress: %d\n", msg.Command, msg.Result, msg.Progress)
					fmt.Printf("Result Param2: %d, Target system: %d, component: %d\n", msg.ResultParam2, msg.TargetSystem, msg.TargetComponent)

				case *ardupilotmega.MessageScaledImu2: // Id 116
					sinceBoot, err := time.ParseDuration(fmt.Sprintf("%dms", msg.TimeBootMs))
					if err != nil {
						panic(err)
					}
					fmt.Printf("RAW IMU2 message sent from drone at: %s after boot\n", sinceBoot)
					fmt.Printf("Accel - x: %d, y: %d, z: %d\n", msg.Xacc, msg.Yacc, msg.Zacc)
					fmt.Printf("Angular speed around - x: %d, y: %d, z: %d\n", msg.Xgyro, msg.Ygyro, msg.Zgyro)
					fmt.Printf("Magnetic field - x: %d, y: %d, z: %d\n", msg.Xmag, msg.Ymag, msg.Zmag)
					fmt.Printf("Temp: %d\n", msg.Temperature)

				case *ardupilotmega.MessagePowerStatus: // Id 125
					fmt.Printf("Power status message from drone at %v\n", time.Now().Format(time.RFC1123))
					fmt.Printf("5V rail voltage: %.2fv, Servo rail voltage: %.2fv, flags: %b\n",
						float32(msg.Vcc)/1000.0, float32(msg.Vservo)/1000.0, msg.Flags)

				case *ardupilotmega.MessageAutopilotVersion: // Id 148
					fmt.Printf("Autopilot version message from drone at %v\n", time.Now().Format(time.RFC1123))
					fmt.Println("Capabilities:")
					if msg.Capabilities&ardupilotmega.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT != 0 {
						fmt.Println("  * MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT: Set")
					} else {
						fmt.Println("  * MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT: Unset")
					}
					if msg.Capabilities&ardupilotmega.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT != 0 {
						fmt.Println("  * MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT: Set")
					} else {
						fmt.Println("  * MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT: Unset")
					}
					if msg.Capabilities&ardupilotmega.MAV_PROTOCOL_CAPABILITY_MISSION_INT != 0 {
						fmt.Println("  * MAV_PROTOCOL_CAPABILITY_MISSION_INT: Set")
					} else {
						fmt.Println("  * MAV_PROTOCOL_CAPABILITY_MISSION_INT: Unset")
					}
					if msg.Capabilities&ardupilotmega.MAV_PROTOCOL_CAPABILITY_COMMAND_INT != 0 {
						fmt.Println("  * MAV_PROTOCOL_CAPABILITY_COMMAND_INT: Set")
					} else {
						fmt.Println("  * MAV_PROTOCOL_CAPABILITY_COMMAND_INT: Unset")
					}
					if msg.Capabilities&ardupilotmega.MAV_PROTOCOL_CAPABILITY_PARAM_UNION != 0 {
						fmt.Println("  * MAV_PROTOCOL_CAPABILITY_PARAM_UNION: Set")
					} else {
						fmt.Println("  * MAV_PROTOCOL_CAPABILITY_PARAM_UNION: Unset")
					}
					if msg.Capabilities&ardupilotmega.MAV_PROTOCOL_CAPABILITY_FTP != 0 {
						fmt.Println("  * MAV_PROTOCOL_CAPABILITY_FTP: Set")
					} else {
						fmt.Println("  * MAV_PROTOCOL_CAPABILITY_FTP: Unset")
					}
					if msg.Capabilities&ardupilotmega.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET != 0 {
						fmt.Println("  * MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET: Set")
					} else {
						fmt.Println("  * MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET: Unset")
					}
					if msg.Capabilities&ardupilotmega.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED != 0 {
						fmt.Println("  * MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED: Set")
					} else {
						fmt.Println("  * MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED: Unset")
					}
					if msg.Capabilities&ardupilotmega.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT != 0 {
						fmt.Println("  * MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT: Set")
					} else {
						fmt.Println("  * MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT: Unset")
					}
					if msg.Capabilities&ardupilotmega.MAV_PROTOCOL_CAPABILITY_TERRAIN != 0 {
						fmt.Println("  * MAV_PROTOCOL_CAPABILITY_TERRAIN: Set")
					} else {
						fmt.Println("  * MAV_PROTOCOL_CAPABILITY_TERRAIN: Unset")
					}
					if msg.Capabilities&ardupilotmega.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET != 0 {
						fmt.Println("  * MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET: Set")
					} else {
						fmt.Println("  * MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET: Unset")
					}
					if msg.Capabilities&ardupilotmega.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION != 0 {
						fmt.Println("  * MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION: Set")
					} else {
						fmt.Println("  * MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION: Unset")
					}
					if msg.Capabilities&ardupilotmega.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION != 0 {
						fmt.Println("  * MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION: Set")
					} else {
						fmt.Println("  * MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION: Unset")
					}
					if msg.Capabilities&ardupilotmega.MAV_PROTOCOL_CAPABILITY_MAVLINK2 != 0 {
						fmt.Println("  * MAV_PROTOCOL_CAPABILITY_MAVLINK2: Set")
					} else {
						fmt.Println("  * MAV_PROTOCOL_CAPABILITY_MAVLINK2: Unset")
					}
					if msg.Capabilities&ardupilotmega.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE != 0 {
						fmt.Println("  * MAV_PROTOCOL_CAPABILITY_MISSION_FENCE: Set")
					} else {
						fmt.Println("  * MAV_PROTOCOL_CAPABILITY_MISSION_FENCE: Unset")
					}
					if msg.Capabilities&ardupilotmega.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY != 0 {
						fmt.Println("  * MAV_PROTOCOL_CAPABILITY_MISSION_RALLY: Set")
					} else {
						fmt.Println("  * MAV_PROTOCOL_CAPABILITY_MISSION_RALLY: Unset")
					}
					if msg.Capabilities&ardupilotmega.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION != 0 {
						fmt.Println("  * MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION: Set")
					} else {
						fmt.Println("  * MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION: Unset")
					}
					fmt.Printf("Firmware ver: %d, middleware ver: %d, OS ver: %d\n", msg.FlightSwVersion,
						msg.MiddlewareSwVersion, msg.OsSwVersion)
					fmt.Printf("HW/board ver: %d, Flight cust ver: %s, Middleware cust: %s, OS cust ver: %s\n",
						msg.BoardVersion, msg.FlightCustomVersion, msg.MiddlewareCustomVersion, msg.OsCustomVersion)
					fmt.Printf("Board vend ID: %d, Prod ID: %d, UID: %d, UID2: %s\n", msg.VendorId,
						msg.ProductId, msg.Uid, msg.Uid2)

				case *ardupilotmega.MessageSensorOffsets: // Id 150
					fmt.Printf("Hardware sensor offset and calibration values message received from drone at: %s\n",
						time.Now().Format(time.RFC1123))
					fmt.Printf("Magnetometer offsets - x: %d, y: %d, z: %d, declination: %0.2f\n", msg.MagOfsX,
						msg.MagOfsY, msg.MagOfsZ, msg.MagDeclination)
					fmt.Printf("Barometer - raw pressure: %d, raw temp: %d\n", msg.RawPress, msg.RawTemp)
					fmt.Printf("Gyro calibration - x: %0.2f, y: %0.2f, z: %0.2f\n", msg.GyroCalX, msg.GyroCalY,
						msg.GyroCalZ)
					fmt.Printf("Accel calibration - x: %0.2f, y: %0.2f, z: %0.2f\n", msg.AccelCalX,
						msg.AccelCalY, msg.AccelCalZ)

				case *ardupilotmega.MessageMeminfo: // Id 152
					fmt.Printf("Memory info message from drone. Heap top: %d, free mem: %d, free mem32: %d\n",
						msg.Brkval, msg.Freemem, msg.Freemem32) // TODO: Find out the units of measurement for these values

				case *ardupilotmega.MessageAhrs: // Id 163
					fmt.Printf("AHRS values message received from drone at: %s\n",
						time.Now().Format(time.RFC1123))
					fmt.Printf("Gyro drift est - x: %0.2f, y: %0.2f, z: %0.2f\n", msg.Omegaix, msg.Omegaiy,
						msg.Omegaiz)
					fmt.Printf("Avg - accel-wgt: %0.2f, renorm: %0.2f, err-roll-pitch: %0.2f, err-yaw %0.2f\n",
						msg.AccelWeight, msg.RenormVal, msg.ErrorRp, msg.ErrorYaw)

				case *ardupilotmega.MessageHwstatus: // Id 165
					fmt.Printf("Hardware status message from drone.  Board voltage: %.2fv, I2C err count: %d\n",
						float32(msg.Vcc)/1000.0, msg.I2cerr)

				case *ardupilotmega.MessageAhrs2: // Id 178
					fmt.Printf("AHRS2 values message received from drone at: %s\n",
						time.Now().Format(time.RFC1123))
					fmt.Printf("Angles - Roll: %0.2f, Pitch: %0.2f, Yaw: %0.2f\n", msg.Roll, msg.Pitch, msg.Yaw)
					fmt.Printf("Alt: %0.2f, Lat: %d, Lon: %d\n", msg.Altitude, msg.Lat, msg.Lng)

				case *ardupilotmega.MessageAhrs3: // Id 182
					fmt.Printf("AHRS3 values message received from drone at: %s\n", time.Now().Format(time.RFC1123))
					fmt.Printf("Angles - Roll: %0.2f, Pitch: %0.2f, Yaw: %0.2f\n", msg.Roll, msg.Pitch, msg.Yaw)
					fmt.Printf("Alt: %0.2f, Lat: %d, Lon: %d\n", msg.Altitude, msg.Lat, msg.Lng)
					fmt.Printf("Test vars - 1: %0.2f, 2: %0.2f, 3: %0.2f, 4: %0.2f\n", msg.V1, msg.V2, msg.V3,
						msg.V4)

				case *ardupilotmega.MessageEkfStatusReport: // Id 193
					fmt.Printf("EKF Status message received from drone at: %s\n", time.Now().Format(time.RFC1123))
					fmt.Println("Flags:")
					if msg.Flags&ardupilotmega.EKF_ATTITUDE != 0 {
						fmt.Println("  * EKF_ATTITUDE: Set")
					} else {
						fmt.Println("  * EKF_ATTITUDE: Unset")
					}
					if msg.Flags&ardupilotmega.EKF_VELOCITY_HORIZ != 0 {
						fmt.Println("  * EKF_VELOCITY_HORIZ: Set")
					} else {
						fmt.Println("  * EKF_VELOCITY_HORIZ: Unset")
					}
					if msg.Flags&ardupilotmega.EKF_VELOCITY_VERT != 0 {
						fmt.Println("  * EKF_VELOCITY_VERT: Set")
					} else {
						fmt.Println("  * EKF_VELOCITY_VERT: Unset")
					}
					if msg.Flags&ardupilotmega.EKF_POS_HORIZ_REL != 0 {
						fmt.Println("  * EKF_POS_HORIZ_REL: Set")
					} else {
						fmt.Println("  * EKF_POS_HORIZ_REL: Unset")
					}
					if msg.Flags&ardupilotmega.EKF_POS_HORIZ_ABS != 0 {
						fmt.Println("  * EKF_POS_HORIZ_ABS: Set")
					} else {
						fmt.Println("  * EKF_POS_HORIZ_ABS: Unset")
					}
					if msg.Flags&ardupilotmega.EKF_POS_VERT_ABS != 0 {
						fmt.Println("  * EKF_POS_VERT_ABS: Set")
					} else {
						fmt.Println("  * EKF_POS_VERT_ABS: Unset")
					}
					if msg.Flags&ardupilotmega.EKF_POS_VERT_AGL != 0 {
						fmt.Println("  * EKF_POS_VERT_AGL: Set")
					} else {
						fmt.Println("  * EKF_POS_VERT_AGL: Unset")
					}
					if msg.Flags&ardupilotmega.EKF_CONST_POS_MODE != 0 {
						fmt.Println("  * EKF_CONST_POS_MODE: Set")
					} else {
						fmt.Println("  * EKF_CONST_POS_MODE: Unset")
					}
					if msg.Flags&ardupilotmega.EKF_PRED_POS_HORIZ_REL != 0 {
						fmt.Println("  * EKF_PRED_POS_HORIZ_REL: Set")
					} else {
						fmt.Println("  * EKF_PRED_POS_HORIZ_REL: Unset")
					}
					if msg.Flags&ardupilotmega.EKF_PRED_POS_HORIZ_ABS != 0 {
						fmt.Println("  * EKF_PRED_POS_HORIZ_ABS: Set")
					} else {
						fmt.Println("  * EKF_PRED_POS_HORIZ_ABS: Unset")
					}
					if msg.Flags&ardupilotmega.EKF_UNINITIALIZED != 0 {
						fmt.Println("  * EKF_UNINITIALIZED: Set")
					} else {
						fmt.Println("  * EKF_UNINITIALIZED: Unset")
					}
					fmt.Printf("Variances - Velocity: %0.2f, HPos: %0.2f, VPos: %0.2f\n", msg.VelocityVariance,
						msg.PosHorizVariance, msg.PosVertVariance)
					fmt.Printf("Variances - Compass: %0.2f, Terrain: %0.2f, Airspeed: %0.2f\n",
						msg.CompassVariance, msg.TerrainAltVariance, msg.AirspeedVariance)

				case *ardupilotmega.MessageVibration: // Id 241
					fmt.Printf("Vibration info message from drone at: %v\n",
						time.Unix(int64(msg.TimeUsec), 0).Format(time.RFC1123))
					fmt.Printf("Axis vibration levels - x: %.2f y: %.2f z: %.2f\n",
						msg.VibrationX, msg.VibrationY, msg.VibrationZ)
					fmt.Printf("Accelerometer clipping counts - 1st: %d 2nd: %d 3rd %d\n",
						msg.Clipping_0, msg.Clipping_1, msg.Clipping_2)

				case *ardupilotmega.MessageStatustext: // Id 253
					fmt.Printf("Text status message from drone at: %v\n", time.Now().Format(time.RFC1123))
					fmt.Printf("Severity: %s, ID: %d, ChunkSeq: %d\n", msg.Severity, msg.Id, msg.ChunkSeq)
					fmt.Printf("Text: %s\n", msg.Text)

				default:
					fmt.Printf("Undecoded message frame received from drone at: %v\n", time.Now().Format(time.RFC1123))
					fmt.Printf("MAVLink version: %d\n", frm.Frame.GetVersion())
					fmt.Printf("System ID: %v\n", frm.SystemId())
					fmt.Printf("Component ID: %v\n", frm.ComponentId())
					fmt.Printf("Message ID: %d\n", frm.Message().GetId())
					fmt.Printf("Message: %+v\n", frm.Message())
					fmt.Printf("Checksum: %v\n", frm.Frame.GetChecksum())
				}

			case 255:
				// This is a message from the Ground Control Station
				switch msg := frm.Message().(type) {
				case *ardupilotmega.MessageHeartbeat: // Id 0
					fmt.Printf("Heartbeat from Ground Control Station at %v\n", time.Now().Format(time.RFC1123))
					fmt.Printf("MAVLink version: %s\n", frm.Frame.GetVersion())
					fmt.Printf("GCS System status: %v\n", msg.SystemStatus)

				case *ardupilotmega.MessageSystemTime: // Id 2
					fmt.Println("System time message received from GCS")
					fmt.Printf("Epoch time (%v), time since boot (%d milliseconds)\n",
						time.Unix(int64(msg.TimeUnixUsec/1000000), int64(msg.TimeUnixUsec%1000000)).Format(time.RFC1123), msg.TimeBootMs)

				case *ardupilotmega.MessageSetMode: // Id 11
					fmt.Printf("Set system mode message from GCS at %v\n", time.Now().Format(time.RFC1123))
					fmt.Printf("Target system: %d", msg.TargetSystem)
					fmt.Printf(", Base mode: ")
					switch msg.BaseMode {
					case ardupilotmega.MAV_MODE_PREFLIGHT:
						// System is not ready to fly, booting, calibrating, etc. No flag is set.
						fmt.Printf("Preflight")
					case ardupilotmega.MAV_MODE_STABILIZE_DISARMED:
						// System is allowed to be active, under assisted RC control.
						fmt.Printf("Disarmed")
					case ardupilotmega.MAV_MODE_STABILIZE_ARMED:
						// System is allowed to be active, under assisted RC control.
						fmt.Printf("Armed")
					case ardupilotmega.MAV_MODE_MANUAL_DISARMED:
						// System is allowed to be active, under manual (RC) control, no stabilization
						fmt.Printf("Manual disarmed")
					case ardupilotmega.MAV_MODE_MANUAL_ARMED:
						// System is allowed to be active, under manual (RC) control, no stabilization
						fmt.Printf("Manual armed")
					case ardupilotmega.MAV_MODE_GUIDED_DISARMED:
						// System is allowed to be active, under autonomous control, manual setpoint
						fmt.Printf("Guided disarmed")
					case ardupilotmega.MAV_MODE_GUIDED_ARMED:
						// System is allowed to be active, under autonomous control, manual setpoint
						fmt.Printf("Guided armed")
					case ardupilotmega.MAV_MODE_AUTO_DISARMED:
						// System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by waypoints)
						fmt.Printf("Auto disarmed")
					case ardupilotmega.MAV_MODE_AUTO_ARMED:
						// System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by waypoints)
						fmt.Printf("Auto armed")
					case ardupilotmega.MAV_MODE_TEST_DISARMED:
						// UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only.
						fmt.Printf("Test disarmed")
					case ardupilotmega.MAV_MODE_TEST_ARMED:
						// UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only.
						fmt.Printf("Test armed")
					default:
						fmt.Printf("Unknown")
					}
					fmt.Printf(", Custom mode: %d\n", msg.CustomMode)

				case *ardupilotmega.MessageParamRequestRead: // Id 20
					if msg.ParamId == "" {
						fmt.Printf("Request from GCS for parameter %d from target system %d, component %d\n",
							msg.ParamIndex, msg.TargetSystem, msg.TargetComponent)
					} else {
						fmt.Printf("Request from GCS for parameter '%s' from target system %d, component %d\n",
							msg.ParamId, msg.TargetSystem, msg.TargetComponent)
					}

				case *ardupilotmega.MessageParamRequestList: // Id 21
					fmt.Printf("Request from GCS for all parameters from target system %d, component %d\n",
						msg.TargetSystem, msg.TargetComponent)

				case *ardupilotmega.MessageMissionRequestList: // Id 43
					fmt.Printf("Request from GCS for the overall list of mission items (type %s) from target system %d, component %d\n",
						msg.MissionType, msg.TargetSystem, msg.TargetComponent)

				case *ardupilotmega.MessageMissionAck: // Id 47
					fmt.Printf("Ack message (mission type %d, result %s) during waypoint handling from GCS for target system %d, component %d\n",
						msg.MissionType, msg.Type, msg.TargetSystem, msg.TargetComponent)

				case *ardupilotmega.MessageMissionRequestInt: // Id 51
					fmt.Printf("Request from GCS for mission item %d, type %s from target system %d, component %d\n",
						msg.Seq, msg.MissionType, msg.TargetSystem, msg.TargetComponent)

				case *ardupilotmega.MessageRequestDataStream: // Id 66
					var startOrStop string
					if msg.StartStop == 1 {
						startOrStop = "start"
					} else {
						startOrStop = "stop"
					}
					fmt.Printf("RequestDataStream (%s) message from GCS, target system %d, component %d\n", startOrStop, msg.TargetSystem, msg.TargetSystem)

				case *ardupilotmega.MessageCommandLong: // Id 76
					fmt.Printf("Command long ('%s') message received from GCS for target system %d, component %d\n",
						msg.Command, msg.TargetSystem, msg.TargetComponent)

				default:
					fmt.Printf("Undecoded message frame received from GCS at: %v\n", time.Now().Format(time.RFC1123))
					fmt.Printf("MAVLink version: %d\n", frm.Frame.GetVersion())
					fmt.Printf("System ID: %v\n", frm.SystemId())
					fmt.Printf("Component ID: %v\n", frm.ComponentId())
					fmt.Printf("Message ID: %d\n", frm.Message().GetId())
					fmt.Printf("Message: %+v\n", frm.Message())
					fmt.Printf("Checksum: %v\n", frm.Frame.GetChecksum())
				}
			}
			fmt.Println()

			// Route frame to every other channel
			node.WriteFrameExcept(frm.Channel, frm.Frame)
		}
	}
}
