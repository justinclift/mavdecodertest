package main

import (
	"fmt"
	"os"
	"time"

	"github.com/aler9/gomavlib"
	"github.com/aler9/gomavlib/dialects/ardupilotmega"
)

func main() {
	// Proxy (or man-in-the-middle ;>) between my rpi connected to a Pixhawk, and QGroundControl running on my local
	// desktop.  At the moment this simply captures the first few frames, prints their details, then exits.
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

	i := 0
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

					fmt.Println()

				case *ardupilotmega.MessageSysStatus: // Id 1
					fmt.Println("System status message from drone")
					fmt.Printf("Mainloop time used %d%%, batt voltage: %d, batt current: %d, batt remain: %d\n",
						msg.Load/10, msg.VoltageBattery, msg.CurrentBattery, msg.BatteryRemaining)
					fmt.Printf("Comm drop rate: %d, comm errors: %d\n", msg.DropRateComm, msg.ErrorsComm)

					fmt.Println()

				case *ardupilotmega.MessageSystemTime: // Id 2
					fmt.Println("System time message received from drone")
					sinceBoot, err := time.ParseDuration(fmt.Sprintf("%dms", msg.TimeBootMs))
					if err != nil {
						panic(err)
					}
					fmt.Printf("Time since drone boot: %s\n", sinceBoot)

					fmt.Println()

				case *ardupilotmega.MessageParamValue: // Id 22
					fmt.Printf("Param message from drone with parameter %s (index %d) value %v\n",
						msg.ParamId, msg.ParamIndex, msg.ParamValue)

					fmt.Println()

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

					fmt.Println()

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

					fmt.Println()

				case *ardupilotmega.MessageScaledPressure: // Id 29
					sinceBoot, err := time.ParseDuration(fmt.Sprintf("%dms", msg.TimeBootMs))
					if err != nil {
						panic(err)
					}
					fmt.Printf("Scaled pressure values message sent from drone at: %s after boot\n", sinceBoot)
					// TODO: Find out how to interpret the "Temperature" value.  It's reporting "4012" just now, on a day
					//       here in Aust that's probably only about 20 degress C at most.
					fmt.Printf("Pressure - Abs: %0.2f, Diff: %0.2f, Temp: %d\n", msg.PressAbs, msg.PressDiff, msg.Temperature)

					fmt.Println()

				case *ardupilotmega.MessageAttitude: // Id 30
					sinceBoot, err := time.ParseDuration(fmt.Sprintf("%dms", msg.TimeBootMs))
					if err != nil {
						panic(err)
					}
					fmt.Printf("Attitude message sent from drone at: %s after boot\n", sinceBoot)
					fmt.Printf("Angles - Roll: %0.2f, Pitch: %0.2f, Yaw: %0.2f\n", msg.Roll, msg.Pitch, msg.Yaw)
					fmt.Printf("Angular speed - Roll: %0.2f, Pitch: %0.2f, Yaw: %0.2f\n", msg.Rollspeed,
						msg.Pitchspeed, msg.Yawspeed)

					fmt.Println()

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

					fmt.Println()

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

					fmt.Println()

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

					fmt.Println()

				case *ardupilotmega.MessageMissionCurrent: // Id 42
					fmt.Printf("Drone message status current mission is # %d\n", msg.Seq)

					fmt.Println()

				// TODO: Fill out the details for these
				case *ardupilotmega.MessageNavControllerOutput: // Id 62
					fmt.Printf("Navigation state controller output message received from drone at: %s\n", time.Now().Format(time.RFC1123))
					//// Current desired roll
					//NavRoll float32
					//// Current desired pitch
					//NavPitch float32
					//// Current desired heading
					//NavBearing int16
					//// Bearing to current waypoint/target
					//TargetBearing int16
					//// Distance to active waypoint
					//WpDist uint16
					//// Current altitude error
					//AltError float32
					//// Current airspeed error
					//AspdError float32
					//// Current crosstrack error on x-y plane
					//XtrackError float32

					fmt.Println()

				case *ardupilotmega.MessageRcChannels: // Id 65
					sinceBoot, err := time.ParseDuration(fmt.Sprintf("%dms", msg.TimeBootMs))
					if err != nil {
						panic(err)
					}
					fmt.Printf("RC channels message sent from drone at: %s after boot\n", sinceBoot)
					//// Timestamp (time since system boot).
					//TimeBootMs uint32
					//// Total number of RC channels being received. This can be larger than 18, indicating that more channels are available but not given in this message. This value should be 0 when no RC channels are available.
					//Chancount uint8
					//// RC channel 1 value.
					//Chan1Raw uint16
					//// RC channel 2 value.
					//Chan2Raw uint16
					//// RC channel 3 value.
					//Chan3Raw uint16
					//// RC channel 4 value.
					//Chan4Raw uint16
					//// RC channel 5 value.
					//Chan5Raw uint16
					//// RC channel 6 value.
					//Chan6Raw uint16
					//// RC channel 7 value.
					//Chan7Raw uint16
					//// RC channel 8 value.
					//Chan8Raw uint16
					//// RC channel 9 value.
					//Chan9Raw uint16
					//// RC channel 10 value.
					//Chan10Raw uint16
					//// RC channel 11 value.
					//Chan11Raw uint16
					//// RC channel 12 value.
					//Chan12Raw uint16
					//// RC channel 13 value.
					//Chan13Raw uint16
					//// RC channel 14 value.
					//Chan14Raw uint16
					//// RC channel 15 value.
					//Chan15Raw uint16
					//// RC channel 16 value.
					//Chan16Raw uint16
					//// RC channel 17 value.
					//Chan17Raw uint16
					//// RC channel 18 value.
					//Chan18Raw uint16
					//// Receive signal strength indicator in device-dependent units/scale. Values: [0-254], 255: invalid/unknown.
					//Rssi uint8

					fmt.Println()

				case *ardupilotmega.MessageVfrHud: // Id 74
					fmt.Printf("HUD Metrics message received from drone at: %s\n", time.Now().Format(time.RFC1123))
					//// Vehicle speed in form appropriate for vehicle type. For standard aircraft this is typically calibrated airspeed (CAS) or indicated airspeed (IAS) - either of which can be used by a pilot to estimate stall speed.
					//Airspeed float32
					//// Current ground speed.
					//Groundspeed float32
					//// Current heading in compass units (0-360, 0=north).
					//Heading int16
					//// Current throttle setting (0 to 100).
					//Throttle uint16
					//// Current altitude (MSL).
					//Alt float32
					//// Current climb rate.
					//Climb float32

					fmt.Println()

				case *ardupilotmega.MessageScaledImu2: // Id 116
					sinceBoot, err := time.ParseDuration(fmt.Sprintf("%dms", msg.TimeBootMs))
					if err != nil {
						panic(err)
					}
					fmt.Printf("RAW IMU2 message sent from drone at: %s after boot\n", sinceBoot)
					//// Timestamp (time since system boot).
					//TimeBootMs uint32
					//// X acceleration
					//Xacc int16
					//// Y acceleration
					//Yacc int16
					//// Z acceleration
					//Zacc int16
					//// Angular speed around X axis
					//Xgyro int16
					//// Angular speed around Y axis
					//Ygyro int16
					//// Angular speed around Z axis
					//Zgyro int16
					//// X Magnetic field
					//Xmag int16
					//// Y Magnetic field
					//Ymag int16
					//// Z Magnetic field
					//Zmag int16
					//// Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C).
					//Temperature int16 `mavext:"true"`

					fmt.Println()

				case *ardupilotmega.MessageSensorOffsets: // Id 150
					fmt.Printf("Hardware sensors offsets and calibrations values message recived from drone at: %s\n",
						time.Now().Format(time.RFC1123))

					//// Magnetometer X offset.
					//MagOfsX int16
					//// Magnetometer Y offset.
					//MagOfsY int16
					//// Magnetometer Z offset.
					//MagOfsZ int16
					//// Magnetic declination.
					//MagDeclination float32
					//// Raw pressure from barometer.
					//RawPress int32
					//// Raw temperature from barometer.
					//RawTemp int32
					//// Gyro X calibration.
					//GyroCalX float32
					//// Gyro Y calibration.
					//GyroCalY float32
					//// Gyro Z calibration.
					//GyroCalZ float32
					//// Accel X calibration.
					//AccelCalX float32
					//// Accel Y calibration.
					//AccelCalY float32
					//// Accel Z calibration.
					//AccelCalZ float32

					fmt.Println()

				case *ardupilotmega.MessageAhrs: // Id 163
					fmt.Printf("AHRS values message recived from drone at: %s\n", time.Now().Format(time.RFC1123))

					// X gyro drift estimate.
					//Omegaix float32 `mavname:"omegaIx"`
					//// Y gyro drift estimate.
					//Omegaiy float32 `mavname:"omegaIy"`
					//// Z gyro drift estimate.
					//Omegaiz float32 `mavname:"omegaIz"`
					//// Average accel_weight.
					//AccelWeight float32
					//// Average renormalisation value.
					//RenormVal float32
					//// Average error_roll_pitch value.
					//ErrorRp float32
					//// Average error_yaw value.
					//ErrorYaw float32

					fmt.Println()

				case *ardupilotmega.MessageAhrs2: // Id 178
					fmt.Printf("AHRS2 values message recived from drone at: %s\n", time.Now().Format(time.RFC1123))
					//// Roll angle.
					//Roll float32
					//// Pitch angle.
					//Pitch float32
					//// Yaw angle.
					//Yaw float32
					//// Altitude (MSL).
					//Altitude float32
					//// Latitude.
					//Lat int32
					//// Longitude.
					//Lng int32

					fmt.Println()

				case *ardupilotmega.MessageAhrs3: // Id 182
					fmt.Printf("AHRS3 values message recived from drone at: %s\n", time.Now().Format(time.RFC1123))
					//// Roll angle.
					//Roll float32
					//// Pitch angle.
					//Pitch float32
					//// Yaw angle.
					//Yaw float32
					//// Altitude (MSL).
					//Altitude float32
					//// Latitude.
					//Lat int32
					//// Longitude.
					//Lng int32
					//// Test variable1.
					//V1 float32
					//// Test variable2.
					//V2 float32
					//// Test variable3.
					//V3 float32
					//// Test variable4.
					//V4 float32

					fmt.Println()

				case *ardupilotmega.MessageEkfStatusReport: // Id 193
					fmt.Printf("EKF Status message recived from drone at: %s\n", time.Now().Format(time.RFC1123))
					//// Flags.
					//Flags EKF_STATUS_FLAGS `mavenum:"uint16"`
					//// Velocity variance.
					//VelocityVariance float32
					//// Horizontal Position variance.
					//PosHorizVariance float32
					//// Vertical Position variance.
					//PosVertVariance float32
					//// Compass variance.
					//CompassVariance float32
					//// Terrain Altitude variance.
					//TerrainAltVariance float32
					//// Airspeed variance.
					//AirspeedVariance float32 `mavext:"true"`

					fmt.Println()

				case *ardupilotmega.MessagePowerStatus: // Id 125
					fmt.Printf("Power status message from drone. 5V rail voltage: %.2fv, Servo rail voltage: %.2fv, flags: %b\n",
						float32(msg.Vcc)/1000.0, float32(msg.Vservo)/1000.0, msg.Flags)

					fmt.Println()

				case *ardupilotmega.MessageMeminfo: // Id 152
					fmt.Printf("Memory info message from drone. Heap top: %d, free mem: %d, free mem32: %d\n",
						msg.Brkval, msg.Freemem, msg.Freemem32) // TODO: Find out the units of measurement for these values
					fmt.Println()

				case *ardupilotmega.MessageHwstatus: // Id 165
					fmt.Printf("Hardware status message from drone.  Board voltage: %.2fv, I2C err count: %d\n",
						float32(msg.Vcc)/1000.0, msg.I2cerr)

					fmt.Println()

				case *ardupilotmega.MessageVibration: // Id 241
					fmt.Printf("Reply message from drone with vibration info at: %v\n", time.Unix(int64(msg.TimeUsec), 0).Format(time.RFC1123))
					fmt.Printf("Axis vibration levels - x: %.2f y: %.2f z: %.2f\n",
						msg.VibrationX, msg.VibrationY, msg.VibrationZ)
					fmt.Printf("Accelerometer clipping counts - 1st: %d 2nd: %d 3rd %d\n",
						msg.Clipping_0, msg.Clipping_1, msg.Clipping_2)

					fmt.Println()

				default:
					fmt.Printf("Drone message frame received at: %v\n", time.Now().Format(time.RFC1123))

					fmt.Printf("MAVLink version: %d\n", frm.Frame.GetVersion())

					fmt.Printf("System ID: %v\n", frm.SystemId())
					fmt.Printf("Component ID: %v\n", frm.ComponentId())
					fmt.Printf("Message ID: %d\n", frm.Message().GetId())

					fmt.Printf("Message: %+v\n", frm.Message())

					fmt.Printf("Checksum: %v\n", frm.Frame.GetChecksum())

					fmt.Println()
				}
				//fmt.Println()
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
					fmt.Printf("GCS message frame received at: %v\n", time.Now().Format(time.RFC1123))

					fmt.Printf("MAVLink version: %d\n", frm.Frame.GetVersion())

					fmt.Printf("System ID: %v\n", frm.SystemId())
					fmt.Printf("Component ID: %v\n", frm.ComponentId())
					fmt.Printf("Message ID: %d\n", frm.Message().GetId())

					fmt.Printf("Message: %+v\n", frm.Message())

					fmt.Printf("Checksum: %v\n", frm.Frame.GetChecksum())
				}

				fmt.Println()

				// Process a limited amount of event frames from the GCS, then exit
				i++
				if i >= 100 {
					os.Exit(0)
				}
			}

			// Route frame to every other channel
			node.WriteFrameExcept(frm.Channel, frm.Frame)
		}
	}
}
