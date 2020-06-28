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
					fmt.Printf("Reply message from drone with parameter %s (index %d) value %v\n",
						msg.ParamId, msg.ParamIndex, msg.ParamValue)

					fmt.Println()

				case *ardupilotmega.MessageMissionCurrent: // Id 42
					fmt.Printf("Drone message status current mission is # %d\n", msg.Seq)

					fmt.Println()

				// TODO: Fill out the details for these
				case *ardupilotmega.MessageGpsRawInt: // Id 24
					//// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
					//TimeUsec uint64
					//// GPS fix type.
					//FixType GPS_FIX_TYPE `mavenum:"uint8"`
					//// Latitude (WGS84, EGM96 ellipsoid)
					//Lat int32
					//// Longitude (WGS84, EGM96 ellipsoid)
					//Lon int32
					//// Altitude (MSL). Positive for up. Note that virtually all GPS modules provide the MSL altitude in addition to the WGS84 altitude.
					//Alt int32
					//// GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
					//Eph uint16
					//// GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
					//Epv uint16
					//// GPS ground speed. If unknown, set to: UINT16_MAX
					//Vel uint16
					//// Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
					//Cog uint16
					//// Number of satellites visible. If unknown, set to 255
					//SatellitesVisible uint8
					//// Altitude (above WGS84, EGM96 ellipsoid). Positive for up.
					//AltEllipsoid int32 `mavext:"true"`
					//// Position uncertainty.
					//HAcc uint32 `mavext:"true"`
					//// Altitude uncertainty.
					//VAcc uint32 `mavext:"true"`
					//// Speed uncertainty.
					//VelAcc uint32 `mavext:"true"`
					//// Heading / track uncertainty
					//HdgAcc uint32 `mavext:"true"`
					//// Yaw in earth frame from north. Use 0 if this GPS does not provide yaw. Use 65535 if this GPS is configured to provide yaw and is currently unable to provide it. Use 36000 for north.
					//Yaw uint16 `mavext:"true"`

				case *ardupilotmega.MessageRawImu: // Id 27
					//// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
					//TimeUsec uint64
					//// X acceleration (raw)
					//Xacc int16
					//// Y acceleration (raw)
					//Yacc int16
					//// Z acceleration (raw)
					//Zacc int16
					//// Angular speed around X axis (raw)
					//Xgyro int16
					//// Angular speed around Y axis (raw)
					//Ygyro int16
					//// Angular speed around Z axis (raw)
					//Zgyro int16
					//// X Magnetic field (raw)
					//Xmag int16
					//// Y Magnetic field (raw)
					//Ymag int16
					//// Z Magnetic field (raw)
					//Zmag int16
					//// Id. Ids are numbered from 0 and map to IMUs numbered from 1 (e.g. IMU1 will have a message with id=0)
					//Id uint8 `mavext:"true"`
					//// Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C).
					//Temperature int16 `mavext:"true"`

				case *ardupilotmega.MessageScaledPressure: // Id 29
					//// Timestamp (time since system boot).
					//TimeBootMs uint32
					//// Absolute pressure
					//PressAbs float32
					//// Differential pressure 1
					//PressDiff float32
					//// Temperature
					//Temperature int16

				case *ardupilotmega.MessageAttitude: // Id 30
					//// Timestamp (time since system boot).
					//TimeBootMs uint32
					//// Roll angle (-pi..+pi)
					//Roll float32
					//// Pitch angle (-pi..+pi)
					//Pitch float32
					//// Yaw angle (-pi..+pi)
					//Yaw float32
					//// Roll angular speed
					//Rollspeed float32
					//// Pitch angular speed
					//Pitchspeed float32
					//// Yaw angular speed
					//Yawspeed float32

				case *ardupilotmega.MessageGlobalPositionInt: // Id 33
					//// Timestamp (time since system boot).
					//TimeBootMs uint32
					//// Latitude, expressed
					//Lat int32
					//// Longitude, expressed
					//Lon int32
					//// Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
					//Alt int32
					//// Altitude above ground
					//RelativeAlt int32
					//// Ground X Speed (Latitude, positive north)
					//Vx int16
					//// Ground Y Speed (Longitude, positive east)
					//Vy int16
					//// Ground Z Speed (Altitude, positive down)
					//Vz int16
					//// Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
					//Hdg uint16

				case *ardupilotmega.MessageRcChannelsRaw: // Id 35
					//// Timestamp (time since system boot).
					//TimeBootMs uint32
					//// Servo output port (set of 8 outputs = 1 port). Flight stacks running on Pixhawk should use: 0 = MAIN, 1 = AUX.
					//Port uint8
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
					//// Receive signal strength indicator in device-dependent units/scale. Values: [0-254], 255: invalid/unknown.
					//Rssi uint8

				case *ardupilotmega.MessageServoOutputRaw: // Id 36
					//// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
					//TimeUsec uint32
					//// Servo output port (set of 8 outputs = 1 port). Flight stacks running on Pixhawk should use: 0 = MAIN, 1 = AUX.
					//Port uint8
					//// Servo output 1 value
					//Servo1Raw uint16
					//// Servo output 2 value
					//Servo2Raw uint16
					//// Servo output 3 value
					//Servo3Raw uint16
					//// Servo output 4 value
					//Servo4Raw uint16
					//// Servo output 5 value
					//Servo5Raw uint16
					//// Servo output 6 value
					//Servo6Raw uint16
					//// Servo output 7 value
					//Servo7Raw uint16
					//// Servo output 8 value
					//Servo8Raw uint16
					//// Servo output 9 value
					//Servo9Raw uint16 `mavext:"true"`
					//// Servo output 10 value
					//Servo10Raw uint16 `mavext:"true"`
					//// Servo output 11 value
					//Servo11Raw uint16 `mavext:"true"`
					//// Servo output 12 value
					//Servo12Raw uint16 `mavext:"true"`
					//// Servo output 13 value
					//Servo13Raw uint16 `mavext:"true"`
					//// Servo output 14 value
					//Servo14Raw uint16 `mavext:"true"`
					//// Servo output 15 value
					//Servo15Raw uint16 `mavext:"true"`
					//// Servo output 16 value
					//Servo16Raw uint16 `mavext:"true"`

				case *ardupilotmega.MessageNavControllerOutput: // Id 62
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

				case *ardupilotmega.MessageRcChannels: // Id 65
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

				case *ardupilotmega.MessageVfrHud: // Id 74
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

				case *ardupilotmega.MessageScaledImu2: // Id 116
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

				case *ardupilotmega.MessageSensorOffsets: // Id 150
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

				case *ardupilotmega.MessageAhrs: // Id 163
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

				case *ardupilotmega.MessageAhrs2: // Id 178
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

				case *ardupilotmega.MessageAhrs3: // Id 182
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

				case *ardupilotmega.MessageEkfStatusReport: // Id 193
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
					fmt.Println("Reply message from drone with vibration info")
					fmt.Printf("Timestamp %v\n", time.Unix(int64(msg.TimeUsec), 0).Format(time.RFC1123))
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
