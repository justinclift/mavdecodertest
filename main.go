package main

import (
	"fmt"
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
		OutSystemId: 10,
	})
	if err != nil {
		panic(err)
	}
	defer node.Close()

	// print every message we receive
	//i := 0
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
			if frm.SystemId() != 1 {
				// This is a message from the Ground Control Station
				switch msg := frm.Message().(type) {
				case *ardupilotmega.MessageHeartbeat:
					fmt.Println("Heartbeat from Ground Control Station")
					fmt.Printf("MAVLink version: %s\n", frm.Frame.GetVersion())

					fmt.Printf("Decoded MAV type: %v\n", msg.Type)
					fmt.Printf("Decoded System status: %v\n", msg.SystemStatus)

				case *ardupilotmega.MessageParamRequestRead: // Id 20
					if msg.ParamId == "" {
						fmt.Printf("Request for parameter %d from target system %d, component %d\n",
							msg.ParamIndex, msg.TargetSystem, msg.TargetComponent)
					} else {
						fmt.Printf("Request for parameter '%s' from target system %d, component %d\n",
							msg.ParamId, msg.TargetSystem, msg.TargetComponent)
					}

				case *ardupilotmega.MessageParamRequestList: // Id 21
					fmt.Printf("Request for all parameters from target system %d, component %d\n",
						msg.TargetSystem, msg.TargetComponent)

				case *ardupilotmega.MessageRequestDataStream:
					var startOrStop string
					if msg.StartStop == 1 {
						startOrStop = "start"
					} else {
						startOrStop = "stop"
					}
					fmt.Printf("RequestDataStream (%s) message from GCS, target system %d, component %d\n", startOrStop, msg.TargetSystem, msg.TargetSystem)

				case *ardupilotmega.MessageCommandLong:
					fmt.Printf("Command long ('%s') message received for target system %d, component %d\n",
						msg.Command, msg.TargetSystem, msg.TargetComponent)

				default:
					fmt.Printf("GCS Message frame received at: %v\n", time.Now().Format(time.RFC1123))

					fmt.Printf("MAVLink version: %d\n", frm.Frame.GetVersion())

					fmt.Printf("System ID: %v\n", frm.SystemId())
					fmt.Printf("Component ID: %v\n", frm.ComponentId())
					fmt.Printf("Message ID: %d\n", frm.Message().GetId())

					fmt.Printf("Message: %+v\n", frm.Message())

					fmt.Printf("Checksum: %v\n", frm.Frame.GetChecksum())
				}

				fmt.Println()

				//// Process a limited amount of event frames from the GCS, then exit
				//i++
				//if i >= 10 {
				//	syscall.Exit(0)
				//}
			}

			// Route frame to every other channel
			node.WriteFrameExcept(frm.Channel, frm.Frame)
		}
	}
}