#!/bin/bash

echo "Starting Pan&Tilt wiggler (use Ctrl+C to interrupt ...)"

while true; do

	rostopic pub -1 /pantilt/orientation_request blueview_pantilt/PanTiltOrientation "{azimuth: 0, elevation: 0}"
	sleep 6

	rostopic pub -1 /pantilt/orientation_request blueview_pantilt/PanTiltOrientation "{azimuth: 180, elevation: 0}"
	sleep 6

	rostopic pub -1 /pantilt/orientation_request blueview_pantilt/PanTiltOrientation "{azimuth: -180, elevation: 0}"

done

echo "Stopping wiggler (and resetting position to zero) ..."
rostopic pub -1 /pantilt/orientation_request blueview_pantilt/PanTiltOrientation "{azimuth: 0, elevation: 0}"

exit 0

