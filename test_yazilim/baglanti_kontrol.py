#!/usr/bin/python
# -*- coding: UTF-8 -*-

print "Bağlantı test"
from time import sleep

# Import DroneKit-Python
from dronekit import connect, VehicleMode

sleep(10)

# Connect to the Vehicle.
#print("Connecting to vehicle on: %s" % (connection_string,))
vehicle = connect('/dev/ttyACM0', wait_ready=True)

# Get some vehicle attributes (state)
print "Get some vehicle attribute values:"
print " GPS: %s" % vehicle.gps_0
print " Battery: %s" % vehicle.battery
print " Last Heartbeat: %s" % vehicle.last_heartbeat
print " Is Armable?: %s" % vehicle.is_armable
print " System status: %s" % vehicle.system_status.state
print " Mode: %s" % vehicle.mode.name    # settable
print " Global Location: %s" % vehicle.location.global_frame
print " Global Location (relative altitude): %s" % vehicle.location.global_relative_frame

# Close vehicle object before exiting script
vehicle.close()

# Shut down simulator
print("Tamamlandı")
