# OBD2 Monitor
# Created at 2020-02-04 14:59:23.359314

from fortebit.polaris import polaris
import obd2

polaris.init()

obd2.start()
while obd2.is_running():
    sleep(200)
    if obd2.is_talking():
        print("RPM:", obd2.rpm, "KM/H:", obd2.kmh, "Temp.C:", obd2.temp)

print("Unexpected exit")