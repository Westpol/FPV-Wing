import matplotlib.pyplot as plt
import csv

timestamps = []
baro_height = []
crsf_throttle = []
gps_speed = []

with open("test.csv") as f:
    reader = csv.reader(f)
    for row in reader:
        try:
            timestamps.append(int(row[0]))
            baro_height.append(float(row[1]))
            crsf_throttle.append(int(row[13]) / 40)
            gps_speed.append(float(row[7]))
        except:
            print(row)

plt.plot(baro_height, label="gyro x")
plt.plot(crsf_throttle, label="Throttle input")
plt.plot(gps_speed, label="GPS speed")
plt.plot()
plt.ylabel("Stuff")
plt.title("Fuck yeh")
plt.grid()
plt.legend()
plt.show()
