import matplotlib.pyplot as plt
import csv

timestamps = []
baro_height = []
crsf_throttle = []

with open("test.csv") as f:
    reader = csv.reader(f)
    for row in reader:
        try:
            timestamps.append(int(row[0]))
            baro_height.append(float(row[3]))
            crsf_throttle.append(int(row[13]) / 40)
        except:
            print(row)

plt.plot(baro_height, label="Baro Height")
plt.plot(crsf_throttle, label="Throttle input")
plt.plot()
plt.ylabel("Stuff")
plt.title("Fuck yeh")
plt.grid()
plt.legend()
plt.show()
