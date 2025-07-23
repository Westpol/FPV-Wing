import matplotlib.pyplot as plt
import csv

timestamps = []
values = []

with open("test.csv") as f:
    reader = csv.reader(f)
    for row in reader:
        timestamps.append(int(row[0]))
        values.append(int(row[1]))

plt.plot(values)
plt.ylabel("Throttle")
plt.title("Throttle Curve")
plt.grid()
plt.show()
