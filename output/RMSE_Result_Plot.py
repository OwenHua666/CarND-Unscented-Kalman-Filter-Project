import csv
import numpy as np
import matplotlib.pyplot as plt
dataset_list = [] 
count = 0
with open('/home/ftb/Self-Driving/carnd-term2/CarND-Extended-Kalman-Filter-Project/build/RMSE_Collect.csv') as csvfile:
	datareader = csv.reader(csvfile, delimiter=' ')
	for row in datareader:
		count += 1
		this_row = []
		for item in row:
			try:
				this_row.append(float(item))
			except ValueError:
				continue
		dataset_list.append(this_row)
dataset_array = np.empty((count-1, 4))
for row in range(0, count-1): # The first row is fake data.
	for col in range(4):
		dataset_array[row, col] = dataset_list[row+1][col]

x_array = np.arange(count-1)
accuracy = [0.11, 0.11, 0.52, 0.52]
plt.figure()
ErrorBound = np.ones(count-1) * accuracy[0]
plt.plot(x_array, dataset_array[:,0], 'b', x_array, ErrorBound, 'r')
plt.xlabel("measurement_steps")
plt.ylabel("RMSE")
plt.axis([0,500,0,0.3])
plt.legend(("RMSE of Px","Error Upper Bound"))
plt.title("RMSE Result of Px (RADAR + LASER)")
plt.grid(True)
plt.show()

plt.figure()
ErrorBound = np.ones(count-1) * accuracy[1]
plt.plot(x_array, dataset_array[:,1], 'b', x_array, ErrorBound, 'r')
plt.xlabel("measurement_steps")
plt.ylabel("RMSE")
plt.axis([0,500,0,0.3])
plt.legend(("RMSE of Py","Error Upper Bound"))
plt.title("RMSE Result of Py (RADAR + LASER)")
plt.grid(True)
plt.show()

plt.figure()
ErrorBound = np.ones(count-1) * accuracy[2]
plt.plot(x_array, dataset_array[:,2], 'b', x_array, ErrorBound, 'r')
plt.xlabel("measurement_steps")
plt.ylabel("RMSE")
plt.axis([0,500,0,5.5])
plt.legend(("RMSE of Vx","Error Upper Bound"))
plt.title("RMSE Result of Vx (RADAR + LASER)")
plt.grid(True)
plt.show()

plt.figure()
ErrorBound = np.ones(count-1) * accuracy[3]
plt.plot(x_array, dataset_array[:,3], 'b', x_array, ErrorBound, 'r')
plt.xlabel("measurement_steps")
plt.ylabel("RMSE")
plt.axis([0,500,0,1.9])
plt.legend(("RMSE of Vy","Error Upper Bound"))
plt.title("RMSE Result of Vy (RADAR + LASER)")
plt.grid(True)
plt.show()
