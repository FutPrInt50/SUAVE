import numpy as np
import pandas as pd

file_name = 'DCDC_Converter.csv'
my_data = np.genfromtxt(file_name, delimiter=',')

y = my_data[0][1:]
modified_data = np.empty((0, 3))

for i, x in enumerate(my_data[1:, 0]):
    voltage = np.ones(np.shape(my_data)[1] - 1) * x
    modified_data = np.append(modified_data, np.array([voltage, y, my_data[i+1][1:]]).T, axis=0)


print(1)