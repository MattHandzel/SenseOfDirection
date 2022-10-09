import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

# def clean_data(data):

#   data = np.array(data, dtype=np.float64)

#   # Get the magnitude of the largest data point

#   print(data.shape)

#   # Get rid of outliers for each data type
#   for i in range(len(data)):
#     data = data[:, data[i] >= (np.mean(data[i]) - 2 * np.std(data[i]))]
#     data = data[:, data[i] <= (np.mean(data[i]) + 2 * np.std(data[i]))]


#   print(data.shape)
  
#   magnitudes = np.sqrt(data[0] ** 2 + data[1] ** 2 + data[2] ** 2)

#   # Divide all of the data by the average of the magnitude
#   data /= np.mean(magnitudes)

#   # Data offsets
#   data[0] -= (np.mean(data, 1)[0])
#   data[1] -= (np.mean(data, 1)[1])
#   data[2] -= (np.mean(data, 1)[2])

#   return data

def clean_data(data):

  data = np.array(data, dtype=np.float64)

  # Get the magnitude of the largest data point

  print(data.shape)

  # Get rid of outliers for each data type
  for i in range(len(data)):
    data = data[:, data[i] >= (np.mean(data[i]) - 2 * np.std(data[i]))]
    data = data[:, data[i] <= (np.mean(data[i]) + 2 * np.std(data[i]))]
  
  print("x center:\t" + str((np.max(data[0]) + np.min(data[0])) / 2))
  print("y center:\t" + str((np.max(data[1]) + np.min(data[1])) / 2))
  print("z center:\t" + str((np.max(data[2]) + np.min(data[2])) / 2))

  # Center data
  data[0] -= (np.max(data[0]) + np.min(data[0])) / 2
  data[1] -= (np.max(data[1]) + np.min(data[1])) / 2
  data[2] -= (np.max(data[2]) + np.min(data[2])) / 2

  # Normalize data
  magnitudes = np.sqrt(data[0] ** 2 + data[1] ** 2 + data[2] ** 2)

  # data[0] /= np.mean(magnitudes)
  # data[1] /= np.mean(magnitudes)
  # data[2] /= np.mean(magnitudes)

  # # Normalize data + soft iron correction
  # x_coeffient = np.mean(np.abs(data),1)[0]
  # y_coefficient = np.mean(np.abs(data),1)[1]
  # z_coefficient = np.mean(np.abs(data),1)[2]
  # print(x_coeffient, y_coefficient, z_coefficient)
  # data[0] /= x_coeffient * 2
  # data[1] /= y_coefficient * 2
  # data[2] /= z_coefficient * 2

  return data

def clean_data_no_soft_iron(data):

  data = np.array(data, dtype=np.float64)

  # Get the magnitude of the largest data point

  print(data.shape)

  # Get rid of outliers for each data type
  for i in range(len(data)):
    data = data[:, data[i] >= (np.mean(data[i]) - 2 * np.std(data[i]))]
    data = data[:, data[i] <= (np.mean(data[i]) + 2 * np.std(data[i]))]
  

  # Center data
  data[0] -= (np.max(data[0]) + np.min(data[0])) / 2
  data[1] -= (np.max(data[1]) + np.min(data[1])) / 2
  data[2] -= (np.max(data[2]) + np.min(data[2])) / 2

  # Normalize data
  magnitudes = np.sqrt(data[0] ** 2 + data[1] ** 2 + data[2] ** 2)

  # data[0] /= np.mean(magnitudes)
  # data[1] /= np.mean(magnitudes)
  # data[2] /= np.mean(magnitudes)

  # Normalize data (w/o soft iron correct)
  x_coefficient = (np.max(data[0]) - np.min(data[0])) / 2
  y_coefficient = (np.max(data[1]) - np.min(data[1])) / 2 
  z_coefficient = (np.max(data[2]) - np.min(data[2])) / 2
  print(x_coefficient / 2, y_coefficient/ 2, z_coefficient / 2)
  data[0] /= x_coefficient 
  data[1] /= y_coefficient 
  data[2] /= z_coefficient 
  return data
def find_approximation_function(data):
  pass  


fig = plt.figure()
ax = plt.axes(projection='3d')
data = pd.read_csv("./data/data_2022-10-08-01 ACCEL.csv")
cmap = "Greens"
# for data in [pd.read_csv("./data/data_2022-09-20-00.csv"), pd.read_csv("./data/data_2022-09-20-01.csv")]:
xData = data['data 0']
yData = data['data 1']
zData = data['data 2']

# Data cleaning

# Remove outliers




# plt.plot(np.arange(len(magnitudes)), magnitudes)

# ax.scatter3D(xData, yData, zData, c=zData, cmap=cmap)

(_xData, _yData, _zData) = clean_data((xData, yData, zData))
ax.scatter3D(_xData, _yData, _zData, c=_zData, cmap="Reds")

(_xData, _yData, _zData) = clean_data_no_soft_iron((xData, yData, zData))
ax.scatter3D(_xData, _yData, _zData, c=_zData, cmap="Blues")
plt.show()




# here is a point (x,y)

# figure out how far away (x,y) is from the center

# get that magnitude

# 
