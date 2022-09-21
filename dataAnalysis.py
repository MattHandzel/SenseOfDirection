import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
fig = plt.figure()
ax = plt.axes(projection='3d')
data = pd.read_csv("./data/data_2022-09-20-02.csv")
cmap = "Greens"
# for data in [pd.read_csv("./data/data_2022-09-20-00.csv"), pd.read_csv("./data/data_2022-09-20-01.csv")]:
xData = data['data 0']
yData = data['data 1']
zData = data['data 2']

magnitudes = np.sqrt(xData**2 + yData**2 + zData**2)

# plt.plot(np.arange(len(magnitudes)), magnitudes)


ax.scatter3D(xData, yData, zData, c=zData, cmap=cmap)
plt.show()

# here is a point (x,y)

# figure out how far away (x,y) is from the center

# get that magnitude

# 
