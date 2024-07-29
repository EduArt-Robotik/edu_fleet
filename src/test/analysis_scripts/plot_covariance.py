import matplotlib.pyplot as plt
import pandas as pd
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D

data_path = 'build/edu_fleet/src/test/'
document_path = 'src/edu_fleet/documentation/formula/plot/'

df = pd.read_csv(data_path + 'extended_kalman_filter_predict_to_time_case1_cov.csv', sep=';')

figure = plt.figure()
ax = Axes3D(figure)
print(df)
surface_plot = ax.plot_trisurf(df.col, df.row, df.value, cmap=cm.jet, linewidth=0.2)

plt.show()