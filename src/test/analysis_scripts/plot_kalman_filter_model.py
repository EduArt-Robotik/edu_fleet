import pandas as pd
import matplotlib.pyplot as plt

data_path = 'build/edu_fleet/src/test/'
document_path = 'src/edu_fleet/documentation/formula/plot/'
plot_files = [
  'prediction-x-based-on-acceleration.csv',
  'prediction-y-based-on-acceleration.csv',
  'prediction-x-based-on-velocity.csv',
  'prediction-y-based-on-velocity.csv',
  'prediction-pose-based-on-velocity-and-yaw-rate.csv'
]

# acceleration and velocity
for i in range(4):
  df = pd.read_csv(data_path + plot_files[i], sep=';')
  df = df[['stamp (s)', 'pos_x (world)', 'pos_y (world)', 'vel_x (vehicle)', 'vel_y (vehicle)', 'acc_x (vehicle)', 'acc_y (vehicle)', 'yaw (world)', 'yaw rate (vehicle)']]

  figure = df.plot(
    title=plot_files[i].replace('.csv', ''),
    xlabel='sec',
    ylabel='m, m/s, m/s^2, rad, rad/s',
    x='stamp (s)').figure

  figure.show()
  figure.savefig(document_path + plot_files[i].replace('.csv', '.pdf'))

# yaw rate
df = pd.read_csv(data_path + plot_files[4], sep=';')
df = df[['pos_x (world)', 'pos_y (world)']]

plot = df.plot(
  title=plot_files[4].replace('.csv', ''),
  xlabel='sec',
  ylabel='m, m/s, m/s^2, rad, rad/s',
  x='pos_x (world)',
  y='pos_y (world)')
plot.set_aspect('equal')
figure = plot.figure

figure.show()
figure.savefig(document_path + plot_files[4].replace('.csv', '.pdf'))

