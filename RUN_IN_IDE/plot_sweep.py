import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
from pathlib import Path

# # setup
# dir_path     = '_results/_data_extracted_20221014_100141_150.csv'
# plot_type    = '2d'   # options: 2d, meshgrid, trisurf
# x_name       = 'hybridization_power'
# y_name       = 'EMS'  # in 2d this is separate lines
# z_name       = 'all'  # in 2d this is y-axis
# cmap         = 'viridis_r'
# show_plots   = False


# setup
dir_path     = '_results/_data_extracted_20230524_093759.csv'
plot_type    = '2d'   # options: 2d, meshgrid, trisurf
x_name       = 'bat_durability_in_months'
y_name       = 'bat_initial_soc'  # in 2d this is separate lines
z_name       = 'sfom_co2'  # in 2d this is y-axis
cmap         = 'viridis_r'
show_plots   = True
constant_parameters = {
                        'bat_e_s_cell': 350, # 350 500
                        'bat_cell_to_pack_factor': 1.5, #1.2 1.5 2.0
                        'bat_cell_mass': 0.07, #0.07 0.045
                        #'bat_durability_target_in_months': 6, # 6 12
                        #'bat_initial_soc': 90, # 0.8 0.9 0.95
                       }

def plot_function(df, x_name, y_name, z_name, plot_type, cmap, dir_path, file_name, show=False):

    # set the X, Y, Z coordinates for plotting
    x_data = df[x_name].to_numpy()
    y_data = df[y_name].to_numpy()
    z_data = df[z_name].to_numpy()

    x_dimension = np.unique(x_data)
    y_dimension = np.unique(y_data)

    X, Y = np.meshgrid(x_dimension, y_dimension)
    Z = np.empty([len(y_dimension), len(x_dimension)])

    for i, y in enumerate(y_dimension):
        for j, x in enumerate(x_dimension):
            index = np.where((x_data == x) & (y_data == y))
            if len(index[0]) > 1:
                raise ValueError('More than one occasion found. Multiple datasets present.')
            elif len(index[0]) == 0:
                Z[i][j] = np.nan
            else:
                Z[i][j] = z_data[index[0][0]]

    print(np.rint(Z))

    if plot_type == '2d':
        # create the figure to plot
        fig = plt.figure(z_name)
        ax = plt.subplot(1, 1, 1)
        for i, y in enumerate(y_dimension):
            xy = np.array([X[i,:], Z[i,:]])
            xy = xy[:, ~np.isnan(xy).any(axis=0)]
            ax.plot(xy[0, :], xy[1, :], label=y_name + ' = ' + str(y), marker='x')
            plt.legend()
        ax.set_title('Constant parameters: ' + str(constant_parameters), fontsize = 9)
        ax.set_xlabel(x_name)
        ax.set_ylabel(z_name)
        #ax.set_ylim([500, 2500])
        plt.grid()
        plt.savefig(dir_path + '/' + file_name + '_' + z_name + '.png')
    else:
        # create the figure to plot
        fig = plt.figure(z_name)
        ax = fig.add_subplot(111, projection="3d")
        if plot_type == 'meshgrid':
            surf = ax.plot_surface(X, Y, Z, cmap=cmap, lw=0.5, rstride=1, cstride=1)
        elif plot_type == 'trisurf':
            surf = ax.plot_trisurf(x_data, y_data, z_data, cmap=cmap)
        ax.set_title('Sweep of ' + z_name + ' over ' + x_name + ' and ' + y_name)
        ax.set_xlabel(x_name)
        ax.set_ylabel(y_name)
        ax.set_zlabel(z_name)
        #ax.invert_xaxis()
        #ax.invert_yaxis()
        cbaxes = fig.add_axes([0.9, 0.2, 0.02, .6])   # x-pos, y-pos, width, height -> measured from bottom left corner
        fig.colorbar(surf, shrink=0.5, cax=cbaxes)
        for i in range(4):
            ax.view_init(elev=30, azim=45 + i*90)
            plt.savefig(dir_path + '/' + file_name + '_' + z_name + '_view_' + str(i) + '.png')

    # display the figure for the user
    if show:
        plt.show()

    # TODO print out/save the figure


dir_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), dir_path)
df = pd.read_csv(dir_path)

for key, value in constant_parameters.items():
    df = df[np.isclose(df[key], value, rtol=1e-05)]
file_name = dir_path.rsplit('/', 1)[-1].split('.')[0]

dir_path = os.path.join(dir_path.rsplit('/', 1)[0], 'plots')
Path(dir_path).mkdir(exist_ok=True)


if z_name == 'all':
    cols = list(df.columns)
    cols = cols[2:]
    num_cols = len(cols)
    cols.remove(x_name)
    cols.remove(y_name)
    for col in cols:
        plot_function(df, x_name, y_name, col, plot_type, cmap, dir_path, file_name, show_plots)
else:
    plot_function(df, x_name, y_name, z_name, plot_type, cmap, dir_path, file_name, show_plots)
