import matplotlib.pyplot as plt
import numpy as np
import json

with open('change_evaprate_maxrange.json','r') as f:
    d = json.load(f)
with open('contrast_phe_and_nopne.json','r') as f:
    d2 = json.load(f)

scenary = {
    0 : [10, 2],
    1 : [5, 1],
    2 : [10, 1],
    3 : [5, 2],
}

fig, ax = plt.subplots(figsize = (8, 8))
ax.grid()

for i in range(2):
    ax.plot(np.linspace(0, 30, 3000), d[i], label = 'mr' + str(scenary[i][0]) + ',er' + str(scenary[i][1]))
    ax.set_xlabel('time/s')
    ax.set_ylabel('Coverage Percent')
    #ax.set_title('Fick Pheromone with Max Range ' + str(scenary[i][0]) + ' and Evaporate ' + str(scenary[i][1]) + '(#uav=20)')
    ax.legend()


ax.plot(np.linspace(0, 30, 3000), d2[6], label = 'mr' + str(scenary[3][0]) + ',er' + str(scenary[3][1]))
ax.set_xlabel('time/s')
ax.set_ylabel('Coverage Percent')
ax.set_title('Fick Pheromone with Changing Max Pheromone Range(mr) and Evaporation rate(er) (#uav=20,no_obs)')
ax.legend()

plt.show()