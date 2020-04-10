import matplotlib.pyplot as plt
import numpy as np
import json

with open('coverage_data/v2_experiment_other3_5_02.json','r') as f:
    d = json.load(f)
with open('coverage_data/contrast_phe_and_nophe.json','r') as f:
    d2 = json.load(f)


fig, ax = plt.subplots(figsize = (8, 8))
ax.grid()


ax.plot(np.linspace(0, 30, 3000), d[2], label = 'v2_stochastic_phe_5_02pstep')
ax.set_xlabel('time/s')
ax.set_ylabel('Coverage Percent')
#ax.set_title('Fick Pheromone with Max Range ' + str(scenary[i][0]) + ' and Evaporate ' + str(scenary[i][1]) + '(#uav=20)')
ax.legend()


ax.plot(np.linspace(0, 30, 3000), d2[3], label = 'v1_stochastic_5_2psecond')
ax.set_xlabel('time/s')
ax.set_ylabel('Coverage Percent')
ax.legend()

ax.plot(np.linspace(0, 30, 3000), d2[7], label = 'v1_stochastic_phe_5_2psecond')
ax.set_xlabel('time/s')
ax.set_ylabel('Coverage Percent')
ax.set_title('v2stochasticphe502pstep_v1stochastic52psecond_v1stochasticphe52psecond')
ax.legend()

plt.show()