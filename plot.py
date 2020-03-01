import matplotlib.pyplot as plt
import numpy as np
import json

with open('coverage_data/UAV_class1.json','r') as f:
    bounce = json.load(f)

with open('coverage_data/UAV_class2.json','r') as f:
    bounce_and_fick = json.load(f)

with open('coverage_data/UAV_class3.json','r') as f:
    fick = json.load(f)

with open('coverage_data/UAV_class4.json','r') as f:
    stochastic = json.load(f)

with open('coverage_data/UAV_class5.json','r') as f:
    fick_one_cross_obstacle = json.load(f)

with open('coverage_data/time.json','r') as f:
    time = json.load(f)

fig = plt.figure()
ax = fig.add_subplot(1,1,1)
ax.grid()

ax.plot(time, bounce, label = 'Bounce Model')
ax.plot(time, bounce_and_fick, label = 'Mixed Model')
ax.plot(time, fick,  label = 'Fick Model')
ax.plot(time, fick_one_cross_obstacle, label = 'Fick Model with One Crosslike Obstacles')
ax.plot(time, stochastic, label = 'Stochastic Model')

ax.legend()
ax.set_xlabel('time/s')
ax.set_ylabel('coverage percentage')
ax.set_title('Coverage Percentage vs Time')
plt.show()