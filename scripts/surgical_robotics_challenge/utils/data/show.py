import numpy as np

data = np.load('/home/lambda/surgical_robotics_challenge/scripts/surgical_robotics_challenge/utils/data/clutch_times.npy', allow_pickle=True)

for i in range(1):
    print(data[i])
print(len(data))
