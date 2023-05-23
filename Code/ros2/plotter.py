import matplotlib.pyplot as plt

x = []
y = []

with open('log.txt') as f:
    lines = f.readlines()
    for line in lines:
        if "[__TIMER__]" in line:
            line = line.split(" ")
            x.append(int(line[-1]))
            y.append(int(line[-2]))

plt.scatter(x, y)
plt.show()
