import matplotlib.pyplot as plt 
ox, oy = [], []
for i in range(0,100):
    ox.append(i)
    oy.append(0)
for i in range(0, 50):
    ox.append(100.0)
    oy.append(i)
for i in range(0,101):
    ox.append(i)
    oy.append(50)
for i in range(0, 51):
    ox.append(0)
    oy.append(i)
for i in range(0, 40):
    ox.append(40)
    oy.append(50.0 - i)
for i in range(0, 40):
    ox.append(60)
    oy.append(50.0 - i)
for i in range(0,41):
    ox.append(i)
    oy.append(10)
for i in range(0,41):
    ox.append(100-i)
    oy.append(10)
plt.plot(ox,oy,'r.')
plt.show()
