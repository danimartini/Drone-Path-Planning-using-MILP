from path_planning import *
plott = 1
f_objective = m.objVal
pos_x = []
pos_y = []
pos_z = []

vel_x = []
vel_y = []
vel_z = []

a_x = []
a_y = []
a_z = []

for n in range(t_steps):
    pos_x.append(px[n].X)
    pos_y.append(py[n].X)
    pos_z.append(pz[n].X)

    vel_x.append(vx[n].X)
    vel_y.append(vy[n].X)
    vel_z.append(vz[n].X)

    a_x.append(ux[n].X)
    a_y.append(uy[n].X)
    a_z.append(uz[n].X)
    # print(pos_x[-1],pos_y[-1],pos_z[-1])
if plott:
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(pos_x, pos_y, pos_z, zdir='z', s=20, c=None, depthshade=True)
    ax.scatter(waypoints[0, :], waypoints[1, :], waypoints[2, :], zdir='z', s=20, c='red', depthshade=True)
    plt.show()
    fig = plt.figure()
    plt.plot(list(range(t_steps)), pos_x)
    plt.plot(list(range(t_steps)), pos_y)
    plt.plot(list(range(t_steps)), pos_z)
    plt.show()
    plt.grid()

    fig = plt.figure()
    plt.plot(list(range(t_steps)), vel_x)
    plt.plot(list(range(t_steps)), vel_y)
    plt.plot(list(range(t_steps)), vel_z)
    plt.show()
    plt.grid()

    fig = plt.figure()
    plt.plot(list(range(t_steps)), a_x)
    plt.plot(list(range(t_steps)), a_y)
    plt.plot(list(range(t_steps)), a_z)
    plt.show()
    plt.grid()