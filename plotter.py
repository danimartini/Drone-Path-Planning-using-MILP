from new_path import *

plott = 1

f_objective = m.objVal

pos_x = []
pos_y = []
pos_z = []

vel_x = []
vel_y = []
vel_z = []

v_n=[]
v_analytical=[]
v_relative = []

a_x = []
a_y = []
a_z = []

u_n=[]
u_analytical=[]
u_relative = []
for n in range(t_steps):
    
    pos_x.append(px[n].X)
    pos_y.append(py[n].X)
    pos_z.append(pz[n].X)

    vel_x.append(vx[n].X)
    vel_y.append(vy[n].X)
    vel_z.append(vz[n].X)

    v_n.append(math.sqrt(V[n].X**2+vx[n].X**2)) #?QW?EQ?EQ?W?ETR?W?@!#$?!@#%?!?^?
    v_analytical.append(math.sqrt(vx[n].X**2+vy[n].X**2+vz[n].X**2))
    if v_n[n]!=0:
        v_relative.append(v_analytical[n]/v_n[n]-1)
    else:
        v_relative.append(v_n[n])


    a_x.append(ux[n].X/g)
    a_y.append(uy[n].X/g)
    a_z.append(uz[n].X/g)

    u_n.append(math.sqrt(U[n].X**2+ux[n].X**2)/g) #?QW?EQ?EQ?W?ETR?W?@!#$?!@#%?!?^?
    u_analytical.append(math.sqrt(ux[n].X**2+uy[n].X**2+uz[n].X**2)/g)
    if u_n[n]!=0:
        u_relative.append(u_analytical[n]/u_n[n]-1)
    else:
        u_relative.append(u_n[n])
    if u_analytical[n]>U_max/g:
        print('VIOLATED MAX ACCELERATION AT', n*delta_t,u_analytical[n], U_max)
    else:
        print(n*delta_t, 'OK')
    if v_analytical[n]>V_max:
        print('VIOLATED MAX VELOCITY AT', n*delta_t,u_analytical[n], U_max)
    else:
        print(n*delta_t, 'OK')

    
if plott:
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(pos_x, pos_y, pos_z, zdir='z', s=20, c=None, depthshade=True)
    ax.scatter(waypoints[0, :], waypoints[1, :], waypoints[2, :], zdir='z', s=20, c='red', depthshade=True)
    plt.show()
    fig = plt.figure()
    plt.plot(list(range(t_steps)), pos_x, label = 'posx')
    plt.plot(list(range(t_steps)), pos_y, label = 'posy')
    plt.plot(list(range(t_steps)), pos_z, label = 'posz')
    plt.show()
    plt.grid()

    fig = plt.figure()
    plt.plot(list(range(t_steps)), vel_x, label='velocity x')
    plt.plot(list(range(t_steps)), vel_y, label='velocity y')
    plt.plot(list(range(t_steps)), vel_z, label='velocity z')
    plt.show()
    plt.grid()
    plt.legend()


    fig = plt.figure()
#    plt.plot(list(range(t_steps)), a_x, label='x accel')
#    plt.plot(list(range(t_steps)), a_y,  label='y accel')
#    plt.plot(list(range(t_steps)), a_z,  label='z accel')
    plt.plot(list(range(t_steps)), v_n, label='numerical vel')
    plt.plot(list(range(t_steps)), v_analytical, label='Analytical vel')
    plt.plot(list(range(t_steps)), v_relative, label='Ratio Analytical-Numerical error')
    plt.legend()
    plt.show()
    plt.grid()

    fig = plt.figure()
#    plt.plot(list(range(t_steps)), a_x, label='x accel')
#    plt.plot(list(range(t_steps)), a_y,  label='y accel')
#    plt.plot(list(range(t_steps)), a_z,  label='z accel')
    plt.plot(list(range(t_steps)), u_n, label='numerical accel')
    plt.plot(list(range(t_steps)), u_analytical, label='Analytical accel')
    plt.plot(list(range(t_steps)), u_relative, label='Ratio Analytical-Numerical error')
    plt.legend()
    plt.show()
    plt.grid()
    
    fig = plt.figure()
#    plt.plot(list(range(t_steps)), u_n, label='numerical accel')
#    plt.plot(list(range(t_steps)), u_analytical, label='Analytical accel')
    plt.plot(list(range(t_steps)), u_relative, label='Ratio Analytical-Numerical error ACCEL')
    plt.plot(list(range(t_steps)), v_relative, label='Ratio Analytical-Numerical error VELOCITY')   
    plt.legend()
    plt.show()
    plt.grid()
    
    ax.plot(pos_x, pos_y, pos_z,label='dt=0.05')
    # ax.scatter(waypoints[0,:], waypoints[1,:], waypoints[2,:], zdir='z', s=80, c='red', depthshade=True,label='Waypoints')
    plt.legend()
    title_str='Trajectory achieved in '+str(t_max)+' seconds'
    plt.title(title_str)
    plt.show()