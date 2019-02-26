from gurobipy import *
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from write_to_text import *
# ============================================================================
#
# Optimization Model
#
# ============================================================================
'Drone dynamics data'
g=9.81        # m/s^2
V_max=118/3.6 # m/s
D_sides=32    # [-]
V_max *= math.cos(math.pi/D_sides)
U_max=5*g    # m/s^2
U_max *= math.cos(math.pi/D_sides)
delta_t=2   # sec
t_max=60    # sec
t_steps=int(round((t_max+delta_t)/delta_t,1)) 	 # [-]
gamma=10*math.pi/180    	 # deg
alpha=10*math.pi/180		 # deg

waypoints =np.array([[8.913, 9.27, 9.27, 5.76, 2.675, 2.787, 6.4045, 6.4045, 2.91, 5.91, 8, 8.913], # x axis
                        [14.642, 10.51, 4.057, 2.749, 6.083, 12.259, 12.5825, 11.8575, 9.0,7.2685, 7.2685+1, 14.642],  #y-axis
                        [0.41, 2.245, 2.899, 2.49, 2.236, 2.36, 1.098, 1.098, 1.625, 2.1565, 2.1565 - 0.5, 0.41]]) # z-axis

velocity_cts=np.array([[0, math.inf,   0,   math.inf, 0, 0, 0, 0, math.inf, math.inf, math.inf, 0],
                       [0, math.inf, math.inf, 0, math.inf, math.inf, math.inf, math.inf, math.inf, 0, math.inf, 0],
                       [ 0, math.inf,   0,     0, 0, 0, 0, 0, math.inf, 0, math.inf, 0]])

# waypoints=waypoints.astype(int)
n_waypoints=waypoints.shape[1]
# ----------------------------------------------------------------------------
# Define Variables to be used.
# ----------------------------------------------------------------------------
m = Model('Path_planning')

'X-coordinate'
px = {}
vx = {}
ux = {}
'Y-coordinate'
py = {}
vy = {}
uy = {}

'Z-coordinate'
pz = {}
vz = {}
uz = {}

'Acceleration'
U={}
'Velocity'
V={}
'Commodity variables for discretization'
theta={}
b={}
ratio=0.0
# ----------------------------------------------------------------------------
# Create Objective FUction
# ----------------------------------------------------------------------------)
for d in range(D_sides+1):
    theta[d]=2*math.pi*(d)/D_sides

for n in range(t_steps):
    U[n]= m.addVar(obj=1-ratio,
                    vtype=GRB.CONTINUOUS,lb=0,
                    name="U_%s" % (n))
    V[n] = m.addVar(obj=1,
                    vtype=GRB.CONTINUOUS, lb=0,
                    name="V_%s" % (n))
    ux[n] = m.addVar(obj=0,
                            vtype=GRB.CONTINUOUS,lb=-GRB.INFINITY,
                            name="ux_%s" % (n))
    uy[n] = m.addVar(obj=0,
                          vtype=GRB.CONTINUOUS,lb=-GRB.INFINITY,
                          name="uy_%s" % (n))
    uz[n] = m.addVar(obj=0,
                          vtype=GRB.CONTINUOUS,lb=-GRB.INFINITY,
                          name="uz_%s" % (n))
    vx[n] = m.addVar(obj=0,
                            vtype=GRB.CONTINUOUS,lb=-GRB.INFINITY,
                            name="Vx_%s" % (n))
    vy[n] = m.addVar(obj=0,
                          vtype=GRB.CONTINUOUS,lb=-GRB.INFINITY,
                          name="Vy_%s" % (n))
    vz[n] = m.addVar(obj=0,
                          vtype=GRB.CONTINUOUS,lb=-GRB.INFINITY,
                          name="Vz_%s" % (n))
    px[n] = m.addVar(obj=0,
                          vtype=GRB.CONTINUOUS,lb=-GRB.INFINITY,
                          name="Px_%s" % (n))
    py[n] = m.addVar(obj=0,
                          vtype=GRB.CONTINUOUS,lb=-GRB.INFINITY,
                          name="Py_%s" % (n))
    pz[n] = m.addVar(obj=0,
                      vtype=GRB.CONTINUOUS,lb=-GRB.INFINITY,
                      name="Pz_%s" % (n))

n_gates=waypoints.shape[1]
for i in range(n_gates):
    for n in range(t_steps):
        b[i,n] = m.addVar(obj=0,
                         vtype=GRB.BINARY,
                         name="b_%s_%s" % (i,n))


Tf = m.addVar(obj=ratio,
             vtype=GRB.CONTINUOUS,lb=0.0,
             name="Tsim")
m.update()

# Optimize
m.setObjective(m.getObjective(), GRB.MINIMIZE)  # The objective is to maximize revenue

# ----------------------------------------------------------------------------
# Create Constraints
# ----------------------------------------------------------------------------

'Max acceleration constraint'
for n in range(t_steps):
    m.addConstr(U[n],
        GRB.LESS_EQUAL, U_max, name='U_ctsmax1_%s' % (n))

    m.addConstr(V[n],
        GRB.LESS_EQUAL, V_max, name='V_ctsmax1_%s' % (n))
'Accelerations Constraint'
for n in range(t_steps):
    for d in range(D_sides):
        m.addConstr(
            math.cos(theta[d])*math.sin(-alpha)*ux[n]+math.sin(theta[d])*math.sin(-alpha)*uy[n]+math.sin(-alpha)*uz[n],
            GRB.LESS_EQUAL,U[n], name='U_cts1_%s_%s' % (n,d))

        m.addConstr(
            math.cos(theta[d]) * math.sin(alpha)*uz[n] + math.sin(theta[d]) * math.sin(alpha) * ux[n] + math.sin(alpha) * uz[n],
            GRB.LESS_EQUAL, U[n], name='U_cts2_%s_%s' % (n, d))

        m.addConstr(
            math.cos(theta[d])*uy[n]+math.sin(theta[d])*uz[n],
            GRB.LESS_EQUAL,U[n], name='U_cts3_%s_%s' % (n,d))


'Max Velocity Constraint'
for n in range(t_steps):
    for d in range(D_sides):
        m.addConstr(
            math.cos(theta[d])*math.sin(gamma)*vx[n]+math.sin(theta[d])*math.sin(gamma)*vy[n]+math.sin(-gamma)*vz[n],
            GRB.LESS_EQUAL,V[n], name='V_cts1_%s_%s' % (n,d))

        m.addConstr(
            math.cos(theta[d]) * math.sin(gamma) * vz[n] + math.sin(theta[d]) * math.sin(gamma) * vx[n] + math.sin(gamma) * vz[n],
            GRB.LESS_EQUAL, V[n], name='V_cts2_%s_%s' % (n, d))

        m.addConstr(
            math.cos(theta[d])*vy[n]+math.sin(theta[d])*vz[n],
            GRB.LESS_EQUAL,V[n], name='V_cts3_%s_%s' % (n,d))

'Integration scheme Constraint'
for n in range(1,t_steps):
    'Position'
    m.addConstr(
        px[n-1]+vx[n-1]*delta_t+1/2*delta_t**2*ux[n-1],
        GRB.EQUAL,px[n], name='Px_cts_%s' % (n))

    m.addConstr(
        py[n-1]+vy[n-1]*delta_t+1/2*delta_t**2*uy[n-1],
        GRB.EQUAL,py[n], name='Py_cts_%s' % (n))
    m.addConstr(
        pz[n-1]+vz[n-1]*delta_t+1/2*delta_t**2*uz[n-1],
        GRB.EQUAL,pz[n], name='Pz_cts_%s' % (n))

    'Velocity'
    m.addConstr(
        vx[n-1]+delta_t*ux[n-1],
        GRB.EQUAL,vx[n], name='Vx_cts_%s' % (n))
    m.addConstr(
        vy[n-1]+delta_t*uy[n-1],
        GRB.EQUAL,vy[n], name='Vy_cts_%s' % (n))
    m.addConstr(
        vz[n-1]+delta_t*uz[n-1],
        GRB.EQUAL,vz[n], name='Vz_cts_%s' % (n))


'Waypoint Constraint'
m.addConstr(
    px[0],
    GRB.EQUAL, waypoints[0,0],name='Wpx')

m.addConstr(
    py[0],
    GRB.EQUAL, waypoints[1,0],name='Wpy')
m.addConstr(
    pz[0],
    GRB.EQUAL, waypoints[2,0],name='Wpz')

# m.addConstr(b[0,0],
#         GRB.EQUAL, 1,name='Mwp_%s' % (i))
#
M=10e6
for n in range(t_steps):
    for i in range(1,n_gates):
        m.addConstr(
            px[n]-waypoints[0,i],
            GRB.LESS_EQUAL, M*(1-b[i,n]),name='Wpx_%s_%s' % (i,n))

        m.addConstr(
            px[n] - waypoints[0, i],
            GRB.GREATER_EQUAL, -M * (1 - b[i, n]), name='Wpox_%s_%s' % (i,n))

        m.addConstr(
            py[n] - waypoints[1, i],
            GRB.LESS_EQUAL, M * (1 - b[i, n]), name='Wpy_%s_%s' % (i,n))

        m.addConstr(
            py[n] - waypoints[1, i],
            GRB.GREATER_EQUAL, -M * (1 - b[i, n]), name='Wpoy_%s_%s' % (i,n))

        m.addConstr(
            pz[n] - waypoints[2, i],
            GRB.LESS_EQUAL, M * (1 - b[i, n]), name='Wpz_%s_%s' % (i,n))

        m.addConstr(
            pz[n] - waypoints[2, i],
            GRB.GREATER_EQUAL, -M * (1 - b[i, n]), name='Wpoz_%s_%s' % (i,n))

'Set meet of waypoint requirement'
for i in range(1,n_gates):
    m.addConstr(
        quicksum(b[i,n] for n in range(t_steps)),
        GRB.EQUAL, 1,name='Mwp_%s' % (i))
#
    m.addConstr(
        quicksum(delta_t*n*b[i,n] for n in range(t_steps)),
        GRB.GREATER_EQUAL, quicksum(delta_t*n*b[i-1,n] for n in range(t_steps)), name='Maxt_%s' % (i))

m.addConstr(
        quicksum(delta_t*n*b[n_gates-1,n] for n in range(t_steps)),
        GRB.LESS_EQUAL, Tf, name='Maxts_%s' % (n_gates-1))

'Initial acceleration Constraint'
for n in [0,t_steps-1]:
    m.addConstr(
            ux[n],
            GRB.EQUAL, 0,name='ux_%s'%(n))
    m.addConstr(
            uy[n],
            GRB.EQUAL, 0,name='uy_%s'%(n))
    m.addConstr(
            uz[n],
            GRB.EQUAL, 0,name='uz_%s'%(n))


'Velocity constaint at waypoint'
for n in range(t_steps):
    for i in range(n_gates):
        if velocity_cts[0,i]<math.inf:
            # m.addConstr(quicksum(vx[n]*b[i,n] for n in range(t_steps)),
            #         GRB.EQUAL, velocity_cts[0,i],name='vx_%s'%(i))
            m.addConstr(
                vx[n] - velocity_cts[0,i],
                GRB.LESS_EQUAL, M * (1 - b[i, n]), name='Wpx_%s_%s' % (i, n))

            m.addConstr(
                vx[n] - velocity_cts[0,i],
                GRB.GREATER_EQUAL, -M * (1 - b[i, n]), name='Wpox_%s_%s' % (i, n))
        if velocity_cts[1,i]<math.inf:
            # m.addConstr(quicksum(vy[n]*b[i,n] for n in range(t_steps)),
            #     GRB.EQUAL, velocity_cts[1,i],name='vy_%s'%(i))
            m.addConstr(
                vy[n] - velocity_cts[1, i],
                GRB.LESS_EQUAL, M * (1 - b[i, n]), name='Wpx_%s_%s' % (i, n))

            m.addConstr(
                vy[n] - velocity_cts[1, i],
                GRB.GREATER_EQUAL, -M * (1 - b[i, n]), name='Wpox_%s_%s' % (i, n))
        if velocity_cts[2,i]<math.inf:
            # m.addConstr(quicksum(vz[n]*b[i,n] for n in range(t_steps)),
            #     GRB.EQUAL, velocity_cts[2,i],name='vz_%s'%(i))
            m.addConstr(
                vz[n] - velocity_cts[2, i],
                GRB.LESS_EQUAL, M * (1 - b[i, n]), name='Wpx_%s_%s' % (i, n))

            m.addConstr(
                vz[n] - velocity_cts[2, i],
                GRB.GREATER_EQUAL, -M * (1 - b[i, n]), name='Wpox_%s_%s' % (i, n))

    m.addConstr(
        uz[n]-0,
        GRB.LESS_EQUAL, M * (1 - b[n_gates-1, n]), name='Wpx_%s_%s' % (i, n))

    m.addConstr(
        uz[n] - 0,
        GRB.GREATER_EQUAL, -M * (1 - b[n_gates-1, n]), name='Wpox_%s_%s' % (i, n))

# ---------------------------------------------------
# -------------------------
# Optimize the Problem
# ----------------------------------------------------------------------------
# Collects the values of the variables "A_%_%_%_%_%_%" and "P_%_%_%_%_%_%" in
# a multi-dimensional array called results_A and results_P respectively.
# Note that every aircraft type has the same amoUt of instances (i.e. the total
# number of aircraft).

m.update()
m.write('path_planning.lp')

'Solve using one of the selected optimization softwares.'
# Set time constraint for optimization (5minutes)
# m.setParam('TimeLimit', 5 * 60)
# m.setParam('MIPgap', 0.009)
m.optimize()
m.write("Path_trajectory.sol")
status = m.status
print(status)
if status == GRB.Status.UNBOUNDED:
    print('The model cannot be solved because it is Unbounded')

elif status == GRB.Status.OPTIMAL:
    f_objective= m.objVal
    pos_x = []
    pos_y = []
    pos_z = []
    t_waypoints=[0]
    for n in range(t_steps):
        pos_x.append(px[n].X)
        pos_y.append(py[n].X)
        pos_z.append(pz[n].X)
        for i in range(n_gates):
            if b[i,n].X == 1:
                print('Waypoint %s at time %s'%(i,n*delta_t))
                t_waypoints.append(n*delta_t)
    file=open('timeswp.txt','w')
    file.writelines(["%s\n" % item for item in t_waypoints])
    file.close()

    tot_time= sum(n*delta_t*b[n_gates-1,n].X for n in range(t_steps))
    print(tot_time)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(pos_x, pos_y, pos_z,label='dt=2')
    ax.scatter(waypoints[0,:], waypoints[1,:], waypoints[2,:], zdir='z', s=80, c='red', depthshade=True,label='Waypoints')
    plt.legend()
    title_str='Trajectory achieved in '+str(t_max)+' seconds'
    plt.title(title_str)
    # plt.show()
    write_text(px, py, pz,tot_time, file_name='path_planning.txt')
    print('Optmization time is ', m.Runtime)

elif status != GRB.Status.INF_OR_UBD and status != GRB.Status.INFEASIBLE:
    print('Optimization was stopped with status %d' % status)
    exit(0)