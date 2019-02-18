from gurobipy import *
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# ============================================================================
#
# Optimization Model
#
# ============================================================================
'Drone dynamics data'
g=9.81        # m/s^2
V_max=188/3.6 # m/s
U_max=5*g     # m/s^2
delta_t=0.1   # sec
t_max=60    # sec
t_steps=int((t_max+delta_t)/delta_t) 	 # [-]
D_sides=32   				 # [-]
gamma=20*math.pi/180    	 # deg
alpha=20*math.pi/180		 # deg

waypoints =10*np.array([[0,  2, 4,  2, 0, -2,-4, -2,0],
                     [0, -2, 0,  2, 0, -2, 0,  2,0],
                     [1,1.5, 2,1.5, 1,1.5, 2,1.5,1]])

waypoints=waypoints.astype(int)
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
Un={}
er={}
'Commodity variables for discretization'
theta={}
# ----------------------------------------------------------------------------
# Create Objective Function
# ----------------------------------------------------------------------------)
for d in range(D_sides):
    theta[d]=2*math.pi*(d+1)/D_sides

# for j in range(9):
#     for i in range(3):
#         er[j,i]= m.addVar(obj=1,
#                         vtype=GRB.CONTINUOUS,
#                         name="E_%s_%s" % (j,i))

for n in range(t_steps):
    Un[n]= m.addVar(obj=1,
                    vtype=GRB.CONTINUOUS,
                    name="U_%s" % (n))

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
m.update()

# Optimize
m.setObjective(m.getObjective(), GRB.MINIMIZE)  # The objective is to maximize revenue

# ----------------------------------------------------------------------------
# Create Constraints
# ----------------------------------------------------------------------------
'Accelerations Constraint'
for n in range(t_steps):
    for d in range(D_sides):
        m.addConstr(
            math.cos(theta[d])*math.sin(-alpha)*ux[n]+math.sin(theta[d])*math.sin(-alpha)*uy[n]+math.sin(-alpha)*uz[n],
            GRB.LESS_EQUAL,U_max, name='U_cts1_%s_%s' % (n,d))

    m.addConstr(
        math.cos(theta[d]) * math.sin(alpha) * uz[n] + math.sin(theta[d]) * math.sin(alpha) * ux[n] + math.sin(alpha) * uz[n],
        GRB.LESS_EQUAL, U_max, name='U_cts2_%s_%s' % (n, d))

    m.addConstr(
        math.cos(theta[d])*uy[n]+math.sin(theta[d])*uz[n],
        GRB.LESS_EQUAL,U_max, name='U_cts3_%s_%s' % (n,d))

'Velocity Constraint'
for n in range(t_steps):
    for d in range(D_sides):
        m.addConstr(
            math.cos(theta[d])*math.sin(gamma)*vx[n]+math.sin(theta[d])*math.sin(gamma)*vy[n]+math.sin(-gamma)*vz[n],
            GRB.LESS_EQUAL,V_max, name='V_cts1_%s_%s' % (n,d))

    m.addConstr(
        math.cos(theta[d]) * math.sin(gamma) * vz[n] + math.sin(theta[d]) * math.sin(gamma) * vx[n] + math.sin(gamma) * vz[n],
        GRB.LESS_EQUAL, V_max, name='V_cts2_%s_%s' % (n, d))

    m.addConstr(
        math.cos(theta[d])*vy[n]+math.sin(theta[d])*vz[n],
        GRB.LESS_EQUAL,V_max, name='V_cts3_%s_%s' % (n,d))

'Integration scheme Constraint'
for n in range(1,t_steps):
    for d in range(D_sides):
        'Position'
        m.addConstr(
            px[n-1]+vx[n-1]*delta_t+1/2*delta_t**2*ux[n-1],
            GRB.EQUAL,px[n], name='Px_cts_%s_%s' % (n,d))

        m.addConstr(
            py[n-1]+vy[n-1]*delta_t+1/2*delta_t**2*uy[n-1],
            GRB.EQUAL,py[n], name='Py_cts_%s_%s' % (n,d))
        m.addConstr(
            pz[n-1]+vz[n-1]*delta_t+1/2*delta_t**2*uz[n-1],
            GRB.EQUAL,pz[n], name='Pz_cts_%s_%s' % (n,d))

        'Velocity'
        m.addConstr(
            vx[n-1]+delta_t*ux[n-1],
            GRB.EQUAL,vx[n], name='Vx_cts_%s_%s' % (n,d))
        m.addConstr(
            vy[n-1]+delta_t*uy[n-1],
            GRB.EQUAL,vy[n], name='Vy_cts_%s_%s' % (n,d))
        m.addConstr(
            vz[n-1]+delta_t*uz[n-1],
            GRB.EQUAL,vz[n], name='Vz_cts_%s_%s' % (n,d))

'Waypoint Constraint'
tolerance = 0.0
t_waypoints =np.linspace(0,t_max,9)
my_list=list(range(waypoints.shape[1]))
for i in my_list:
    j=int(t_waypoints[i]/delta_t)
    m.addConstr(
        px[j],
        GRB.EQUAL, waypoints[0,i])

    m.addConstr(
        py[j],
        GRB.EQUAL, waypoints[1,i])

    m.addConstr(
        pz[j],
        GRB.EQUAL, waypoints[2,i])

    # m.addConstr(
    #     px[j],
    #     GRB.GREATER_EQUAL, waypoints[0, i] - er[j])
    # m.addConstr(
    #     py[j],
    #     GRB.GREATER_EQUAL, waypoints[1, i] - er[j])
    # m.addConstr(
    #     pz[j],
    #     GRB.GREATER_EQUAL, waypoints[2, i] - er[j])
# ---------------------------------------------------
# -------------------------
# Optimize the Problem
# ----------------------------------------------------------------------------
# Collects the values of the variables "A_%_%_%_%_%_%" and "P_%_%_%_%_%_%" in
# a multi-dimensional array called results_A and results_P respectively.
# Note that every aircraft type has the same amount of instances (i.e. the total
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
    print('The model cannot be solved because it is unbounded')

elif status == GRB.Status.OPTIMAL:
    f_objective= m.objVal
    pos_x = []
    pos_y = []
    pos_z = []
    for n in range(int(t_max/delta_t)):
        pos_x.append(px[n].X)
        pos_y.append(py[n].X)
        pos_z.append(pz[n].X)
        print(px[n].X,py[n].X,pz[n].X)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(pos_x, pos_y, pos_z, zdir='z', s=20, c=None, depthshade=True)
    ax.scatter(waypoints[0,:], waypoints[1,:], waypoints[2,:], zdir='z', s=80, c='red', depthshade=True)
    plt.show()

elif status != GRB.Status.INF_OR_UNBD and status != GRB.Status.INFEASIBLE:
    print('Optimization was stopped with status %d' % status)
    exit(0)