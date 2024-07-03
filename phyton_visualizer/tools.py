from roblib import *

def draw_field_perso(ax, f, q_points, p_points, xmin, xmax, ymin, ymax, a):
    Mx = arange(xmin, xmax, a)
    My = arange(ymin, ymax, a)
    X1, X2 = meshgrid(Mx, My)
    VX, VY = f(X1, X2, q_points, p_points)
    R = sqrt(VX ** 2 + VY ** 2)
    quiver(Mx, My, VX / R, VY / R)


def clamp(x,val1,val2):
    #clamp value x between val1 and val2
    new_x = x
    if new_x>val1 and new_x>val2:
        new_x = max(val1,val2)
    elif new_x<val1 and new_x<val2:
        new_x = min(val1,val2)
    return new_x

def force(r,c):
    return c/r

def torque(r,f):
    return r*f

def val_sign(x):
    #doesn't return 0 compared to the classic sign function
    if x>=0.0:
        return 1
    else:
        return -1

def rotation_matrix(θ):
    return array([[cos(θ),-sin(θ)],[sin(θ),cos(θ)]])

def draw_line(ax, a, b, color='gray'):
    plot2D_perso(ax, hstack((a, b)), color)
    # plot2D(a, 'ro')
    # plot2D(b, 'ro')

def sav_monitor(monitor,file):
    file.write(str(monitor)[1:-1]+"\n")

def draw_monitor(filename,legend,begin=0,title="tab",show=True):
    end = len(legend) + begin
    table = ""
    #getting datas
    with open(filename) as f:
        lines = f.readlines()
        for line in lines :
            values = line.split(",")
            values = np.array([float(x) for x in values])
            if table == "":
                table = values
            else:
                table = np.vstack((table,values))

    #draw datas
    if begin < 0 or len(legend) < 1:
        print("Error draw_monitor(): begin index should be >= 0 (0 = first values after time) and legend list should be > 0")
    elif len(table[0])-1 < begin+len(legend):
        print("Error draw_monitor(): Not enough data found in the file for datas asked")
        print("number of values found (including time): ",len(table[0]))
        print("number of values printable: ", len(table[0])-1)
        print("begin index asked : ",begin)
        print("nb data asked (=nb labels) : ", len(legend))
        print("indexes avaiblable to print : 0 to ", len(table[0])-1)
        print("indexes asked : " + str(begin) + " to " + str(begin+len(legend)))
    else:
        times = table[:, 0]
        #draw simulation values
        fig, ax = plt.subplots()
        for i in range(len(legend)):
            ax.plot(times, table[:,i+begin+1], label=legend[i])
        ax.set_xlabel('time')
        ax.set_title(title)
        leg = ax.legend(loc='upper right')
        if show:
            plt.ion()
            plt.show()

def to_radian(degree):
    return degree*np.pi/180

def to_degree(radian):
    return radian*180/np.pi

def set_value(index,target,dt,table,measures_table="",wait_joints=False,tolerance=0,speed=to_radian(130)):
    """
    Bring a variable to the value wanted with a speed given.
    table = table where the variable is.
    index = index of the variable to modify in a table.
    target = value wanted for the variable.
    speed = speed to converge to this (in value/s) (if the variable is a deg value, then it will represent deg/s).
    dt = time of loop simulation.
    measure = measure of the joint or other variable that need to reach the target.
    wait_joints = If true, the command will wait that the measure reach the target, else, when the command as reached the target, it end.
    """

    err = target - table[index]
    err2 = 0
    if wait_joints:
        if type(measures_table) is not str:
            if len(measures_table) == len(table):
                err2 = target - measures_table[index]
            else:
                print("ERREUR set_value() : The measure table need to have same lenght of table")
        else:
            print("ERREUR set_value() : The measure table is a str but need to be a table")

    table[index] = table[index] + sign(err)*min(abs(err),abs(speed*dt))

    #print("mes : ",measures_table[index])
    #print("tag : ",target)
    #print("Err : ",err)
    #print("cmd : ",table[index])

    if abs(err2) <= tolerance and abs(err) <= 0 :
        return 1
    else:
        return 0


def compute_alpha(beta, epsilon, a):
    """
    :param beta: angle wanted for the leg support axis (0 = vertical)
    :param epsilon: angle of the knees. (knees have the same angle)
    :param a: Lenght of the parts of the leg (all part have same dimensions)
    :return: alpha needed to have theta we want with the epsilon
    """
    sigma = to_radian(180) - epsilon
    z = a*sqrt((5/4)-cos(sigma))
    gamma = -arccos(((3*(a**2))+(4*(z**2)))/(8*z*a))
    alpha = beta + gamma
    return alpha

def compute_alpha2(beta, epsilon):
    alpha = (epsilon/2) - beta
    return -alpha

def simul_spring(k,h,hdamp,t,a,h1,m,g,epsilon_natural):
    """
    Simulate the motor joint position, during the landing, as if they are free. So that we have the springs theoric behavior for the landing.
    :param k: k constant of one spring (there are two of them on the kangaroo leg).
    :param h: Maximum height of the jump.(m)
    :param hdamp: height when the kangaroo starts to touch the floor. damping part.(m)
    :param t: time from the top of the jump (start of the fall) (m)
    :param a: leg part lenght (m)
    :param h1: height between ankle joint and center of body.(m) (should correspond to kagaroo height measured by the simulator)
    :param m: mass of the robot (kg)
    :param g: gravity absolute value (>0) (m.s-2)
    :return: Current height during the damping (for debugging) and theoric position for the knees (rad).
    """

    tdamp = sqrt(-2*(hdamp-h)/g)
    t2 = t-tdamp #damping part time
    if t < tdamp:
        return -0.5*g*t**2 + h, epsilon_natural
    else:
        #bloc A
        vhdamp = -sqrt(2*g*(h-hdamp))
        K = 2*k

        #bloc C
        zdamp = ((2*m)/(2*m+K*(t2**2)))*((((K*hdamp)/(2*m))*(t2**2))+(vhdamp*t2)-(0.5*g*(t2**2))+hdamp)

        #bloc B
        epsilon = np.pi - arccos(((-((zdamp-h1)**2))/(8*(a**2)))+(5/8))

        return zdamp,epsilon

if __name__ == "__main__":
    #test clamp
    x = 4
    print(clamp(x,-1,2))
    x = 2
    print(clamp(x,-1,2))
    x = 1
    print(clamp(x,-1,2))
    x = 0
    print(clamp(x,-1,2))
    x = -0.5
    print(clamp(x,-1,2))
    x = -4
    print(clamp(x,-1,2))
    x = -1
    print(clamp(x,-1,2))

    #compute alpha test
    """print(to_degree(compute_alpha(to_radian(0),to_radian(130),0.075)))"""

    #compute alpha 2 test
    """print(to_degree(compute_alpha2(to_radian(0),to_radian(-40))))"""
    """print(to_degree(compute_alpha2(to_radian(0), to_radian(-111.9))))"""

    #draw monitor and controler test
    """
    last_command = [0, 0, 0, 0, 0, 0, 0, 0, 0]
    measures = last_command.copy()
    end = 0
    target = 10
    time = []
    cmd = []
    trg = []
    mes = []
    t = 0
    dt = 0.1
    wait_joints= True
    tolerance = 0.1
    while end != 1:
        end = set_value(0, target, dt, last_command, measures, wait_joints= wait_joints, tolerance=tolerance, speed=1)
        measures = [measures[ind]+0.1*(last_command[ind]-measures[ind]) for ind in range(len(last_command))]
        time.append(t)
        cmd.append(last_command[0])
        trg.append(target)
        mes.append(measures[0])
        t += dt

    target = 0
    end = 0
    while end != 1:
        end = set_value(0, target, dt, last_command, measures, wait_joints= wait_joints, tolerance=tolerance, speed=1)
        measures = [measures[ind]+0.1*(last_command[ind]-measures[ind]) for ind in range(len(last_command))]
        time.append(t)
        cmd.append(last_command[0])
        trg.append(target)
        mes.append(measures[0])
        t += dt

    figure()
    plot(time,cmd,label="cmd")
    plot(time,trg,label="trg")
    plot(time, mes, label="mes")
    legend()
    show()"""

    """
    filename = "logs/simu09_05_2023__14_28_41.txt"
    legend = ["cmd_joint_ankle_L", "cmd_joint_ankle_R", "cmd_joint_leg_up_L", "cmd_joint_leg_up_R", "cmd_joint_leg_down_L", "cmd_joint_leg_down_R"]
    legend2 = ["joint_ankle_L", "joint_ankle_R", "joint_leg_up_L", "joint_leg_up_R", "joint_leg_down_L", "joint_leg_down_R"]
    draw_monitor(filename,legend,begin=0,title="commands",show=False)
    draw_monitor(filename, legend2,begin=6,title="mesures",show=True) #the last show all the figures before
    """

    #simul springs test
    #the damping theory
    """
    k = 200
    h = 0.5
    hdamp = 0.29
    a = 0.075
    h1 = 0.075
    m = 1
    g = 9.81
    epsilon_natural = to_radian(40)

    epsilons = []
    heights = []
    times = arange(0,10,0.05)

    for t in times :
        #height,epsilon = simul_spring(k,h,hdamp,t,a,h1,m,g,epsilon_natural)
        height, epsilon = simul_spring(k, 0.43929338455200195, hdamp, t, a,h1,m,g,epsilon_natural)
        epsilons.append(epsilon)
        heights.append(height)

    figure("knees degree(rad)")
    plot(times, epsilons)
    figure("height of robot")
    plot(times, heights)
    show()
    """