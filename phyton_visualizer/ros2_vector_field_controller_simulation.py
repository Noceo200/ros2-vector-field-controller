from tools import * 

def f_diff(x,u):
    x,u  = x.flatten(), u.flatten() 
    v,θ = x[2],x[3]    
    return array([[v*cos(θ)],[v*sin(θ)],[u[0]],[u[1]]])

def f_omni(x,u,v_d):
    x,u  = x.flatten(), u.flatten() #x,y,v,θ and  
    v,θ = x[2],x[3] 
    if norm(v_d) > 0.0:
        return array([[v*v_d[0]/norm(v_d)],[v*v_d[1]/norm(v_d)],[u[0]],[u[1]]])
    else:
        return array([[0.0],[0.0],[u[0]],[u[1]]])

def clamp_not_zero(val,min_lim,max_lim,null_radius): #values should be norms > 0.0
    new_val=val
    if val > max_lim:
        new_val = max_lim
    elif val < min_lim and val>null_radius:
        new_val = min_lim
    elif val < null_radius:
        new_val = 0.0
    return new_val

"""
Vector field parameters.
"""

def vector_field(x,y,q_points, p_point):  #-grad(V) #x1 and x2 can be matrix because of draw_vector_field function
    min_spd_norm =  0.1 #use only to clamp minimal attractive command
    max_spd_norm = 1.0 #used for repulsive, attractive and final commands clamping
    null_spd_radius = 0.01 #define minimal cmd_vel sent to be considered
    min_goal_dist = 0.1 #minimal distance between pa and p (if used) to consider the robot successful 
    ka_force = 3.0 #coeff attractivity
    kr_dist = 0.0 #coeff de repoussement (exponential position), can be negative
    kr_force = 3.0 #1.0 #mutiply repulsive force

    """
    Compute clamped repulsive command
    """
    cl_rep_cmd_x, cl_rep_cmd_y = compute_rep_cmd(x,y,q_points,kr_dist,kr_force,max_spd_norm)

    """
    Compute clamped attractivity command
    """
    cl_attr_cmd_x, cl_attr_cmd_y, point_mode = compute_attr_cmd(x,y,p_point,cmd_vel,ka_force,min_spd_norm,max_spd_norm,null_spd_radius)
 
    """
    Merge and clamp commands
    """

    cmd_x = cl_rep_cmd_x + cl_attr_cmd_x #can be matrix
    cmd_y = cl_rep_cmd_y + cl_attr_cmd_y #can be matrix
    remap_ratio = 0.0 #can become a matrix

    if isinstance(cmd_x, (list, tuple, np.ndarray)): #specific to python program that need to deal with matrix
        #print("cl_attr_cmd_x: (50,50):",cl_attr_cmd_x[50,50])
        #print("cl_attr_cmd_y: (50,50):",cl_attr_cmd_y[50,50])
        #print("cl_rep_cmd_x: (50,50):",cl_rep_cmd_x[50,50])
        #print("cl_rep_cmd_y: (50,50):",cl_rep_cmd_y[50,50])
        remap_ratio = np.empty((len(x), len(y)))
        for row in range(len(cmd_x)):
            for col in range(len(cmd_x[row])): 
                #filling matrix
                cmd_x_ = cmd_x[row,col]
                cmd_y_ = cmd_y[row,col]
                cmd_vect = np.array([[cmd_x_],[cmd_y_]])
                if(norm(cmd_vect)>0.0):
                    remap_ratio[row,col] = clamp_not_zero(norm(cmd_vect),0.0,max_spd_norm,0.0)/norm(cmd_vect)
    else: 
        cmd_vect = np.array([[cmd_x],[cmd_y]])
        if(norm(cmd_vect)>0.0): #else it will keep default value of 0.0
            remap_ratio = clamp_not_zero(norm(cmd_vect),0.0,max_spd_norm,0.0)/norm(cmd_vect)

    cl_cmd_x = cmd_x*remap_ratio #can be matrix
    cl_cmd_y = cmd_y*remap_ratio #can be matrix

    if point_mode:
        if isinstance(cl_cmd_x, (list, tuple, np.ndarray)):
            for row in range(len(cl_cmd_x)):
                for col in range(len(cl_cmd_x[row])): 
                    x_ = x[row,col]
                    y_ = y[row,col]
                    if norm(np.array([[x_],[y_]]) - p_point) <= min_goal_dist:
                        cl_cmd_x[row,col] = 0.0
                        cl_cmd_y[row,col] = 0.0
        else:
            if norm(np.array([[x],[y]]) - p_point) <= min_goal_dist:
                cl_cmd_x = 0.0
                cl_cmd_y = 0.0

    return cl_cmd_x,cl_cmd_y

def compute_rep_cmd(x,y,q_points,kr_dist,kr_force,max_spd_norm): #x and y can be matrix in this python simulator
    minus_grad_v_x = 0.0
    minus_grad_v_y = 0.0
    for point in q_points:
        norm_term = sqrt(((x-point[0,0])**2) + (y-point[1,0])**2)
        exp_term = exp(kr_dist-norm_term)
        if norm_term.all() > 0.0:
            minus_grad_v_x += kr_force*(x-point[0,0])*(exp_term/norm_term)
            minus_grad_v_y += kr_force*(y-point[1,0])*(exp_term/norm_term)

    remap_ratio=0.0
    if isinstance(minus_grad_v_x, (list, tuple, np.ndarray)): #specific to python program that need to deal with matrix
        remap_ratio = np.empty((len(minus_grad_v_x), len(minus_grad_v_x)))
        for row in range(len(minus_grad_v_x)):
            for col in range(len(minus_grad_v_x[row])): 
                #filling matrix
                cmd_x_ = minus_grad_v_x[row,col]
                cmd_y_ = minus_grad_v_y[row,col]
                cmd_vect = np.array([[cmd_x_],[cmd_y_]])
                if(norm(cmd_vect)>0.0): #else it will keep default value of 0.0
                    remap_ratio[row,col] = clamp_not_zero(norm(cmd_vect),0.0,max_spd_norm,0.0)/norm(cmd_vect)
    else:
        cmd_vect = np.array([[minus_grad_v_x],[minus_grad_v_y]])
        if(norm(cmd_vect)>0.0): #else it will keep default value of 0.0
            remap_ratio = clamp_not_zero(norm(cmd_vect),0.0,max_spd_norm,0.0)/norm(cmd_vect)

    cl_rep_cmd_x = minus_grad_v_x*remap_ratio #can be matrix
    cl_rep_cmd_y = minus_grad_v_y*remap_ratio #can be matrix
    return cl_rep_cmd_x, cl_rep_cmd_y

def compute_attr_cmd(x,y,p_point,cmd_vel,ka_force,min_spd_norm,max_spd_norm,null_spd_radius): #x and y can be matrix in this python simulator
    point_mode = False
    if norm(cmd_vel) > null_spd_radius: #then we should apply the received CMD rather than doing the point following
        remap_ratio = clamp_not_zero(norm(cmd_vel),min_spd_norm,max_spd_norm,0.0)/norm(cmd_vel)
        cl_attr_cmd_x = cmd_vel[0,0]*remap_ratio
        cl_attr_cmd_y = cmd_vel[1,0]*remap_ratio
        return cl_attr_cmd_x, cl_attr_cmd_y, point_mode
    else: #otherwise we do the point following
        point_mode = True
        cmd_d_x = ka_force*(p_point[0,0]-x)
        cmd_d_y = ka_force*(p_point[1,0]-y)

        remap_ratio = 0.0 #might become a matrix

        if isinstance(cmd_d_x, (list, tuple, np.ndarray)):
            #print("cmd_d_x: (50,50):",cmd_d_x[50,50])
            #print("cmd_d_y: (50,50):",cmd_d_y[50,50])
            remap_ratio = np.empty((len(cmd_d_x), len(cmd_d_x)))
            for row in range(len(cmd_d_x)):
                for col in range(len(cmd_d_x[row])): 
                    #filling matrix
                    cmd_d_x_ = cmd_d_x[row,col]
                    cmd_d_y_ = cmd_d_y[row,col]
                    cmd_d_vect_ = np.array([[cmd_d_x_],[cmd_d_y_]])
                    if norm(cmd_d_vect_) > 0.0: #other remap ratio keep default value of 0.0
                        remap_ratio[row,col] = clamp_not_zero(norm(cmd_d_vect_),min_spd_norm,max_spd_norm,0.0001)/norm(cmd_d_vect_)
            #print("remap_ratio: (50,50):",remap_ratio[50,50])
        else:
            cmd_d_vect = np.array([[cmd_d_x],[cmd_d_y]])
            if norm(cmd_d_vect) > 0.0: #other remap ratio keep default value of 0.0
                remap_ratio = clamp_not_zero(norm(cmd_d_vect),min_spd_norm,max_spd_norm,0.0001)/norm(cmd_d_vect)

        cl_attr_cmd_x = cmd_d_x*remap_ratio #can be matrixes
        cl_attr_cmd_y = cmd_d_y*remap_ratio #can be matrixes

        return cl_attr_cmd_x, cl_attr_cmd_y, point_mode

#list of repusive points
q_points = [
    #array([[-3.0],[-3.0]])
    ] 

for i in np.arange(-4,5,2):
    for y in np.arange(-4,5,2):
        q_points.append(array([[i],[y]]))


#Attractive point
p_point = array([[3.0],[3.0]])

#constant speed command for teleoperation
cmd_vel = array([[0.0],[0.0]])

#simu parameters
x = array([[-6,0,0,0]]).T #x,y,v,θ
θbar = 0
dt = 0.1
s = 7
ax=init_figure(-s,s,-s,s)
pause(1) #to have time to record

pos_sav = []
#simulation
for t in arange(0,50,dt):
    #mesures
    mx, my, mv, mθ = x.flatten()

    #commandes
    #p_point = array([[3*cos((t*2)/10)], [3*sin((t*2)/10)]])
    vhat = array([[0.0],[0.0]]) #array([[-0.3*2*sin((t*2)/10)],[0.3*2*cos((t*2)/10)]]) #dérivée de phat #array([[0.0],[0.0]])
    #q_points[0] = array([[cos(-t/10)+p_point[0,0]], [sin(-t/10)+p_point[1,0]]])

    #teleoperation cmd
    #cmd_vel = array([[1.2*cos((t*2)/2)], [1.2*sin((t*2)/2)]])

    #control
    wx, wy = vector_field(mx,my,q_points,p_point) #gradient à suivre
    w = array([wx,wy])
    vbar = norm(w) #max(min(norm(w),1.0),norm(vhat)*1.1) #vitesse capée dans le vector field direct
    #print(vbar)
    θbar = angle(w)
    u=array([[vbar-mv],[sawtooth(θbar-mθ)]])
    θbar_omni = sawtooth(angle(p_point-array([[mx],[my]])))
    u_omni = array([[vbar-mv],[sawtooth(θbar_omni-mθ)]])

    #simulation
    #x=x+dt*f_diff(x,u)
    x=x+dt*f_omni(x,u_omni,w)
    pos_sav.append(np.array([[x[0]],[x[1]]]))

    #affichage
    clear(ax)
    for point in q_points:
        draw_disk(ax,point,0.1,"magenta")
    draw_disk(ax,p_point,0.1,"green")
    draw_tank(x[[0,1,3]],'red',0.1) # x,y,θ
    draw_disk(ax,x[[0,1]],0.05,"blue")
    draw_field_perso(ax,vector_field,q_points, p_point,-s,s,-s,s,0.2)
    for pos in pos_sav:
        draw_disk(ax,pos,0.03,"blue")

print("FINISHED")
pause(20)    


