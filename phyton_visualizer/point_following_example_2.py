from tools import * 

def f_diff(x,u):
    x,u  = x.flatten(), u.flatten() 
    v,θ = x[2],x[3]    
    return array([[v*cos(θ)],[v*sin(θ)],[u[0]],[u[1]]])

def f_omni(x,u,v_d):
    x,u  = x.flatten(), u.flatten() #x,y,v,θ and  
    v,θ = x[2],x[3] 
    return array([[v*v_d[0]/norm(v_d)],[v*v_d[1]/norm(v_d)],[u[0]],[u[1]]])

"""
Vector field parameters.
"""

min_spd_x = -1.0
min_spd_y = -1.0
max_spd_x = 1.0
max_spd_y = 1.0
gap_radius_x = 0.1
gap_radius_y = 0.1
null_radius_x = 0.1
null_radius_y = 0.1
ka = 1.0 #coeff attractivity
kr = 0.5 #coeff de repoussement (exponential position)

def vector_field(x,y,q_points, p_points):  #-grad(V) #x1 and x2 can be matrix because of draw_vector_field function

    grad_v_x = 0.0
    grad_v_y = 0.0
    for point in q_points:
        norm_term = sqrt(((x-point[0,0])**2) + (y-point[1,0])**2)
        exp_term = exp(kr-norm_term)
        if norm_term.all() !=0:
            grad_v_x += (x-point[0,0])*(exp_term/norm_term)
            grad_v_y += (y-point[1,0])*(exp_term/norm_term)
    for point in p_points:
        norm_term = sqrt(((x-point[0,0])**2) + (y-point[1,0])**2)**(ka-2)
        attr_int_x = (x-point[0,0])*(-ka*norm_term)
        attr_int_y = (y-point[1,0])*(-ka*norm_term)
        if isinstance(attr_int_x, (list, tuple, np.ndarray)): #we clamp attractivity, otherwise it will have a huge impact on points far from it, but we need to keep the good direction
            grad_v_x += [[clamp_not_zero(x,min_spd_x,max_spd_x,gap_radius_x,null_radius_x) for x in row] for row in attr_int_x]
            grad_v_y += [[clamp_not_zero(y,min_spd_y,max_spd_y,gap_radius_y,null_radius_y) for y in row] for row in attr_int_y]
        else:
            grad_v_x += clamp_not_zero(attr_int_x,min_spd_x,max_spd_x,gap_radius_x,null_radius_x)
            grad_v_y += clamp_not_zero(attr_int_y,min_spd_y,max_spd_y,gap_radius_y,null_radius_y)

    print(grad_v_x)

    return grad_v_x+cmd_off[0,0],grad_v_y+cmd_off[1,0]

def clamp_not_zero(val,min_lim,max_lim,gap_radius,null_radius):
    new_val=val
    if val > max_lim:
        new_val = max_lim
    elif val < min_lim:
        new_val = min_lim
    elif abs(val) < gap_radius:
        new_val = gap_radius
        if abs(val) < null_radius:
            new_val = 0.0
    return new_val

#list of repusive points
q_points = [
    array([[-3],[-3]])
    ] 

for i in np.arange(-4,5,2):
    for y in np.arange(-4,5,2):
        q_points.append(array([[i],[y]]))


#list of attractive points
p_points = [
    array([[3],[3]])
    ] 

#constant speed offset (used for teleoperation for example)
cmd_off = array([[0.0],[0.0]])

x = array([[0,0,0,0]]).T #x,y,v,θ
θbar = 0
dt = 0.1
s = 5
ax=init_figure(-s,s,-s,s)
pause(5) 
for t in arange(0,50,dt):
    #mesures
    mx, my, mv, mθ = x.flatten()

    #commandes
    p_points[0] = array([[3*cos((t*2)/10)], [3*sin((t*2)/10)]])
    vhat = array([[0.0],[0.0]]) #array([[-0.3*2*sin((t*2)/10)],[0.3*2*cos((t*2)/10)]]) #dérivée de phat #array([[0.0],[0.0]])
    q_points[0] = array([[cos(-t/10)+p_points[0][0,0]], [sin(-t/10)+p_points[0][1,0]]])

    #teleoperation cmd
    #cmd_off = array([[0.5*cos((t*2)/2)], [0.5*sin((t*2)/2)]])

    #control
    wx, wy = vector_field(mx,my,q_points,p_points) #gradient à suivre
    w = array([wx,wy])
    vbar = clamp_not_zero(norm(w),min_spd_y,max_spd_y,gap_radius_y,null_radius_y) #max(min(norm(w),1.0),norm(vhat)*1.1) #vitesse capée dans le vector field direct
    θbar = angle(w)
    u=array([[vbar-mv],[sawtooth(θbar-mθ)]])
    θbar_omni = sawtooth(angle(p_points[0]-array([[mx],[my]])))
    u_omni = array([[vbar-mv],[sawtooth(θbar_omni-mθ)]])

    #simulation
    #x=x+dt*f_diff(x,u)
    x=x+dt*f_omni(x,u_omni,w)

    #affichage
    clear(ax)
    for point in q_points:
        draw_disk(ax,point,0.1,"magenta")
    for point in p_points:
        draw_disk(ax,point,0.1,"green")
    draw_tank(x[[0,1,3]],'red',0.1) # x,y,θ
    draw_disk(ax,x[[0,1]],0.05,"blue")
    draw_field_perso(ax,vector_field,q_points, p_points,-s,s,-s,s,0.2)

pause(1)    


