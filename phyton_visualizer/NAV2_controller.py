from tools import * 

def f_diff(x,u):
    x,u  = x.flatten(), u.flatten() 
    v,θ = x[2],x[3]    
    return array([[v*cos(θ)],[v*sin(θ)],[u[0]],[u[1]]])

def f_omni(x,u,v_d):
    x,u  = x.flatten(), u.flatten() #x,y,v,θ and  
    v,θ = x[2],x[3] 
    return array([[v*v_d[0]/norm(v_d)],[v*v_d[1]/norm(v_d)],[u[0]],[u[1]]])

def vector_field(x1,x2,q_points, p_points):  #-grad(V) #x1 and x2 can be matrix because of draw_vector_field function
    coef = 1 #coeff de repoussement
    grad_v_x = 0.0
    grad_v_y = 0.0
    for point in q_points:
        nq1 = coef*(x1-point[0,0])
        nq2 = coef*(x2-point[1,0])
        norm3 = (nq1**2+nq2**2)**(3/2)
        if norm3.all() !=0:
            grad_v_x += -nq1/norm3
            grad_v_y += -nq2/norm3
    for point in p_points:
        grad_v_x += 2*(x1-point[0,0])
        grad_v_y += 2*(x2-point[1,0])
    return -grad_v_x,-grad_v_y


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
    vhat = array([[-0.3*2*sin((t*2)/10)],[0.3*2*cos((t*2)/10)]]) #dérivée de phat
    q_points[0] = array([[cos(-t/10)+p_points[0][0,0]], [sin(-t/10)+p_points[0][1,0]]])

    #control
    wx, wy = vector_field(mx,my,q_points,p_points) #gradient à suivre
    w = array([wx,wy])
    vbar = norm(vhat)*1.1 #min(0.5*norm(w),0.2) #vitesse capée
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
        draw_disk(ax,point,0.2,"magenta")
    for point in p_points:
        draw_disk(ax,point,0.2,"green")
    draw_tank(x[[0,1,3]],'red',0.1) # x,y,θ
    draw_field_perso(ax,vector_field,q_points, p_points,-s,s,-s,s,0.4)

pause(1)    


