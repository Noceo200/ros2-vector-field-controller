from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py

def f(x,u):
    x,u  = x.flatten(), u.flatten()
    v,θ = x[2],x[3]    
    return array([[v*cos(θ)],[v*sin(θ)],[u[0]],[u[1]]])

def f1(x1,x2):  #-grad(V)
    coef = 0.5#coeff de repoussement
    nq1 = coef*(x1-qhat[0,0])
    nq2 = coef*(x2-qhat[1,0])
    norm3 = (nq1**2+nq2**2)**(3/2)
    return vhat[0,0]-2*(x1-phat[0,0])+ nq1/norm3,vhat[1,0]-2*(x2-phat[1,0])+nq2/norm3

phat = array([[3],[3]]) #point attirant
qhat = array([[1],[1]]) #point repoussant
vhat = array([[0],[0]]) #vitesse du point attirant
x = array([[0,0,0,0]]).T #x,y,v,θ
θbar = 0
dt = 0.1
s = 5
ax=init_figure(-s,s,-s,s)

for t in arange(0,50,dt):
    #mesures
    mx, my, mv, mθ = x.flatten()

    #commandes
    phat = array([[cos(t/10)], [2*sin(t/10)]])
    vhat = array([[-0.1*sin(t/10)],[0.2*cos(t/10)]]) #dérivée de phat
    qhat = array([[2*cos(t/5)], [2*sin(t/5)]])

    #control
    wx, wy = f1(mx,my) #gradient à suivre
    w = array([wx,wy])
    vbar = min(0.5*norm(w),1) #vitesse capée à 1
    θbar = angle(w)
    u=array([[vbar-mv],[sawtooth(θbar-mθ)]])

    #simulation
    x=x+dt*f(x,u)

    #affichage
    clear(ax)
    draw_disk(ax,qhat,0.2,"magenta")
    draw_disk(ax,phat,0.2,"green")
    draw_tank(x[[0,1,3]],'red',0.1) # x,y,θ
    draw_field(ax,f1,-s,s,-s,s,0.4)

pause(1)    


