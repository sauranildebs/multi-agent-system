#!/usr/bin/env python
# license removed for brevity
import rospy
import array
from std_msgs.msg import String
from visualization_msgs.msg import *
import numpy
import matplotlib.pyplot as plt
from matplotlib import path
from math import *
import copy
import mavros
from mavros_msgs.msg import GlobalPositionTarget
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from mavros_msgs.srv import * 
#from planar import Polygon
#global lat, lon , ini_lat , ini_lon , flag2;
#I = []
#global lat, lon , ini_lat , ini_lon ;
# global lat 
# global lon
limiting_angle=numpy.radians(40)
ini_angle1 = 0.0
heading_angle1 = 0.0
ini_local_lat1 = 0.0
ini_local_lon1 = 0.0
local_lat1 = 0.0
local_lon1 = 0.0
lat = 0.0
lon = 0.0 
ini_lat1 = 0.0
ini_lon1 = 0.0
flag2 = 0 
flag31 = 0
flag41 = 0

ini_angle2 = 0.0
heading_angle2 = 0.0
ini_local_lat2 = 0.0
ini_local_lon2 = 0.0
local_lat2 = 0.0
local_lon2 = 0.0
lat = 0.0
lon = 0.0 
ini_lat2 = 0.0
ini_lon2 = 0.0
flag2 = 0 
flag32 = 0
flag42 = 0
#type_mask = None
#coor_frame = None
# def callback(msg):
#     message = "recieved  " + msg.data
#     print(message)


    #return S

def singlecollectingPositionwithin(S,Pc,X,Y,destination):
    #print("In singlecollectingPositionwithin ...............")
    DGCM = []
    for i in range(numpy.shape(numpy.transpose(X))[0]):
        DGCM.append(numpy.sqrt(numpy.square(X[i] - destination[0]) + numpy.square(Y[i] - destination[1])))
    #if(len(X) != 0):
    idx = []
    idx.append(DGCM.index(max(DGCM)))
    #print("idx: " , idx , " popopopopopopopopopopopopop")
    if(len(idx) != 0):
        #print("Sssssssssssss: " , S)
        #S = numpy.zeros([1,2])
        S[0,0] = X[idx[0]]                                                                                 
        S[0,1] = Y[idx[0]]
        #pt = numpy.asarray(pt)
        #print("numpy.ndim(S): " , numpy.ndim(S)  , " " , S)
        S,s2 = moveSheperd(S,destination[:],Pc,-1)   
    #print("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
    return S    
    

def formcircle(Sh,centre,radius,ang,q):
    #print("In formcircle ...................")
    s = numpy.shape(Sh)[0]
    Sh2 = copy.deepcopy(Sh)
    points = getcircularpoints(centre,radius,s,ang,q)
    #print("points: " , points , " yyyyyyyyyyyyyyyyyyyyyy  s: ", s , " Sh2: " , Sh2)
    S = numpy.zeros([numpy.shape(Sh)[0],2])
    for i in range(numpy.shape(Sh)[0]):
        D = []
        for j in range(numpy.shape(Sh2)[0]):
            D.append(numpy.sqrt(numpy.square(points[j][0] - Sh2[j][0]) + numpy.square(points[j][1] - Sh2[j][1])))
        #print("FORMCIRCLE D = : " , D , " points: " , points)
        idx = D.index(min(D))
        S[i] = [points[idx][0],points[idx][1]]
        Sh2[i] = [-1000,-1000]       
        points[idx] = [10000,10000]
        #print(idx , " S[i] " , S[i] , " \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\")
    Sh = S
    #print("in formcircle Sh: " , Sh, )
    #print(" zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz")
    return Sh

def getcircularpoints(centre,radius,Nopoints,ang,q):
    #Nopoints = no. of sheperds + 1 
    #print("In getcircularpoints")
    #print("Nopoints: " , Nopoints , "  " , numpy.zeros([1,2]))
    #print("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^")
    #print("center " , centre , " rad " , radius , " ang " , ang)
    if(Nopoints%2 == 0):
        ang = ang - 0.5*q*(22.0/7   )/Nopoints
    points = numpy.zeros([Nopoints,2])    
    for i in range(Nopoints):
        points[i,0] = centre[0]+radius*numpy.cos(ang+((i-ceil((Nopoints)/2))*q*3.14/Nopoints));
        points[i,1] = centre[1]+radius*numpy.sin(ang+((i-ceil((Nopoints)/2))*q*3.14/Nopoints));   
    #print("points in getcircular: " , points)
    #print("OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO")
    return points           

def surroundEffectCollector( Sh,X,Y,ra,Pd,Pc,destination,f,mindisShep,rhoash,alpha,ang):
    #print("IN surroundEffectCollector ...............................")
    RoS = []
    a = 1
    XGCM = numpy.mean(X)
    YGCM = numpy.mean(Y)
    GCM = [XGCM,YGCM]
    D = []
    for i in range(len(X)):
        D.append(numpy.sqrt(numpy.square(X[i] - GCM[0]) + numpy.square(Y[i] - GCM[1])))
    Sh = formcircle(Sh,GCM,alpha*a*0.25*max(D)*mindisShep,ang,2)
    #print("Sh suuroundEffectColl after FORMCIRCLE: " , Sh)
    return Sh 
    #print("EEEEEEEEEEEEEEEEEEEEEE")


def surroundEffect(Sh,X,Y,ra,Pd,destination,q,mindisShep,rhoash,rs):
    #print("In surroundEffect .......................")
    radius = 60
    XGCM = numpy.mean(X)
    YGCM = numpy.mean(Y)
    coS = [numpy.mean(Sh[:,0]),numpy.mean(Sh[:,1])]
    #print("should be mean of sheperd pos coS : " , coS , " cccccccccccccccccc")
    point1 = [XGCM,YGCM]
    diff = [point1[0] - coS[0] , point1[1] - coS[1]]
    if(0.5*numpy.linalg.norm(diff) < 3*ra):
        a = 3*ra
    else:
        a = 0.5*numpy.linalg.norm(diff)
    pt1 = numpy.asarray(point1)
    destination = numpy.asarray(destination)
    centre,notused = moveSheperd(pt1,destination,a,1)
    #print("after moveSheperd centre: " , centre , " also Sh before formcircle: " , Sh)
    ang = numpy.arctan2(destination[1] - point1[1] , destination[0] - point1[0])
    Sh = formcircle(Sh,centre,radius,ang,q)
    return Sh       
    #print("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&")

def monitorspill(Sh,I,X,Y,ra,Pd,Pc,destination,q,f,rhoash,rs,l,mindisShep):
    #print("In monitorspill ..........................")
    mode = 6
    #currentmode = []
    GCM=[numpy.mean(X), numpy.mean(Y)];
    rad = 35
    if(len(l) != 0):
        Shup = Sh[l,:]
        Sh1 = copy.deepcopy(Shup)
    #print("I : " , I , " eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee")    
    Shbase=Sh[I,:]
    #print("Shbase: " , Shbase , " fffffffffffffffffffffffffffffff")
    ang = numpy.arctan2(GCM[1]-destination[1],GCM[0]-destination[0])
    #points = getcircularpoints( GCM,rad,numpy.shape(Shup)[0]+1,ang,q);
    points = getcircularpoints( GCM,rad,len(l)+1,ang,q);
    if(len(l) == 0):
        l = []
        for i in range(1): #instead of numpy.shape(Shup)[0] as Shup not defined
            midpoint = [numpy.sum(points[i:i+2,0]),numpy.sum(points[i:i+2,1])]
            D = []
            for i in range(numpy.shape(Sh1)[0]):
                D.append(numpy.sqrt(numpy.square(Sh1[i][0] - midpoint[0]) + numpy.square(Sh1[i][1] - midpoint[1])))
            idx = D.index(min(D))
            l.append(idx)
            Sh1[idx] = [-1000,-1000]
    #for i in range(numpy.shape(Sh)[0]):
    #check if the below will work ?? , else put a for loop or google for path with arrays
    NetX = numpy.zeros(numpy.shape(Shbase)[0] + numpy.shape(points)[0])
    NetY = numpy.zeros(numpy.shape(Shbase)[0] + numpy.shape(points)[0])    
    #print("NetX : " , NetX )
    #print("NetY : " , NetY)
    #print("Shbase: " , Shbase , " " , Shbase[0][0])
    #print("points:  " , points)
    #print("GGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG")    
    for i in range(numpy.shape(Shbase)[0] + numpy.shape(points)[0]):
        if(i < numpy.shape(Shbase)[0]):
            NetX[i] = Shbase[i][0]
            NetY[i] = Shbase[i][1]
        else:
            NetX[i] = points[numpy.shape(Shbase)[0] - i][0]
            NetY[i] = points[numpy.shape(Shbase)[0] - i][0]

    #print("NetX : " , NetX)
    #print("NetY : " , NetY)
    PTS = numpy.asarray([NetX,NetY]).reshape(numpy.shape(NetX)[0],2)

    #print("jojojjojojjojojojojojojojojo")        
    
    #p = path.Path([NetX,NetY])
    #p = path.Path([points[:,0],points[:,1]])
    #print("PTS:  " , PTS, " cccccccccccccccccccccccccccccccccccccc")
    p = path.Path([[PTS[i][0],PTS[i][1]] for i in range(numpy.shape(NetX)[0])])
    #p = geometry.Polygon([[PTS[i][0],PTS[i][1]] for i in range(numpy.shape(NetX)[0])])
    #print("p: " , p , "ppppppppppppppppppppppppppppppppppppp")
    Xt = numpy.transpose(X)
    Yt = numpy.transpose(Y)
    pts = numpy.zeros([numpy.shape(Xt)[0],2])
    for j in range(numpy.shape(Xt)[0]):
        pts[j][0] = Xt[j]
        pts[j][1] = Yt[j]
    IN = p.contains_points(pts) 
    idx = numpy.where(IN == False)
    Xout = []
    Yout = []           
    for j in idx[0]:
        Xout.append(X[j])
        Yout.append(Y[j])
    if(len(Xout) != 0 and numpy.shape(Shup)[0]>1):
        #print("numpy.shape(Shup)[0]: " , numpy.shape(Shup)[0] , "tttttttttttttttttttttttttttt")
        for i in range(numpy.shape(Shup)[0]):
            p = path.Path([[(GCM[0],GCM[1]),(points[i,0],points[i,1]),(points[i+1,0],points[i+1,1])]])
            Xt = numpy.transpose(Xout)
            Yt = numpy.transpose(Yout)
            pts = numpy.zeros([numpy.shape(Xt)[0],2])
            for j in range(numpy.shape(Xt)[0]):
                pts[j][0] = Xt[j]
                pts[j][1] = Yt[j]
            IN = p.contains_points(pts)             
            idx = numpy.where(IN == True)
            Xin = []
            Yin = []
            for j in idx[0]:
                Xin.append(Xout[j]) 
                Yin.append(Yout[j])
            DGCM = []
            idx = []
            for j in range(len(Xin)):
                DGCM.append(numpy.sqrt(numpy.square(Xin[i] - numpy.mean(Xin)) + numpy.square(Yin[i] - numpy.mean(Yin))))        
                if(DGCM[j] > 0.5*f):
                    idx.append(j)
            if(len(idx) == 0):
                Sh[l[i]] = singleherdingPosition(Sh[l[i]],Pc,Xin,Yin,GCM);      
            else:
                Sh[l[i]] = singlecollectingPositionwithin(Sh[l[i]],0.25*Pd,Xin,Yin,[numpy.mean(Xin), numpy.mean(Yin)])   
    else:
        Shbase = surroundEffect(Sh,X,Y,ra,0.25*Pd,destination,q,mindisShep,rhoash,rs);          
        Shup = []
    Shup=Sh[l,:];
    return Shup,l       
    #print("JJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJ")

    
def movestrayseperate(Sh,X,Y,ra,Pd,Pc,destination,q,mindisShep,rhoash,rs,I,f):
    #print("In movestrayseperate : ,,,,,,,,,,,,,,,,,,,I: " , I)
    XGCM = numpy.mean(X)
    YGCM = numpy.mean(Y)
    mode = 5
    q = 1.8
    f = ra*numpy.log(len(X))
    CGCM = [XGCM,YGCM]
    dist = numpy.zeros(numpy.shape(Sh)[0])
    if(len(I) == 0):
        for i in range(numpy.shape(Sh)[0]):
            dist[i] = numpy.sqrt(numpy.square(Sh[i][0] - destination[0]) + numpy.square(Sh[i][1] - destination[1]))
        I = numpy.argsort(dist)
        I = I[-3:0] # check that the last 3 entries are coming
        #print("I : " , I , " I n movestray blahblahblah")   
    l = []    
    for i in range(numpy.shape(Sh)[0]):
        l.append(i)
    l = numpy.setdiff1d(l,I) #l has only those entries which arent in I
    Shbase = Sh[I,:]
    Shup = Sh[l,:]
    Shbase = surroundEffect(Shbase,X,Y,ra,0.25*Pd,destination,q,mindisShep,rhoash,rs)
    #In monitorspill why Sh and why not Shup
    #print("Sh: " , Sh , " I: " , I , " X: " , X , "Y: " , Y)
    #print("mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm ")
    Shup,l = monitorspill(Sh,I,X,Y,ra,Pd,Pc,destination,q,f,rhoash,rs,l,mindisShep)
    Sh[I,:] = Shbase
    Sh[l,:] = Shup  
    return Sh
    #print("LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL")

def herdingPosition(Sh,Pc,X,Y,destination):
    #print("In herdingPosition:  ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,")
    X1 = X
    Y1 = Y
    diff = numpy.shape(Sh)[0] - len(X)
    S = copy.deepcopy(Sh)
    for i in range(numpy.shape(Sh)[0]-diff):
        SGCM = []
        for j in range(len(X)):
            SGCM.append(numpy.sqrt(numpy.square(X[j] - Sh[i,0]) + numpy.square(Y[j] - Sh[i,1])))
        idx1 = SGCM.index(min(SGCM))
        S,s = moveSheperd(numpy.asarray([X[idx1],Y[idx1]]),destination,Pc,-1)
        #S[i,0] = S1[0]
        #S[i,1] = S1[1]
        numpy.delete(X,idx1)
        numpy.delete(Y,idx1)

    if(diff <= 0):
        for i in range(numpy.shape(Sh)[0]-diff+1,numpy.shape(Sh)[0]):
            SGCM =[]
            for j in range(len(X1)):
                SGCM.append(numpy.sqrt(numpy.square(X1[j] - Sh[i,0]) + numpy.square(Y1[j] - Sh[i,1])))      
            idx1 = SGCM.index(min(SGCM))
            S,s = moveSheperd(numpy.asarray([X[idx1],Y[idx1]]),destination,Pc,-1)
            #S[i,0] = S1[0]
            #S[i,1] = S1[1]  
    Sh = S    
    return Sh
    #print("QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ")      


def collectstrayseperate( Sh,X,Y,ra,Pd,Pc,destination,f,mindisShep,rhoash,rs,ang,pimultiple):
    #print("In COLLECTSTRAYseparate ..........................")
    mode = 4
    XGCM = numpy.mean(X)
    YGCM = numpy.mean(Y)
    GCM = [XGCM,YGCM]
    D = []
    for i in range(len(X)):
        D.append(numpy.sqrt(numpy.square(X[i] - GCM[0]) + numpy.square(Y[i] - GCM[1])))
    #points = numpy.zeros(numpy.shape(Sh)[0] + 1)
    points1 = getcircularpoints( [XGCM,YGCM],max(D)+rs/2,numpy.shape(Sh)[0],ang,pimultiple)    
    points = numpy.zeros([numpy.shape(Sh)[0] + 1,2])
    #print("points befre resizing: " , points1 , " fffffffffffffffffffffffffffffff " , points )
    for i in range(numpy.shape(Sh)[0]):
        points[i,0] = points1[i,0]
        points[i,1] = points1[i,1]
    points[numpy.shape(Sh)[0],0] = points1[0,0]
    points[numpy.shape(Sh)[0],1] = points1[0,1]    
    #print("points after reszing: " , points , " gggggggggggggggggggggggggggggg")
    #points[numpy.shape(points)[0]][0] = points[0][0]
    #points[numpy.shape(points)[0]][1] = points[0][1]
    Sh1 = copy.deepcopy(Sh)
    order = []
    #midpoint = []
    for i in range(numpy.shape(points)[0] - 1):
        midpoint = [numpy.sum(points[i:i+2][0]),numpy.sum(points[i:i+2][1])]
        midpoint[0] = 0.5*midpoint[0]
        midpoint[1] = 0.5*midpoint[1]
        #midpoint[1] = 0.5*numpy.sum(points[i:i+2][1])
        dist = []
        #print("Sh1: " , Sh1 , " ffffffffffffffffffffffff midpt: " , midpoint )
        for j in range(numpy.shape(Sh1)[0]):
            dist.append(numpy.sqrt(numpy.square(Sh1[j][0] - midpoint[0]) + numpy.square(Sh1[j][1] - midpoint[1])))
        idx = dist.index(min(dist))
        order.append(idx)
        Sh1[idx] = [-1000,-1000]
    idx1 = []
    for i in range(numpy.shape(Sh)[0]):
        p = path.Path([(GCM[0],GCM[1]),(points[i,0],points[i,1]),(points[i+1,0],points[i+1,1])])
        Xt = numpy.transpose(X)
        Yt = numpy.transpose(Y) 
        pts = numpy.zeros([numpy.shape(Xt)[0],2])
        for j in range(numpy.shape(Xt)[0]):
            pts[j][0] = Xt[j]
            pts[j][0] = Yt[j]
        IN = p.contains_points(pts)
        idx = numpy.where(IN == True)
        Xin = []
        Yin = []
        for j in (idx[0]):
            Xin.append(X[j])
            Yin.append(Y[j])    
        D = []  
        idx = []
        for j in range(numpy.shape(Xin)[0]):
            D.append(numpy.sqrt(numpy.square(Xin[j] - GCM[0]) + numpy.square(Yin[j] - GCM[1]))) 
        #print("D: " , D , " DDDDDDDDDDDDDDDDDDDDDDDDD")
        if(len(D) != 0):
            idx.append(D.index(max(D)))
            last = [Xin[idx[0]],Yin[idx[0]]]
        #print("idx: " , idx , " idxidxidxidxidxidxidixdixi")
        if(len(idx) != 0):
            #print("S before singlecollectPos called :  " , Sh[order[i]] )
            #S = Sh[order[i]] 
            Sh[order[i]] = singlecollectingPosition(Sh[order[i]],last,XGCM,YGCM,Pd/4);
            #Sh[order[i]] = S 
            #print("AFTER FUNC CALLED: S = " ,  "  " , Sh[order[i]])
        else:
            idx1.append(i)
        
        if(idx1):
            dist = []
            for j in range(numpy.shape(Sh)[0]):
                dist.append(numpy.sqrt(numpy.square(Sh[j][0] - GCM[0]) + numpy.square(Sh[j][0] - GCM[1])))
            D = min(dist)
            #print("Sh: " , Sh[order[idx1[0]]], "  order: " , order , "  idx1: " , idx1 , "  movemovemovemovemovemove")
            for j in range(len(idx1)):
                Sh[order[idx1[j]]],s = moveSheperd(numpy.asarray(GCM).reshape(1,2),Sh[order[idx1[j]]].reshape(1,2),D,+1);

    NetX = numpy.zeros(numpy.shape(Sh)[0] + 1)
    NetY = numpy.zeros(numpy.shape(Sh)[0] + 1)
    for i in range(numpy.shape(Sh)[0] + 1):
        if(i < numpy.shape(Sh)[0]):
            NetX[i] = Sh[i][0]
            NetY[i] = Sh[i][1]
        else:
            NetX[i] = XGCM
            NetY[i] = YGCM

    PTS = numpy.asarray([NetX,NetY]).reshape(numpy.shape(NetX)[0],2)
    p = path.Path([[PTS[i][0],PTS[i][1]] for i in range(numpy.shape(NetX)[0])])
    Xt = numpy.transpose(X)
    Yt = numpy.transpose(Y)     
    pts = numpy.zeros([numpy.shape(Xt)[0],2])
    for j in range(numpy.shape(Xt)[0]):
        pts[j][0] = Xt[j]
        pts[j][1] = Yt[j]
    IN = p.contains_points(pts) 
    idx = numpy.where(IN == True)
    Xin = []
    Yin = []
    for i in idx[0]:
        Xin.append(X[i])
        Yin.append(Y[i])
    D = []
    idx = []
    for i in range(len(Xin)):
        D.append(numpy.sqrt(numpy.square(Xin[i] - GCM[0]) + numpy.square(Yin[i] - GCM[1]))) 
        if(D[i] > rs):
            idx.append(i)
    if(len(idx) == 0):
        mode = 1   
    return Sh,mode             
    #print("WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW")

def purepredator(Sh,X,Y,ra,Pd,Pc,destination,q,f,rhoash,rs,I):
    #print("In PUREPREDATOR ..................I = " , I , " " )
    pimultiple = (22.0/7.0)*q
    mode = 6
    Sh1 = copy.deepcopy(Sh)
    #currentmode = []
    #print("numpy.mean(X) " , numpy.mean(X) , "lenX: " , len(X))
    if(len(X) != 0):
        GCM = [numpy.mean(X),numpy.mean(Y)]
        rad = 60
        ang = numpy.arctan2(GCM[1]-destination[1],GCM[0]-destination[0])       
        #print("numpy.shape(Sh)[0] : " , numpy.shape(Sh)[0] , " GCM " , GCM)
        #print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
        points = getcircularpoints(GCM,rad/2,numpy.shape(Sh)[0]+1,ang,q) #if Sh is array
        #points = getcircularpoints([0,0] , 10, 3, 0, 1)
        #print("points " , points  , " " , points[0,:])
        #print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
        #if Sh is a list
        #points = getcircularpoints(GCM,rad/2,len(Sh[0])+1,ang,q)         
        if(len(I) == 0):
            I = []
            #midpoint = numpy.zeros([numpy.shape(Sh)])          
            #print("midpoint before for loop " , midpoint , " " )
            for i in range(numpy.shape(Sh)[0]):
                midpoint = [numpy.sum(points[i:i+2,0]),numpy.sum(points[i:i+2,1])]
                #print("midpoint before getting mul by 1/2: " , midpoint)
                if(numpy.shape(Sh)[0] > 1):
                    midpoint[0] = 0.5*midpoint[0]
                    midpoint[1] = 0.5*midpoint[1]   
                #print("midpoint " , midpoint)
                #midpoint = 0.5*[numpy.sum(points[i:i+1,0]),numpy.sum(points[i:i+1,1])]
                #midpoint[i][1] = 0.5*numpy.sum(points[i:i+1,1])                    
                #print("######################################################")
                D = []
                for j in range(numpy.shape(Sh)[0]):
                    D.append(numpy.sqrt(numpy.square(midpoint[0] - Sh1[j][0]) + numpy.square(midpoint[0] - Sh1[j][1])))
                idx = D.index(min(D))
                Sh1[idx,0] = -1000
                Sh1[idx,1] = -1000
                #print("D: " , D  , " idx: " , idx , "rororrorororrororrorroro") 
                I.append(idx)                
        #print("numpy.shape(Sh)[0] " , numpy.shape(Sh)[0] , " I= " , I)       
        for i in range(numpy.shape(Sh)[0]):
            p = path.Path([(GCM[0],GCM[1]),(points[i,0],points[i,1]),(points[i+1,0],points[i+1,1])])
            #print(numpy.shape(X) , " kkkkkkkkkkkkkkkk")
            Xt = numpy.transpose(X)
            Yt = numpy.transpose(Y)
            pts = numpy.zeros([numpy.shape(Xt)[0],2])
            #pts[:,0] = Xt
            #pts[:,1] = Yt
            #pts = [Xt,Yt]
            for j in range(numpy.shape(Xt)[0]):
                pts[j][0] = Xt[j]
                pts[j][1] = Yt[j]
            #print("just pts: " , pts)
            #print("this is the path: " , p)
            #print("X: " , pts[0][0] , " " ,pts[1][0] , " " , pts[0][1] , " " , pts[1][1] , " lolololooololololo")
            IN = p.contains_points(pts)
            #print("INNNNNNNNNNNNNNNNNN " , IN)
            idx = numpy.where(IN == True)
            #print(idx[0])
            #print("sssssssssssssssssssssssssssss")
            Xin = []
            Yin = []
            for j in (idx[0]):
                Xin.append(X[j])
                Yin.append(Y[j])
            DGCM = []
            for j in range(numpy.shape(Xin)[0]):
                DGCM.append(numpy.sqrt(numpy.square(Xin[j] - numpy.mean(Xin)) + numpy.square(Yin[j] - numpy.mean(Yin))))
            idx = []
            #print("DGCM" , DGCM , " " ,numpy.shape(Xin)[0])
            for j in range(numpy.shape(Xin)[0]):
                if(DGCM[j] > 0.5*f):
                    idx.append(j)   
            #MANGE or ASK ABOUT I:
            #print("I " , I , " i[o] " , I[0] , Sh)
            #print("Sh : " , Sh[I[1]] , " " , Sh[I[1],:])                    
            if(len(idx) == 0 and len(Xin) != 0):
                #print("in if, prior to calling singleherding : i= " , i )
                #print(" I[i]: " , I[i] , " Sh[I[i]] " , Sh[I[i]] , " ", type(Sh[I[i]]))
                #print("Xin " , Xin )
                #print(" Yin " , Yin)
                #print("Pc " , Pc , " destination " , destination)
                #changing Sh[I[i]] to S below to use in stepspertimestep
                S = Sh[I[i]]
                #Sh[I[i]] = singleherdingPosition( Sh[I[i]],Pc,Xin,Yin,destination)
                #print("Going in singleherdingPosition: " , Sh[I[i]])
                Sh[I[i]] = singleherdingPosition( Sh[I[i]].reshape(1,2),Pc,Xin,Yin,destination)
                S = Sh[I[i]]
                #print("New Pos " , Sh[I[i]])
                #Sh[I[i]],direct = stepspertimestepSheperd(S,Sh[I[i]],1)
                #currentmode[I[i]] = 1 
            else:
                if(len(Xin) != 0):
                    #print("Going in singlecollectingPositionWithin")
                    mindisShep = 6
                    #mode = 0
                    #print("Going in singlecollectingPositionwithin, " , Sh[I[i]])
                    Sh[I[i]] = singlecollectingPositionwithin( Sh[I[i]].reshape(1,2),0.25*Pd,Xin,Yin,[numpy.mean(Xin), numpy.mean(Yin)])     
                    #currentmode[I[i]] = 1 
                    #collectstrayseperate(Sh,X,Y,ra,Pd,Pc,destination,f,mindisShep,rhoash,rs,ang,pimultiple)
                    #S = movestrayseperate(Sh,X,Y,ra,Pd,Pc,destination,q,mindisShep,rhoash,rs,I,f)
                    #Sh,direct = stepspertimestep(S,Sh,1) #sheep speed = 1            
    #print("PurePredator Ending with Sh: " , Sh)            
    #print("PPPPPPPPPPPPPPPPPPPPPPPPPPPPP")
    return Sh , I

def sheperdMovement( Sh,X,Y,ra,Pd,Pc,destination,f,mindisShep,rhoash,alpha,ang,mode,nin,ds,rs,method,predmasterslave,decrad,q,I):
    #print("Sh: " , Sh , " YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY")
    XGCM = numpy.mean(X)
    YGCM = numpy.mean(Y)
    Sh1 = copy.deepcopy(Sh)
    GCM = [XGCM,YGCM]
    X1 = X
    Y1 = Y
    #currentmode = []
    i = 1
    if(mode!=5 and mode!=6):
        #print("NOT ENTERING HERE >>>>>>>>>>")
        I = []
    #Below function not converted   
    if(numpy.shape(Sh)[0] >= len(X)):
       herdingPosition(Sh,Pc,X,Y,destination)
    if(numpy.shape(Sh)[0] == 1):
        #print("IN singlesheperdMovement  ..................." , Sh)
        Sh = singlesheperdMovement(Sh,X,Y,ra,Pd,Pc,destination,f)        
        #print("AFTER EFFECTS of above func: "  ,Sh)
        #Sh = Sh.reshape(1,2)
        #Sh1 = Sh.reshape(1,2)
        #Sh1[0,0] = 1000
        #print("Let's see if Sh1 affects Sh: " , Sh , " AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
        #IT DOES CHANGE
    elif(mode == 4):
        #print("IN MODE = 4 " ) 
        ang = 0
        pimultiple = 2
        Sh , mode = collectstrayseperate( Sh,X,Y,GCM,Pd,Pc,destination,f,mindisShep,rhoash,rs,ang,pimultiple)
        #print("In ShpMovement after CollStraySeparate: " , Sh)
        if(predmasterslave==1):
            mode = 5
        elif(predmasterslave==3):
            mode = 6
        elif(predmasterslave==2 and mode == 1):
            mode = 1
    elif(mode == 6):
        Sh, I = purepredator(Sh,X,Y,ra,Pd,Pc,destination,q,f,rhoash,rs,I)
        #print("in SHPMOvement after PUREPREDATOR " , Sh)
    elif(mode == 5):
        Sh = movestrayseperate(Sh,X,Y,ra,Pd,Pc,destination,q,mindisShep,rhoash,rs,I);
    else:
        DGCM = []
        idx = []
        for i in range(len(X)):
            DGCM.append(numpy.sqrt(numpy.square(X[i] - XGCM) + numpy.square(Y[i] - YGCM)))
            if(DGCM[i] > f):
                idx.append(i)                                       
        if(len(idx) <= len(X) and mode == 1):
            #print("entering surround EFFect ")
            mode = 1
            alpha = 1
            ang = 0 
            Sh = surroundEffect(Sh,X,Y,ra,Pd,destination,q,mindisShep,rhoash,rs);
        elif(len(idx) > numpy.shape(Sh)[0] or mode == 2):
            mode = 2
            Sh = surroundEffectCollector(Sh,X1,Y1,ra,Pd,Pc,destination,i,mindisShep,rhoash,alpha,ang);
            D1 = []
            #print("D1 found with sh and sh1: ")
            #for i in range(1,10):
            for j in range(numpy.shape(Sh)[0]):
                D1.append(numpy.sqrt(numpy.square(Sh[j][0] - Sh1[j][0]) + numpy.square(Sh[j][1] - Sh1[j][1])))
            p = path.Path([[Sh[j][0],Sh[j][1]] for j in range(numpy.shape(Sh)[0])])
            Xt = numpy.transpose(X1)                
            Yt = numpy.transpose(Y1)
            pts = numpy.zeros([numpy.shape(Xt)[0],2])
            for j in range(len(Xt)):
                pts[j][0] = Xt[j]
                pts[j][1] = Yt[j]
            IN = p.contains_points(pts)
            IN1 = numpy.where(IN == True)
            #print("IN1: " , IN1 , " p : " , p , "sum(D1): ", sum(D1))   
            IN1 = len(IN1[0])
            if((sum(D1) < 10 or IN1 == len(X1)) and method == 1):
                #print("MODE CHANGES TO 4 ")
                mode = 4
            elif(method == 2):
                D = []
                for i in range(numpy.shape(Sh)[0]):
                    D.append(numpy.sqrt(numpy.square(Sh[i][0] - Sh1[i][0]) + numpy.square(Sh[i][1] - Sh1[i][1])))
                #print("In SheperdMovement D = " , D , " sum(D): " , sum(D))
                if(sum(D) < 10):
                    alpha=decrad*alpha
                    dist = []
                    for i in range(numpy.shape(Sh)[0]):
                        dist.append(numpy.sqrt(numpy.square(Sh[i][0] - XGCM) + numpy.square(Sh[i][1] - YGCM)))
                    D = max(dist)
                    if(D < rs):
                        mode = 1
                    ang = ang + 0.5
    #print("numpy.shape(Sh)[0]: " , numpy.shape(Sh) , "  " , Sh , " ", Sh1, " 111111111111111111111111111111")                 
    #if(numpy.shape(Sh)[0] == 1):
        #Sh,direction = stepspertimestepSheperd(Sh,Sh1,ds);                        
    #else:    
    #for i in range(numpy.shape(numpy.asarray(Sh))[0]):
    Sh,direction = stepspertimestepSheperd(Sh,Sh1,ds);   #ds = 1.5                      
    print("Sh after last for loop : " , Sh , " mode " , mode)    
    return Sh,mode,I

def singlesheperdMovement(S,X,Y,ra,Pd,Pc,destination,f):
    #print("singlesheperdMovement in Sh : " , S , "  " , numpy.shape(X) , " |||||||||||||||||")
    D = numpy.zeros(numpy.shape(X)[0])
    idx = []
    l = numpy.shape(X)
    for i in range(l[0]):
        D[i] = numpy.sqrt(numpy.square(X[i] - S[0][0]) + numpy.square(Y[i] - S[0][1]))
        #print(i, " D[i] " , D[i] )
        if(D[i] < ra):
            idx.append(i)
    #currentmode = 2 
    #print("idx should be empty: " , idx , " XXXXXXXXXXXXXXXXXXXXXXXXXXXx")
    if(len(idx) == 0):
        Xgcm = numpy.mean(X)
        Ygcm = numpy.mean(Y)
        Dgcm = []
        ind = []
        for i in range(l[0]):  # or len if X is a list and not an array     
            Dgcm.append(numpy.sqrt(numpy.square(X[i] - Xgcm) + numpy.square(Y[i] - Ygcm)))
            #print(i, " dgcm should have some: " , Dgcm[i])
            if(Dgcm[i] > f):
                ind.append(i)
        if(len(ind) == 0):
            #print("entering singleherdingPosition: " , S , " HHHHHHHHHHHHHHHHHHHHHHHHHH")
            S = singleherdingPosition(S,Pc,X,Y,destination)
            #print("After HERDING: aka new value " , S , " hhhhhhhhhhhhhhhhhhhhhh" )
            #currentmode = 1;            
        else:
            pos = Dgcm.index(max(Dgcm))
            last = [X[pos],Y[pos]]
            #print("entering singlecollectingPosition: " , S , " CCCCCCCCCCCCCCCCCCCCC")
            S = singlecollectingPosition(S,last,Xgcm,Ygcm,Pd)
            #print("entering singlecollectingPosition: " , S , " ccccccccccccccccc")
            #currentmode = 2             
    return S
            
def singleherdingPosition(S,Pc,X,Y,destination):
    #print("In HERDING " , S)
    #S2 = copy.deepcopy(S)
    #S2 = [0,0]
    l = numpy.shape(X)
    #Dgcm = numpy.zeros(numpy.shape(X))
    Dgcm = []
    for i in range(l[0]):  # or len if X is a list and not an array     
        Dgcm.append(numpy.sqrt(numpy.square(X[i] - destination[0]) + numpy.square(Y[i] - destination[1])))
    #idx = numpy.argmax(Dgcm,axis = 0)
    pos = Dgcm.index(max(Dgcm))
    idx = []
    for i in range(len(Dgcm)):
        if(Dgcm[i] == Dgcm[pos]):
            idx.append(i)
    #print("X and Y at POS: " , X[pos] , " " , Y[pos] , " " , S[0] , " ",S[0,0])
    if(len(idx) != 0):
        agent = numpy.zeros(2)
        agent[0] = X[pos]
        agent[1] = Y[pos]
        #agent = numpy.asarray([X[pos],Y[pos]])
        #print("agent " , agent)
        #S1 = S
        S[0,0] = agent[0]
        #S[0][1] = Y[pos]
        #print("Entering Move pt1: " , S , " pt2: " , destination , " " , agent)
        S,s2 = moveSheperd(agent,destination[:],Pc,-1) #s2 is useless
        #S,s2 = move(S,S,0,0)
        #S = S1
        #print("S after move : " , S ,"  ", )
        #print("hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh ") 
    #S,s2 = stepspertimestepSheperd(S,S2,1.5)   
    #numpy.asarray(S1)
    #print("S after limiting its steps: " , S , " ssssssssssssssssssssssssssss " )
    return S

def singlecollectingPosition( S,last,XGCM,YGCM,Pd):
    #print("last = : " , last , " XGCM: " , XGCM , " YGCM: " , YGCM , " numpy.asarray([XGCM,YGCM]) " , numpy.asarray([XGCM,YGCM]).reshape(2,1))
    #print("in singlecollecting: " , type(S) , "  " , S)
    G = numpy.zeros(2)
    G[0] = XGCM
    G[1] = YGCM
    #print("G; " , G , " GGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG")
    l,s = moveSheperd(numpy.asarray(last),G,Pd,-1)
    #print("l = " , l , " llllllllllllllllllllllllllllllllllllllllll")
    #l = l.reshape(1,2)
    S,s = moveSheperd(S,l,1,1)
    #S = S.reshape(1,2)
    #print("INSIDE Func singlecollecting: " , S , )
    #print("cccccccccccccccccccccccccccccccccccccc") 
    return S


# def stepspertimestepSheperd(currentpoints,initialpoints,steps): # to limit the step taken per unit time
# #steps: steps to be taken towards currentpoints from initialpoints
#     difference=currentpoints-initialpoints
#     norm=numpy.linalg.norm(difference,axis=0)
#     l=numpy.shape(difference)
#     print("in stepper  L : " , l , " " , norm, " llolololooloolooololo " , currentpoints)
#     direction=numpy.zeros((2,1))
#     #for i in range(1):
#     step=steps
#     if norm<steps:
#         step=norm
#     if norm==0:
#         unitvec=[0,0]
#     else:
#         unitvec=[difference[0]/norm,difference[1]/norm]
#         direction[0]=unitvec[0]
#         direction[1]=unitvec[1]
#     currentpoints[0] = currentpoints[0]+unitvec[0]*step
#     currentpoints[1] = currentpoints[1]+unitvec[1]*step
#     print("currentpoints " , currentpoints , " poppppopopopopo")
#     return currentpoints,direction

def stepspertimestepSheperd(currentpoints,initialpoints,steps): # to limit the step taken per unit time
#steps: steps to be taken towards currentpoints from initialpoints
    #print("currentpoints: " , currentpoints , " initials " , initialpoints,)
    difference=currentpoints-initialpoints    
    #norm=numpy.linalg.norm(difference[:],axis=0)
    norm = numpy.zeros(numpy.shape(currentpoints)[0])
    for i in range(numpy.shape(currentpoints)[0]):
        norm[i] = float(sqrt(pow(difference[i][0],2) + pow(difference[i][1],2)))
    l=numpy.shape(difference)
    #print("Difference " , difference, " DDDDDDDDDDDDDDDDDDDDDDDDDDD " , norm)
    #print("in stepspertime L: " , l ," norm: " , norm ) 
    direction=numpy.zeros((l[0],2))
    for i in range(l[0]):
        step=steps
        if norm[i]<steps:
            step=norm[i]
        if norm[i]==0:
            unitvec=[0,0]
        else:
            unitvec=[difference[i][0]/norm[i],difference[i][1]/norm[i]]
            direction[i,0]=unitvec[0]
            direction[i,1]=unitvec[1]
        #print("diff: " , difference ,"univec: " , unitvec , " norm: " , norm, " CEHK")    
        currentpoints[i,0]=initialpoints[i,0]+unitvec[0]*step
        currentpoints[i,1]=initialpoints[i,1]+unitvec[1]*step
    #print("NEW currentpts: " , currentpoints,)
    #print(" NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN")    
    return currentpoints,direction

def moveSheperd( point1,point2,intensity,direction ): 
    #print("point1 : " , point1, "  point2 " , point2, " PPPPPPPPPPPPPPPP")
    point1 = numpy.asarray(point1).reshape(1,2)
    point2 = numpy.asarray(point2).reshape(1,2)   
    if numpy.shape(point1)[0]!=0 and numpy.shape(point2)[0]!=0:
        #difference=numpy.asarray(point1).reshape(1,2)-numpy.asarray(point2).reshape(1,2)
        difference=point1-point2
        #print("point1 : " , point1, "  point2 " , point2, " pppppppppp")   
        norm = numpy.zeros(numpy.shape(point1)[0])
        for i in range(numpy.shape(point1)[0]):
            norm[i] = numpy.sqrt(numpy.square(difference[i][0]) + numpy.square(difference[i][1]))
        l=numpy.shape(point2)
        #print("numpy.shape(point1) " , numpy.shape(point1) , "  ",point1[0], " " , norm , )
        #if numpy.shape(point1)[0]>1:
        for i in range(numpy.shape(point2)[0]):
            #print(i , "  " , " iiiiiiiiiiiiiiiiiiiiiiiiiiiii")
            if norm[i]==0:
                unitvec=[0,0]
            else:
                unitvec=[difference[i][0]/norm[i],difference[i][1]/norm[i]]
            #print("unitvec " , unitvec , " " , )    
            point1[i][0]=point1[i][0]-direction*(intensity)*unitvec[0]
            point2[i][0]=point2[i][0]+direction*(intensity)*unitvec[0]
            point1[i][1]=point1[i][1]-direction*(intensity)*unitvec[1]
            point2[i][1]=point2[i][1]+direction*(intensity)*unitvec[1]
        # else:
        #     #print("here fear")
        #     if norm==0:
        #         unitvec=[0,0]
        #     else:
        #         unitvec=[difference[0]/norm,difference[1]/norm]
        #     point1[0][0]=point1[0][0]-direction*(intensity)*unitvec[0]
        #     point2[0][0]=point2[0][0]+direction*(intensity)*unitvec[0]
        #     point1[0][1]=point1[0][1]-direction*(intensity)*unitvec[1]
        #     point2[0][1]=point2[0][1]+direction*(intensity)*unitvec[1]
    #print("changed points: " , point1 , "  " , point2 , " "  )        
    return point1,point2

def move( point1,point2,intensity,direction ):    
    if numpy.shape(point1)[0]!=0 and numpy.shape(point2)[0]!=0:
        difference=numpy.asarray(point1)-numpy.asarray(point2)
        #difference=point1-point2
        norm=numpy.linalg.norm(difference,axis=0)
        l=numpy.shape(point2)
        if numpy.ndim(point1)>1:
            for i in range(l[1]):
                if norm[i]==0:
                    unitvec=[0,0]
                else:
                    unitvec=[difference[0][i]/norm[i],difference[1][i]/norm[i]]
                point1[0,i]=point1[0,i]-direction*(intensity)*unitvec[0]
                point2[0,i]=point2[0,i]+direction*(intensity)*unitvec[0]
                point1[1,i]=point1[1,i]-direction*(intensity)*unitvec[1]
                point2[1,i]=point2[1,i]+direction*(intensity)*unitvec[1]
        else:
            if norm==0:
                unitvec=[0,0]
            else:
                unitvec=[difference[0]/norm,difference[1]/norm]
            point1[0]=point1[0]-direction*(intensity)*unitvec[0]
            point2[0]=point2[0]+direction*(intensity)*unitvec[0]
            point1[1]=point1[1]-direction*(intensity)*unitvec[1]
            point2[1]=point2[1]+direction*(intensity)*unitvec[1]
    return point1,point2

def inertialEffect( initpoints,direction,h ):
    inertiapoints=numpy.zeros((2,numpy.size(direction)/2))
    # print direction
    if numpy.size(direction)!=0:
        inertiapoints=initpoints+h*direction
    return inertiapoints

def inertialEffect( initpoints,direction,h ):
    inertiapoints=initpoints
    # print direction
    if numpy.size(direction)!=0:
        for i in range(len(initpoints[0])):
            inertiapoints[0][i] = initpoints[0][i] + h*direction[0][i]
            inertiapoints[1][i] = initpoints[1][i] + h*direction[1][i]
    return inertiapoints    

def clusteringEffect(initpoints,n,c,rs,Shepherd): #the clustering of the sheep
    Shepherdsize=numpy.shape(Shepherd)
    clusterpoints=initpoints
    for j in range(Shepherdsize[0]):
        if numpy.size(Shepherd)==2:
            S=Shepherd
        else:
            S=Shepherd[j]
        D=numpy.linalg.norm(initpoints.T-S,axis=1)
        closeagents=numpy.where((D<(rs)))  # Check why this is happening
        for i in range(numpy.size(closeagents)):
            agent1=initpoints[:,closeagents[0][i]] 
            D=numpy.linalg.norm(initpoints.T-agent1,axis=1)
            sortedD=numpy.argsort(D)
            sortedDindex=numpy.where((sortedD<=n-1))
            agent2=(initpoints[:,sortedDindex[0]]).mean(1)
            clusterpoints[:,closeagents[0][i]],agent2=move(agent1,agent2,c,1)
    return clusterpoints

def angularnoiseEffect( initpoints,e,p ):   # The noise function
    X=initpoints[0,:]
    Y=initpoints[1,:]
    l=numpy.shape(X)
    errorpoints=initpoints
    r=numpy.random.rand(1,l[0])
    for i in range(l[0]):
        if p>r[0][i]:
            X[i]=X[i]+e*numpy.random.randint(-1,1)
            Y[i]=Y[i]+e*numpy.random.randint(-1,1)
    errorpoints[0]=X
    errorpoints[1]=Y
    return errorpoints

def setGuidedMode1():
    rospy.wait_for_service('rover1/mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy('rover1/mavros/set_mode', mavros_msgs.srv.SetMode)
        isModeChanged = flightModeService(custom_mode='GUIDED') #return true or false
    except rospy.ServiceException, e:
        print "GUIDED Mode could not be set."%e

def setGuidedMode2():
    rospy.wait_for_service('rover2/mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy('rover2/mavros/set_mode', mavros_msgs.srv.SetMode)
        isModeChanged = flightModeService(custom_mode='GUIDED') #return true or false
    except rospy.ServiceException, e:
        print "GUIDED Mode could not be set."%e


def setArm1():
    rospy.wait_for_service('rover1/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('rover1/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(True)
    except rospy.ServiceException, e:
        print "Arming failed"%e

def setArm2():
    rospy.wait_for_service('rover2/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('rover2/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(True)
    except rospy.ServiceException, e:
        print "Arming failed"%e


def selfRepellingEffect( initialpoints,ra,rhoa):
    l=numpy.shape(initialpoints)
    repelledpoints=initialpoints
    for i in range(l[1]):
        D =numpy.linalg.norm(initialpoints.T-initialpoints[:,i],axis=1)
        closeagents=numpy.where((D>0) & (D<ra)) 
        if numpy.size(closeagents)!=0:
            agent1=numpy.reshape(numpy.repeat(initialpoints[:,i],numpy.size(closeagents)),(2,numpy.size(closeagents)))
            agent1,agent2=move(agent1,initialpoints[:,closeagents[0]],rhoa,-1)
            repelledpoints[:,i] = agent1.mean(1)
    return repelledpoints

def sheperdingEffect(initpoints,Shepherd,rs,rhos): #the fear of the predator
    Shepherdsize=numpy.shape(Shepherd)
    shepherdingpoints=initpoints
    for j in range(Shepherdsize[0]):
        if numpy.size(Shepherd)==2:
            S=Shepherd
        else:
            S=Shepherd[j]
        D=numpy.linalg.norm(shepherdingpoints.T-S,axis=1)
        closeagents=numpy.where((D<(rs)))                                    
        S=numpy.reshape(numpy.repeat(S,numpy.size(closeagents)),(2,numpy.size(closeagents)))
        N=numpy.shape(shepherdingpoints)
        NS,shepherdingpoints[:,closeagents[0]]=move(S,shepherdingpoints[:,closeagents[0]],rhos,-1)
    return shepherdingpoints

# def sheepmovement(initpoints,e,p,ra,rhoa,dps,Shepherd,direction,n,c,rs,rhos,h):
#     clusterpoints=clusteringEffect(initpoints,n,c,rs,Shepherd)
#     errorpoints=angularnoiseEffect(initpoints,e,p) # The effect of noise    <----working
#     repelledpoints=selfRepellingEffect(initpoints,ra,rhoa)         #<--working
#     fearpoints=sheperdingEffect(initpoints,Shepherd,rs,rhos)
#     inertiapoints=inertialEffect(initpoints[:],direction[:],h)
#     allpoints=repelledpoints+fearpoints+errorpoints+clusterpoints+inertiapoints
#     initpoints,direction=stepspertimestep(allpoints,initpoints,dps)   #<--working
    #return initpoints

def sheepmovements(initpoints,n,c,rs,Shepherd,ra,rhoa,rhos,unitvec,h,e,dps,mode,destination,p):
    copypoints = copy.deepcopy(initpoints)
    if(n != 1):
        initpoints = clusteringEffect(initpoints,n,c,rs,Shepherd)
    idx1 = []
    X = initpoints[0,:]
    Y = initpoints[1,:]
    for i in range(len(X)):
        dist = sqrt(pow(initpoints[0][i] - destination[0],2) + pow(initpoints[1][i] - destination[1],2))
        if(dist >= 10):
            idx1.append(i)
    initpoints1 = sheperdingEffect(initpoints,Shepherd,rs,rhos)
    for i in range(len(initpoints[0])):
        if(i not in idx1):
            initpoints1[0][i] = initpoints[0][i]
            initpoints1[1][i] = initpoints[1][i]        

    #print("initpoints " , initpoints )
    #print("inipoints1 " , initpoints1)        
    initpoints = abs(initpoints - initpoints1)
    initpoints[0] = initpoints[0] + initpoints[1]
    idx = []
    for i in range(len(initpoints[0])):
        if(initpoints[0][i] != 0):
            idx.append(i)
    #idx = idx.intersect(idx1)
    idx.sort()
    idx1.sort()
    #print("idx: " , idx , " idx1: " , idx1 , " " , len(idx) , " " , len(idx1))
    j = 0; k = 0;
    temp =[]
    while(j < len(idx) and k < len(idx1)):
        if(idx[j] == idx1[k]):
            temp.append(j)
            j = j+1; k=k+1;
        elif(idx[j] > idx1[k]):
            k = k + 1
        else:
            j = j + 1
    #print("before intertial initpoints")
    #print(initpoints)        
    if(len(idx)!=0 and len(unitvec)!= 0):
        initpoints1[:,idx] = inertialEffect(initpoints1[:,idx],unitvec[:,idx],h)
    initpoints = initpoints1
    #print("AFTER IT")
    #print(initpoints)
    #r = numpy.random.randint(len(X))
    #for i in range()    
    initpoints = angularnoiseEffect(initpoints,e,p)

    initpoints[:,idx1] = selfRepellingEffect(initpoints[:,idx1],ra,rhos)
    idx = []
    for i in range(len(initpoints[0])):
        dist = sqrt(pow(initpoints[0][i] - destination[0],2) + pow(initpoints[1][i] - destination[1],2))
        if(dist >= 10):
            idx.append(i)
    if(len(idx) != 0):
        initpoints[:,idx],unitvec1 = stepspertimestep(initpoints[:,idx],copypoints[:,idx],dps)
        unitvec[:,idx] = unitvec1[:,:]
    #print("idx: " , idx)
    #print("idx1: " , idx1)
    if(len(idx) != 0):
        initpoints1 = initpoints[:,idx]
        R = numpy.random.rand(2,len(idx))
        initpoints1 = initpoints1 + R
        idx1 = []
        for i in range(len(initpoints1[0])):
            dist = sqrt(pow(initpoints1[0][i] - destination[0],2) + pow(initpoints1[1][i] - destination[1],2))
            if(dist > 10):
                idx1.append(i)
        if(len(idx1)!= 0 ):
            for i in idx1:
                initpoints1[:,i] = initpoints[:,idx[i]]
        initpoints[:,idx] = initpoints1            
    else:
        initpoints,unitvec = stepspertimestep(initpoints,copypoints,dps)    


def stepspertimestep(currentpoints,initialpoints,steps): # to limit the step taken per unit time
#steps: steps to be taken towards currentpoints from initialpoints
    difference=currentpoints-initialpoints
    norm=numpy.linalg.norm(difference,axis=0)
    l=numpy.shape(difference)
    #print("in stepper  L : " , l , " llolololooloolooololo " ) 
    direction=numpy.zeros((2,l[1]))
    for i in range(l[1]):
        step=steps
        if norm[i]<steps:
            step=norm[i]
        if norm[i]==0:
            unitvec=[0,0]
        else:
            unitvec=[difference[0][i]/norm[i],difference[1][i]/norm[i]]
            direction[0,i]=unitvec[0]
            direction[1,i]=unitvec[1]
        currentpoints[0][i]=initialpoints[0][i]+unitvec[0]*step
        currentpoints[1][i]=initialpoints[1][i]+unitvec[1]*step
    return currentpoints,direction

def get_position_local1(msg):    
    global local_lat1 , local_lon1 , flag31 , ini_local_lat1 , ini_local_lon1;
    local_lat1 = msg.pose.pose.position.x;
    local_lon1 = msg.pose.pose.position.y;

    if(flag31 == 0):
        ini_local_lat1 = local_lat1;
        ini_local_lon1 = local_lon1;
        flag31 = 1;

    local_lat1 = local_lat1 - ini_local_lat1
    local_lon1 = local_lon1 - ini_local_lon1    


def get_position_local2(msg):    
    global local_lat2 , local_lon2 , flag32 , ini_local_lat2 , ini_local_lon2;
    local_lat2 = msg.pose.pose.position.x;
    local_lon2 = msg.pose.pose.position.y;

    if(flag32 == 0):
        ini_local_lat2 = local_lat2;
        ini_local_lon2 = local_lon2;
        flag32 = 1;

    local_lat2 = local_lat2 - ini_local_lat2
    local_lon2 = local_lon2 - ini_local_lon2    



def get_compass1(msg):
    global heading_angle1 , flag41 , ini_angle1
    heading_angle1 = msg.data    
    if(flag41 == 0):
        #ini_angle = heading_angle
        ini_angle1 = 90
        flag41 = 1



def get_compass2(msg):
    global heading_angle2 , flag42 , ini_angle2
    heading_angle2 = msg.data    
    if(flag42 == 0):
        #ini_angle = heading_angle
        ini_angle2 = 90
        flag42 = 1

def main():
    rospy.init_node('sheep_sheperd', anonymous=True)

    pub_marker = rospy.Publisher('rover2_sheep_sheperd', MarkerArray , queue_size=1)
    #WHY ARE WE USING A PUBLISHER TO PUBLISH THE POS. Shoudn;t we use a SUBSRIBER
    #rmove = rospy.Publisher('/mavros/setpoint_raw/global', GlobalPositionTarget, queue_size = 10)
    #rmove_sub = rospy.Subscriber('/mavros/setpoint_raw/global' , GlobalPositionTarget, get_position)
    #rmove_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix , get_position)
    rmove_sub_local1 = rospy.Subscriber('/rover1/mavros/global_position/local', Odometry, get_position_local1)
    rvel1 = rospy.Publisher('/rover1/mavros/setpoint_velocity/cmd_vel' , TwistStamped ,queue_size = 1)
    rcompass1 = rospy.Subscriber('/rover1/mavros/global_position/compass_hdg', Float64 , get_compass1)

    rmove_sub_local2 = rospy.Subscriber('/rover2/mavros/global_position/local', Odometry, get_position_local2)
    #rvel2 = rospy.Publisher('/rover2/mavros/setpoint_velocity/cmd_vel' , TwistStamped ,queue_size = 1)
    rcompass2 = rospy.Subscriber('/rover2/mavros/global_position/compass_hdg', Float64 , get_compass2)    

    e=0.3 # strength of error movement
    p=0.05 # probability of an error
    Xstartpos=20.0 #swarm initial position
    Ystartpos=0.0
    h = 0.5 #strength of inertial force
    N = 100 # number of agents
    N1 = copy.deepcopy(N)
    ra=2.0  # distance for self repulsion
    mindisShep = 6
    n=numpy.ceil(0.9*N1)
    Pd = ra*log(N)
    Pc = ra
    threshold = 10.0
    dps=1 # speed of agents
    
    f = N^(1/2)
    rhoa=2.0# strength of self repulsion 
    rhos=1.0#strength of predatory repulsion 
    q = 1.8
    #I = []
    rhoash = 1.0
    #Shepherd=[-0,100]
    
    #S1 = [0,120]
    S2 = [60,90]
    S3 = [-60,90]
    S4 = [30,100]
    nos = 2 #Number of sheperd
    #Sh = numpy.asarray([S1,S2,S3,S4]).reshape(nos,2)
    #Sh = numpy.asarray([S1,S2,S3]).reshape(nos,2) #Sheperd Array  
    #S1 = [(temp_x - 111320)/100.0,(temp_y - 110567)/100.0]
    S1 = [0,10]
    S2 = [0,-10]
    Sh = numpy.asarray([S1,S2]).reshape(nos,2)
    #Sh = numpy.asarray([S2,S3]).reshape(nos,2)
    rs = 15  #radius of influence
    c=1.05 #strength of clustering
    destination=[50,0] #destination
    x=Xstartpos+numpy.random.rand(1,N) #swarm initialization
    y=Ystartpos+numpy.random.rand(1,N) #gives random values from [0,1)
    initpoints=numpy.concatenate((x,y),axis=0)
    direction=numpy.zeros((2,N))
    unitvec=numpy.zeros((2,N))

    alpha = 1
    ang = 0
    mode = 2
    nin = 0 
    method = 1 
    predmasterslave = 3 
    ds = 1.5
    decrad = 0.9
    I = []

    #N = 100    

    rate = rospy.Rate(10) # 10hz

    #rov_vel = Twist()
    #rov = PositionTarget()    
    rov1 = TwistStamped()
    rov2 = TwistStamped()
    M = MarkerArray()    
    #x = []
    #y = []


    shape = Marker.CUBE;
    for i in range(N+nos+1): #to include shepherd too
        marker = Marker()
        # print("i " , i)
        marker.header.frame_id = "/rover2_sheep_sheperd";
        marker.header.stamp = rospy.get_rostime();
        

        marker.ns = "mul_shp";
        
        marker.id = i;
        
        marker.type = shape;
        
        marker.action = Marker.ADD;

        if(i < N):
            marker.pose.position.x = x[0][i];            

            marker.pose.position.y = y[0][i];
        
        marker.pose.position.z = 0;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
   
        marker.lifetime = rospy.Duration();

        if(i >= N and i<N+nos):
            marker.ns = "sheperd"
            marker.pose.position.x = Sh[i-N][0];
            marker.pose.position.y = Sh[i-N][1];
            if(i == N):
                marker.color.r = 1.0;
                marker.color.g = 0.0;
            if(i == N+1):
                marker.color.g = 1.0;
                marker.color.b = 1.0;    

        if(i >= N+nos):
            marker.ns = "Point"
            marker.pose.position.x = destination[0]
            marker.pose.position.y = destination[1]
            marker.type = Marker.CYLINDER
            marker.color.b = 1.0;
            marker.color.r = 0.0;
            marker.scale.x = 20.0;
            marker.scale.y = 20.0;
            marker.scale.z = 0.5;   
        #x.append(marker.pose.position.x)
        #y.append(marker.pose.position.y)


        M.markers.append(marker)
        # print("id = " , M.markers[i].id , "ini pos " , x[0][i%N] , " , " , y[0][i%N])

        m=numpy.size(initpoints[0])
        # print("this will be the new N " , m)

    count = 0;    
    flag = 0;

    arr = numpy.zeros((1,10))
    outthreshold = []
    for i in range(N):
        outthreshold.append(i)

    p = 0
    count2 = 0
    flag5 = 0
    global ini_angle
    #global lat, lon , ini_lat , ini_lon 

    t1 = rospy.get_rostime()
    #print("aligned with 0 aka N")    
    angle_covered = 0.0

    # while(1):
    #     print(heading_angle)
    #     rov.twist.linear.x = 0.07
    #     rov.twist.angular.z = 1.0
    #     rvel.publish(rov)
    #     t1 = rospy.get_time()
        # while(rospy.get_time() < t1 + 0.2):
        #     print('')



    while(heading_angle1 <= 80 or heading_angle1 >= 100):  
        print('inside first while loop')          
        rov1.twist.linear.x = 0.0
        #rov2.twist.linear.x = 10.0
        rov1.twist.angular.z = 1.0
        rvel1.publish(rov1)
        

    # while(heading_angle2 <= 80 or heading_angle2 >= 100):  
    #     print('inside second while loop')          
    #     rov2.twist.linear.x = 0.00
    #     rov2.twist.angular.z = 1.0
    #     rvel2.publish(rov2)

        #rospy.Duration(0.2)
        # print("FFFFFFFFFFFFFFFFFFFFFFFFFFFFF") 
    # while(1):  
    #     print('run forward')          
    #     rov1.twist.linear.x = 0.5
    #     rov2.twist.linear.x = 10.0
    #     rov1.twist.angular.z = 0.0
    #     rov2.twist.angular.z = 0.0
    #     #rov1.twist.angular.z = 1.0
    #     rvel1.publish(rov1)
    #     rvel2.publish(rov2)


    while not rospy.is_shutdown():    
        print('in MARKER LOOP')
        #FROM SITL WITH PATIENCE :P
        scale = 5
        Sh[0,0] = local_lat1*5 
        Sh[0,1] = local_lon1*5
        Sh[1,0] = local_lat2*5 
        Sh[1,1] = local_lon2*5
       

        pub_marker.publish(M)        
        N=numpy.size(initpoints[0])
        Pd=ra*numpy.log(N1)
        Pc=ra
        f=ra*numpy.sqrt(N1)
        #COPY OF SHPERD BEFORE IT CHANGES::
        temp_sh = copy.deepcopy(Sh)

        sheepmovements(initpoints,n,c,rs,Sh,ra,rhoa,rhos,unitvec,h,e,dps,mode,destination,p)
        #sheepmovement(initpoints,e,p,ra,rhoa,dps,Shepherd,direction,n,c,rs,rhos,h)
        #sheperdmovement(Sh,initpoints[0],initpoints[1],ra,Pd,Pc,destination,f,mindisShep,rhoash,rs,I)
        Sh,mode,I = sheperdMovement(Sh,initpoints[0][outthreshold],initpoints[1][outthreshold],ra,Pd,Pc,destination,f,mindisShep,rhoash,alpha,ang,mode,nin,ds,rs,method,predmasterslave,decrad,q,I)
        
        Sh = Sh/scale
        # LIMIT ANGLE HERE

        # dx = (Sh[0,0] - temp_sh[0,0])
        # dy = (Sh[0,1] - temp_sh[0,1])
        # V =  sqrt(pow(dx,2) + pow(dy,2))
        # theta = numpy.arctan2(dy,dx)
        # if numpy.absolute(theta)>(limiting_angle):
        #     if theta<0:
        #         Sh[0,0]=temp_sh[0,0]+V*temp_sh[0,0]*numpy.cos(-(limiting_angle))
        #     else:
        #         Sh[0,1]=temp_sh[0,1]+V*temp_sh[0,1]*numpy.sin(-(limiting_angle))

        #rospy.Duration(0.05)
        #plt.pause(0.05) --- similar to ros::Duration(0.05) or rospy.Duration(0.05)
        #plt.clf() --- clears the plot screen blah blah
        #print(initpoints)
        # print("yolo mode " , mode , " " , )
        # print("sheperd:" , Sh)
        #print("I : " , I)
        GCM = [numpy.mean(initpoints[0][:]),numpy.mean(initpoints[1][:])]
        Dcheck = sqrt(pow(Sh[0][0] - GCM[0],2) + pow(Sh[0][1] - GCM[1],2))
        #for i in range(nos):
            #Dcheck.append(sqrt(pow(Sh[i][0] - numpy.mean(initpoints[0][:]),2) + pow(Sh[i][1] - numpy.mean(initpoints[1][:],2))))
        #print("oloy " ,  )
        #print("Distance from GCM: " , Dcheck)
        for i in range(N):
            M.markers[i].pose.position.x = initpoints[0][i] 
            M.markers[i].pose.position.y = initpoints[1][i] 
        for i in range(nos):    
            M.markers[N1+i].pose.position.x = temp_sh[i][0];
            M.markers[N1+i].pose.position.y = temp_sh[i][1];
            #x[0][i] = M.markers[i].pose.position.x
            #y[0][i] = M.markers[i].pose.position.y
            #print(i , " x " , M.markers[i].pose.position.x , " id " , M.markers[i].id)
        temp_initpoints = copy.deepcopy(initpoints)    
        
        dist = []    
        flag = 0
        #print("Destination: " ,destination)
        for i in range(N):
            dist.append(sqrt(pow(initpoints[0][i] - destination[0],2) + pow(initpoints[1][i] - destination[1],2)))
            if(sqrt(pow(initpoints[0][i] - destination[0],2) + pow(initpoints[1][i] - destination[1],2)) <= threshold):
                flag = flag + 1

        outthreshold = []        
        for i in range(N):
            dist.append(sqrt(pow(initpoints[0][i] - destination[0],2) + pow(initpoints[1][i] - destination[1],2)))
            if(sqrt(pow(initpoints[0][i] - destination[0],2) + pow(initpoints[1][i] - destination[1],2)) > threshold):
                #flag = flag + 1
                outthreshold.append(i)
                #print("Deleting " , i  ," " )
        #print(dist)
        #print("sheeps pos " , initpoints[])
        #if(flag >= 1):
            #for i in cordi:
                #print(i , " " , initpoints[0][i] , " " , initpoints[1][i])
            #break        
                #temp_initpoints = numpy.delete(temp_initpoints,i,1)
                
        # for i in reversed(cordi):
        #     initpoints = numpy.delete(initpoints,i,1)
        #     direction = numpy.delete(direction,i,1)
        #     print("dist: " , dist[i] , )
        #print("DELETING " , numpy.shape(initpoints))
        #initpoints = temp_initpoints   

        #SITL STARTS     
        #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        

        dx1 = (Sh[0,0] - temp_sh[0,0]/scale)
        dy1 = (Sh[0,1] - temp_sh[0,1]/scale)
        V1 =  sqrt(pow(dx1,2) + pow(dy1,2))*5
        theta1 = numpy.arctan2(dy1,dx1)        

        dx2 = (Sh[1,0] - temp_sh[1,0]/scale)
        dy2 = (Sh[1,1] - temp_sh[1,1]/scale)
        V2 =  sqrt(pow(dx2,2) + pow(dy2,2))*5
        theta2 = numpy.arctan2(dy2,dx2)        

        #rov.position.x = ini_lat + dx
        #rov.position.y = ini_lon + dy
        print("V1 " ,V1 ," V2 " , V2)
        rov1.twist.linear.x = 10        
        rov1.twist.angular.z = 0.00;
        thetadeg1 = theta1*(180)/(22.0/7)

        # rov2.twist.linear.x = 10        
        # rov2.twist.angular.z = 0.00;
        thetadeg2 = theta2*(180)/(22.0/7)
        
        if(thetadeg1 < 0):
            thetadeg1 = 360 + thetadeg1;

        if(thetadeg2 < 0):
            thetadeg2 = 360 + thetadeg2;

        # if(heading_angle > ini_angle and heading_angle < 360):
        #     angle_covered = heading_angle - ini_angle;
        # elif(heading_angle > 0):
        #     angle_covered = heading_angle    

        # while(heading_angle1 < beta1 - 15 or heading_angle1 > beta1 + 15):
                        
        #     rov1.twist.linear.x = 0.1                                         #0.5 Changed here
            
        #     if(heading_angle1 <= beta1):
        #         if(beta1 - heading_angle1 <= 180):
        #             yaw_to_publish1 = -1.0
        #         else:
        #             yaw_to_publish1 = 1.0
        #     else:
        #         if(heading_angle1 - beta1 <= 180):
        #             yaw_to_publish1 = 1.0
        #         else:
        #             yaw_to_publish1 = -1.0                    
        #     speed_of_angular_change1=5
        #     rov1.twist.angular.z = yaw_to_publish1
        #     print('beta1 ' , beta1 , " " , heading_angle1 , " " , yaw_to_publish1)                
        #     rvel1.publish(rov1)
        #     #rospy.Duration(0.1)
        
        #AS POSITIVE IS TOWARDS UPWARDS
        beta1 = (ini_angle1 - thetadeg1)
        if(beta1 > 360):
            beta1 = beta1%360
        elif(beta1 < 0):
            beta1 = beta1%360 #SWEET CONVERSION : -40%360 = 320 :p  

        beta2 = (ini_angle2 - thetadeg2)
        if(beta2 > 360):
            beta2 = beta2%360
        elif(beta2 < 0):
            beta2 = beta2%360 #SWEET CONVERSION : -40%360 = 320 :p    
            
        while(heading_angle1 < beta1 - 15 or heading_angle1 > beta1 + 15):
                        
            rov1.twist.linear.x = 0.0                                         #0.5 Changed here
            
            if(heading_angle1 <= beta1):
                if(beta1 - heading_angle1 <= 180):
                    yaw_to_publish1 = -1.0
                else:
                    yaw_to_publish1 = 1.0
            else:
                if(heading_angle1 - beta1 <= 180):
                    yaw_to_publish1 = 1.0
                else:
                    yaw_to_publish1 = -1.0                    
            
            rov1.twist.angular.z = yaw_to_publish1
            print('beta1 ' , beta1 , " " , heading_angle1 , " " , yaw_to_publish1)                
            rvel1.publish(rov1)
            # time1 = rospy.get_rostime()
            # while(rospy.get_rostime() < time1 + rospy.Duration(0.5)):
            #     print('in time loop')        

            #rospy.Duration(0.1)

        # while(heading_angle2 < beta2 - 15 or heading_angle2 > beta2 + 15 or heading_angle1 < beta1 - 15 or heading_angle1 > beta1 + 15):
            
        #     if(heading_angle2 < beta2 - 15 or heading_angle2 > beta2 + 15):            
        #         rov2.twist.linear.x = 0.0                                         #0.5 Changed here
                
        #         if(heading_angle2 <= beta2):
        #             if(beta2 - heading_angle2 <= 180):
        #                 yaw_to_publish2 = -1.0
        #             else:
        #                 yaw_to_publish2 = 1.0
        #         else:
        #             if(heading_angle2 - beta2 <= 180):
        #                 yaw_to_publish2 = 1.0
        #             else:
        #                 yaw_to_publish2 = -1.0                    
        #         speed_of_angular_change2 = 5
        #         rov2.twist.angular.z = yaw_to_publish2
        #         print('beta2 ' , beta2 , " " , heading_angle2 , " " , yaw_to_publish2)              
        #         rvel2.publish(rov2)

        #     if(heading_angle1 < beta1 - 15 or heading_angle1 > beta1 + 15):
        #         rov1.twist.linear.x = 0.0                                         #0.5 Changed here
                
        #         if(heading_angle1 <= beta1):
        #             if(beta1 - heading_angle1 <= 180):
        #                 yaw_to_publish1 = -1.0
        #             else:
        #                 yaw_to_publish1 = 1.0
        #         else:
        #             if(heading_angle1 - beta1 <= 180):
        #                 yaw_to_publish1 = 1.0
        #             else:
        #                 yaw_to_publish1 = -1.0                    
        #         speed_of_angular_change1=5
        #         rov1.twist.angular.z = yaw_to_publish1
        #         print('beta1 ' , beta1 , " " , heading_angle1 , " " , yaw_to_publish1)              
        #         rvel1.publish(rov1)
                    

        rov1.twist.linear.x = 10.0
        rov1.twist.angular.z = 0.0   
            

        #rov2.twist.linear.x = 10
        #rov2.twist.angular.z = 0.0   
        
        #rvel2.publish(rov2)    
        rvel1.publish(rov1)
        time1 = rospy.get_rostime()
        while(rospy.get_rostime() < time1 + rospy.Duration(1.0)):
            print('in time loop')
            rov1.twist.linear.x = 10.0        
            rvel1.publish(rov1)
        #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if(flag >= n):
            break        
        count = count + 1;
        if(count > 1000):            
            break    
        rate.sleep()




if __name__ == '__main__':
    #x = []
    #y = []
    #lat = None
    #lon = None
    #ini_lat = None
    #ini_lon = None
    #flag2 = 0
    try:
        #setGuidedMode1()
        #setGuidedMode2()
        #setArm1()
        #setArm2()
        main()
    except rospy.ROSInterruptException:
        pass
