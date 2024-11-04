from math import ceil, floor, sqrt, cos, sin, tan, atan2, asin
import copy
import numpy

from math import pi as PI

def distance(pose1, pose2):
    """ compute Euclidean distance for 2D """
    return sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)+0.001

def RVO_update(X, V_des, V_current, ws_model, pairs):
    """ compute best velocity given the desired velocity, current velocity and workspace model"""
    ROB_RAD = ws_model['robot_radius']+0.1
    V_opt = list(V_current)
    for i in range(len(X)):
        vA = V_current[i]
        pA = X[i]
        RVO_BA_all = []
        
        # Find this agent's pair
        pair_index = next((pair[1] for pair in pairs if pair[0] == i), next((pair[0] for pair in pairs if pair[1] == i), None))
        
        for j in range(len(X)):
            if i!=j:
                vB = V_current[j]
                pB = X[j]
                
                # If j is the pair of i, use a larger radius
                if j == pair_index:
                    dist_BA = distance(pA, pB)
                    theta_BA = atan2(pB[1]-pA[1], pB[0]-pA[0])
                    if dist_BA < 2*ROB_RAD:
                        dist_BA = 2*ROB_RAD
                    theta_BAort = asin(2*ROB_RAD/dist_BA)
                    theta_ort_left = theta_BA+theta_BAort
                    bound_left = [cos(theta_ort_left), sin(theta_ort_left)]
                    theta_ort_right = theta_BA-theta_BAort
                    bound_right = [cos(theta_ort_right), sin(theta_ort_right)]
                    RVO_BA = [(pA[0]+pB[0])/2+(vB[0]+vA[0])/2, (pA[1]+pB[1])/2+(vB[1]+vA[1])/2], bound_left, bound_right, dist_BA, 2*ROB_RAD
                else:
                    # use RVO
                    transl_vB_vA = [pA[0]+0.5*(vB[0]+vA[0]), pA[1]+0.5*(vB[1]+vA[1])]
                    dist_BA = distance(pA, pB)
                    theta_BA = atan2(pB[1]-pA[1], pB[0]-pA[0])
                    if 2*ROB_RAD > dist_BA:
                        dist_BA = 2*ROB_RAD
                    theta_BAort = asin(2*ROB_RAD/dist_BA)
                    theta_ort_left = theta_BA+theta_BAort
                    bound_left = [cos(theta_ort_left), sin(theta_ort_left)]
                    theta_ort_right = theta_BA-theta_BAort
                    bound_right = [cos(theta_ort_right), sin(theta_ort_right)]
                    RVO_BA = [transl_vB_vA, bound_left, bound_right, dist_BA, 2*ROB_RAD]
                
                RVO_BA_all.append(RVO_BA)
        
        # Add collision avoidance for links
        for pair in pairs:
            if i not in pair:
                # Treat the link as a line segment obstacle
                p1, p2 = X[pair[0]], X[pair[1]]
                
                # Check if the robot's path intersects with the link
                if line_intersection(pA, [pA[0]+vA[0], pA[1]+vA[1]], p1, p2):
                    # If intersection occurs, create a repulsive force
                    closest_point = closest_point_on_segment(pA, p1, p2)
                    dist_BA = distance(pA, closest_point)
                    if dist_BA < ROB_RAD:
                        dist_BA = ROB_RAD
                    theta_BA = atan2(closest_point[1]-pA[1], closest_point[0]-pA[0])
                    theta_BAort = asin(ROB_RAD/dist_BA)
                    theta_ort_left = theta_BA+theta_BAort
                    bound_left = [cos(theta_ort_left), sin(theta_ort_left)]
                    theta_ort_right = theta_BA-theta_BAort
                    bound_right = [cos(theta_ort_right), sin(theta_ort_right)]
                    RVO_BA = [closest_point, bound_left, bound_right, dist_BA, ROB_RAD]
                    RVO_BA_all.append(RVO_BA)
        
        # Add collision avoidance for circular static obstacles
        for obstacle in ws_model['circular_obstacles']:
            obstacle_center = [obstacle[0], obstacle[1]]
            obstacle_radius = obstacle[2]
            dist_BA = distance(pA, obstacle_center) - obstacle_radius
            if dist_BA < ROB_RAD:
                dist_BA = ROB_RAD
            theta_BA = atan2(obstacle_center[1]-pA[1], obstacle_center[0]-pA[0])
            theta_BAort = asin(ROB_RAD/dist_BA)
            theta_ort_left = theta_BA+theta_BAort
            bound_left = [cos(theta_ort_left), sin(theta_ort_left)]
            theta_ort_right = theta_BA-theta_BAort
            bound_right = [cos(theta_ort_right), sin(theta_ort_right)]
            RVO_BA = [obstacle_center, bound_left, bound_right, dist_BA, ROB_RAD]
            RVO_BA_all.append(RVO_BA)
        
        vA_post = intersect(pA, V_des[i], RVO_BA_all)
        V_opt[i] = vA_post[:]
    return V_opt

def intersect(pA, vA, RVO_BA_all):
    # print '----------------------------------------'
    # print 'Start intersection test'
    norm_v = distance(vA, [0, 0])
    suitable_V = []
    unsuitable_V = []
    for theta in numpy.arange(0, 2*PI, 0.1):
        for rad in numpy.arange(0.02, norm_v+0.02, norm_v/5.0):
            new_v = [rad*cos(theta), rad*sin(theta)]
            suit = True
            for RVO_BA in RVO_BA_all:
                p_0 = RVO_BA[0]
                left = RVO_BA[1]
                right = RVO_BA[2]
                dif = [new_v[0]+pA[0]-p_0[0], new_v[1]+pA[1]-p_0[1]]
                theta_dif = atan2(dif[1], dif[0])
                theta_right = atan2(right[1], right[0])
                theta_left = atan2(left[1], left[0])
                if in_between(theta_right, theta_dif, theta_left):
                    suit = False
                    break
            if suit:
                suitable_V.append(new_v)
            else:
                unsuitable_V.append(new_v)                
    new_v = vA[:]
    suit = True
    for RVO_BA in RVO_BA_all:                
        p_0 = RVO_BA[0]
        left = RVO_BA[1]
        right = RVO_BA[2]
        dif = [new_v[0]+pA[0]-p_0[0], new_v[1]+pA[1]-p_0[1]]
        theta_dif = atan2(dif[1], dif[0])
        theta_right = atan2(right[1], right[0])
        theta_left = atan2(left[1], left[0])
        if in_between(theta_right, theta_dif, theta_left):
            suit = False
            break
    if suit:
        suitable_V.append(new_v)
    else:
        unsuitable_V.append(new_v)
    #----------------------        
    if suitable_V:
        # print 'Suitable found'
        vA_post = min(suitable_V, key = lambda v: distance(v, vA))
    else:
        # print 'Suitable not found'
        tc_V = dict()
        for unsuit_v in unsuitable_V:
            tc_V[tuple(unsuit_v)] = 0
            tc = []
            for RVO_BA in RVO_BA_all:
                p_0 = RVO_BA[0]
                left = RVO_BA[1]
                right = RVO_BA[2]
                dist = RVO_BA[3]
                rad = RVO_BA[4]
                dif = [unsuit_v[0]+pA[0]-p_0[0], unsuit_v[1]+pA[1]-p_0[1]]
                theta_dif = atan2(dif[1], dif[0])
                theta_right = atan2(right[1], right[0])
                theta_left = atan2(left[1], left[0])
                if in_between(theta_right, theta_dif, theta_left):
                    small_theta = abs(theta_dif-0.5*(theta_left+theta_right))
                    if abs(dist*sin(small_theta)) >= rad:
                        rad = abs(dist*sin(small_theta))
                    big_theta = asin(abs(dist*sin(small_theta))/rad)
                    dist_tg = abs(dist*cos(small_theta))-abs(rad*cos(big_theta))
                    if dist_tg < 0:
                        dist_tg = 0                    
                    tc_v = dist_tg/distance(dif, [0,0])
                    tc.append(tc_v)
            tc_V[tuple(unsuit_v)] = min(tc)+0.001
        WT = 0.2
        vA_post = min(unsuitable_V, key = lambda v: ((WT/tc_V[tuple(v)])+distance(v, vA)))
    return vA_post 

def in_between(theta_right, theta_dif, theta_left):
    if abs(theta_right - theta_left) <= PI:
        if theta_right <= theta_dif <= theta_left:
            return True
        else:
            return False
    else:
        if (theta_left <0) and (theta_right >0):
            theta_left += 2*PI
            if theta_dif < 0:
                theta_dif += 2*PI
            if theta_right <= theta_dif <= theta_left:
                return True
            else:
                return False
        if (theta_left >0) and (theta_right <0):
            theta_right += 2*PI
            if theta_dif < 0:
                theta_dif += 2*PI
            if theta_left <= theta_dif <= theta_right:
                return True
            else:
                return False

def compute_V_des(X, goal, V_max):
    V_des = []
    for i in range(len(X)):
        dif_x = [goal[i][k]-X[i][k] for k in range(2)]
        norm = distance(dif_x, [0, 0])
        norm_dif_x = [dif_x[k]*V_max[i]/norm for k in range(2)]
        V_des.append(norm_dif_x[:])
        if reach(X[i], goal[i], 0.1):
            V_des[i][0] = 0
            V_des[i][1] = 0
    return V_des
            
def reach(p1, p2, bound=0.5):
    if distance(p1,p2)< bound:
        return True
    else:
        return False

def closest_point_on_segment(p, a, b):
    """
    Find the closest point on segment ab to point p
    """
    ax, ay = a
    bx, by = b
    px, py = p
    
    # Vector from a to b
    abx = bx - ax
    aby = by - ay
    
    # Vector from a to p
    apx = px - ax
    apy = py - ay
    
    # Project ap onto ab, computing the scale factor
    t = (apx * abx + apy * aby) / (abx * abx + aby * aby)
    
    if t < 0:
        return a  # Beyond a
    elif t > 1:
        return b  # Beyond b
    else:
        return [ax + t * abx, ay + t * aby]  # On the segment

def line_intersection(p1, p2, p3, p4):
    """Check if two line segments intersect"""
    def ccw(A, B, C):
        return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])
    return ccw(p1,p3,p4) != ccw(p2,p3,p4) and ccw(p1,p2,p3) != ccw(p1,p2,p4)