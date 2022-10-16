#%%
import numpy as np
import copy
from numpy import linalg as la
import matplotlib.pyplot as plt
#%%
def Graham(points, Plot = True):
    stack, theta, ind = [], [], []

    s_ind = np.where(points[:,1] == np.min(points[:,1]))[0] #row index of corrdinate w min y value
    start = np.array([points[s_ind,0], points[s_ind,1]]).reshape([2,])
    points = np.delete(points, s_ind, 0)
    points_rel = points - start #relative poisitions
    stack.append(start)

    for i in range(0,len(points_rel)):
        theta.append(np.arccos(points_rel[i,0]/la.norm(points_rel[i,:]))) #angle wrt x axis [1 0 0]

    theta_copy = copy.copy(theta)

    for i in range(0, len(theta)):
        min_tht = min(theta)
        ind.append(theta_copy.index(min_tht))
        theta.remove(min_tht)

    stack.append(points[ind[0],:]) #smallest angle point is guarenteed on convex hull

    for i in range (1,len(ind)):
        cp = np.cross((stack[-1] - stack[-2]), (points[ind[i],:] - stack[-1]))
        if cp >= 0:
            stack.append(points[ind[i],:])
        else:
            while cp < 0:
                del stack[-1]
                cp = np.cross((stack[-1] - stack[-2]), (points[ind[i],:] - stack[-1]))
            stack.append(points[ind[i],:])

    if Plot:
        xmin, xmax = min(np.min(points[:,0]), start[0]), max(np.max(points[:,0]), start[0])
        ymin, ymax = min(np.min(points[:,1]), start[1]), max(np.max(points[:,1]), start[1])
        fig = plt.figure(figsize = (10,10))
        ax = fig.add_subplot(111)
        ax.set_xlim(xmin,xmax)
        ax.set_ylim(ymin,ymax)
        ax.axis('equal')
        plt.scatter(np.append(points[:,0], start[0]), np.append(points[:,1], start[1]), marker = 'o')
        for i in range(0, len(stack)-1):
            plt.plot([stack[i][0], stack[i+1][0]], [stack[i][1], stack[i+1][1]], color='red')

        plt.plot([stack[0][0], stack[-1][0]], [stack[0][1], stack[-1][1]], color = 'red')
        plt.show()

    return stack