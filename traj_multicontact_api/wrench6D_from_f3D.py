import numpy as np

def contacts_from_footrect_center():
    """
    Compute the relative contact points from foot center of a Talos foot.
    
    Order is clockwise, starting from bottom left (looking at a forward pointing foot from above).
    Expressed in local foot coordinates. 
    """
    lx = 0.1
    ly = 0.65
    contacts = np.zeros((3,4))  # [F_p_FCi]
    contacts[0, :] = [-lx, -lx, lx, lx]
    contacts[1, :] = [-ly, ly, -ly, ly]
    contacts[2, :] = [0] * 4
    return contacts

def contact_force_to_wrench(contacts, cf):
    """
    Compute the wrench at the end effector center expressed in world coordinates.
    
    Each contact force (at a foot for instance) is represented in MAPI as a set of 
    4 forces at the corners of the contact polygone (foot -> rectangle). These forces,
    as the other quantities, are expressed in world coordinates. From this 4 3D forces, 
    we can compute the 6D wrench at the center of the origin point of the contacts vectors.
    cf: corner forces ordered as [fx1, fy1, fz1, fx2, fy2... fz4]
    """
    cfr = cf.reshape((3,4), order='F')  # reshape in Fortran style to have forces as columns
    return np.hstack([
            cfr.sum(axis=1),
            sum(np.cross(contacts[:,k], cfr[:,k]) for k in range(4))
                     ])

contacts = contacts_from_footrect_center()
f_arr_foot_left = np.random.rand(12)
f_arr_foot_right = np.random.rand(12)
wrench6D_foot_left = contact_force_to_wrench(contacts, f_arr_foot_left)
wrench6D_foot_right = contact_force_to_wrench(contacts, f_arr_foot_right)
