#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
from cs476.msg import Chain2D
import math


def get_chain_msg():
    """Return a message from the "chain_config" channel.

    This function will wait until a message is received.
    """
    # TODO: Implement this function
    rospy.init_node('chain_msg', anonymous=True)
    msg = rospy.wait_for_message('chain_config', Chain2D)

    return msg


    raise NotImplementedError


def plot_chain(config, W, L, D):
    """Plot a 2D kinematic chain A_1, ..., A_m

    @type config: a list [theta_1, ..., theta_m] where theta_1 represents the angle between A_1 and the x-axis,
        and for each i such that 1 < i <= m, \theta_i represents the angle between A_i and A_{i-1}.
    @type W: float, representing the width of each link
    @type L: float, representing the length of each link
    @type D: float, the distance between the two points of attachment on each link
    """

    (joint_positions, link_vertices) = get_link_positions(config, W, L, D)

    fig, ax = plt.subplots()
    plot_links(link_vertices, ax)
    plot_joints(joint_positions, ax)
    ax.axis("equal")
    plt.show()


def plot_links(link_vertices, ax):
    """Plot the links of a 2D kinematic chain A_1, ..., A_m on the axis ax

    @type link_vertices: a list [V_1, ..., V_m] where V_i is the list of [x,y] positions of vertices of A_i
    """

    for vertices in link_vertices:
        x = [vertex[0] for vertex in vertices]
        y = [vertex[1] for vertex in vertices]

        x.append(vertices[0][0])
        y.append(vertices[0][1])
        ax.plot(x, y, "k-", linewidth=2)


def plot_joints(joint_positions, ax):
    """Plot the joints of a 2D kinematic chain A_1, ..., A_m on the axis ax

    @type joint_positions: a list [p_1, ..., p_{m+1}] where p_i is the position [x,y] of the joint between A_i and A_{i-1}
    """
    x = [pos[0] for pos in joint_positions]
    y = [pos[1] for pos in joint_positions]
    ax.plot(x, y, "k.", markersize=10)


def get_link_positions(config, W, L, D):
    """Compute the positions of the links and the joints of a 2D kinematic chain A_1, ..., A_m

    @type config: a list [theta_1, ..., theta_m] where theta_1 represents the angle between A_1 and the x-axis,
        and for each i such that 1 < i <= m, \theta_i represents the angle between A_i and A_{i-1}.
    @type W: float, representing the width of each link
    @type L: float, representing the length of each link
    @type D: float, the distance between the two points of attachment on each link

    @return: a tuple (joint_positions, link_vertices) where
        * joint_positions is a list [p_1, ..., p_{m+1}] where p_i is the position [x,y] of the joint between A_i and A_{i-1}
        * link_vertices is a list [V_1, ..., V_m] where V_i is the list of [x,y] positions of vertices of A_i
    """
    # TODO: Implement this function


    joint_positions = []
    num_links = len(config)
    link_vertices = []
    if num_links == 0:
        return [], []
    anglex=0
    angley=0
    joint_positions.append([0, 0])
    for i in range(num_links):

        #In this part calculated position and made sure that the angles are with respect to X axis and for this need to 
        #add them up using the variable anglex and angley
        x = joint_positions[-1][0] + D * math.cos(config[i]+anglex)   
        
        
        y = joint_positions[-1][1] + D * math.sin(config[i]+angley)
        anglex=anglex+config[i]
        angley=angley+config[i]
        joint_positions.append([x, y])

    # Calculate link vertices
        #here few parts were really tricky. For that I manually calculated value of angle 45+90 , 90+180 and 45 manually 
        # to comply with the transition matrix calculation.


        ######################################Acknowledgement#######################################
        #The idea of angle calculation was taken from Md Rayhanul Islam as I was struggling here to make it look ok
        #The idea of this calculation was taken from the book and few youtube resources. 

        ######################################Acknowledgement#######################################


        
        widthSq=math.sqrt((W/2)**2 + (W/2)**2)
        x1 = joint_positions[i][0] + widthSq * math.cos(anglex+2.356)
        y1 = joint_positions[i][1] + widthSq * math.sin(angley+2.356)
        x2 = joint_positions[i][0] + widthSq* math.cos(anglex+3.927)
        y2 = joint_positions[i][1] + widthSq * math.sin(angley+3.927)
        x3 = x + widthSq * math.cos(anglex-.785)
        y3 = y + widthSq * math.sin(angley-.785)
        x4 = x + widthSq * math.cos(anglex+.785)
        y4 = y + widthSq * math.sin(angley+.785)
        link_vertices.append([[x1, y1], [x2, y2], [x3, y3], [x4, y4]])

    return (joint_positions, link_vertices)
    raise NotImplementedError


if __name__ == "__main__":
    chain = get_chain_msg()
    print("chain msg",chain)
    plot_chain(chain.config, chain.W, chain.L, chain.D)
