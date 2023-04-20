import math
from geometry import get_euclidean_distance, get_nearest_point_on_line, get_nearest_point_on_dubins_line

import dubins


class Edge:
    """A base class for storing edge information, including its geometry"""

    def __init__(self, s1, s2, step_size=0.1):
        """The constructor

        @type s1: a float indicating the state at the begining of the edge
        @type s2: a float indicating the state at the end of the edge
        @type step_size: a float indicating the length between consecutive states
            in the discretization
        """
        # The origin of the edge
        self.s1 = s1

        # The destination of the edge
        self.s2 = s2

        # The step size for discretizing the edge
        self.step_size = step_size


    def __str__(self):
        return "(" + str(self.s1) + "," + str(self.s2) + ")"

    def get_origin(self):
        """Return the point at the beginning of the edge"""
        return self.s1

    def get_destination(self):
        """Return the point at the end of the edge"""
        return self.s2

    def get_step_size(self):
        return self.step_size

    def get_cost(self):
        """Return the cost of the edge"""
        return self.get_length()

    def get_path(self):
        """Return the path, representing the geometry of the edge"""
        return [self.s1, self.s2]

    def reverse(self):
        """Reverse the origin/destination of the edge"""
        tmp = self.s1
        self.s1 = self.s2
        self.s2 = tmp

    def get_discretized_state(self, i):
        """Return the i^{th} discretized state"""
        raise NotImplementedError

    def get_nearest_point(self, state):
        """Compute the nearest point on this edge to the given state

        @return (s, t) where s is the point on the edge that is closest to state
        and it is at distance t*length from the beginning of the edge
        """
        raise NotImplementedError

    def split(self):
        """Split the edge at distance t/length where length is the length of this edge

        @return (edge1, edge2) edge1 and edge2 are the result of splitting the original edge
        """
        raise NotImplementedError

    def get_length(self):
        """Return the length of the edge"""
        raise NotImplementedError


class EdgeDubins(Edge):

    def __init__(self, s1, s2, step_size=0.1):
        super().__init__(s1, s2, step_size)

        # Store useful information so that they do not need to be recomputed
        self.line_segment = s2 - s1  # The line segment from s1 to s2
        self.length = get_euclidean_distance(
            self.s1, self.s2
        )  # the length of this line segment
        if self.length > 0:
            # for discretization purpose
            self.tstep = min(step_size / self.length, 1)

        # The number of discretized state
        self.num_discretized_states = len(
            self.get_all_discretized_states_points())
        self.parent_edge_start = ()

    def reverse(self):
        """Reverse the origin/destination of the edge"""
        super().reverse()
        self.line_segment = self.s2 - self.s1

    def get_discretized_state(self, i):
        config=self.get_all_discretized_states_points()
        """Return the i^{th} discretized state"""
        if i == 0:
            return self.s1
        if i == self.num_discretized_states - 1:
            return self.s2
        if i >= len(config)-1:
            #print("here",i,self.num_discretized_states-1)
            return None
        #print(len(config),i)
        return config[i]
        

    def get_all_discretized_states_points(self):
        path = dubins.shortest_path(self.s1, self.s2, 0.5)
        configurations, _ = path.sample_many(self.step_size)
        configurations.append(tuple(self.s2))
        return configurations

    def get_nearest_point(self, state):
        """Compute the nearest point on this edge to the given state
        @return (s, t) where s is the point on the edge that is closest to state
        and it is at distance t*length from the beginning of the edge
        """

        #return get_nearest_point_on_dubins_line(paths, state)
        return get_nearest_point_on_line(self.s1, self.s2, state)

    def split(self, t):
        """Split the edge at distance t/length where length is the length of this edge
        @return (edge1, edge2) edge1 and edge2 are the result of splitting the original edge
        """
        s = self.s1 + t * self.line_segment
        e1 = EdgeDubins(self.s1, s, self.step_size)
        e2 = EdgeDubins(s, self.s2, self.step_size)
        e2.parent_edge_start = e1.s1
        return (
            e1, e2
        )

    def get_length(self):
        """Return the length of the edge"""
        return self.length



