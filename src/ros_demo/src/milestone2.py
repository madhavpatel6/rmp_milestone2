#!/usr/bin/env python
import math
import pprint
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
pp = pprint.PrettyPrinter(indent=4)

grid_size_x = rospy.get_param('/grid_size_x')
grid_size_y = rospy.get_param('/grid_size_y')
grid_step = rospy.get_param('/grid_step')

real_size_x = grid_size_x + ((grid_size_x - 1)*(1/grid_step - 1))
real_size_y = grid_size_y + ((grid_size_y - 1)*(1/grid_step - 1))

print 'Real Size %d, %d' % (real_size_x, real_size_y)
def convert_vertex_to_xy(v):
    x = math.floor(((v - 1) / real_size_y)) * grid_step - math.floor(grid_size_x / 2.0) + 1 - grid_size_x % 2
    y = ((v - 1) % real_size_y) * grid_step - math.floor(grid_size_y / 2.0) + 1 - grid_size_y % 2
    return x, y

def convert_xy_to_vertex(x, y):
    dx = x + math.floor(grid_size_x / 2.0) - 1 + grid_size_x % 2
    dy = y + math.floor(grid_size_y / 2.0) - 1 + grid_size_y % 2
    return int(1 + (dx * real_size_y)/grid_step + dy/grid_step)

def check_bounds(x, y, grid_size_x, grid_size_y):
    x_min = - math.floor(grid_size_x / 2.0) + 1 - grid_size_x % 2
    x_max = math.floor(grid_size_x / 2.0)
    y_min = - math.floor(grid_size_y / 2.0) + 1 - grid_size_y % 2
    y_max = math.floor(grid_size_y / 2.0)
    return x_min <= x <= x_max and y_min <= y <= y_max

def create_graph(grid_size_x, grid_size_y):
    graph = {}
    x_min = - math.floor(grid_size_x / 2.0) + 1 - grid_size_x % 2
    x_max = math.floor(grid_size_x / 2.0)
    y_min = - math.floor(grid_size_y / 2.0) + 1 - grid_size_y % 2
    y_max = math.floor(grid_size_y / 2.0)
    for i in range(1, int(real_size_y*real_size_x) + 1):
        x, y = convert_vertex_to_xy(i)
        graph[i] = [[x, y], [], []]
        if check_bounds(x - grid_step, y, grid_size_x, grid_size_y):
            graph[i][1].append([convert_xy_to_vertex(x - grid_step, y), 1])
        if check_bounds(x + grid_step, y, grid_size_x, grid_size_y):
            graph[i][1].append([convert_xy_to_vertex(x + grid_step, y), 1])
        if check_bounds(x, y - grid_step, grid_size_x, grid_size_y):
            graph[i][1].append([convert_xy_to_vertex(x, y - grid_step), 1])
        if check_bounds(x, y + grid_step, grid_size_x, grid_size_y):
            graph[i][1].append([convert_xy_to_vertex(x, y + grid_step), 1])
    for k in graph.keys():
        for e in graph[k][1]:
            graph[e[0]][2].append(k)
    return graph

class DStar:
    def __init__(self):
        self.graph = create_graph(grid_size_x, grid_size_y)
        self.g = {}
        self.rhs = {}
        self.s_start = 1
        self.s_last = self.s_start
        self.km = 0
        self.U = []
        self.goal = [22, -17]
        self.goal_vertex = convert_xy_to_vertex(self.goal[0], self.goal[1])
        print('Goal position (%d, %d) -> %d' % (self.goal[0], self.goal[1], self.goal_vertex))
        self.initialize()
        self.pub = rospy.Publisher('/goal', Int32, queue_size=10)

    def calculate_key(self, s):
        return [min(self.g[s], self.rhs[s]) + self.heuristic(self.s_start, s) + self.km, min(self.g[s], self.rhs[s])]

    def compare_key_less_than(self, k1, k2):
        if k1[0] < k2[0]:
            return True
        elif k1[0] == k2[0] and k1[1] < k2[1]:
            return True
        else:
            return False

    def compare_key_less_than_equal(self, k1, k2):
        if k1[0] < k2[0]:
            return True
        elif k1[0] == k2[0] and k1[1] <= k2[1]:
            return True
        else:
            return False

    def heuristic(self, s, goal):
        s_x, s_y = convert_vertex_to_xy(s)
        goal_x, goal_y = convert_vertex_to_xy(goal)
        return math.sqrt((s_x - goal_x)**2 + (s_y - goal_y)**2)

    def initialize(self):
        self.km = 0
        for i in range(1, int(real_size_x * real_size_y) + 1):
            self.rhs[i] = float('inf')
            self.g[i] = float('inf')
        self.rhs[self.goal_vertex] = 0
        self.U.append([self.goal_vertex, self.calculate_key(self.goal_vertex)])
        pp.pprint(self.U)

    def update_vertex(self, u):
        successors = self.graph[u][1]
        if u != self.goal_vertex:
            min_successors = min(successors, key=lambda x: x[1] + self.g[x[0]])
            self.rhs[u] = min_successors[1] + self.g[min_successors[0]]
        # If U is already in U we remove it
        for i in range(len(self.U)):
            if self.U[i][0] == u:
                del self.U[i]
                break

        if self.g[u] != self.rhs[u]:
            self.U.append([u, self.calculate_key(u)])

    def compute_shortest_path(self):
        while self.compare_key_less_than(self.get_top_of_queue_key(), self.calculate_key(self.s_start)) or \
                self.rhs[self.s_start] != self.g[self.s_start]:
            k_old = self.get_top_of_queue_key()
            u = self.pop_top_of_queue()[0]

            if self.compare_key_less_than(k_old, self.calculate_key(u)):
                self.U.append([u, self.calculate_key(u)])
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for p in self.graph[u][2]:
                    self.update_vertex(p)
            else:
                self.g[u] = float('inf')
                for p in self.graph[u][2]:
                    self.update_vertex(p)
                self.update_vertex(u)

    def get_top_of_queue(self):
        minimum = None
        for v in self.U:
            if minimum is None:
                minimum = v
            if self.compare_key_less_than(v[1], minimum[1]):
                minimum = v
        return minimum

    def get_top_of_queue_key(self):
        return self.get_top_of_queue()[1]

    def pop_top_of_queue(self):
        top = self.get_top_of_queue()
        index = self.U.index(top)
        del self.U[index]
        return top

    def move_to_vertex(self, v):
        rospy.loginfo('Moving to vertex ' + str(v))
        self.pub.publish(v)
        while True:
            node = rospy.wait_for_message('/current_node', String).data
            # print('Current Node', int(node.data))
            if int(node) == v:
                break
            rospy.sleep(0.5)
        rospy.loginfo('Completed move to vertex ' + str(v))

    def check_updated_edge_costs(self, costs):
        print costs
        changed = False
        for current_node, edge, cost in costs:
            for c in self.graph[current_node][1]:
                if c[0] == edge and c[1] != cost:
                    print('Updated cost that was different %d -> %d = %f to %f' % (current_node, c[0], c[1], cost))
                    c[1] = cost
                    self.update_vertex(current_node)
                    changed = True
        return changed

    def read_edge_costs(self):
        #   rospy.loginfo('Reading in the current edge costs')
        costs = rospy.wait_for_message('/edge_costs', String).data
        costs_split = costs.split('\n')
        cost_list = []
        curr_node = None
        for c in costs_split:
            cost = c.split('\t')
            if len(cost) == 1:
                continue
            current_node, neighboring_node, edge_cost = int(cost[0]), int(cost[1]), float(cost[2])
            cost_list.append([current_node, neighboring_node, edge_cost])

        return cost_list

    def main_loop(self):
        self.s_start = int(rospy.wait_for_message('/current_node', String).data)
        self.s_last = self.s_start

        self.compute_shortest_path()
        self.s_last = self.s_start
        while self.s_start != self.goal_vertex:
            self.s_start = min(self.graph[self.s_start][1], key=lambda e: e[1] + self.g[e[0]])[0]
            self.move_to_vertex(self.s_start)
            edge_costs = self.read_edge_costs()
            updated = self.check_updated_edge_costs(edge_costs)
            if updated:
                self.km += self.heuristic(self.s_last, self.s_start)
                self.s_last = self.s_start
                self.compute_shortest_path()


def main():
    rospy.init_node('milestone2')
    d = DStar()
    d.main_loop()

if __name__ == "__main__":
    main()