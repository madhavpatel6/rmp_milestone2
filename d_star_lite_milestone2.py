#D*Lite Python
import Queue
import math
import time
import matplotlib.pyplot as plt
import matplotlib
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32

def create_Graph(x_steps,y_steps,dx,dy,weight):
	#Graph={}
	TL=(2*y_steps-1)*(2*x_steps-1)-(2*x_steps-2)
	TR=(2*y_steps-1)*(2*x_steps-1)
	BR=(2*x_steps-1)
	#print(TL,TR,BR)
	up=(2*x_steps-1)
	down=-(2*x_steps-1)
	Graph={1:[[-dx*(x_steps-1),-dy*(y_steps-1)],[[2,weight],[2*x_steps,weight]]]}
	Graph[2*x_steps-1]=[[dx*(x_steps-1),-dy*(y_steps-1)],[[(x_steps-2),weight],[4*x_steps-2,weight]]]
	Graph[(2*y_steps-1)*(2*x_steps-1)-(2*x_steps-2)]=[[-dx*(x_steps-1),dy*(y_steps-1)],[[(2*y_steps-1)*(2*x_steps-1)-(2*x_steps-2)+1,weight],[(2*y_steps-1)*(2*x_steps-1)-(2*x_steps-2)-(2*x_steps-1),weight]]]
	Graph[(2*y_steps-1)*(2*x_steps-1)]=[[dx*(x_steps-1),dy*(y_steps-1)],[[(2*y_steps-1)*(2*x_steps-1)-1,weight],[(2*y_steps-1)*(2*x_steps-1)-(2*x_steps-1),weight]]]
	for n,i in zip(range(2,2*x_steps-1),range(-dx*(x_steps-1)+dx,dx*x_steps-dx,dx)):#bottom side
		#print(n,i)
		Graph[n]=[[i,-dy*(y_steps-1)],[[n-1,weight],[n+up,weight],[n+1,weight]]]

	for n,i in zip(range(TL+1,TR),range(-dx*(x_steps-1)+dx,dx*x_steps-dx,dx)):#top side
		#print(n,i)
		Graph[n]=[[i,dy*(y_steps-1)],[[n-1,weight],[n+down,weight],[n+1,weight]]]
	
	for n,i in zip(range(1+up,TL+down+1,up),range(-dy*(y_steps-1)+dy,dy*y_steps-dy,dy)):#left side
		#print(n,i)
		Graph[n]=[[-dx*(x_steps-1),i],[[n+up,weight],[n+down,weight],[n+1,weight]]]
		
	for n,i in zip(range(BR+up,TR+down+1,up),range(-dy*(y_steps-1)+dy,dy*y_steps-dy,dy)):#right side
		#print(n,i)
		Graph[n]=[[dx*(x_steps-1),i],[[n+up,weight],[n+down,weight],[n-1,weight]]]	
	
	
	
	for nx,i in zip(range(2+up,2*x_steps-1+up),range(-dx*(x_steps-1)+dx,dx*x_steps-dx,dx)):
		for ny,j in zip(range((2*y_steps-1)),range(-dy*(y_steps-1)+dy,dy*y_steps-dy,dy)):
			#print(nx,ny,i,j)
			curr=nx+ny*up
			Graph[curr]=[[i,j],[[curr+1,weight],[curr-1,weight],[curr+up,weight],[curr+down,weight]]]
	#print(Graph[25])
	#or i in range(1,TR):
	#	print(Graph[i])		
	return Graph

def Calc_keys(s):
	k1=min([g[s],rhs[s]])+heuristic(s,goal)+Km
	k2=min([g[s],rhs[s]])
	key=[k1,k2]
	#print("calc kyes: ",key)
	return key

def init():
	global U, S, Km, rhs, g
	U=Queue.PriorityQueue(maxsize=0)
	S=[]
	Keys=Graph.keys()
	for key in Keys:
		S.append(key)
	Km=0
	rhs={}
	g={}
	for i in range(len(S)):
		rhs[S[i]]=float("inf")
		g[S[i]]=float("inf")
	rhs[goal]=0
	U.put((Calc_keys(goal)[0],goal))
	#U.put((2,goal))
	#print(U.queue)
	#print("Queue Init",U.get())
	

def Update_Vertex(u):
	if int(u) != int(goal):
		#print(u,goal)
		s_succ=Graph[u][1]
		temp=[]
		for s in s_succ:
			temp.append(s[1]+g[s[0]])
		#print("u is: ",u,"Temp: ", temp)
		rhs[u]=min(temp)
	u_U=False
	for uu in U.queue:
		if uu[1] is u:
			u_U=True
			break
	if u_U is True:
		U.get(uu)
	if rhs[u] != g[u]:
		U.put((Calc_keys(goal)[0],u))
	#print(rhs)
def Calc_shortest_path():
	counter=0
	print(U.queue[0][0],start,rhs[start],g[start])
	while U.queue[0][0] < 9 or float(rhs[start]) != float(g[start]):
		KU=U.get()
		k_old=KU[0]
		u=KU[1]
		#print("u: ",u,"kold: ",k_old,"Key: ",Calc_keys(u))
		if k_old < Calc_keys(u)[0]:
			#print("if")
			U.put((Calc_keys(u)[0],u))		
		elif g[u] > rhs[u]:
			#print("elif")
			g[u]=rhs[u]
			s_pred=Graph[u][1]
			for s in s_pred:
				Update_Vertex(s[0])
		else:
			#print("else")
			g[u]=float('inf')
			for s in Graph[u][1]:
				Update_Vertex(s[0])
			Update_Vertex(u)
		
		counter=counter+1
		if counter>=1000000000:
			#print(U.queue)
			#counter=0
			g_keys=g.keys()
			good=[]
			for i in g_keys:
				if g[i] is not rhs[i]:
					good.append(i)
			x_g=[]
			y_g=[]
			for i in good:
				x_g.append(Graph[i][0][0])
				y_g.append(Graph[i][0][1])
			#plt.plot(x_g,y_g,'go')
			#plt.show()
	print(counter)	
	
def heuristic(s,s_p):
	s_coords=Graph[s][0]
	goal_coords=Graph[s_p][0]
	distance=((goal_coords[0]-s_coords[0])**2+(goal_coords[1]-s_coords[1])**2)**0.5
	#print("Distance: ",distance)
	return distance

def move_to_vertex(pub, v):
    rospy.loginfo('Attempting to move to vertex ' + str(v))
    for i in range(10):
    	pub.publish(v)
    print("Im stupid",v,pub)
    while True:
        node = rospy.wait_for_message('/current_node', String).data
        #print('Current Node', int(node.data))
	pub.publish(v)
        if int(node) == v:
            break
        rospy.sleep(0.5)
    rospy.loginfo('Successfully moved to vertex ' + str(v))	

def read_edge_costs():
    rospy.loginfo('Reading in the current edge costs')
    costs = rospy.wait_for_message('/edge_costs', String).data
    print(costs)
    costs_split = costs.split('\n')
    cost_list = []
    for c in costs_split:
        cost = c.split('\t')
        if len(cost) == 1:
            continue
        current_node, neighboring_node, edge_cost = int(cost[0]), int(cost[1]), float(cost[2])
        cost_list.append([current_node, neighboring_node, edge_cost])
        rospy.loginfo('%d -> %d : %f' % (current_node, neighboring_node, edge_cost))
    return cost_list

def update_Graph(Costs):
	u_to_update=[]
	for i in Costs:
		for j in Graph[i[0]][1]:
			if i[1] == j[0] and i[2] != j[1]:
				Temp=Graph[i[0]][1]
				Temp[Temp.index(j)]=[i[1],i[2]]
				u_to_update.append(i[0])
				print("Update stuff:", Temp,u_to_update)
	return u_to_update

def convert_xy_to_vertex(x, y):
    grid_size_x=5
    grid_size_y=5
    dx = x + math.floor(grid_size_x / 2.0) - 1 + grid_size_x % 2
    dy = y + math.floor(grid_size_y / 2.0) - 1 + grid_size_y % 2
    return int(1 + (dx * grid_size_y) + dy)

global goal, start, Graph
rospy.init_node('milestone2')
pub = rospy.Publisher('/goal', Int32,queue_size=100)
time.sleep(5)
Graph=create_Graph(30,30,1,1,1)#float('inf'))
kk=Graph.keys()
print(max(kk))
goal=convert_xy_to_vertex(22, -17)
print(Graph[goal])
print("Goal: ",goal)
start=15
snode = rospy.wait_for_message('/current_node', String).data
start=int(snode)
print("Start is: ",start)
PLOT=False
if PLOT is True:

	x=[]
	y=[]
	for i in kk:
		x.append(Graph[i][0][0])
		y.append(Graph[i][0][1])
	plt.plot(x, y, 'ro')
	plt.plot(Graph[goal][0][0],Graph[goal][0][1],'go')
	plt.plot(Graph[start][0][0],Graph[start][0][1],'go')
	plt.show()
s_last=start
init()
Calc_shortest_path()
path=[]
Costs=read_edge_costs()#read edge costs
edge_update=update_Graph(Costs)	
if len(edge_update) > 0:
	Km=Km+heuristic(s_last,start)
	s_last=start
	for i in edge_update:
		Update_Vertex(i)
	Calc_shortest_path()
while start != goal:
	s_succ=Graph[start][1]
	temp=[]
	for s in s_succ:
		temp.append(s[1]+g[s[0]])
	#print(temp.index(min(temp)))
	start=s_succ[temp.index(min(temp))][0]
	path.append(start)
	move_to_vertex(pub,start)#move to start
	for i in range(1):
		Costs=read_edge_costs()#read edge costs
		#time.sleep(0.5)
	edge_update=update_Graph(Costs)	
	if len(edge_update) > 0:
		Km=Km+heuristic(s_last,start)
		s_last=start
		for i in edge_update:
			Update_Vertex(i)
		Calc_shortest_path()
#x_p=[]
#y_p=[]
#for i in path:
#	x_p.append(Graph[i][0][0])
#	y_p.append(Graph[i][0][1])
#plt.plot(x_p, y_p, 'ro')
#plt.show()