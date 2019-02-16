# Essential imports
from Tkinter import *
import xml.etree.ElementTree as ET
import math
from Queue import *
import struct

# some constants about the earth
MPERLAT = 111000 # meters per degree of latitude, approximately
MPERLON = MPERLAT * math.cos(42*math.pi/180) # meters per degree longitude at 42N

# Bounds of Map (Parts of Oshawa which we are familiar with)
LEFTBOUND = -78.9103
RIGHTBOUND = -78.8197
TOPBOUND= 43.9738
BOTBOUND = 43.8986

# Window size
WINWIDTH = 800
WINHEIGHT = 800

# Node Class
class Node():
    def __init__(self,p,e):
        self.pos = p
        self.ways = []
        self.elev = e

# Way Class
class Way():
    def __init__(self,n,t):
        self.name = n
        self.type = t
        self.nodes = []

# Edge Class
class Edge():
    def __init__(self, w, src, d):
        self.way = w
        self.dest = d
        self.cost = distance(src,d)

# Path making Class
class Path():
    def __init__(self,n,w):
        self.nodes = n
        self.ways = w

    # Heuristic Function
    def heuristic(self,node,gnode):
        dist = distance(node, gnode)
        elev = elevation(node, gnode)

        if elev > 0:
            return dist * 100
        elif elev < 0:
            return dist * .01
        elif elev == 0:
            return dist

    # A* Implementation
    def a_star(self,start,goal):

        # Visited setup
        visited = {}
        visited[start] = None
        
        # Frontier setup
        frontier = Queue()
        frontier.put((self.heuristic(start,goal),start))
        
        # Cost setup
        costs = {}
        costs[start] = 0

        while not frontier.empty():
            curr, curr = frontier.get()
            if curr == goal:
                print "Estimated time is " + str(round(costs[goal]*60/5000, 2)) + " minutes." #5 km/hr on flat
                return self.make_path(visited,goal)
            for edge in curr.ways:
                newcost = costs[curr] + edge.cost
                if edge.dest not in visited or newcost < costs[edge.dest]:
                    visited[edge.dest] = (curr, edge.way)
                    costs[edge.dest] = newcost
                    frontier.put((self.heuristic(edge.dest,goal)+newcost,edge.dest))

    # Prepares selected path
    def make_path(self,visit,goal):
        nodes = []
        ways = []
        current = goal
        nodes.append(current)
        while visit[current] is not None:
            prev, way = visit[current]
            ways.append(way.name)
            nodes.append(prev)
            current = prev
        nodes.reverse()
        ways.reverse()
        return nodes,ways

# Window Class
class MyWin(Frame):
    def __init__(self,master,nodes,ways,elevs):
        self.array = {}
        self.nodes = nodes
        self.ways = ways
        self.elevs = elevs
        self.startnode = None
        self.goalnode = None
        self.path = Path(nodes,ways)
        thewin = Frame(master)
        
        w = Canvas(thewin, width=WINWIDTH, height=WINHEIGHT, cursor="crosshair")
        w.bind("<Button-1>", self.mapclick)
        w.bind("<Motion>", self.maphover)

        # Create map
        for way in self.ways:
            nlist = self.ways[way].nodes
            this = self.coord_convert(self.nodes[nlist[0]].pos)
            if len(self.nodes[nlist[0]].ways) > 2:
                self.array[((int)(this[0]),(int)(this[1]))] = nlist[0]
            for n in range(len(nlist)-1):
                next_val = self.coord_convert(self.nodes[nlist[n+1]].pos)
                self.array[((int)(next_val[0]),(int)(next_val[1]))] = nlist[n+1]
                w.create_line(this[0],this[1],next_val[0],next_val[1])
                this = next_val

        # GUI Elements (Path and Nodes)
        w.create_line(0,0,0,0,fill='blue',width=2,tag='path')
        w.create_oval(0,0,0,0,fill='black',tag='start')
        w.create_oval(0,0,0,0,fill='white',tag='goal')
        w.pack(fill=BOTH)
        self.canvas = w

        thewin.pack()

    # Keep track of last node
    def maphover(self,event):
        for (dx,dy) in [(0,0),(-1,0),(0,-1),(1,0),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
            check = (event.x+dx,event.y+dy)
            if check in self.array:
                self.lastnode = self.array[check]
                last = self.coord_convert(self.nodes[self.lastnode].pos)
                self.canvas.coords('lastdot',(last[0]-2,last[1]-2,last[0]+2,last[1]+2))

    # Clearing function
    def clear(self):
        print "Clearing screen"
        self.goalnode = None
        self.startnode = None
        self.canvas.coords('start',(0,0,0,0))
        self.canvas.coords('goal',(0,0,0,0))
        self.canvas.coords('path',(0,0,0,0))

    # Onclick function
    def mapclick(self,event):
        
        # If both are selected, clear map
        if self.startnode and self.goalnode is not None:
            self.clear()
        
        # If start node doesnt exist, selected node will become start node
        if self.startnode is None:
            self.startnode = self.nodes[self.lastnode]
            value = self.coord_convert(self.startnode.pos)
            self.canvas.coords('start',(value[0]-2,value[1]-2,value[0]+2,value[1]+2))
        
        # If goal node doesn't exist, selected node will become goal node
        elif self.goalnode is None:
            self.goalnode = self.nodes[self.lastnode]
            value = self.coord_convert(self.goalnode.pos)
            self.canvas.coords('goal',(value[0]-2,value[1]-2,value[0]+2,value[1]+2))
            self.route()

    def route(self):
        # Test difference between start to goal and goal to start trips using elevation
        nodes,ways = self.path.a_star(self.goalnode, self.startnode)

        nodes,ways = self.path.a_star(self.startnode, self.goalnode)
        lastway = ""
        for wayname in ways:
            if wayname != lastway:
                lastway = wayname
        coords = []
        for node in nodes:
            npos = self.coord_convert(node.pos)
            coords.append(npos[0])
            coords.append(npos[1])

        self.canvas.coords('path',*coords)

    # Finds coordinates compared to location on window
    def coord_convert(self,latlon):
        x = (latlon[1]-LEFTBOUND)*(WINWIDTH/(abs(LEFTBOUND)-abs(RIGHTBOUND)))
        y = (TOPBOUND-latlon[0])*(WINHEIGHT/(TOPBOUND-BOTBOUND))
        return x,y

# Calculate distance between 2 nodes
def distance(node1, node2):
    distx = (node2.pos[0]-node1.pos[0])*MPERLON
    disty = (node2.pos[1]-node1.pos[1])*MPERLAT
    return math.sqrt(distx*distx+disty*disty)

def elevation(node1, node2):
    elev = node2.elev - node1.elev 
    return elev

# Read BIL File
def read_elevations(efilename):
    efile = open(efilename)
    estr = efile.read()
    elevs = []
    for spot in range(0,len(estr),2):
        elevs.append(struct.unpack('<h',estr[spot:spot+2])[0])
    return elevs

# Read OSM File
def read_xml(filename, elevs):

    tree = ET.parse(filename)
    root = tree.getroot()

    nodes = dict()
    ways = dict()
    waytypes = set()

    for item in root:
        if item.tag == 'node':
            coords = ((float)(item.get('lat')),(float)(item.get('lon')))
            row = (int)(44 - coords[0]) * 1201
            col = (int)(1-(abs(coords[1]))+78) * 1201
            elevation = elevs[(row*1201)+col]
            nodes[(long)(item.get('id'))] = Node(coords,elevation)            
        

        elif item.tag == 'way':
            oneway = False
            nodename = 'Generic road'
            for subitem in item:
                if subitem.tag == 'tag' and subitem.get('k') == 'highway':
                    nodetype = subitem.get('v')
                    wayid = (long)(item.get('id'))
                    ways[wayid] = Way(nodename,nodetype)
                    nlist = []
                    for subitem in item:
                        if subitem.tag == 'nd':
                            nlist.append((long)(subitem.get('ref')))
                    thisn = nlist[0]
                    for n in range(len(nlist)-1):
                        nextn = nlist[n+1]
                        nodes[thisn].ways.append(Edge(ways[wayid],nodes[thisn],nodes[nextn]))
                        thisn = nextn
                    if not oneway:
                        thisn = nlist[-1]
                        for n in range(len(nlist)-2,-1,-1):
                            nextn = nlist[n]
                            nodes[thisn].ways.append(Edge(ways[wayid],nodes[thisn],nodes[nextn]))
                            thisn = nextn                
                    ways[wayid].nodes = nlist
                if subitem.tag == 'tag' and subitem.get('k') == 'name':
                    nodename = subitem.get('v')
                if subitem.tag == 'tag' and subitem.get('k') == 'oneway':
                    oneway = True
    return nodes,ways

def main():
    print "Reading BIL"
    elevs = read_elevations("n43_w079_3arc_v2.bil")
    print "Reading XML"
    nodes,ways = read_xml("dbv.osm", elevs)

    master = Tk()
    master.title("Assignment One")
    thewin = MyWin(master,nodes,ways,elevs)
    print "\nInstructions:"
    print "Click two locations to find path between two nodes. Click new location to clear previous path\n"

    mainloop()

if __name__ == "__main__":
    main()