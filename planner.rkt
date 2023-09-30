#lang dssl2

import cons
import "project-lib/graph.rkt"
import "project-lib/dictionaries.rkt"
import "project-lib/binheap.rkt"
import "project-lib/stack-queue.rkt"

struct kons:
   let pos
   let data

### Basic Types ###

#  - Latitudes and longitudes are numbers:
let Lat?  = num?
let Lon?  = num?

#  - Point-of-interest categories and names are strings:
let Cat?  = str?
let Name? = str?

### Raw Item Types ###

#  - Raw positions are 2-element vectors with a latitude and a longitude
let RawPos? = TupC[Lat?, Lon?]

#  - Raw road segments are 4-element vectors with the latitude and
#    longitude of their first endpoint, then the latitude and longitude
#    of their second endpoint
let RawSeg? = TupC[Lat?, Lon?, Lat?, Lon?]

#  - Raw points-of-interest are 4-element vectors with a latitude, a
#    longitude, a point-of-interest category, and a name
let RawPOI? = TupC[Lat?, Lon?, Cat?, Name?]

### Contract Helpers ###

# ListC[T] is a list of `T`s (linear time):
let ListC = Cons.ListC
# List of unspecified element type (constant time):
let List? = Cons.list?


interface TRIP_PLANNER:

    # Returns the positions of all the points-of-interest that belong to
    # the given category.
    def locate_all(
            self,
            dst_cat:  Cat?           # point-of-interest category
        )   ->        ListC[RawPos?] # positions of the POIs

    # Returns the shortest route, if any, from the given source position
    # to the point-of-interest with the given name.
    def plan_route(
            self,
            src_lat:  Lat?,          # starting latitude
            src_lon:  Lon?,          # starting longitude
            dst_name: Name?          # name of goal
        )   ->        ListC[RawPos?] # path to goal

    # Finds no more than `n` points-of-interest of the given category
    # nearest to the source position.
    def find_nearby(
            self,
            src_lat:  Lat?,          # starting latitude
            src_lon:  Lon?,          # starting longitude
            dst_cat:  Cat?,          # point-of-interest category
            n:        nat?           # maximum number of results
        )   ->        ListC[RawPOI?] # list of nearby POIs


class TripPlanner (TRIP_PLANNER):
    let ROADs: VecC[RawSeg?] #the road segments
    let POIs: VecC[RawPOI?] #the points of interest
    let Graph: WuGraph? #graph of planner
    let id_pos: AssociationList? #dictionary of mapping position to unique number
    let pos_id: AssociationList? #dictionary of unique number to mapping position
    let neighbors: VecC[ListC[num?]] # neighbors of each position in graph
    
    def __init__(self, road_segments, Points_of_interest):
        self.ROADs = road_segments
        self.POIs = Points_of_interest
        self.Graph = WuGraph(road_segments.len() * 2)
        self.id_pos = AssociationList()
        self.pos_id = AssociationList()
        let count = 0
        let first = True
        for road in road_segments:
            if self.id_pos.mem?([road[0], road[1]]) == False:
                if first != True: count = count + 1
                self.id_pos.put([road[0], road[1]], count)
                self.pos_id.put(count, [road[0], road[1]])
                first = False
            if self.id_pos.mem?([road[2], road[3]]) == False:
                count = count + 1
                self.id_pos.put([road[2], road[3]], count)
                self.pos_id.put(count, [road[2], road[3]])
            self.Graph.set_edge(self.id_pos.get([road[0], road[1]]), self.id_pos.get([road[2], road[3]]), self.find_dist(road))
        self.neighbors = [None; count + 1]
        for pos in range(count + 1):
            self.neighbors[pos] = self.Graph.get_adjacent(pos)

        
    def locate_all(self, dst_cat: Cat?)-> ListC[RawPos?]:
        let result = AssociationList() #keeps track of visited pos
        let track = None # the final list of pois with no duplicates
        for poi in self.POIs: 
            if poi[2] == dst_cat and result.mem?([poi[0], poi[1]]) == False:
                result.put([poi[0], poi[1]], 0)
                track = cons([poi[0], poi[1]], track)
        return track

        
    def plan_route(self, src_lat: Lat?, src_lon: Lon?, dst_name: Name?):
        let nodes_dist = BinHeap[kons?](self.Graph.len(), λ x, y: x.data < y.data)
        let start = self.id_pos.get([src_lat, src_lon])
        let valid = False
        let dest
        for poi in self.POIs: #assert the destination is a poi else its a non_existant destination
            if poi[3] == dst_name:
                dest = self.id_pos.get([poi[0], poi[1]])
                valid = True
                break
        if valid == False: return None
        let visited = [False; self.id_pos.len()]
        let pred =  [None; self.id_pos.len()]
        let dist = [inf; self.id_pos.len()]
        dist[start] = 0
        nodes_dist.insert(kons(start, 0))
        let curr
        while nodes_dist.len() != 0: #djikstras
            curr = nodes_dist.find_min()
            nodes_dist.remove_min()
            if visited[curr.pos] == False:
                visited[curr.pos] = True
                let neybor = self.neighbors[curr.pos]
                while neybor != None:
                    if curr.data + self.Graph.get_edge(curr.pos, neybor.data) < dist[neybor.data]:
                        dist[neybor.data] = curr.data + self.Graph.get_edge(curr.pos, neybor.data)
                        pred[neybor.data] = curr.pos
                        nodes_dist.insert(kons(neybor.data, dist[neybor.data]))
                        neybor = neybor.next
                    else:
                        neybor = neybor.next
        let result = None # set final path by traversing from dest back to start else if not possible its unreachable                 
        curr = dest
        while curr != start and curr != None:
            result = cons(self.pos_id.get(curr), result)
            curr = pred[curr]
        if curr == None: 
           return None
        else:
            result = cons(self.pos_id.get(curr), result)
            return result
            
         
        
        
        
        
    def find_nearby(self, src_lat: Lat?, src_lon: Lon?, dst_cat: Cat?, n: nat?):
        let poi_s = self.locate_all(dst_cat)
        let nodes_dist = BinHeap[kons?](self.Graph.len(), λ x, y: x.data < y.data)
        let start = self.id_pos.get([src_lat, src_lon])
        let pos_dist = AssociationList()
        let pos_poi = AssociationList()
        let postn = poi_s
        while postn != None :#initialize position to poi dictionary and poi to distance from source dictionary
            for i in range(self.POIs.len()):
                if postn.data == [self.POIs[i][0], self.POIs[i][1]] and self.POIs[i][2] == dst_cat:
                   pos_poi.put(self.id_pos.get(postn.data), self.POIs[i])
                   break
            if postn == [src_lat, src_lon]:
               pos_dist.put(self.id_pos.get(postn.data), 0)
               postn = postn.next 
            pos_dist.put(self.id_pos.get(postn.data), inf)
            postn = postn.next 
        let visited = [False; self.id_pos.len()]
        let pred =  [None; self.id_pos.len()]
        let dist = [inf; self.id_pos.len()]
        dist[start] = 0
        nodes_dist.insert(kons(start, 0))
        let curr
        while nodes_dist.len() != 0: # djikstras with specific distances for required pois
            curr = nodes_dist.find_min()
            nodes_dist.remove_min()
            if visited[curr.pos] == False:
                visited[curr.pos] = True
                let neybor = self.neighbors[curr.pos]
                while neybor != None:
                    if curr.data + self.Graph.get_edge(curr.pos, neybor.data) < dist[neybor.data]:
                        dist[neybor.data] = curr.data + self.Graph.get_edge(curr.pos, neybor.data)
                        if pos_dist.mem?(neybor.data):
                           pos_dist.put(neybor.data, curr.data + self.Graph.get_edge(curr.pos, neybor.data))
                        pred[neybor.data] = curr.pos
                        nodes_dist.insert(kons(neybor.data, dist[neybor.data]))
                        neybor = neybor.next
                    else:
                        neybor = neybor.next             
        let sort_dist = BinHeap[kons?](pos_dist.len(), λ x, y: x.data < y.data)
        postn = poi_s
        let dest
        if pos_dist.mem?(start): pos_dist.put(start, 0)
        while postn != None:# assert reachability before adding to sorting heap
            dest = self.id_pos.get(postn.data)
            while dest != start and dest != None:
               dest = pred[dest]
            if dest == start:
               sort_dist.insert(kons(self.id_pos.get(postn.data), pos_dist.get(self.id_pos.get(postn.data))))
               postn = postn.next
            else:
               postn = postn.next     
        let result = None #prepare result  by taking top n pois in heap
        for i in range(n):
            if sort_dist.len() != 0:
               result = cons(pos_poi.get(sort_dist.find_min().pos), result)
               sort_dist.remove_min()
            else:
                break
        if result != None:
           return result
        else: 
            return None

    def find_dist(self, segment: RawSeg?)->num?:
        let length = (((segment[0] - segment[2])**2)+((segment[1] - segment[3])**2))**0.5
        return length
        
#   ^ YOUR CODE GOES HERE 


def my_first_example():
    return TripPlanner([[0,0, 0,1], [0,0, 1,0]],
                       [[0,0, "bar", "The Empty Bottle"],
                        [0,1, "food", "Pierogi"]])

test 'My first locate_all test':
    assert my_first_example().locate_all("food") == \
        cons([0,1], None)

test 'My first plan_route test':
    assert my_first_example().plan_route(0, 0, "Pierogi") == \
       cons([0,0], cons([0,1], None))

test 'My first find_nearby test':
    assert my_first_example().find_nearby(0, 0, "food", 1) == \
       cons([0,1, "food", "Pierogi"], None)
       
let eight_principles = ["Know your rights.", "Acknowledge your sources.", "Protect your work.", "Avoid suspicion.", "Do your own work.", "Never falsify a record or permit another person to do so.", "Never fabricate data, citations, or experimental results.", "Always tell the truth when discussing your work with your instructor."]
for strng in  eight_principles:
    print(strng)