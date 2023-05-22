import heapq
import random

class Paths:
    turnLeft = 15
    turnRight = 15
    straight = 10

    graph = {
        'A' : {'E' : turnLeft},
        'B' : {'D' : straight, 'G' : turnRight},
        'C' : {'A' : straight, 'G' : turnLeft},
        'D' : {'I' : turnRight},
        'E' : {'L' : turnLeft, 'O' : straight},
        'F' : {'B' : turnRight},
        'G' : {'K' : turnRight, 'N' : turnLeft, 'Q' : straight},
        'H' : {'A' : turnLeft, 'D' : turnRight},
        'I' : {'M' : turnRight, 'S' : straight},
        'J' : {'C' : turnLeft},
        'K' : {'F' : turnRight, 'O' : turnLeft},
        'L' : {'H' : turnLeft,  'Q' : turnRight}, # 'N' : straight,  deleted just for now
        'M' : {'H' : turnRight, 'K' : straight, 'Q' : turnLeft},
        'N' : {'J' : turnLeft, 'S' : turnRight},
        'O' : {'V' : turnLeft},
        'P' : {'F' : straight, 'L' : turnRight},
        'Q' : {'U' : turnRight, 'X' : turnLeft},
        'R' : {'H' : straight, 'K' : turnLeft, 'N' : turnRight},
        'S' : {'W' : turnRight},
        'T' : {'J' : straight, 'M' : turnLeft},
        'U' : {'P' : turnRight},
        'V' : {'R' : turnLeft, 'X' : straight},
        'W' : {'R' : turnRight, 'U' : straight},
        'X' : {'T' : turnLeft}
    }

    dirGraph = {
        'A' : {'E' : "turnLeft"},
        'B' : {'D' : "straight", 'G' : "turnRight"},
        'C' : {'A' : "straight", 'G' : "turnLeft"},
        'D' : {'I' : "turnRight"},
        'E' : {'L' : "turnLeft", 'O' : "straight"},
        'F' : {'B' : "turnRight"},
        'G' : {'K' : "turnRight", 'N' : "turnLeft", 'Q' : "straight"},
        'H' : {'A' : "turnLeft", 'D' : "turnRight"},
        'I' : {'M' : "turnRight", 'S' : "straight"},
        'J' : {'C' : "turnLeft"},
        'K' : {'F' : "turnRight", 'O' : "turnLeft"},
        'L' : {'H' : "turnLeft", 'N' : "straight", 'Q' : "turnRight"},
        'M' : {'H' : "turnRight", 'K' : "straight", 'Q' : "turnLeft"},
        'N' : {'J' : "turnLeft", 'S' : "turnRight"},
        'O' : {'V' : "turnLeft"},
        'P' : {'F' : "straight", 'L' : "turnRight"},
        'Q' : {'U' : "turnRight", 'X' : "turnLeft"},
        'R' : {'H' : "straight", 'K' : "turnLeft", 'N' : "turnRight"},
        'S' : {'W' : "turnRight"},
        'T' : {'J' : "straight", 'M' : "turnLeft"},
        'U' : {'P' : "turnRight"},
        'V' : {'R' : "turnLeft", 'X' : "straight"},
        'W' : {'R' : "turnRight", 'U' : "straight"},
        'X' : {'T' : "turnLeft"}
    }

    def randRoad(self, curNode): # Function selects random road for mode 2 taxi that is at an intersaction.
        dic = Paths.dirGraph[curNode]
        arr = list(dic.keys())
        length = len(dic)
        rand = random.randint(1, length)
        nextNode = arr[rand-1]
        nextNodeInt = ord(nextNode)-65
        return nextNode, nextNodeInt, dic[nextNode]# ex) 'A', 65, turnRight
    
    def nextRoad(self, curNode, curPath): # A, AEO
        index_of_current_road = curPath.index(curNode) #0
        nextNode = curPath[index_of_current_road+1] # ex) E
        dic = Paths.dirGraph[curNode] # {'E' : "turnLeft"}
        nextNodeInt = ord(nextNode)-65
        return nextNode, nextNodeInt, dic[nextNode]# ex) 'E', 4, turnLeft

    def GPP(self, start, end): # Global path planning algorithm. dijkstra used for this.
        # Create a dictionary to store the distance of each node from the starting node
        distances = {node: float('inf') for node in Paths.graph}
        distances[start] = 0

        # Create a dictionary to store the shortest path to each node
        paths = {start: []}

        # Create a priority queue to store nodes to visit
        pq = [(0, start)]

        while pq:
            # Get the node with the shortest distance
            (distance, current_node) = heapq.heappop(pq)

            # If we have found the end node, return the shortest path
            if current_node == end:
                return paths[current_node] + [current_node]

            # Otherwise, update the distances and paths to neighboring nodes
            for neighbor, weight in Paths.graph[current_node].items():
                new_distance = distance + weight
                if new_distance < distances[neighbor]:
                    distances[neighbor] = new_distance
                    paths[neighbor] = paths[current_node] + [current_node]
                    heapq.heappush(pq, (new_distance, neighbor))

        # If we reach here, there is no path from start to end
        return None

    def directions(self, path): # Tells you which way to go ex) right, left, straight
        length = len(path)
        queue = []
        for i in range(0, length - 1):
            curNode = path[i]
            nextNode = path[i+1]
            dir = Paths.dirGraph[curNode][nextNode]
            queue.append(dir)
        return queue

# test case
"""
path = Paths()
curPath = path.GPP('A', 'G')
print(curPath)

direct = path.directions(curPath)
print(direct)
"""