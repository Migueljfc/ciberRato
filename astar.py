from heapq import *
from math import dist

#dist used as heuristic


def astar(start, goal,array,walls):

    neighbors = [(0,2),(0,-2),(2,0),(-2,0)]

    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:dist(start, goal)}
    oheap = []

    heappush(oheap, (fscore[start], start))
    
    while oheap:

        current = heappop(oheap)[1]

        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return data

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j            
            tentative_g_score = gscore[current] + dist(current, neighbor)
            
            if neighbor not in array:
                continue

            # Make sure walkable terrain
            if i == 2 and ((current[0]+1,current[1]) in walls):
                continue
            if i == -2 and ((current[0]-1,current[1]) in walls):
                continue
            if j == 2 and ((current[0],current[1]+1) in walls):
                continue
            if j == -2 and ((current[0],current[1]-1) in walls):
                continue

                
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
                
            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + dist(neighbor, goal)
                heappush(oheap, (fscore[neighbor], neighbor))
           
    return None