import math
import heapq
import scipy.spatial

def distance_of(from_node, to_node, map):
    return scipy.spatial.distance.euclidean(map.intersections[from_node], map.intersections[to_node])

def shortest_path(map, start, goal):    
    if (start == goal):     # if we start at the goal vertex, we just need to return that node, and we are done
        return [start]   
 
    paths = []              # this list will hold the active paths with the explored nodes per route
    solutions = []          # a list of successfull set of solution paths from start to goal
    
    # We will update the "Best Overall G" of our solutions, whenever a successfull solution with smaller G comes up
    # the reason is that if we have already successfull solutions, if our expansions produce higher G+H results than
    # our best overall G score, then we can stop our algorithm
    best_overall_g = math.inf
    
    
    # the visited nodes will be a list of
    # [g+h, g, [nodes_visited]]
    
    # the first one, h+g, will be used for measuring which node has the smaller overall score, so it gets selected
    
    # At first node, for distance we get the euclidian distance of start to goal (h), our g is 0 because its the first node
    heapq.heappush(paths, [0, 0, [start], {start:1}])

    # keep exploring...
    while (True):
    
        # get the shortest cost path from the min heap with the lowest distance penalty
        # that is pull the BEST path so far from the min heap
        shortest_path = heapq.heappop(paths)

        
        cur_node = shortest_path[2][len(shortest_path[2])-1]  # the last intersection (node) in our path
        cur_path = shortest_path[2]                           # the list with the full path from the first to the current node
        gh       = shortest_path[0]                           # the G+H score for the last intersection
        g        = shortest_path[1]                           # the G for our path from the first to the last intersection
        

        # if the currently visiting node is the goal node, we have a match!
        if cur_node == goal:                          
            # add to the solutions min heap the solution path, together with its G for sorting
            heapq.heappush(solutions, [g, cur_path])  

            # if this is indeed the best G score of all solutions, save the score... this will be useful later
            if best_overall_g > g:
                best_overall_g = g
            continue


        # we will need to keep track of the best G+H in our expansion...
        # so if we already have a solution with cheaper overall G+H than our current expansion
        # then we can terminate the algorithm, as there is an optimum solution
        best_local_gh = math.inf

        # for all the edges (roads) coming out of this node (intersection) in our adjacency list...
        for next_node in map.roads[cur_node]:
            if (next_node in cur_path): # do not explore already visited nodes
                continue

            # "H" is the heuristic distance between our next node, to the final node
            h = distance_of(next_node, goal, map)
            
            # the "new G" will be the G of our preious path, plus the distance between our current and next node
            new_g = g + distance_of(cur_node, next_node, map)

            # the updated path with be a copy of our existing path, plus the next visiting node
            updated_path = cur_path.copy()
            updated_path.append(next_node)
            
            # We will add a new record to our paths heap, with the new G, H, and the updated path
            path_record = [new_g+h, new_g, updated_path]      
            heapq.heappush(paths, path_record)
            
            if best_local_gh > new_g+h:
                best_local_gh = new_g+h
        
        # if there is at least one solution, and from all the above explorations we ended up with
        # G+H scores, higher than the final G score, means that there is no better solution, so we break
        if (len(solutions)>=1 and best_local_gh > best_overall_g):
            break;
    
    # return the smallest G scored solution from our heap
    return heapq.heappop(solutions)[1]