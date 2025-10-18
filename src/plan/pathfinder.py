import heapq
from src.floor import * # Import all the sector types

class Node:
    """A blueprint for a node in the A* search grid."""
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position
    
    def __lt__(self, other):
        return self.f < other.f

def find_optimal_path(sector_map_snapshot, start_coord, end_coord):
    """
    Returns a list of coordinates as a path from the given start to the given end.
    """
    start_node = Node(None, start_coord)
    end_node = Node(None, end_coord)

    open_list = []
    closed_list = []
    heapq.heappush(open_list, (start_node.f, start_node))

    # --- REFINEMENT 1: Get map dimensions ONCE ---
    map_data = sector_map_snapshot._SectorMapSnapshot__sector_map._SectorMap__data
    map_height = len(map_data)
    map_width = len(map_data[0])

    while len(open_list) > 0:
        current_node = heapq.heappop(open_list)[1]

        if current_node in closed_list:
            continue
        
        closed_list.append(current_node)

        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1]
        
        children = []
        current_pos = current_node.position
        current_sector = sector_map_snapshot._SectorMapSnapshot__sector_map.get_sector(current_pos)
        
        possible_moves = []
        # --- REFINEMENT 2: Refined Rule Logic ---
        if isinstance(current_sector, Buffer_Zone_Sector):
            possible_moves = [(0, -1), (0, 1)]  # Up, Down
        elif isinstance(current_sector, Highway_Left_Sector):
            possible_moves = [(-1, 0), (0, -1), (0, 1)] # Left, Up, Down
        elif isinstance(current_sector, Highway_Right_Sector):
            possible_moves = [(1, 0), (0, -1), (0, 1)]  # Right, Up, Down
        else: # For complex zones like QC/Yard, trust the built-in function
            valid_next_coords = sector_map_snapshot.get_moveable_to_coordinates(current_pos)
            if valid_next_coords:
                for coord in valid_next_coords:
                    possible_moves.append((coord.x - current_pos.x, coord.y - current_pos.y))

        for move in possible_moves:
            next_pos = Coordinate(current_pos.x + move[0], current_pos.y + move[1])
            
            if not (0 <= next_pos.y < map_height and 0 <= next_pos.x < map_width):
                continue
            
            occupators = sector_map_snapshot.get_occupators(next_pos)
            if occupators:
                continue

            new_node = Node(current_node, next_pos)
            children.append(new_node)

        for child in children:
            if child in closed_list:
                continue

            child.g = current_node.g + 1
            child.h = abs(child.position.x - end_node.position.x) + abs(child.position.y - end_node.position.y)
            child.f = child.g + child.h

            if any(open_node for open_node in open_list if child == open_node[1] and child.g > open_node[1].g):
                continue
            
            heapq.heappush(open_list, (child.f, child))
            
    return None