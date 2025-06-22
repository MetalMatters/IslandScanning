# pathfinding.py
import math
try:
    import networkx as nx
    NETWORKX_AVAILABLE = True
except ImportError:
    NETWORKX_AVAILABLE = False
    # Main script handles NETWORKX_AVAILABLE check.

try:
    from shapely.geometry import Point, LineString
    from shapely.ops import unary_union
    SHAPELY_AVAILABLE_FOR_PATHFINDING = True # Local check if needed
except ImportError:
    SHAPELY_AVAILABLE_FOR_PATHFINDING = False

# Import utility functions like are_points_close and dist_sq
from . import gcode_parser # Use relative import
from .geometry_operations import geometry_hash

def build_boundary_graph(clipped_cells_data, cfg_coord_epsilon):
    if not NETWORKX_AVAILABLE or not SHAPELY_AVAILABLE_FOR_PATHFINDING:
        # print("NetworkX or Shapely not available in pathfinding.build_boundary_graph")
        return None # Return None if dependencies not met
    
    G = nx.Graph()
    all_boundaries = []
    for cell_data in clipped_cells_data:
        poly = cell_data['shapely_poly_clipped']
        if poly.boundary.geom_type == 'MultiLineString':
            all_boundaries.extend(list(poly.boundary.geoms)) # Use .geoms
        elif poly.boundary.geom_type == 'LineString': 
            all_boundaries.append(poly.boundary)
            
    if not all_boundaries: return G 
    try:
        unioned_boundaries = unary_union(all_boundaries)
    except Exception: # Handle issues with unary_union
        # print("Warning: Unary union of boundaries failed. Graph might be incomplete.")
        return G # Return empty graph or handle error as appropriate
        
    if unioned_boundaries.is_empty: return G

    line_segments_to_process = []
    if unioned_boundaries.geom_type == 'MultiLineString':
        line_segments_to_process = list(unioned_boundaries.geoms) # Use .geoms
    elif unioned_boundaries.geom_type == 'LineString':
        line_segments_to_process = [unioned_boundaries]

    for segment in line_segments_to_process:
        if segment.geom_type == 'LineString' and segment.length > cfg_coord_epsilon:
            coords = list(segment.coords)
            for i in range(len(coords) - 1):
                p1_t = (coords[i][0], coords[i][1]) # Ensure tuple for hashability
                p2_t = (coords[i+1][0], coords[i+1][1])
                
                # Add edge with length as weight
                G.add_edge(p1_t, p2_t, weight=math.sqrt(gcode_parser.dist_sq(p1_t, p2_t)))
    return G

def _add_point_to_graph_robustly(point_coords, graph_obj,
                                 cfg_node_merge_epsilon, cfg_on_segment_epsilon,
                                 cfg_endpoint_distinction_epsilon,
                                 cfg_coord_epsilon, cfg_grid_cell_size):
    if not NETWORKX_AVAILABLE or not SHAPELY_AVAILABLE_FOR_PATHFINDING: return None
        
    point_t = (point_coords[0], point_coords[1])

    # 1. Check if point_t is already an existing node (or very close)
    for node in list(graph_obj.nodes()): 
        if gcode_parser.are_points_close(point_t, node, cfg_node_merge_epsilon): 
            return node 

    # 2. If not an existing node, try to find the closest segment it lies on and split it
    candidate_edge_split = None 
    min_dist_to_segment_line = float('inf')

    for u, v in list(graph_obj.edges()): 
        segment_line = LineString([u, v])
        
        if segment_line.length > cfg_coord_epsilon: 
            projected_frac = segment_line.project(Point(point_t))
            
            if -cfg_coord_epsilon < projected_frac < segment_line.length + cfg_coord_epsilon:
                proj_point_on_segment = segment_line.interpolate(projected_frac).coords[0]
                current_dist_to_proj = math.sqrt(gcode_parser.dist_sq(point_t, proj_point_on_segment))

                if current_dist_to_proj < cfg_on_segment_epsilon: 
                    if current_dist_to_proj < min_dist_to_segment_line:
                        min_dist_to_segment_line = current_dist_to_proj
                        proj_point_t_candidate = (proj_point_on_segment[0], proj_point_on_segment[1])
                        candidate_edge_split = (u, v, proj_point_t_candidate, current_dist_to_proj)

    if candidate_edge_split:
        u, v, proj_point_t, current_dist_to_proj = candidate_edge_split
        
        final_proj_node = proj_point_t
        for existing_node in list(graph_obj.nodes()):
            if gcode_parser.are_points_close(proj_point_t, existing_node, cfg_node_merge_epsilon):
                final_proj_node = existing_node 
                break
        
        if final_proj_node == proj_point_t and not graph_obj.has_node(final_proj_node):
             graph_obj.add_node(final_proj_node)

        if not gcode_parser.are_points_close(final_proj_node, u, cfg_endpoint_distinction_epsilon) and \
           not gcode_parser.are_points_close(final_proj_node, v, cfg_endpoint_distinction_epsilon):
                        
            if graph_obj.has_edge(u, v):
                graph_obj.remove_edge(u, v)

            graph_obj.add_edge(u, final_proj_node, weight=math.sqrt(gcode_parser.dist_sq(u, final_proj_node)))
            graph_obj.add_edge(final_proj_node, v, weight=math.sqrt(gcode_parser.dist_sq(final_proj_node, v)))
            
            graph_obj.add_node(point_t) 
            graph_obj.add_edge(point_t, final_proj_node, weight=current_dist_to_proj)
            return point_t 
        else:
            target_node_on_boundary = u if gcode_parser.are_points_close(final_proj_node, u, cfg_endpoint_distinction_epsilon) else v
            graph_obj.add_node(point_t)
            graph_obj.add_edge(point_t, target_node_on_boundary, weight=current_dist_to_proj)
            return point_t

    # 3. Fallback: connect to the closest existing node
    closest_node_in_graph = None
    min_dist_to_any_node = float('inf')
    for node in graph_obj.nodes():
        dist_val = math.sqrt(gcode_parser.dist_sq(point_t, node))
        if dist_val < min_dist_to_any_node:
            min_dist_to_any_node = dist_val
            closest_node_in_graph = node
    
    if closest_node_in_graph and min_dist_to_any_node < cfg_grid_cell_size * 2.5: # Heuristic distance limit 
        graph_obj.add_node(point_t)
        graph_obj.add_edge(point_t, closest_node_in_graph, weight=min_dist_to_any_node)
        return point_t
    
    return None 

def find_shortest_path_on_boundary(graph, start_point, end_point,
                                   cfg_node_merge_epsilon, cfg_on_segment_epsilon,
                                   cfg_endpoint_distinction_epsilon,
                                   cfg_coord_epsilon, cfg_grid_cell_size):
    if not NETWORKX_AVAILABLE or not SHAPELY_AVAILABLE_FOR_PATHFINDING: return None
    if not graph or start_point is None or end_point is None:
        return None

    if gcode_parser.are_points_close(start_point, end_point, cfg_coord_epsilon * 2): # If points are very close, direct line
        return LineString([start_point, end_point])

    temp_graph = graph.copy() 

    connected_start_node = _add_point_to_graph_robustly(
        start_point, temp_graph,
        cfg_node_merge_epsilon, cfg_on_segment_epsilon, cfg_endpoint_distinction_epsilon,
        cfg_coord_epsilon, cfg_grid_cell_size
    )
    connected_end_node = _add_point_to_graph_robustly(
        end_point, temp_graph,
        cfg_node_merge_epsilon, cfg_on_segment_epsilon, cfg_endpoint_distinction_epsilon,
        cfg_coord_epsilon, cfg_grid_cell_size
    )

    if connected_start_node is None or connected_end_node is None:
        return None

    try:
        path_nodes = nx.dijkstra_path(temp_graph, source=connected_start_node, target=connected_end_node, weight='weight')
        if len(path_nodes) > 1:
            return LineString(path_nodes) # Path found
    except nx.NetworkXNoPath:
        # print(f"Debug: No path found between {connected_start_node} and {connected_end_node} in temp_graph.")
        return None # No path exists
    except Exception as e:
        print(f"Error during boundary pathfinding: {e}. Skipping connection.")
        return None # Other error
        
    return None # Should not be reached if logic is correct

class RoutingCache:
    def __init__(self):
        self.cache = {}

    def get_routing(self, cell_id):
        return self.cache.get(cell_id)

    def update_routing(self, cell_id, infill_segments):
        self.cache[cell_id] = infill_segments

class ConnectionCache:
    def __init__(self):
        self.cache = {}

    def get_connection(self, from_id, to_id):
        return self.cache.get((from_id, to_id))

    def update_connection(self, from_id, to_id, path):
        self.cache[(from_id, to_id)] = path