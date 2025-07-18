import math
import hashlib
try:
    from shapely.geometry import Polygon, MultiPolygon, Point, LineString
    from shapely.ops import unary_union
    from shapely.validation import make_valid
    from shapely.affinity import rotate
    SHAPELY_AVAILABLE = True
except ImportError:
    SHAPELY_AVAILABLE = False
    # This module should not be used if Shapely is not available.
    # The main script handles the SHAPELY_AVAILABLE check.

# Import from gcode_parser for utility functions if needed directly
from . import gcode_parser # Use relative import if this is part of a package

def identify_candidate_regions_shapely(layer_wall_loops, cfg_min_area_thresh, cfg_coord_epsilon):
    if not SHAPELY_AVAILABLE:
        # This print might be redundant if main_script already checks and exits/warns
        # print("Shapely not available in geometry_operations.identify_candidate_regions_shapely")
        return []
    if not layer_wall_loops:
        return []

    shapely_polys_data = []
    processed_loop_indices = set()

    for i, loop_pts in enumerate(layer_wall_loops):
        if i in processed_loop_indices or len(loop_pts) < 3:
            continue
        current_polys_from_loop = []
        try:
            poly = Polygon(loop_pts)
            if not poly.is_valid:
                poly = make_valid(poly)
            if poly.geom_type == 'Polygon':
                current_polys_from_loop.append(poly)
            elif poly.geom_type == 'MultiPolygon':
                for p_part in poly.geoms: 
                    if p_part.geom_type == 'Polygon':
                        current_polys_from_loop.append(p_part)
        except Exception: 
            continue

        for idx, p_valid in enumerate(current_polys_from_loop):
            if p_valid.area > cfg_min_area_thresh:
                poly_id_str = f"{i}-{idx}" if len(current_polys_from_loop) > 1 else str(i)
                shapely_polys_data.append({
                    'id': poly_id_str, 
                    'shapely_poly': p_valid,
                    'area': p_valid.area,
                    'children_polys': [], 
                    'parent_poly_id': None 
                })
        processed_loop_indices.add(i)

    if not shapely_polys_data:
        return []
    
    shapely_polys_data.sort(key=lambda x: x['area'], reverse=True)

    for i in range(len(shapely_polys_data)):
        current_node_data = shapely_polys_data[i]
        current_poly = current_node_data['shapely_poly']
        tightest_parent_data = None
        min_parent_area_diff = float('inf') 

        for j in range(i): 
            potential_parent_candidate_data = shapely_polys_data[j]
            candidate_parent_poly = potential_parent_candidate_data['shapely_poly']
            try:
                if candidate_parent_poly.is_valid and current_poly.is_valid and \
                   candidate_parent_poly.contains(current_poly.representative_point()): 
                    intersection_area = current_poly.intersection(candidate_parent_poly).area
                    if abs(intersection_area - current_poly.area) < (current_poly.area * 0.01 + cfg_coord_epsilon**2): 
                        area_diff = candidate_parent_poly.area - current_poly.area
                        if area_diff >= 0 and area_diff < min_parent_area_diff: 
                            tightest_parent_data = potential_parent_candidate_data
                            min_parent_area_diff = area_diff
            except Exception: 
                pass 

        if tightest_parent_data:
            current_node_data['parent_poly_id'] = tightest_parent_data['id']
            tightest_parent_data['children_polys'].append(current_poly) 

    all_laminae_shapely_polys = []
    for node_data in shapely_polys_data: # Iterate through ALL polygons. Each will form the base of a lamina.
        
        parent_poly_for_lamina = node_data['shapely_poly'] 
        direct_children_shapely_objects = node_data['children_polys']

        lamina_result = parent_poly_for_lamina 
        
        if direct_children_shapely_objects:
            try:
                valid_children = [ch for ch in direct_children_shapely_objects if ch.is_valid and not ch.is_empty]
                if not valid_children:
                    children_union = None
                else:
                    children_union = unary_union(valid_children) 

                if children_union and not children_union.is_empty:
                    potential_lamina = parent_poly_for_lamina.difference(children_union)
                    
                    if potential_lamina.is_empty:
                        continue 
                    
                    if not potential_lamina.is_valid:
                        potential_lamina = make_valid(potential_lamina)

                    lamina_result = potential_lamina
                    
            except Exception: 
                pass 

        if not lamina_result or lamina_result.is_empty:
            continue

        if not lamina_result.is_valid:
            lamina_result = make_valid(lamina_result)

        if lamina_result and not lamina_result.is_empty and lamina_result.area > cfg_min_area_thresh:
            if lamina_result.geom_type == 'Polygon':
                all_laminae_shapely_polys.append(lamina_result)
            elif lamina_result.geom_type == 'MultiPolygon':
                for p in lamina_result.geoms: 
                    if p.is_valid and not p.is_empty and p.area > cfg_min_area_thresh:
                        all_laminae_shapely_polys.append(p)

    candidate_regions_for_classification = []
    if all_laminae_shapely_polys:
        for shapely_poly in all_laminae_shapely_polys:
            if shapely_poly.is_valid and shapely_poly.exterior and shapely_poly.area > cfg_min_area_thresh :
                coords = list(shapely_poly.exterior.coords)
                interiors = []
                if hasattr(shapely_poly, 'interiors'): 
                    interiors = [list(interior.coords) for interior in shapely_poly.interiors]

                candidate_regions_for_classification.append({
                    'shapely_poly': shapely_poly,
                    'path_float': coords,
                    'interiors_float': interiors,
                    'centroid': (shapely_poly.centroid.x, shapely_poly.centroid.y)
                })
        candidate_regions_for_classification.sort(key=lambda p: (p['centroid'][1], p['centroid'][0]))
    
    return candidate_regions_for_classification


def generate_infill_segments_for_polygon(polygon, angle_degrees, line_spacing,
                                         cfg_coord_epsilon, cfg_x_order_left_to_right, cfg_y_order_top_to_bottom):
    if not SHAPELY_AVAILABLE: return [], None, None
    if polygon.is_empty or not polygon.is_valid:
        return [], None, None

    minx, miny, maxx, maxy = polygon.bounds
    centroid_x, centroid_y = polygon.centroid.x, polygon.centroid.y
    
    bbox_width = maxx - minx
    bbox_height = maxy - miny
    max_extent = math.sqrt(bbox_width**2 + bbox_height**2) + line_spacing * 2 

    sweep_half_dim = max_extent / 2
    sweep_start_x = centroid_x - sweep_half_dim
    sweep_end_x = centroid_x + sweep_half_dim
    
    infill_lines = []
    num_lines = math.ceil(max_extent / line_spacing) + 1 
    start_y_for_sweep = centroid_y - (num_lines / 2) * line_spacing + (line_spacing / 2) 

    for i in range(num_lines):
        y_coord = start_y_for_sweep + i * line_spacing
        line = LineString([(sweep_start_x, y_coord), (sweep_end_x, y_coord)])
        infill_lines.append(line)

    rotated_infill_lines = [rotate(line, angle_degrees, origin=(centroid_x, centroid_y), use_radians=False) for line in infill_lines]

    clipped_segments = []
    for rotated_line in rotated_infill_lines:
        try:
            intersection = rotated_line.intersection(polygon)
            if not intersection.is_empty:
                if intersection.geom_type == 'LineString':
                    if intersection.length > cfg_coord_epsilon * 5: 
                        clipped_segments.append(intersection)
                elif intersection.geom_type == 'MultiLineString':
                    for segment in intersection.geoms: 
                        if segment.geom_type == 'LineString' and segment.length > cfg_coord_epsilon * 5:
                            clipped_segments.append(segment)
        except Exception: 
            pass


    if not clipped_segments:
        return [], None, None

    y_sort_multiplier = -1 if cfg_y_order_top_to_bottom else 1
    x_sort_multiplier = 1 if cfg_x_order_left_to_right else -1

    def get_point_sweep_value(point_tuple): 
        return (y_sort_multiplier * point_tuple[1], x_sort_multiplier * point_tuple[0])

    oriented_segments = []
    for segment in clipped_segments:
        p1 = segment.coords[0]
        p2 = segment.coords[-1]
        
        if get_point_sweep_value(p1) > get_point_sweep_value(p2):
            oriented_segments.append(LineString([p2, p1]))
        else:
            oriented_segments.append(segment)

    oriented_segments.sort(key=lambda s: get_point_sweep_value(s.coords[0]))

    final_infill_elements = [] 
    first_point_of_cell_infill = None
    last_point_of_cell_infill = None
    last_processed_start_point = None

    for i, segment in enumerate(oriented_segments):
        primary_seg_start = segment.coords[0]
        primary_seg_end = segment.coords[-1]

        if first_point_of_cell_infill is None:
            first_point_of_cell_infill = primary_seg_start

        if last_processed_start_point is not None and \
           not gcode_parser.are_points_close(last_processed_start_point, primary_seg_start, cfg_coord_epsilon): 
            connecting_line = LineString([last_processed_start_point, primary_seg_start])
            if connecting_line.length > cfg_coord_epsilon:
                final_infill_elements.append((connecting_line, "connection"))

        primary_line = LineString([primary_seg_start, primary_seg_end])
        if primary_line.length > cfg_coord_epsilon:
            final_infill_elements.append((primary_line, "primary"))
        
        retrace_line = LineString([primary_seg_end, primary_seg_start])
        if retrace_line.length > cfg_coord_epsilon:
            final_infill_elements.append((retrace_line, "retrace"))
        
        last_processed_start_point = primary_seg_start
        
        if i == len(oriented_segments) - 1:
            last_point_of_cell_infill = primary_seg_start 

    return final_infill_elements, first_point_of_cell_infill, last_point_of_cell_infill


def generate_and_mask_grid(classified_regions, wall_loops, travel_segments,
                           cfg_grid_cell_size, cfg_bounds_buffer,
                           cfg_infill_region_shrink_offset, cfg_coord_epsilon, cfg_min_area_thresh,
                           cfg_x_order_left_to_right, cfg_y_order_top_to_bottom):
    if not SHAPELY_AVAILABLE:
        # print("Shapely not available in geometry_operations.generate_and_mask_grid") # Redundant if main handles
        return []

    if not classified_regions:
        # print("No classified regions to mask against.") # Potentially verbose
        return []

    all_coords = []
    for loop in wall_loops:
        all_coords.extend(loop)
    for seg_start, seg_end in travel_segments:
        all_coords.append(seg_start)
        all_coords.append(seg_end)
    for region_data in classified_regions:
        if region_data['shapely_poly']: 
            all_coords.append(region_data['shapely_poly'].bounds[0:2]) 
            all_coords.append(region_data['shapely_poly'].bounds[2:4]) 

    if not all_coords:
        # print("No coordinates found to determine print bounds.") # Potentially verbose
        return []

    min_x = min(p[0] for p in all_coords) - cfg_bounds_buffer
    max_x = max(p[0] for p in all_coords) + cfg_bounds_buffer
    min_y = min(p[1] for p in all_coords) - cfg_bounds_buffer
    max_y = max(p[1] for p in all_coords) + cfg_bounds_buffer # Corrected 'all_rds' to 'all_coords'

    infill_polygons = [r['shapely_poly'] for r in classified_regions if r['contains_infill'] and r['shapely_poly']]
    if not infill_polygons:
        # print("No infillable regions found for masking.") # Potentially verbose
        return []

    infill_mask = unary_union(infill_polygons)
    if not infill_mask.is_valid:
        infill_mask = make_valid(infill_mask) 

    if cfg_infill_region_shrink_offset > cfg_coord_epsilon: 
        try:
            shrunk_infill_mask = infill_mask.buffer(-cfg_infill_region_shrink_offset)
            if not shrunk_infill_mask.is_empty and shrunk_infill_mask.is_valid:
                infill_mask = shrunk_infill_mask
            # else: # Potentially verbose warning if buffer results in empty/invalid
                # print(f"Warning: Buffering infill mask by -{cfg_infill_region_shrink_offset}mm resulted in empty or invalid. Keeping original.")
        except Exception: # Broad exception for buffer issues
            # print(f"Error buffering infill mask: {e}. Keeping original mask.") # Potentially verbose
            pass


    if infill_mask.is_empty:
        # print("Unified infill mask is empty after unioning and optional buffering.") # Potentially verbose
        return []

    clipped_grid_cells_data = [] 
    start_x = -25.0  # Or set to your desired absolute origin X
    start_y = -25.0  # Or set to your desired absolute origin Y

    grid_x_idx_counter = 0
    current_x = start_x
    while current_x < max_x:
        grid_y_idx_counter = 0
        current_y = start_y
        while current_y < max_y:
            grid_cell = Polygon([
                (current_x, current_y),
                (current_x + cfg_grid_cell_size, current_y),
                (current_x + cfg_grid_cell_size, current_y + cfg_grid_cell_size),
                (current_x, current_y + cfg_grid_cell_size),
                (current_x, current_y) 
            ])

            try:
                intersection_result = grid_cell.intersection(infill_mask)
            except Exception: intersection_result = None

            if intersection_result and not intersection_result.is_empty:
                polys_to_add = []
                if intersection_result.geom_type == 'Polygon':
                    polys_to_add.append(intersection_result)
                elif intersection_result.geom_type == 'MultiPolygon':
                    polys_to_add.extend(list(intersection_result.geoms)) 

                for poly_part in polys_to_add:
                    if poly_part.geom_type == 'Polygon' and poly_part.area > cfg_min_area_thresh / 100.0: 
                        clipped_grid_cells_data.append({
                            'grid_x_idx': grid_x_idx_counter,
                            'grid_y_idx': grid_y_idx_counter,
                            'shapely_poly_clipped': poly_part,
                            'infill_segments': [], 
                            'infill_angle': None, 
                            'cell_infill_start_pt': None, 
                            'cell_infill_end_pt': None,    
                            'cell_infill_start_pt_on_boundary': None, 
                            'cell_infill_end_pt_on_boundary': None    
                        })
            current_y += cfg_grid_cell_size
            grid_y_idx_counter += 1
        current_x += cfg_grid_cell_size
        grid_x_idx_counter += 1

    y_sort_multiplier = -1 if cfg_y_order_top_to_bottom else 1
    x_sort_multiplier = 1 if cfg_x_order_left_to_right else -1
    clipped_grid_cells_data.sort(key=lambda c: (y_sort_multiplier * c['grid_y_idx'], x_sort_multiplier * c['grid_x_idx']))
    return clipped_grid_cells_data

def geometry_hash(polygon, decimals=2):
    """Returns a tolerant hash for a polygon using centroid, area, and perimeter, rounded."""
    centroid = tuple(round(c, decimals) for c in polygon.centroid.coords[0])
    area = round(polygon.area, decimals)
    perimeter = round(polygon.length, decimals)
    return hash((centroid, area, perimeter))

def cells_are_equivalent(cell1, cell2, area_tol=0.9, centroid_tol=0.9, overlap_threshold=0.98):
    """
    Compare two grid cells to determine if they are geometrically equivalent.
    Now includes polygon intersection overlap ratio to detect boundary changes.
    
    Args:
        cell1, cell2: Cell dictionaries containing 'shapely_poly_clipped'
        area_tol: Maximum allowed difference in area
        centroid_tol: Maximum allowed distance between centroids
        overlap_threshold: Minimum intersection-over-union (IoU) ratio required
    
    Returns:
        Boolean indicating if cells are equivalent
    """
    poly1 = cell1['shapely_poly_clipped']
    poly2 = cell2['shapely_poly_clipped']
    
    # Check basic properties (existing checks)
    area_diff = abs(poly1.area - poly2.area)
    centroid1 = poly1.centroid
    centroid2 = poly2.centroid
    centroid_dist = math.hypot(centroid1.x - centroid2.x, centroid1.y - centroid2.y)
    
    # Early rejection for obvious differences
    if area_diff >= area_tol or centroid_dist >= centroid_tol:
        return False
    
    # Check actual geometric overlap (new check)
    try:
        # Calculate intersection area
        intersection = poly1.intersection(poly2)
        intersection_area = intersection.area
        
        # Calculate union area
        union_area = poly1.area + poly2.area - intersection_area
        
        # Calculate IoU (Intersection over Union)
        if union_area > 0:
            overlap_ratio = intersection_area / union_area
            return overlap_ratio >= overlap_threshold
        return False
    except Exception:
        # If there's any error in geometric operations, consider cells different
        return False