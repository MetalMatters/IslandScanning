# main_script.py (your refactored original script)
import sys
import os
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.append(SCRIPT_DIR)
    
import time
import math # For any direct math ops if needed
from collections import defaultdict
from pathlib import Path

# Import from your new modules
import config as cfg # Alias for brevity
import gcode_parser
import geometry_operations
import pathfinding
import plotting_utils # This will attempt to import matplotlib

# Shapely and NetworkX imports & availability checks remain important at the top level
try:
    from shapely.geometry import Point, LineString # Still needed for Point in classification, LineString for toolpaths
    # Other specific shapely imports might be localized to geometry_operations.py
    SHAPELY_AVAILABLE = True
except ImportError:
    SHAPELY_AVAILABLE = False
    print("Shapely library not found. Please install with: pip install shapely")

try:
    import networkx as nx # Main script might still check NETWORKX_AVAILABLE
    NETWORKX_AVAILABLE = True
except ImportError:
    NETWORKX_AVAILABLE = False
    print("NetworkX library not found. Please install with: pip install networkx")

# Deprecated pyclipper functions (can be removed if not used elsewhere)
def to_int_point(p): pass
def to_float_point(p): pass
def to_int_path(path): pass
def to_float_path(path): pass

def path_centroid(path_tuples): # This could also go into a general utils.py
    if not path_tuples or len(path_tuples) < 1: return (0, 0)
    x_coords = [p[0] for p in path_tuples]
    y_coords = [p[1] for p in path_tuples]
    num_points = len(path_tuples)
    if num_points == 0: return (0,0)
    return (sum(x_coords) / num_points, sum(y_coords) / num_points)

# --- Main Execution ---
if __name__ == "__main__":
    overall_start_time = time.time()

    if not SHAPELY_AVAILABLE:
        exit("Shapely library is required. Please install it.")
    # NetworkX is preferred for masked_grid mode but not strictly a showstopper if only direct jumps are acceptable
    if cfg.PLOT_MODE == "masked_grid" and not NETWORKX_AVAILABLE:
        print("Warning: NetworkX library not found. Inter-cell boundary connections will be direct jumps.")

    gcode_file = Path(cfg.GCODE_FILE_PATH_STR)
    if not gcode_file.is_file():
        print(f"Error: G-code file not found at '{gcode_file}'. Please verify BASE_WORKING_DIRECTORY and GCODE_FILE_NAME.")
        exit()
    if cfg.PLOT_LAYER_NUMBER is not None and not isinstance(cfg.PLOT_LAYER_NUMBER, int):
        exit("PLOT_LAYER_NUMBER must be int or None.")
    if cfg.PLOT_MODE not in ["classified_regions", "masked_grid"]:
        exit("PLOT_MODE must be 'classified_regions' or 'masked_grid'.")

    # --- 1. G-code Parsing ---
    stage_start_time = time.time()
    gcode_data_by_layer, layer_z_values = gcode_parser.extract_gcode_features(
        gcode_file,
        cfg.ABS_E_CHANGE_FOR_RETRACTION_CMD,
        cfg.MIN_E_DELTA_FOR_PRINTING_WHEN_NOT_RETRACTED,
        cfg.moved_xy_threshold,
        cfg.min_segment_length_epsilon_sq,
        cfg.COORD_EPSILON
    )
    print(f"Time for G-code feature extraction: {time.time() - stage_start_time:.3f} seconds")

    print("\n--- Summary ---")
    if not gcode_data_by_layer:
        print("No G-code features were extracted.")
    else:
        for layer_n, data_for_l in sorted(gcode_data_by_layer.items()):
            print(f"Layer {layer_n}: {len(data_for_l['wall_loops'])} wall loops, "
                  f"{len(data_for_l['travel_segments'])} travel segments, "
                  f"{len(data_for_l['original_infill_segments'])} original infill segments."
                  f" Parsed Z: {layer_z_values.get(layer_n, 'N/A')}")

    if cfg.PLOT_LAYER_NUMBER is not None and gcode_data_by_layer:
        plotting_section_start_time = time.time()
        layer_data_to_plot = gcode_data_by_layer.get(cfg.PLOT_LAYER_NUMBER)

        if not layer_data_to_plot:
            print(f"No data found for plotting Layer Number = {cfg.PLOT_LAYER_NUMBER}.")
        else:
            wall_loops_for_plot_layer = layer_data_to_plot.get('wall_loops', [])
            original_infill_for_layer = layer_data_to_plot.get('original_infill_segments', [])
            travel_segments_to_plot = layer_data_to_plot.get('travel_segments', [])

            # --- 2. Identify Candidate Regions (Shapely) ---
            stage_start_time = time.time()
            candidate_regions = []
            if wall_loops_for_plot_layer:
                candidate_regions = geometry_operations.identify_candidate_regions_shapely(
                    wall_loops_for_plot_layer, cfg.MIN_AREA_THRESH_FLOAT, cfg.COORD_EPSILON
                )
            print(f"Time for identifying candidate regions (Shapely): {time.time() - stage_start_time:.3f} seconds")

            # --- 3. Classify Regions ---
            classified_regions_to_plot = []
            if candidate_regions:
                stage_start_time_classify = time.time() # Renamed to avoid conflict
                print(f"Classifying {len(candidate_regions)} candidate regions for layer {cfg.PLOT_LAYER_NUMBER}.")
                for region_idx, region_data in enumerate(candidate_regions):
                    shapely_poly_region = region_data['shapely_poly']
                    test_poly = shapely_poly_region
                    try: # Buffer for robust infill check
                        buffered = shapely_poly_region.buffer(-cfg.COORD_EPSILON / 50.0, resolution=4)
                        if not buffered.is_empty and buffered.is_valid: test_poly = buffered
                    except Exception: pass

                    region_infill_segments = []
                    if original_infill_for_layer and test_poly.is_valid and test_poly.area > (cfg.MIN_AREA_THRESH_FLOAT / 100.0):
                        points_to_test_per_segment = 3
                        for seg_idx, (seg_start, seg_end) in enumerate(original_infill_for_layer):
                            if gcode_parser.segment_length_sq(seg_start, seg_end) < (cfg.min_segment_dist_threshold)**2:
                                continue
                            segment_fully_within = True
                            for i_pt in range(1, points_to_test_per_segment + 1):
                                if not segment_fully_within: break
                                ratio = i_pt / (points_to_test_per_segment + 1)
                                pt_x = seg_start[0] * (1-ratio) + seg_end[0] * ratio
                                pt_y = seg_start[1] * (1-ratio) + seg_end[1] * ratio
                                test_point_shapely = Point(pt_x, pt_y) # Shapely Point
                                try:
                                    if not test_poly.contains(test_point_shapely):
                                        segment_fully_within = False; break
                                except Exception: segment_fully_within = False; break
                            if segment_fully_within:
                                region_infill_segments.append((seg_start, seg_end))

                    contains_significant_infill = False
                    longest_infill_track_in_region = None
                    longest_len_sq = 0.0
                    if region_infill_segments:
                        for s_start, s_end in region_infill_segments:
                            current_len_sq = gcode_parser.segment_length_sq(s_start, s_end)
                            if current_len_sq > longest_len_sq:
                                longest_len_sq = current_len_sq
                                longest_infill_track_in_region = (s_start, s_end)
                        if longest_infill_track_in_region and longest_len_sq >= cfg.MIN_SIGNIFICANT_TRACK_LENGTH_SQ:
                            contains_significant_infill = True
                    region_data['contains_infill'] = contains_significant_infill
                    region_data['longest_track'] = longest_infill_track_in_region
                    classified_regions_to_plot.append(region_data)
                print(f"Time for classifying regions: {time.time() - stage_start_time_classify:.3f} seconds")

            # --- Matplotlib specific setup ---
            fig, ax = None, None
            CAN_PLOT = False
            try:
                import matplotlib.pyplot # Check if available before setting up plot
                fig, ax = plotting_utils.setup_plot()
                CAN_PLOT = True
            except ImportError:
                print("\nMatplotlib not installed. Plotting will be skipped.")

            plot_title_str = ""
            all_toolpaths_ordered = [] # For masked_grid mode

            if cfg.PLOT_MODE == "classified_regions":
                plot_title_str = f"G-code Features & Classified Regions (Shapely) for Layer = {cfg.PLOT_LAYER_NUMBER} ({len(classified_regions_to_plot)} candidates)"
                if CAN_PLOT and ax is not None:
                    plotting_utils.plot_classified_regions_on_ax(ax, classified_regions_to_plot, cfg.MIN_SIGNIFICANT_TRACK_LENGTH_SQ)
            elif cfg.PLOT_MODE == "masked_grid":
                plot_title_str = f"Infill Masked Grid Layer {cfg.PLOT_LAYER_NUMBER} ({len(classified_regions_to_plot)} infillable regions) - X:{'L-R' if cfg.X_ORDER_LEFT_TO_RIGHT else 'R-L'}, Y:{'T-B' if cfg.Y_ORDER_TOP_TO_BOTTOM else 'B-T'}"
                print(f"Generating and plotting masked grid for layer {cfg.PLOT_LAYER_NUMBER} (Mode: Masked Grid).")

                stage_start_time = time.time()
                masked_grid_cells_data = geometry_operations.generate_and_mask_grid(
                    classified_regions_to_plot, wall_loops_for_plot_layer, travel_segments_to_plot,
                    cfg.GRID_CELL_SIZE, cfg.GRID_BOUNDS_BUFFER, cfg.INFILL_REGION_SHRINK_OFFSET,
                    cfg.COORD_EPSILON, cfg.MIN_AREA_THRESH_FLOAT,
                    cfg.X_ORDER_LEFT_TO_RIGHT, cfg.Y_ORDER_TOP_TO_BOTTOM
                )
                print(f"Time for generating and masking grid: {time.time() - stage_start_time:.3f} seconds")

                # Infill Generation for each cell
                stage_start_time = time.time()
                current_infill_angle_idx = 0
                print(f"Generating infill for {len(masked_grid_cells_data)} grid cells.")
                for cell_number, cell_data in enumerate(masked_grid_cells_data):
                    cell_poly = cell_data['shapely_poly_clipped']
                    current_angle = cfg.INFILL_ANGLES_CYCLE[current_infill_angle_idx]
                    cell_infill_elements, cell_start_pt, cell_end_pt = \
                        geometry_operations.generate_infill_segments_for_polygon(
                            cell_poly, current_angle, cfg.INFILL_LINE_SPACING,
                            cfg.COORD_EPSILON, cfg.X_ORDER_LEFT_TO_RIGHT, cfg.Y_ORDER_TOP_TO_BOTTOM
                        )
                    cell_data['infill_segments'] = cell_infill_elements
                    cell_data['infill_angle'] = current_angle
                    cell_data['execution_order'] = cell_number + 1
                    cell_data['cell_infill_start_pt'] = cell_start_pt
                    cell_data['cell_infill_end_pt'] = cell_end_pt

                    # Project to boundary
                    if cell_poly.boundary and cell_start_pt and cell_end_pt:
                        try:
                            poly_boundary = cell_poly.boundary
                            if poly_boundary.geom_type == 'MultiLineString' and len(list(poly_boundary.geoms)) == 1: # Use list() for MultiLineString
                                poly_boundary = list(poly_boundary.geoms)[0]
                            elif poly_boundary.geom_type != 'LineString':
                                poly_boundary = None
                            
                            if poly_boundary and poly_boundary.length > cfg.COORD_EPSILON:
                                cell_data['cell_infill_start_pt_on_boundary'] = poly_boundary.interpolate(poly_boundary.project(Point(cell_start_pt))).coords[0]
                                cell_data['cell_infill_end_pt_on_boundary'] = poly_boundary.interpolate(poly_boundary.project(Point(cell_end_pt))).coords[0]
                            else:
                                cell_data['cell_infill_start_pt_on_boundary'], cell_data['cell_infill_end_pt_on_boundary'] = None, None
                        except Exception: cell_data['cell_infill_start_pt_on_boundary'], cell_data['cell_infill_end_pt_on_boundary'] = None, None
                    else: cell_data['cell_infill_start_pt_on_boundary'], cell_data['cell_infill_end_pt_on_boundary'] = None, None
                    current_infill_angle_idx = (current_infill_angle_idx + 1) % len(cfg.INFILL_ANGLES_CYCLE)
                print(f"Time for infill generation for all cells: {time.time() - stage_start_time:.3f} seconds")

                # Inter-Cell Connections & Toolpath Assembly
                inter_cell_conn_start_time = time.time()
                boundary_graph = None
                if NETWORKX_AVAILABLE:
                    print("Building boundary graph for inter-cell connections...")
                    boundary_graph = pathfinding.build_boundary_graph(masked_grid_cells_data, cfg.COORD_EPSILON)
                    if boundary_graph and boundary_graph.number_of_nodes() > 0:
                        print(f"Graph built with {boundary_graph.number_of_nodes()} nodes and {boundary_graph.number_of_edges()} edges.")
                    else: print("Warning: Boundary graph is empty or could not be built. Inter-cell connections will be direct jumps.")

                print("Assembling ordered toolpaths including inter-cell connections...")
                for i, current_cell_data in enumerate(masked_grid_cells_data):
                    if i > 0: # Add inter-cell connection from previous cell to current
                        prev_cell_data = masked_grid_cells_data[i-1]
                        from_pt_boundary = prev_cell_data.get('cell_infill_end_pt_on_boundary')
                        to_pt_boundary = current_cell_data.get('cell_infill_start_pt_on_boundary')
                        from_pt_internal = prev_cell_data.get('cell_infill_end_pt')
                        to_pt_internal = current_cell_data.get('cell_infill_start_pt')
                        connection_made = False
                        if NETWORKX_AVAILABLE and boundary_graph and boundary_graph.number_of_nodes() > 0 and \
                           from_pt_boundary and to_pt_boundary and from_pt_internal and to_pt_internal:
                            path_along_boundary = pathfinding.find_shortest_path_on_boundary(
                                boundary_graph, from_pt_boundary, to_pt_boundary,
                                cfg.NODE_MERGE_EPSILON, cfg.ON_SEGMENT_EPSILON, cfg.ENDPOINT_DISTINCTION_EPSILON,
                                cfg.COORD_EPSILON, cfg.GRID_CELL_SIZE
                            )
                            if path_along_boundary and path_along_boundary.length > cfg.COORD_EPSILON:
                                # Part 1: Jump from internal end of previous cell to its boundary projection
                                if not gcode_parser.are_points_close(from_pt_internal, from_pt_boundary, cfg.COORD_EPSILON):
                                    all_toolpaths_ordered.append((LineString([from_pt_internal, from_pt_boundary]), "inter_cell_direct_jump_part"))
                                # Part 2: The actual path along the boundary
                                all_toolpaths_ordered.append((path_along_boundary, "inter_cell_boundary_connection"))
                                # Part 3: Jump from boundary path end to internal start of current cell
                                last_boundary_path_pt = path_along_boundary.coords[-1]
                                # Connect boundary path end to internal start, possibly via to_pt_boundary projection
                                if not gcode_parser.are_points_close(last_boundary_path_pt, to_pt_internal, cfg.COORD_EPSILON):
                                    if not gcode_parser.are_points_close(last_boundary_path_pt, to_pt_boundary, cfg.COORD_EPSILON):
                                         all_toolpaths_ordered.append((LineString([last_boundary_path_pt, to_pt_boundary]), "inter_cell_direct_jump_part"))
                                    if not gcode_parser.are_points_close(to_pt_boundary, to_pt_internal, cfg.COORD_EPSILON):
                                        all_toolpaths_ordered.append((LineString([to_pt_boundary, to_pt_internal]), "inter_cell_direct_jump_part"))
                                connection_made = True
                        if not connection_made and from_pt_internal and to_pt_internal: # Fallback direct jump
                            if not gcode_parser.are_points_close(from_pt_internal, to_pt_internal, cfg.COORD_EPSILON):
                                all_toolpaths_ordered.append((LineString([from_pt_internal, to_pt_internal]), "inter_cell_direct_jump"))
                    # Add current cell's internal infill segments
                    if current_cell_data['infill_segments']:
                        all_toolpaths_ordered.extend(current_cell_data['infill_segments'])
                print(f"Time for inter-cell connections and toolpath assembly: {time.time() - inter_cell_conn_start_time:.3f} seconds")

                if CAN_PLOT and ax is not None:
                    plotting_utils.plot_masked_grid_on_ax(ax, masked_grid_cells_data, all_toolpaths_ordered)
            
            # --- Common Plotting Elements & Finalize (if plotting is possible) ---
            if CAN_PLOT and fig is not None and ax is not None:
                stage_start_time = time.time()
                plotting_utils.plot_common_elements_on_ax(ax, wall_loops_for_plot_layer, travel_segments_to_plot)
                plotting_utils.finalize_plot(fig, ax, plot_title_str, cfg.OUTPUT_PLOT_FILE_PATH_STR)
                print(f"Time for common plot elements and final setup: {time.time() - stage_start_time:.3f} seconds")

            # --- Placeholder for G-code Generation call ---
            if cfg.PLOT_MODE == "masked_grid" and cfg.ENABLE_GCODE_GENERATION:
                print("\n G-code generation logic would go here, using 'all_toolpaths_ordered'.")
                # target_z = layer_z_values.get(cfg.PLOT_LAYER_NUMBER)
                # if target_z is not None:
                #    gcode_generator.generate_gcode_from_toolpaths(...) # Example call
                # else:
                #    print(f"SKIPPING G-code generation: Z-height for layer {cfg.PLOT_LAYER_NUMBER} not found.")

        print(f"Total time for plotting section for layer {cfg.PLOT_LAYER_NUMBER}: {time.time() - plotting_section_start_time:.3f} seconds")

    elif cfg.PLOT_LAYER_NUMBER is not None:
         print(f"No G-code features extracted, so cannot plot Layer Number = {cfg.PLOT_LAYER_NUMBER}.")

    print(f"\n--- Total script execution time: {time.time() - overall_start_time:.3f} seconds ---")