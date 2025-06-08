# main_script.py
import sys
import os
import time
import math # For any direct math ops if needed (e.g. sqrt, though dist_sq is in gcode_parser)
from pathlib import Path
from collections import defaultdict # If used directly in main

# --- Add SCRIPT_DIR to sys.path for robust module imports ---
# This should be at the very top, before other project imports.
# SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# if SCRIPT_DIR not in sys.path:
#     sys.path.append(SCRIPT_DIR)
# For package structure, if __init__.py exists and you run with -m:
# from . import config as cfg
# from . import gcode_parser ... etc.
# For flat structure with sys.path modification:
from . import config as cfg
from . import gcode_parser
from . import geometry_operations
from . import pathfinding
from . import plotting_utils # This module will handle its own matplotlib import status
from . import gcode_generator  # NEW: Import the G-code generator module

# --- Check for external library availability ---
# Shapely (used by geometry_operations and pathfinding for Point/LineString)
try:
    from shapely.geometry import Point, LineString # Still needed in main for Point in classification logic
    SHAPELY_AVAILABLE = True
except ImportError:
    SHAPELY_AVAILABLE = False
    print("CRITICAL ERROR: Shapely library not found. This script heavily relies on it. Please install with: pip install shapely")
    # exit() # Consider exiting if Shapely is absolutely mandatory from the start

# NetworkX (used by pathfinding)
try:
    import networkx as nx
    NETWORKX_AVAILABLE = True
except ImportError:
    NETWORKX_AVAILABLE = False
    # This is a soft dependency if only direct jumps are acceptable for masked_grid mode.
    # print("Warning: NetworkX library not found. Boundary pathfinding will not be available.") # Handled later

# Matplotlib (used by plotting_utils)
# plotting_utils.py checks MATPLOTLIB_AVAILABLE internally.

# --- Helper functions that might remain in main or move to a general utils.py ---
def path_centroid(path_tuples):
    if not path_tuples or len(path_tuples) < 1: return (0, 0)
    x_coords = [p[0] for p in path_tuples]
    y_coords = [p[1] for p in path_tuples]
    num_points = len(path_tuples)
    if num_points == 0: return (0,0) # Should be caught by "not path_tuples"
    return (sum(x_coords) / num_points, sum(y_coords) / num_points)

# --- Main Execution ---
if __name__ == "__main__":
    overall_start_time = time.time()

    if not SHAPELY_AVAILABLE:
        print("Exiting due to missing Shapely library.")
        exit()

    if cfg.PLOT_MODE == "masked_grid" and not NETWORKX_AVAILABLE:
        print("Warning: NetworkX library not found. Inter-cell boundary connections will default to direct jumps.")

    gcode_file_path = Path(cfg.GCODE_FILE_PATH_STR)
    if not gcode_file_path.is_file():
        print(f"Error: G-code file not found at '{gcode_file_path}'. Please verify BASE_WORKING_DIRECTORY and GCODE_FILE_NAME in config.py.")
        exit()
    if cfg.PLOT_LAYER_NUMBER is None or not isinstance(cfg.PLOT_LAYER_NUMBER, int):
        print(f"Error: PLOT_LAYER_NUMBER in config.py must be an integer (e.g., 123). Current value: {cfg.PLOT_LAYER_NUMBER}")
        exit()
    if cfg.PLOT_MODE not in ["classified_regions", "masked_grid"]:
        print(f"Error: PLOT_MODE in config.py must be 'classified_regions' or 'masked_grid'. Current value: {cfg.PLOT_MODE}")
        exit()

    # --- 1. G-code Parsing ---
    print(f"\n--- Stage 1: G-code Parsing (Layer {cfg.PLOT_LAYER_NUMBER}) ---")
    stage_start_time = time.time()
    gcode_data_by_layer, layer_z_values = gcode_parser.extract_gcode_features(
        gcode_file_path,
        cfg.ABS_E_CHANGE_FOR_RETRACTION_CMD,
        cfg.MIN_E_DELTA_FOR_PRINTING_WHEN_NOT_RETRACTED,
        cfg.moved_xy_threshold,
        cfg.min_segment_length_epsilon_sq,
        cfg.COORD_EPSILON
    )
    print(f"Time for G-code feature extraction: {time.time() - stage_start_time:.3f} seconds")

    print("\n--- G-code Parsing Summary ---")
    if not gcode_data_by_layer:
        print("No G-code features were extracted from the input file.")
    else:
        for layer_n, data_for_l in sorted(gcode_data_by_layer.items()):
            # Only print summary for the target layer if many layers exist, or a few around it
            if abs(layer_n - cfg.PLOT_LAYER_NUMBER) < 3 or len(gcode_data_by_layer) < 10:
                print(f"  Layer {layer_n}: {len(data_for_l['wall_loops'])} wall loops, "
                      f"{len(data_for_l['travel_segments'])} travel segments, "
                      f"{len(data_for_l['original_infill_segments'])} original infill segments."
                      f" Parsed Z: {layer_z_values.get(layer_n, 'N/A')}")
        if cfg.PLOT_LAYER_NUMBER not in gcode_data_by_layer:
            print(f"Warning: Target layer {cfg.PLOT_LAYER_NUMBER} not found in parsed G-code data.")
            # exit() # Or allow to continue if G-code generation is off

    # --- Processing for the Target Layer ---
    if cfg.PLOT_LAYER_NUMBER is not None and gcode_data_by_layer and cfg.PLOT_LAYER_NUMBER in gcode_data_by_layer:
        processing_section_start_time = time.time()
        layer_data_to_process = gcode_data_by_layer.get(cfg.PLOT_LAYER_NUMBER)
        target_layer_z = layer_z_values.get(cfg.PLOT_LAYER_NUMBER)

        if not layer_data_to_process: # Should be caught by "cfg.PLOT_LAYER_NUMBER in gcode_data_by_layer"
            print(f"No data found for processing Layer Number = {cfg.PLOT_LAYER_NUMBER}.")
        else:
            print(f"\n--- Processing Layer {cfg.PLOT_LAYER_NUMBER} (Z={target_layer_z if target_layer_z is not None else 'Unknown'}) ---")
            wall_loops_for_layer = layer_data_to_process.get('wall_loops', [])
            original_infill_for_layer = layer_data_to_process.get('original_infill_segments', [])
            travel_segments_for_layer = layer_data_to_process.get('travel_segments', [])

            # --- 2. Identify Candidate Regions (Shapely) ---
            print("\n--- Stage 2: Identifying Candidate Regions ---")
            stage_start_time = time.time()
            candidate_regions = []
            if wall_loops_for_layer:
                candidate_regions = geometry_operations.identify_candidate_regions_shapely(
                    wall_loops_for_layer, cfg.MIN_AREA_THRESH_FLOAT, cfg.COORD_EPSILON
                )
            print(f"Identified {len(candidate_regions)} candidate regions.")
            print(f"Time for identifying candidate regions: {time.time() - stage_start_time:.3f} seconds")

            # --- 3. Classify Regions ---
            print("\n--- Stage 3: Classifying Regions for Infill ---")
            classified_regions_to_use = [] # Will hold the final classified data
            if candidate_regions:
                stage_start_time_classify = time.time()
                print(f"Classifying {len(candidate_regions)} candidate regions...")
                for region_idx, region_data in enumerate(candidate_regions):
                    shapely_poly_region = region_data['shapely_poly']
                    test_poly = shapely_poly_region # Start with original region
                    try: # Buffer for robust infill check; small negative buffer
                        buffered = shapely_poly_region.buffer(-cfg.COORD_EPSILON / 50.0, resolution=4)
                        if buffered and not buffered.is_empty and buffered.is_valid:
                            test_poly = buffered
                    except Exception: pass # If buffering fails, use original

                    region_infill_segments_found = []
                    # Check if original_infill_for_layer has data and test_poly is valid and has area
                    if original_infill_for_layer and test_poly.is_valid and test_poly.area > (cfg.MIN_AREA_THRESH_FLOAT / 100.0):
                        points_to_test_per_segment = 3 # Midpoints for checking containment
                        for seg_idx, (seg_start, seg_end) in enumerate(original_infill_for_layer):
                            if gcode_parser.segment_length_sq(seg_start, seg_end) < (cfg.min_segment_dist_threshold)**2:
                                continue # Skip very short segments
                            segment_fully_within = True
                            for i_pt in range(1, points_to_test_per_segment + 1):
                                if not segment_fully_within: break
                                ratio = i_pt / (points_to_test_per_segment + 1)
                                pt_x = seg_start[0] * (1-ratio) + seg_end[0] * ratio
                                pt_y = seg_start[1] * (1-ratio) + seg_end[1] * ratio
                                test_point_shapely = Point(pt_x, pt_y) # Shapely Point for .contains
                                try:
                                    if not test_poly.contains(test_point_shapely):
                                        segment_fully_within = False; break
                                except Exception: # Handle geometry errors during .contains
                                    segment_fully_within = False; break
                            if segment_fully_within:
                                region_infill_segments_found.append((seg_start, seg_end))

                    contains_significant_infill_flag = False
                    longest_infill_track_in_region_coords = None
                    if region_infill_segments_found:
                        longest_len_sq = 0.0
                        for s_start, s_end in region_infill_segments_found:
                            current_len_sq = gcode_parser.segment_length_sq(s_start, s_end)
                            if current_len_sq > longest_len_sq:
                                longest_len_sq = current_len_sq
                                longest_infill_track_in_region_coords = (s_start, s_end)
                        if longest_infill_track_in_region_coords and longest_len_sq >= cfg.MIN_SIGNIFICANT_TRACK_LENGTH_SQ:
                            contains_significant_infill_flag = True
                    
                    # Update region_data (which is an element of candidate_regions)
                    region_data['contains_infill'] = contains_significant_infill_flag
                    region_data['longest_track'] = longest_infill_track_in_region_coords
                    classified_regions_to_use.append(region_data) # Add to the list to be used
                print(f"Time for classifying regions: {time.time() - stage_start_time_classify:.3f} seconds")
            else:
                print("No candidate regions to classify.")

            # --- Plotting Setup (matplotlib is checked in plotting_utils) ---
            fig, ax = None, None
            if plotting_utils.MATPLOTLIB_AVAILABLE:
                fig, ax = plotting_utils.setup_plot()
            else:
                print("\nMatplotlib not available. Plotting will be skipped.")

            plot_title_main_str = ""
            all_toolpaths_ordered_for_gcode = [] # This will hold (LineString, type) for G-code and plotting

            # --- PLOT_MODE specific logic ---
            if cfg.PLOT_MODE == "classified_regions":
                print("\n--- Mode: Classified Regions ---")
                plot_title_main_str = f"Classified Regions - Layer {cfg.PLOT_LAYER_NUMBER} ({len(classified_regions_to_use)} regions)"
                if fig and ax: # Check if plotting is possible
                    plotting_utils.plot_classified_regions_on_ax(ax, classified_regions_to_use, cfg.MIN_SIGNIFICANT_TRACK_LENGTH_SQ)

            elif cfg.PLOT_MODE == "masked_grid":
                print("\n--- Mode: Masked Grid (for Plotting and G-code Generation) ---")
                plot_title_main_str = f"Masked Grid Infill - Layer {cfg.PLOT_LAYER_NUMBER} - X:{'L>R' if cfg.X_ORDER_LEFT_TO_RIGHT else 'R>L'}, Y:{'T>B' if cfg.Y_ORDER_TOP_TO_BOTTOM else 'B>T'}"
                
                # --- 4. Generate and Mask Grid ---
                print("\n--- Stage 4: Generating Masked Grid Cells ---")
                stage_start_time = time.time()
                masked_grid_cells_data_list = geometry_operations.generate_and_mask_grid(
                    classified_regions_to_use, wall_loops_for_layer, travel_segments_for_layer,
                    cfg.GRID_CELL_SIZE, cfg.GRID_BOUNDS_BUFFER, cfg.INFILL_REGION_SHRINK_OFFSET,
                    cfg.COORD_EPSILON, cfg.MIN_AREA_THRESH_FLOAT,
                    cfg.X_ORDER_LEFT_TO_RIGHT, cfg.Y_ORDER_TOP_TO_BOTTOM
                )
                print(f"Generated {len(masked_grid_cells_data_list)} masked grid cells.")
                print(f"Time for generating and masking grid: {time.time() - stage_start_time:.3f} seconds")

                if masked_grid_cells_data_list:
                    # --- 5. Generate Infill for each Grid Cell ---
                    print("\n--- Stage 5: Generating Infill within Grid Cells ---")
                    stage_start_time = time.time()
                    current_infill_angle_idx = 0
                    for cell_number, cell_data_dict in enumerate(masked_grid_cells_data_list):
                        cell_poly = cell_data_dict['shapely_poly_clipped']
                        current_angle = cfg.INFILL_ANGLES_CYCLE[current_infill_angle_idx]
                        cell_infill_elements, cell_start_pt, cell_end_pt = \
                            geometry_operations.generate_infill_segments_for_polygon(
                                cell_poly, current_angle, cfg.INFILL_LINE_SPACING,
                                cfg.COORD_EPSILON, cfg.X_ORDER_LEFT_TO_RIGHT, cfg.Y_ORDER_TOP_TO_BOTTOM
                            )
                        # Update the dictionary IN PLACE
                        cell_data_dict['infill_segments'] = cell_infill_elements # List of (LineString, type)
                        cell_data_dict['infill_angle'] = current_angle
                        cell_data_dict['execution_order'] = cell_number + 1 # 1-based for display
                        cell_data_dict['cell_infill_start_pt'] = cell_start_pt # Internal start of this cell's infill
                        cell_data_dict['cell_infill_end_pt'] = cell_end_pt   # Internal end of this cell's infill

                        # Project internal start/end points to the cell's boundary
                        if cell_poly.boundary and cell_start_pt and cell_end_pt:
                            try:
                                poly_boundary = cell_poly.boundary
                                # Handle MultiLineString if it's effectively a single closed loop
                                if poly_boundary.geom_type == 'MultiLineString' and len(list(poly_boundary.geoms)) == 1:
                                    poly_boundary = list(poly_boundary.geoms)[0]
                                elif poly_boundary.geom_type != 'LineString':
                                    poly_boundary = None # Cannot project to non-LineString
                                
                                if poly_boundary and poly_boundary.length > cfg.COORD_EPSILON:
                                    cell_data_dict['cell_infill_start_pt_on_boundary'] = poly_boundary.interpolate(poly_boundary.project(Point(cell_start_pt))).coords[0]
                                    cell_data_dict['cell_infill_end_pt_on_boundary'] = poly_boundary.interpolate(poly_boundary.project(Point(cell_end_pt))).coords[0]
                                else: # Ensure keys exist even if projection fails
                                    cell_data_dict['cell_infill_start_pt_on_boundary'] = None
                                    cell_data_dict['cell_infill_end_pt_on_boundary'] = None
                            except Exception: # Catch any Shapely errors during projection
                                cell_data_dict['cell_infill_start_pt_on_boundary'] = None
                                cell_data_dict['cell_infill_end_pt_on_boundary'] = None
                        else: # Ensure keys exist
                            cell_data_dict['cell_infill_start_pt_on_boundary'] = None
                            cell_data_dict['cell_infill_end_pt_on_boundary'] = None
                        current_infill_angle_idx = (current_infill_angle_idx + 1) % len(cfg.INFILL_ANGLES_CYCLE)
                    print(f"Time for infill generation for all cells: {time.time() - stage_start_time:.3f} seconds")

                    # --- 6. Assemble Inter-Cell Connections & Final Toolpath Order ---
                    print("\n--- Stage 6: Assembling Inter-Cell Connections and Final Toolpaths ---")
                    inter_cell_conn_assembly_start_time = time.time()
                    boundary_graph_for_paths = None
                    if NETWORKX_AVAILABLE:
                        boundary_graph_for_paths = pathfinding.build_boundary_graph(masked_grid_cells_data_list, cfg.COORD_EPSILON)
                        if boundary_graph_for_paths and boundary_graph_for_paths.number_of_nodes() > 0:
                            print(f"Boundary graph built with {boundary_graph_for_paths.number_of_nodes()} nodes and {boundary_graph_for_paths.number_of_edges()} edges.")
                        else: print("Warning: Boundary graph is empty or could not be built. Inter-cell connections default to direct jumps.")
                    
                    for i, current_cell_data_dict in enumerate(masked_grid_cells_data_list):
                        # Add Inter-cell connection path (if not the first cell)
                        if i > 0:
                            prev_cell_data_dict = masked_grid_cells_data_list[i-1]
                            # Points for connecting previous cell's end to current cell's start
                            from_pt_boundary_prev = prev_cell_data_dict.get('cell_infill_end_pt_on_boundary')
                            to_pt_boundary_curr = current_cell_data_dict.get('cell_infill_start_pt_on_boundary')
                            from_pt_internal_prev = prev_cell_data_dict.get('cell_infill_end_pt') # End of retrace of prev cell
                            to_pt_internal_curr = current_cell_data_dict.get('cell_infill_start_pt') # Start of first primary of current cell

                            connection_path_segments_added = False
                            if NETWORKX_AVAILABLE and boundary_graph_for_paths and boundary_graph_for_paths.number_of_nodes() > 0 and \
                               from_pt_boundary_prev and to_pt_boundary_curr and from_pt_internal_prev and to_pt_internal_curr:
                                path_on_boundary_line = pathfinding.find_shortest_path_on_boundary(
                                    boundary_graph_for_paths, from_pt_boundary_prev, to_pt_boundary_curr,
                                    cfg.NODE_MERGE_EPSILON, cfg.ON_SEGMENT_EPSILON, cfg.ENDPOINT_DISTINCTION_EPSILON,
                                    cfg.COORD_EPSILON, cfg.GRID_CELL_SIZE
                                )
                                if path_on_boundary_line and path_on_boundary_line.length > cfg.COORD_EPSILON / 10: # Ensure path has some length
                                    # Part 1: Travel from internal end of previous cell to its boundary projection
                                    if not gcode_parser.are_points_close(from_pt_internal_prev, from_pt_boundary_prev, cfg.COORD_EPSILON):
                                        all_toolpaths_ordered_for_gcode.append((LineString([from_pt_internal_prev, from_pt_boundary_prev]), "inter_cell_direct_jump_part"))
                                    
                                    # Part 2: The actual path along the boundary
                                    all_toolpaths_ordered_for_gcode.append((path_on_boundary_line, "inter_cell_boundary_connection"))
                                    
                                    # Part 3: Travel from boundary path end to internal start of current cell
                                    last_boundary_path_pt = path_on_boundary_line.coords[-1]
                                    # Ensure connection from boundary path end to the internal start of the current cell
                                    if not gcode_parser.are_points_close(last_boundary_path_pt, to_pt_internal_curr, cfg.COORD_EPSILON):
                                        # If boundary path does not end exactly at to_pt_boundary_curr, connect to it first
                                        if not gcode_parser.are_points_close(last_boundary_path_pt, to_pt_boundary_curr, cfg.COORD_EPSILON):
                                             all_toolpaths_ordered_for_gcode.append((LineString([last_boundary_path_pt, to_pt_boundary_curr]), "inter_cell_direct_jump_part"))
                                        # Then connect from to_pt_boundary_curr to to_pt_internal_curr
                                        if not gcode_parser.are_points_close(to_pt_boundary_curr, to_pt_internal_curr, cfg.COORD_EPSILON):
                                            all_toolpaths_ordered_for_gcode.append((LineString([to_pt_boundary_curr, to_pt_internal_curr]), "inter_cell_direct_jump_part"))
                                    connection_path_segments_added = True
                            
                            if not connection_path_segments_added and from_pt_internal_prev and to_pt_internal_curr:
                                # Fallback: Direct jump if boundary path failed or points missing/NetworkX unavailable
                                if not gcode_parser.are_points_close(from_pt_internal_prev, to_pt_internal_curr, cfg.COORD_EPSILON): # Avoid zero-length jump
                                    all_toolpaths_ordered_for_gcode.append((LineString([from_pt_internal_prev, to_pt_internal_curr]), "inter_cell_direct_jump"))
                        
                        # Add current cell's internal infill segments (already ordered: primary, retrace, connection...)
                        if current_cell_data_dict['infill_segments']:
                            all_toolpaths_ordered_for_gcode.extend(current_cell_data_dict['infill_segments'])
                    print(f"Assembled {len(all_toolpaths_ordered_for_gcode)} toolpath segments.")
                    print(f"Time for inter-cell connections and toolpath assembly: {time.time() - inter_cell_conn_assembly_start_time:.3f} seconds")

                    # Plotting for masked_grid mode
                    if fig and ax:
                        plotting_utils.plot_masked_grid_on_ax(ax, masked_grid_cells_data_list, all_toolpaths_ordered_for_gcode)
                else: # No masked_grid_cells_data_list
                    print("No masked grid cells were generated, skipping infill and G-code generation.")
            
            # --- Common Plotting Elements & Finalize Plot (if plotting is active) ---
            if fig and ax: # Only if Matplotlib setup was successful
                print("\n--- Finalizing Plot ---")
                stage_start_time = time.time()
                plotting_utils.plot_common_elements_on_ax(ax, wall_loops_for_layer, travel_segments_for_layer)
                plotting_utils.finalize_plot(fig, ax, plot_title_main_str, cfg.OUTPUT_PLOT_FILE_PATH_STR)
                print(f"Time for common plot elements and final plot display/save: {time.time() - stage_start_time:.3f} seconds")

            # --- 7. G-code Generation (if enabled for masked_grid mode) ---
            if cfg.PLOT_MODE == "masked_grid" and cfg.ENABLE_GCODE_GENERATION:
                print("\n--- Stage 7: G-code Generation ---")
                if not all_toolpaths_ordered_for_gcode:
                    print("No toolpaths were generated for G-code output. Skipping G-code generation.")
                elif target_layer_z is None:
                    print(f"SKIPPING G-code generation: Z-height for layer {cfg.PLOT_LAYER_NUMBER} could not be determined.")
                else:
                    gcode_gen_start_time = time.time()
                    print(f"Attempting G-code generation for layer {cfg.PLOT_LAYER_NUMBER} at Z={target_layer_z:.3f}...")
                    gcode_generator.generate_gcode_from_toolpaths(
                        all_toolpaths_ordered_for_gcode,
                        target_layer_z, # Must be a float
                        cfg.GENERATED_GCODE_FILE_PATH_STR,
                        cfg.FEEDRATE_PRIMARY_INFILL,
                        cfg.FEEDRATE_BOUNDARY_CONNECTION,
                        cfg.FEEDRATE_INTRA_CELL_CONNECTION,
                        cfg.FEEDRATE_TRAVEL,
                        cfg.EXTRUSION_MULTIPLIER,
                        cfg.INITIAL_E_VALUE_FOR_GENERATED_LAYER,
                        cfg.COORD_EPSILON # Pass for filtering tiny segments in G-code gen
                    )
                    print(f"Time for G-code generation: {time.time() - gcode_gen_start_time:.3f} seconds")
            elif cfg.ENABLE_GCODE_GENERATION and cfg.PLOT_MODE != "masked_grid":
                print("\nNote: G-code generation is enabled but PLOT_MODE is not 'masked_grid'. G-code will not be generated.")


        print(f"\nTotal time for processing layer {cfg.PLOT_LAYER_NUMBER}: {time.time() - processing_section_start_time:.3f} seconds")

    elif cfg.PLOT_LAYER_NUMBER is not None: # Target layer was specified but not found or no data
         print(f"No G-code features extracted or target layer {cfg.PLOT_LAYER_NUMBER} not found, cannot process/plot.")

    print(f"\n--- Total script execution time: {time.time() - overall_start_time:.3f} seconds ---")