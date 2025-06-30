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
from .pathfinding import RoutingCache, ConnectionCache
from .geometry_operations import geometry_hash, cells_are_equivalent
routing_cache = RoutingCache()
connection_cache = ConnectionCache()

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

def classify_regions_with_infill(candidate_regions, layer_data, min_significant_track_length_sq):
    """
    Classify regions based on whether they contain infill.
    
    Args:
        candidate_regions: List of region data dictionaries
        layer_data: Dictionary of layer-specific data including original_infill_segments
        min_significant_track_length_sq: Threshold for "significant" infill
        
    Returns:
        List of classified regions
    """
    classified_regions_to_use = []
    original_infill_segments = layer_data.get('original_infill_segments', [])
    
    for region_idx, region_data in enumerate(candidate_regions):
        shapely_poly_region = region_data['shapely_poly']
        test_poly = shapely_poly_region
        try:
            buffered = shapely_poly_region.buffer(-cfg.COORD_EPSILON / 50.0, resolution=4)
            if buffered and not buffered.is_empty and buffered.is_valid:
                test_poly = buffered
        except Exception:
            pass

        region_infill_segments_found = []
        if original_infill_segments and test_poly.is_valid and test_poly.area > (cfg.MIN_AREA_THRESH_FLOAT / 100.0):
            points_to_test_per_segment = 3
            for seg_idx, (seg_start, seg_end) in enumerate(original_infill_segments):
                if gcode_parser.segment_length_sq(seg_start, seg_end) < (cfg.min_segment_dist_threshold)**2:
                    continue
                segment_fully_within = True
                for i_pt in range(1, points_to_test_per_segment + 1):
                    if not segment_fully_within: break
                    ratio = i_pt / (points_to_test_per_segment + 1)
                    pt_x = seg_start[0] * (1 - ratio) + seg_end[0] * ratio
                    pt_y = seg_start[1] * (1 - ratio) + seg_end[1] * ratio
                    test_point_shapely = Point(pt_x, pt_y)
                    try:
                        if not test_poly.contains(test_point_shapely):
                            segment_fully_within = False; break
                    except Exception:
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
            if longest_infill_track_in_region_coords and longest_len_sq >= min_significant_track_length_sq:
                contains_significant_infill_flag = True

        region_data['contains_infill'] = contains_significant_infill_flag
        region_data['longest_track'] = longest_infill_track_in_region_coords
        classified_regions_to_use.append(region_data)
    
    return classified_regions_to_use

# Add this helper function after your other utility functions
def print_normal(message):
    """Print only if not in minimal output mode"""
    if not cfg.MINIMAL_OUTPUT_MODE:
        print(message)

def print_progress(current, total, message="Processing"):
    """Always print progress updates, even in minimal mode"""
    percent = 100.0 * current / total if total > 0 else 0
    print(f"{message}: {percent:.1f}% complete ({current}/{total})")

# Add at the beginning of the script, after imports
def ensure_output_directory():
    """Create the output directory if it doesn't exist"""
    if not cfg.OUTPUT_DIRECTORY.exists():
        try:
            cfg.OUTPUT_DIRECTORY.mkdir(parents=True)
            print_normal(f"Created output directory: {cfg.OUTPUT_DIRECTORY}")
        except Exception as e:
            print(f"Error creating output directory: {e}")
            exit(1)

# --- Main Execution ---
if __name__ == "__main__":
    overall_start_time = time.time()
    
    ensure_output_directory()  # Make sure output directory exists
    
    if cfg.SINGLE_GCODE_FILE_OUTPUT:
        combined_gcode_path = str(cfg.OUTPUT_DIRECTORY / cfg.SINGLE_GCODE_OUTPUT_FILENAME)
        gcode_generator.initialize_combined_gcode_file(combined_gcode_path, cfg.GCODE_STUB_START)

    if not SHAPELY_AVAILABLE:
        print("Exiting due to missing Shapely library.")
        exit()

    if cfg.PLOT_MODE == "masked_grid" and not NETWORKX_AVAILABLE:
        print("Warning: NetworkX library not found. Inter-cell boundary connections will default to direct jumps.")

    gcode_file_path = Path(cfg.GCODE_FILE_PATH_STR)
    if not gcode_file_path.is_file():
        print(f"Error: G-code file not found at '{gcode_file_path}'. Please verify BASE_WORKING_DIRECTORY and GCODE_FILE_NAME in config.py.")
        exit()
    if cfg.PLOT_MODE not in ["classified_regions", "masked_grid"]:
        print(f"Error: PLOT_MODE in config.py must be 'classified_regions' or 'masked_grid'. Current value: {cfg.PLOT_MODE}")
        exit()

    # --- 1. G-code Parsing ---
    print_normal(f"\n--- Stage 1: G-code Parsing (All Layers) ---")
    stage_start_time = time.time()
    gcode_data_by_layer, layer_z_values = gcode_parser.extract_gcode_features(
        gcode_file_path,
        cfg.ABS_E_CHANGE_FOR_RETRACTION_CMD,
        cfg.MIN_E_DELTA_FOR_PRINTING_WHEN_NOT_RETRACTED,
        cfg.moved_xy_threshold,
        cfg.min_segment_length_epsilon_sq,
        cfg.COORD_EPSILON
    )
    print_normal(f"Time for G-code feature extraction: {time.time() - stage_start_time:.3f} seconds")

    print_normal("\n--- G-code Parsing Summary ---")
    if not gcode_data_by_layer:
        print_normal("No G-code features were extracted from the input file.")
        exit()
    else:
        for layer_n, data_for_l in sorted(gcode_data_by_layer.items()):
            print_normal(f"  Layer {layer_n}: {len(data_for_l['wall_loops'])} wall loops, "
                  f"{len(data_for_l['travel_segments'])} travel segments, "
                  f"{len(data_for_l['original_infill_segments'])} original infill segments."
                  f" Parsed Z: {layer_z_values.get(layer_n, 'N/A')}")

    # --- Model Boundaries Processing (New Logic) ---
    all_model_boundaries_processed = False
    model_boundaries = {}
    if not all_model_boundaries_processed:
        print_normal("\n--- Processing All Model Boundaries (One Time Only) ---")
        boundary_start_time = time.time()
        
        # Process all boundaries for all layers at once
        for layer_num_to_process in sorted(gcode_data_by_layer.keys()):
            wall_loops = gcode_data_by_layer[layer_num_to_process]['wall_loops']
            travel_segments = gcode_data_by_layer[layer_num_to_process]['travel_segments']
            
            # Process boundaries for this layer
            classified_regions = geometry_operations.identify_candidate_regions_shapely(
                wall_loops, cfg.MIN_AREA_THRESH_FLOAT, cfg.COORD_EPSILON
            )
            classified_regions_to_use = classify_regions_with_infill(
                classified_regions, gcode_data_by_layer[layer_num_to_process], 
                cfg.MIN_SIGNIFICANT_TRACK_LENGTH_SQ
            )
            
            # Store in our model_boundaries dictionary
            model_boundaries[layer_num_to_process] = {
                'wall_loops': wall_loops,
                'travel_segments': travel_segments,
                'classified_regions': classified_regions,
                'classified_regions_to_use': classified_regions_to_use
            }
        
        all_model_boundaries_processed = True
        print_normal(f"Processed all model boundaries in {time.time() - boundary_start_time:.3f} seconds")

    # --- Multi-layer Processing Loop ---
    prev_cells_by_idx = None  # <-- Place before your for layer_num in ... loop
    all_model_boundaries_processed = False
    model_boundaries = {}
    total_layers = len(gcode_data_by_layer)
    for layer_idx, layer_num in enumerate(sorted(gcode_data_by_layer.keys())):
        # Show progress regardless of minimal mode
        print_progress(layer_idx + 1, total_layers, "Processing layers")
        
        if prev_cells_by_idx is None:
            prev_cells_by_idx = {}

        processing_section_start_time = time.time()
        layer_data_to_process = gcode_data_by_layer[layer_num]
        target_layer_z = layer_z_values.get(layer_num)

        print_normal(f"\n--- Processing Layer {layer_num} (Z={target_layer_z if target_layer_z is not None else 'Unknown'}) ---")
        wall_loops_for_layer = layer_data_to_process.get('wall_loops', [])
        original_infill_for_layer = layer_data_to_process.get('original_infill_segments', [])
        travel_segments_for_layer = layer_data_to_process.get('travel_segments', [])

        # --- 2. Identify Candidate Regions (Shapely) ---
        print_normal("\n--- Stage 2: Identifying Candidate Regions ---")
        stage_start_time = time.time()
        candidate_regions = []
        if wall_loops_for_layer:
            candidate_regions = geometry_operations.identify_candidate_regions_shapely(
                wall_loops_for_layer, cfg.MIN_AREA_THRESH_FLOAT, cfg.COORD_EPSILON
            )
        print_normal(f"Identified {len(candidate_regions)} candidate regions.")
        print_normal(f"Time for identifying candidate regions: {time.time() - stage_start_time:.3f} seconds")

        # --- 3. Classify Regions ---
        print_normal("\n--- Stage 3: Classifying Regions for Infill ---")
        classified_regions_to_use = []
        if candidate_regions:
            stage_start_time_classify = time.time()
            print_normal(f"Classifying {len(candidate_regions)} candidate regions...")
            for region_idx, region_data in enumerate(candidate_regions):
                shapely_poly_region = region_data['shapely_poly']
                test_poly = shapely_poly_region
                try:
                    buffered = shapely_poly_region.buffer(-cfg.COORD_EPSILON / 50.0, resolution=4)
                    if buffered and not buffered.is_empty and buffered.is_valid:
                        test_poly = buffered
                except Exception:
                    pass

                region_infill_segments_found = []
                if original_infill_for_layer and test_poly.is_valid and test_poly.area > (cfg.MIN_AREA_THRESH_FLOAT / 100.0):
                    points_to_test_per_segment = 3
                    for seg_idx, (seg_start, seg_end) in enumerate(original_infill_for_layer):
                        if gcode_parser.segment_length_sq(seg_start, seg_end) < (cfg.min_segment_dist_threshold)**2:
                            continue
                        segment_fully_within = True
                        for i_pt in range(1, points_to_test_per_segment + 1):
                            if not segment_fully_within: break
                            ratio = i_pt / (points_to_test_per_segment + 1)
                            pt_x = seg_start[0] * (1 - ratio) + seg_end[0] * ratio
                            pt_y = seg_start[1] * (1 - ratio) + seg_end[1] * ratio
                            test_point_shapely = Point(pt_x, pt_y)
                            try:
                                if not test_poly.contains(test_point_shapely):
                                    segment_fully_within = False; break
                            except Exception:
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

                region_data['contains_infill'] = contains_significant_infill_flag
                region_data['longest_track'] = longest_infill_track_in_region_coords
                classified_regions_to_use.append(region_data)
            print_normal(f"Time for classifying regions: {time.time() - stage_start_time_classify:.3f} seconds")
        else:
            print_normal("No candidate regions to classify.")

        # --- Plotting Setup ---
        fig, ax = None, None
        if cfg.ENABLE_PLOTTING and plotting_utils.MATPLOTLIB_AVAILABLE:
            # Only create matplotlib objects if this is the layer we want to plot
            if layer_num == cfg.PLOT_LAYER_NUMBER:
                fig, ax = plotting_utils.setup_plot()
                print_normal(f"\nPreparing to plot layer {layer_num}")
            else:
                print_normal(f"Skipping plot setup for layer {layer_num} (not the target layer)")
        elif cfg.ENABLE_PLOTTING and not plotting_utils.MATPLOTLIB_AVAILABLE:
            print_normal("\nMatplotlib not available. Plotting will be skipped.")
        else:
            print_normal("\nPlotting disabled via configuration.")

        plot_title_main_str = ""
        all_toolpaths_ordered_for_gcode = []

        # --- PLOT_MODE specific logic ---
        if cfg.PLOT_MODE == "classified_regions":
            print_normal("\n--- Mode: Classified Regions ---")
            
        elif cfg.PLOT_MODE == "masked_grid":
            print_normal("\n--- Mode: Masked Grid (for Plotting and G-code Generation) ---")
            
            # --- 4. Generate and Mask Grid ---
            print_normal("\n--- Stage 4: Generating Masked Grid Cells ---")
            stage_start_time = time.time()
            masked_grid_cells_data_list = geometry_operations.generate_and_mask_grid(
                classified_regions_to_use, wall_loops_for_layer, travel_segments_for_layer,
                cfg.GRID_CELL_SIZE, cfg.GRID_BOUNDS_BUFFER, cfg.INFILL_REGION_SHRINK_OFFSET,
                cfg.COORD_EPSILON, cfg.MIN_AREA_THRESH_FLOAT,
                cfg.X_ORDER_LEFT_TO_RIGHT, cfg.Y_ORDER_TOP_TO_BOTTOM
            )
            print_normal(f"Generated {len(masked_grid_cells_data_list)} masked grid cells.")
            print_normal(f"Time for generating and masking grid: {time.time() - stage_start_time:.3f} seconds")

            if masked_grid_cells_data_list:
                # --- 5. Generate Infill for each Grid Cell ---
                print_normal("\n--- Stage 5: Generating Infill within Grid Cells ---")
                stage_start_time = time.time()
                current_infill_angle_idx = 0
                for cell_number, cell_data_dict in enumerate(masked_grid_cells_data_list):
                    grid_x_idx = cell_data_dict['grid_x_idx']
                    grid_y_idx = cell_data_dict['grid_y_idx']
                    cell_data_dict['column_id'] = (grid_x_idx, grid_y_idx)
                    cell_data_dict['execution_order'] = cell_number + 1  # <-- ADD THIS LINE

                    cell_x = -25.0 + grid_x_idx * cfg.GRID_CELL_SIZE
                    cell_y = -25.0 + grid_y_idx * cfg.GRID_CELL_SIZE
                    cell_poly = cell_data_dict['shapely_poly_clipped']

                    prev_cell = prev_cells_by_idx.get((grid_x_idx, grid_y_idx))
                    equivalent = False
                    if prev_cell:
                        equivalent = cells_are_equivalent(cell_data_dict, prev_cell)
                    
                    # --- Only recompute if not equivalent or no previous cell ---
                    if prev_cell and equivalent:
                        reused_routing = routing_cache.get_routing((grid_x_idx, grid_y_idx))
                        if reused_routing is not None:
                            cell_data_dict['infill_segments'] = reused_routing
                            cell_data_dict['routing_reused'] = True
                            # Set any other fields you need for plotting or downstream logic
                            continue  # Skip recomputation

                    # If not reused, recompute as usual
                    current_angle = cfg.INFILL_ANGLES_CYCLE[current_infill_angle_idx]
                    cell_infill_elements, cell_start_pt, cell_end_pt = \
                        geometry_operations.generate_infill_segments_for_polygon(
                            cell_poly, current_angle, cfg.INFILL_LINE_SPACING,
                            cfg.COORD_EPSILON, cfg.X_ORDER_LEFT_TO_RIGHT, cfg.Y_ORDER_TOP_TO_BOTTOM
                        )
                    cell_data_dict['infill_segments'] = cell_infill_elements
                    cell_data_dict['infill_angle'] = current_angle
                    cell_data_dict['execution_order'] = cell_number + 1
                    cell_data_dict['cell_infill_start_pt'] = cell_start_pt
                    cell_data_dict['cell_infill_end_pt'] = cell_end_pt
                    cell_data_dict['column_id'] = (grid_x_idx, grid_y_idx)
                    routing_cache.update_routing(cell_data_dict['column_id'], cell_infill_elements)
                    cell_data_dict['routing_reused'] = False

                    # Update the previous cells dictionary
                    prev_cells_by_idx[(grid_x_idx, grid_y_idx)] = cell_data_dict

                    if cell_poly.boundary and cell_start_pt and cell_end_pt:
                        try:
                            poly_boundary = cell_poly.boundary
                            if poly_boundary.geom_type == 'MultiLineString' and len(list(poly_boundary.geoms)) == 1:
                                poly_boundary = list(poly_boundary.geoms)[0]
                            elif poly_boundary.geom_type != 'LineString':
                                poly_boundary = None
                            if poly_boundary and poly_boundary.length > cfg.COORD_EPSILON:
                                cell_data_dict['cell_infill_start_pt_on_boundary'] = poly_boundary.interpolate(poly_boundary.project(Point(cell_start_pt))).coords[0]
                                cell_data_dict['cell_infill_end_pt_on_boundary'] = poly_boundary.interpolate(poly_boundary.project(Point(cell_end_pt))).coords[0]
                            else:
                                cell_data_dict['cell_infill_start_pt_on_boundary'] = None
                                cell_data_dict['cell_infill_end_pt_on_boundary'] = None
                        except Exception:
                            cell_data_dict['cell_infill_start_pt_on_boundary'] = None
                            cell_data_dict['cell_infill_end_pt_on_boundary'] = None
                    else:
                        cell_data_dict['cell_infill_start_pt_on_boundary'] = None
                        cell_data_dict['cell_infill_end_pt_on_boundary'] = None
                    current_infill_angle_idx = (current_infill_angle_idx + 1) % len(cfg.INFILL_ANGLES_CYCLE)
                print_normal(f"Time for infill generation for all cells: {time.time() - stage_start_time:.3f} seconds")

                # --- 6. Assemble Inter-Cell Connections & Final Toolpath Order ---
                print_normal("\n--- Stage 6: Assembling Inter-Cell Connections and Final Toolpaths ---")
                inter_cell_conn_assembly_start_time = time.time()
                boundary_graph_for_paths = None
                if NETWORKX_AVAILABLE:
                    boundary_graph_for_paths = pathfinding.build_boundary_graph(masked_grid_cells_data_list, cfg.COORD_EPSILON)
                    if boundary_graph_for_paths and boundary_graph_for_paths.number_of_nodes() > 0:
                        print_normal(f"Boundary graph built with {boundary_graph_for_paths.number_of_nodes()} nodes and {boundary_graph_for_paths.number_of_edges()} edges.")
                    else:
                        print_normal("Warning: Boundary graph is empty or could not be built. Inter-cell connections default to direct jumps.")

                for i, current_cell_data_dict in enumerate(masked_grid_cells_data_list):
                    if i > 0:
                        prev_cell_data_dict = masked_grid_cells_data_list[i - 1]
                        from_id = prev_cell_data_dict['column_id']
                        to_id = current_cell_data_dict['column_id']
                        from_pt_boundary_prev = prev_cell_data_dict.get('cell_infill_end_pt_on_boundary')
                        to_pt_boundary_curr = current_cell_data_dict.get('cell_infill_start_pt_on_boundary')
                        from_pt_internal_prev = prev_cell_data_dict.get('cell_infill_end_pt')
                        to_pt_internal_curr = current_cell_data_dict.get('cell_infill_start_pt')

                        connection_path_segments_added = False
                        if NETWORKX_AVAILABLE and boundary_graph_for_paths and boundary_graph_for_paths.number_of_nodes() > 0 and \
                           from_pt_boundary_prev and to_pt_boundary_curr and from_pt_internal_prev and to_pt_internal_curr:
                            cached_path = connection_cache.get_connection(from_id, to_id)
                            if cached_path is not None:
                                all_toolpaths_ordered_for_gcode.append((cached_path, "inter_cell_boundary_connection"))
                                connection_path_segments_added = True
                            else:
                                path_on_boundary_line = pathfinding.find_shortest_path_on_boundary(
                                    boundary_graph_for_paths, from_pt_boundary_prev, to_pt_boundary_curr,
                                    cfg.NODE_MERGE_EPSILON, cfg.ON_SEGMENT_EPSILON, cfg.ENDPOINT_DISTINCTION_EPSILON,
                                    cfg.COORD_EPSILON, cfg.GRID_CELL_SIZE
                                )
                                if path_on_boundary_line and path_on_boundary_line.length > cfg.COORD_EPSILON / 10:
                                    all_toolpaths_ordered_for_gcode.append((path_on_boundary_line, "inter_cell_boundary_connection"))
                                    connection_cache.update_connection(from_id, to_id, path_on_boundary_line)
                                    connection_path_segments_added = True

                        if not connection_path_segments_added and from_pt_internal_prev and to_pt_internal_curr:
                            if not gcode_parser.are_points_close(from_pt_internal_prev, to_pt_internal_curr, cfg.COORD_EPSILON):
                                all_toolpaths_ordered_for_gcode.append((LineString([from_pt_internal_prev, to_pt_internal_curr]), "inter_cell_direct_jump"))

                    if current_cell_data_dict['infill_segments']:
                        all_toolpaths_ordered_for_gcode.extend(current_cell_data_dict['infill_segments'])
                print_normal(f"Assembled {len(all_toolpaths_ordered_for_gcode)} toolpath segments.")
                print_normal(f"Time for inter-cell connections and toolpath assembly: {time.time() - inter_cell_conn_assembly_start_time:.3f} seconds")

                # --- Clean up toolpaths by clipping against model boundary
                if all_toolpaths_ordered_for_gcode:
                    print_normal("\n--- Cleaning up toolpaths with boundary clipping ---")
                    stage_start_time = time.time()
                    
                    # Create combined boundary from all classified regions
                    from shapely.ops import unary_union
                    try:
                        combined_boundary = unary_union([region['shapely_poly'] for region in classified_regions_to_use])
                        
                        # Clip all toolpaths against the combined boundary
                        clipped_toolpaths = []
                        skipped_count = 0
                        
                        for path, path_type in all_toolpaths_ordered_for_gcode:
                            try:
                                if path.intersects(combined_boundary):
                                    clipped_path = path.intersection(combined_boundary)
                                    
                                    # Only keep non-empty geometries
                                    if not clipped_path.is_empty:
                                        # Handle different geometry types that might result from intersection
                                        if clipped_path.geom_type == 'LineString':
                                            clipped_toolpaths.append((clipped_path, path_type))
                                        elif clipped_path.geom_type == 'MultiLineString':
                                            # Break multiline into individual line segments
                                            for line in clipped_path.geoms:
                                                if len(list(line.coords)) >= 2:  # Only keep lines with at least 2 points
                                                    clipped_toolpaths.append((line, path_type))
                                        # Skip GeometryCollection, Point, etc.
                                    else:
                                        skipped_count += 1
                                else:
                                    skipped_count += 1
                            except Exception as e:
                                print(f"Error processing path: {e}")
                        
                        # Replace original toolpaths with clipped ones
                        print_normal(f"Toolpath cleanup: {len(all_toolpaths_ordered_for_gcode)} → {len(clipped_toolpaths)} paths")
                        print_normal(f"Removed {skipped_count} paths completely outside boundary")
                        all_toolpaths_ordered_for_gcode = clipped_toolpaths
                        
                        print_normal(f"Time for toolpath boundary clipping: {time.time() - stage_start_time:.3f} seconds")
                    except Exception as e:
                        print(f"WARNING: Toolpath boundary clipping failed: {e}")
                        print_normal("Continuing with original toolpaths")

                if fig and ax:
                    plotting_utils.plot_masked_grid_on_ax(ax, masked_grid_cells_data_list, all_toolpaths_ordered_for_gcode)
            else:
                print_normal("No masked grid cells were generated, skipping infill and G-code generation.")

        # --- Common Plotting Elements & Finalize Plot (if plotting is active) ---
        if cfg.ENABLE_PLOTTING and fig and ax:
            # Only generate an SVG file for the specific layer number configured in cfg.PLOT_LAYER_NUMBER
            if layer_num == cfg.PLOT_LAYER_NUMBER:
                print_normal("\n--- Finalizing Plot ---")
                stage_start_time = time.time()
                
                plotting_utils.plot_common_elements_on_ax(ax, wall_loops_for_layer, travel_segments_for_layer)
                
                # Use per-layer output file name
                output_plot_file = str(cfg.OUTPUT_DIRECTORY / f"infill_layer_{layer_num}_plot_v_final.svg")
                
                # Always show the plot for the specifically chosen layer
                show_plot = True
                
                plotting_utils.finalize_plot(fig, ax, plot_title_main_str, output_plot_file, show_plot=show_plot)
                print_normal(f"Time for plotting layer {layer_num}: {time.time() - stage_start_time:.3f} seconds")
            else:
                # For all other layers, don't generate SVG files
                print_normal(f"Skipping plot generation for layer {layer_num} (not the configured plotting layer)")
                
                # Clear the plot for the next layer to avoid memory buildup
                if fig and ax:
                    ax.clear()

        # --- 7. G-code Generation (if enabled for masked_grid mode) ---
        print_normal(f"[DEBUG] PLOT_MODE: {cfg.PLOT_MODE}, ENABLE_GCODE_GENERATION: {cfg.ENABLE_GCODE_GENERATION}")
        print_normal(f"[DEBUG] all_toolpaths_ordered_for_gcode length: {len(all_toolpaths_ordered_for_gcode)}")
        print_normal(f"[DEBUG] target_layer_z: {target_layer_z}")

        if cfg.PLOT_MODE == "masked_grid" and cfg.ENABLE_GCODE_GENERATION:
            print_normal("\n--- Stage 7: G-code Generation ---")
            if not all_toolpaths_ordered_for_gcode:
                print_normal("No toolpaths were generated for G-code output. Skipping G-code generation.")
            elif target_layer_z is None:
                print_normal(f"SKIPPING G-code generation: Z-height for layer {layer_num} could not be determined.")
            else:
                # Include wall segments first in the toolpath list
                complete_toolpaths = []
                
                # Add wall segments with appropriate classification
                for wall_loop in wall_loops_for_layer:
                    for i in range(len(wall_loop) - 1):
                        # Create LineString for each wall segment
                        wall_segment = LineString([wall_loop[i], wall_loop[i+1]])
                        complete_toolpaths.append((wall_segment, "wall_segment"))
                
                # Then add infill and connections
                complete_toolpaths.extend(all_toolpaths_ordered_for_gcode)
                
                # Generate G-code lines for this layer
                layer_gcode_lines = gcode_generator.generate_gcode_for_layer(
                    complete_toolpaths, target_layer_z,
                    cfg.FEEDRATE_PRIMARY_INFILL,
                    cfg.FEEDRATE_BOUNDARY_CONNECTION,
                    cfg.FEEDRATE_INTRA_CELL_CONNECTION,
                    cfg.FEEDRATE_TRAVEL,
                    cfg.FEEDRATE_WALL,
                    cfg.COORD_EPSILON
                )
                
                # For the target layer, always create a separate file
                if layer_num == cfg.PLOT_LAYER_NUMBER:
                    target_layer_output_path = str(cfg.OUTPUT_DIRECTORY / f"layer_{layer_num}_only.gcode")
                    
                    # Add start, layer content, and end stubs to make a complete file
                    try:
                        with open(target_layer_output_path, 'w') as f:
                            # Write start stub
                            f.write(cfg.GCODE_STUB_START + '\n')
                            
                            # Write layer G-code
                            for line in layer_gcode_lines:
                                f.write(line + '\n')
                            
                            # Write end stub
                            f.write(cfg.GCODE_STUB_END + '\n')
                        
                        print_normal(f"Created separate G-code file for layer {layer_num}: {target_layer_output_path}")
                    except IOError as e:
                        print(f"ERROR: Could not write separate layer G-code file: {e}")
                
                # Continue with combined file if enabled
                if cfg.SINGLE_GCODE_FILE_OUTPUT:
                    # Get next layer's Z height if available
                    next_layer_nums = [l for l in gcode_data_by_layer.keys() if l > layer_num]
                    next_z = None
                    if next_layer_nums:
                        next_layer = min(next_layer_nums)
                        next_z = layer_z_values.get(next_layer)
                    
                    # Append to the combined file
                    combined_gcode_path = str(cfg.OUTPUT_DIRECTORY / cfg.SINGLE_GCODE_OUTPUT_FILENAME)
                    gcode_generator.append_layer_to_combined_gcode(
                        combined_gcode_path, 
                        layer_gcode_lines,
                        cfg.GCODE_STUB_LAYER_TRANSITION
                    )

        print_normal(f"\nTotal time for processing layer {layer_num}: {time.time() - processing_section_start_time:.3f} seconds")

    # Finalize the combined G-code file if we're using that mode
    if cfg.SINGLE_GCODE_FILE_OUTPUT:
        combined_gcode_path = str(cfg.OUTPUT_DIRECTORY / cfg.SINGLE_GCODE_OUTPUT_FILENAME)
        gcode_generator.finalize_combined_gcode_file(combined_gcode_path, cfg.GCODE_STUB_END)
        print(f"\n--- Combined G-code file created: {combined_gcode_path} ---")

    print_normal(f"\n--- Total script execution time: {time.time() - overall_start_time:.3f} seconds ---")

# Modify the generate_gcode_for_layer function to use G1 for all moves:

def generate_gcode_for_layer(
    ordered_toolpaths, layer_z_height,
    cfg_feedrate_primary,
    cfg_feedrate_boundary,
    cfg_feedrate_intra_conn,
    cfg_feedrate_travel,
    cfg_feedrate_wall,
    cfg_coord_epsilon=0.01):
    """
    Generate G-code lines for a single layer without writing to a file.
    Modified for laser-based LPBF printer (no extrusion references).
    All moves use G1 for proper interpolation and timing control.
    """
    gcode_lines = []
    current_x, current_y, current_z = None, None, None
    
    # Ensure layer_z_height is a float
    try:
        target_z = float(layer_z_height)
    except (ValueError, TypeError):
        print(f"ERROR: Invalid layer_z_height for G-code generation: {layer_z_height}.")
        return gcode_lines
    
    # Just include a minimal layer comment (no header)
    gcode_lines.append(f"; Layer at Z={target_z:.3f}")
    
    # Initial move to layer Z height - use G1 instead of G0
    gcode_lines.append(f"G1 Z{target_z:.3f} F{int(cfg_feedrate_travel)} ; Move to layer height")
    current_z = target_z
    
    first_xy_move_in_layer_sequence = True
    laser_is_on = False  # Track laser state
    
    for toolpath_idx, toolpath_item in enumerate(ordered_toolpaths):
        if not isinstance(toolpath_item, tuple) or len(toolpath_item) != 2:
            continue
            
        line_string_obj, segment_type = toolpath_item
        
        if not line_string_obj or not hasattr(line_string_obj, 'coords') or not list(line_string_obj.coords):
            continue
            
        points = list(line_string_obj.coords)
        if len(points) < 2:
            continue
            
        # Determine if this segment requires laser on or off
        requires_laser_on = segment_type in ["primary_infill", "cell_infill", "wall_segment", "inter_cell_boundary_connection"]
        
        # Determine feedrate based on segment type
        if segment_type == "primary_infill" or segment_type == "cell_infill":
            current_feedrate = cfg_feedrate_primary
        elif segment_type == "wall_segment":
            current_feedrate = cfg_feedrate_wall
        elif segment_type == "inter_cell_boundary_connection":
            current_feedrate = cfg_feedrate_boundary
        elif segment_type in ["retrace", "inter_cell_direct_jump", "inter_cell_direct_jump_part"]:
            current_feedrate = cfg_feedrate_travel
        else:
            current_feedrate = cfg_feedrate_primary
            
        # Always use G1 for all moves to ensure interpolation
        gcode_prefix = "G1"
        
        # Iterate through segments within the LineString
        for i in range(len(points) - 1):
            p_start = points[i]
            p_end = points[i+1]
            
            target_x_val, target_y_val = round(p_end[0], 3), round(p_end[1], 3)
            
            # Skip zero-length segments
            if gcode_parser.are_points_close(p_start, p_end, cfg_coord_epsilon / 1000.0):
                continue
                
            # Handle initial positioning or discontinuities
            if first_xy_move_in_layer_sequence:
                start_x_val_initial, start_y_val_initial = round(p_start[0], 3), round(p_start[1], 3)
                # Use G1 instead of G0 for initial positioning
                gcode_lines.append(f"G1 X{start_x_val_initial} Y{start_y_val_initial} F{int(cfg_feedrate_travel)} ; Initial position")
                current_x, current_y = start_x_val_initial, start_y_val_initial
                first_xy_move_in_layer_sequence = False
            elif current_x is not None and current_y is not None and \
                 not gcode_parser.are_points_close((current_x, current_y), p_start, cfg_coord_epsilon * 2.0):
                # Discontinuity detected - need to move to new start position
                if laser_is_on:
                    gcode_lines.append("M5 ; laser off for repositioning")
                    laser_is_on = False
                
                start_x_val_jump, start_y_val_jump = round(p_start[0], 3), round(p_start[1], 3)
                # Use G1 instead of G0 for repositioning
                gcode_lines.append(f"G1 X{start_x_val_jump} Y{start_y_val_jump} F{int(cfg_feedrate_travel)} ; Reposition")
                current_x, current_y = start_x_val_jump, start_y_val_jump
            
            # Turn laser on/off as needed
            if requires_laser_on and not laser_is_on:
                gcode_lines.append("M3 S100 ; laser on")
                laser_is_on = True
            elif not requires_laser_on and laser_is_on:
                gcode_lines.append("M5 ; laser off")
                laser_is_on = False
                
            # Generate the move command
            gcode_command = f"{gcode_prefix} X{target_x_val} Y{target_y_val} F{int(current_feedrate)}"
            gcode_lines.append(gcode_command)
            
            current_x, current_y = target_x_val, target_y_val
    
    # Ensure laser is off at end of layer
    if laser_is_on:
        gcode_lines.append("M5 ; laser off at layer end")
    
    return gcode_lines
