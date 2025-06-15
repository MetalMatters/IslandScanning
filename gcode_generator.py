# gcode_generator.py
import math
import time
from pathlib import Path

# Assuming gcode_parser.are_points_close and gcode_parser.dist_sq are available
# If this file is in the same package, a relative import is good.
from . import gcode_parser # For are_points_close, dist_sq
# from . import config as cfg # If you need to access config directly, though prefer passing values

def generate_gcode_from_toolpaths(
    ordered_toolpaths,    layer_z_height,
    output_gcode_filepath_str,
    cfg_feedrate_primary,
    cfg_feedrate_boundary,
    cfg_feedrate_intra_conn,
    cfg_feedrate_travel,
    cfg_extrusion_multiplier,
    cfg_initial_e_value=0.0,
    cfg_coord_epsilon=0.01):

    print(f"[DEBUG] Entered generate_gcode_from_toolpaths")
    print(f"[DEBUG] Number of toolpaths: {len(ordered_toolpaths)}")
    print(f"[DEBUG] Output file: {output_gcode_filepath_str}")
    print(f"[DEBUG] Layer Z height: {layer_z_height}")

    gcode_lines = []
    current_x, current_y, current_z, current_e = None, None, None, cfg_initial_e_value
    
    # Ensure layer_z_height is a float
    try:
        target_z = float(layer_z_height)
    except (ValueError, TypeError):
        print(f"ERROR: Invalid layer_z_height for G-code generation: {layer_z_height}. Aborting G-code generation.")
        return

    # Standard G-code Header
    gcode_lines.append(f"; Generated G-code for layer at Z={target_z:.3f}")
    gcode_lines.append(f"; Script: {Path(__file__).name if '__file__' in globals() else 'interactive_script'}") # Get script name
    gcode_lines.append(f"; Generation Time: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    gcode_lines.append("")
    gcode_lines.append("G21 ; Set units to millimeters")
    gcode_lines.append("G90 ; Set to absolute positioning")
    gcode_lines.append("M82 ; Set extruder to absolute mode")
    # Optional: Add fan commands M106/M107 if needed
    # Optional: Add comments about infill type, angle, etc.

    # Initial move to layer Z height (if not already there)
    # This assumes previous G-code might have left Z at a different height.
    # A G0 FXXXX is used for Z moves typically.
    gcode_lines.append(f"G0 Z{target_z:.3f} F{int(cfg_feedrate_travel)} ; Move to layer height")
    current_z = target_z

    first_xy_move_in_layer_sequence = True # Tracks if it's the very first XY move of this generated sequence

    for toolpath_idx, (line_string_obj, segment_type) in enumerate(ordered_toolpaths):
        if not line_string_obj or not hasattr(line_string_obj, 'coords') or not list(line_string_obj.coords):
            # print(f"Warning: Skipping empty or invalid LineString in toolpath element {toolpath_idx}.")
            continue

        points = list(line_string_obj.coords) # List of (x,y) tuples
        if len(points) < 2:
            # print(f"Warning: LineString with < 2 points in toolpath element {toolpath_idx}. Skipping.")
            continue

        feedrate_to_use = None
        extrude_this_segment_type = False
        gcode_prefix = "G0" # Default to travel

        if segment_type == "primary":
            feedrate_to_use = cfg_feedrate_primary
            extrude_this_segment_type = True
            gcode_prefix = "G1"
        elif segment_type == "inter_cell_boundary_connection":
            feedrate_to_use = cfg_feedrate_boundary
            extrude_this_segment_type = True
            gcode_prefix = "G1"
        elif segment_type == "connection": # Intra-cell connection
            feedrate_to_use = cfg_feedrate_intra_conn
            extrude_this_segment_type = True
            gcode_prefix = "G1"
        elif segment_type in ["retrace", "inter_cell_direct_jump", "inter_cell_direct_jump_part"]:
            feedrate_to_use = cfg_feedrate_travel
            extrude_this_segment_type = False
            gcode_prefix = "G0"
        else:
            # print(f"Warning: Unknown segment type '{segment_type}' for G-code generation. Treating as travel.")
            feedrate_to_use = cfg_feedrate_travel
            extrude_this_segment_type = False
            gcode_prefix = "G0"
        
        # Iterate through segments within the LineString
        for i in range(len(points) - 1):
            p_start = points[i]
            p_end = points[i+1]

            target_x_val, target_y_val = round(p_end[0], 3), round(p_end[1], 3)

            # Skip zero-length segments that might arise from rounding or geometry issues
            # Use a very tight epsilon for this check, specific to G-code generation precision.
            if gcode_parser.are_points_close(p_start, p_end, cfg_coord_epsilon / 1000.0):
                continue

            gcode_command_parts = [gcode_prefix]

            # Handle initial XY positioning or discontinuities
            if first_xy_move_in_layer_sequence:
                start_x_val_initial, start_y_val_initial = round(p_start[0], 3), round(p_start[1], 3)
                gcode_lines.append(f"G0 X{start_x_val_initial} Y{start_y_val_initial} F{int(cfg_feedrate_travel)} ; Initial XY position for generated sequence")
                current_x, current_y = start_x_val_initial, start_y_val_initial
                first_xy_move_in_layer_sequence = False
            # Check if current_x/y are set and if there's a jump needed from previous segment's end
            elif current_x is not None and current_y is not None and \
                 not gcode_parser.are_points_close((current_x, current_y), p_start, cfg_coord_epsilon * 2.0): # Allow slightly larger epsilon for chaining
                # Discontinuity: current position doesn't match start of this segment. Travel to it.
                start_x_val_jump, start_y_val_jump = round(p_start[0], 3), round(p_start[1], 3)
                gcode_lines.append(f"G0 X{start_x_val_jump} Y{start_y_val_jump} F{int(cfg_feedrate_travel)}")
                current_x, current_y = start_x_val_jump, start_y_val_jump

            # Add X and Y coordinates
            gcode_command_parts.append(f"X{target_x_val}")
            gcode_command_parts.append(f"Y{target_y_val}")
            # Z is assumed to be modal from the initial G0 Z command for the layer.

            # REMOVE E value for laser G-code
            # if extrude_this_segment_type:
            #     segment_length = math.sqrt(gcode_parser.dist_sq(p_start, p_end)) # Use original points for length
            #     delta_e = cfg_extrusion_multiplier * segment_length
            #     current_e += delta_e
            #     gcode_command_parts.append(f"E{current_e:.5f}") # Typical E precision

            if feedrate_to_use is not None:
                gcode_command_parts.append(f"F{int(feedrate_to_use)}")

            gcode_lines.append(" ".join(gcode_command_parts))
            current_x, current_y = target_x_val, target_y_val # Update current position to the end of this segment

    # Standard G-code Footer
    gcode_lines.append("")
    gcode_lines.append(f"; End of generated layer G-code for Z={target_z:.3f}")
    gcode_lines.append(f"; Final E value for this layer: {current_e:.5f}") # Informative
    # Optional: Add M107 (Fan Off), etc. if this G-code is meant to be standalone.

    try:
        print("[DEBUG] Attempting to write G-code file...")
        with open(output_gcode_filepath_str, 'w') as f:
            for line in gcode_lines:
                f.write(line + '\n')
        print(f"SUCCESS: Generated G-code saved to: {output_gcode_filepath_str}")
    except IOError as e:
        print(f"ERROR: Could not write G-code file to '{output_gcode_filepath_str}': {e}")

    print("[DEBUG] Exiting generate_gcode_from_toolpaths")