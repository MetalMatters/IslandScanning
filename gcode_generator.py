# gcode_generator.py
import math
import time
from pathlib import Path

# Assuming gcode_parser.are_points_close and gcode_parser.dist_sq are available
# If this file is in the same package, a relative import is good.
from . import gcode_parser # For are_points_close, dist_sq
from . import laser_control
from . import config as cfg

def generate_gcode_from_toolpaths(
    ordered_toolpaths,    layer_z_height,
    output_gcode_filepath_str,
    cfg_feedrate_primary,
    cfg_feedrate_boundary,
    cfg_feedrate_intra_conn,
    cfg_feedrate_travel,
    cfg_feedrate_wall,  # Add this parameter
    cfg_extrusion_multiplier,
    cfg_initial_e_value=0.0,
    cfg_coord_epsilon=0.01):

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

        # Determine feedrate based on segment type
        if segment_type == "primary_infill" or segment_type == "cell_infill":
            current_feedrate = cfg_feedrate_primary
        elif segment_type == "wall_segment":  # Add this condition
            current_feedrate = cfg_feedrate_wall
        elif segment_type == "inter_cell_boundary_connection":
            current_feedrate = cfg_feedrate_boundary
        elif segment_type == "inter_cell_direct_jump":
            current_feedrate = cfg_feedrate_intra_conn
        else:  # Default to primary infill rate for any unspecified types
            current_feedrate = cfg_feedrate_primary

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
        # Ensure the directory exists
        output_path = Path(output_gcode_filepath_str)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        
        # Write the file
        with open(output_gcode_filepath_str, 'w') as f:
            for line in gcode_lines:
                f.write(line + '\n')
    except IOError as e:
        print(f"ERROR: Could not write G-code file to '{output_gcode_filepath_str}': {e}")

# Add these new functions

def initialize_combined_gcode_file(output_filepath, start_stub):
    """Initialize the combined G-code file with the start stub."""
    try:
        # Create the directory if it doesn't exist
        output_path = Path(output_filepath)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        
        # Generate customized start G-code with proper laser control
        customized_start = laser_control.create_start_gcode(
            power=cfg.LASER_POWER, 
            laser_ramp_file=cfg.LASER_RAMP_FILE
        )
        
        # Write the start stub to the file
        with open(output_filepath, 'w') as f:
            f.write(customized_start + '\n')
            
    except IOError as e:
        print(f"ERROR: Could not initialize combined G-code file '{output_filepath}': {e}")

def append_layer_to_combined_gcode(output_filepath, layer_gcode, transition_stub_template):
    """Append a layer's G-code to the combined file with a transition stub."""
    try:
        # Generate customized layer transition
        customized_transition = laser_control.create_layer_transition(
            power=cfg.LASER_POWER
        )
        
        # Append both layer G-code and transition in a single operation
        # to avoid unwanted spacing
        with open(output_filepath, 'a') as f:
            # Write regular layer G-code
            for i, line in enumerate(layer_gcode):
                f.write(line + '\n')
            
            # Write transition stub with ONLY ONE newline between
            f.write(customized_transition + '\n')
            
    except IOError as e:
        print(f"ERROR: Could not append layer to combined G-code file '{output_filepath}': {e}")

def finalize_combined_gcode_file(output_filepath, end_stub):
    """Finalize the combined G-code file with the end stub."""
    try:
        # Generate customized end G-code with proper laser control
        customized_end = laser_control.create_end_gcode(g98_boundary=cfg.G98_BOUNDARY)
        
        # Append the end stub to the file
        with open(output_filepath, 'a') as f:
            f.write('\n' + customized_end + '\n')
            
    except IOError as e:
        print(f"ERROR: Could not finalize combined G-code file '{output_filepath}': {e}")

def generate_gcode_for_layer(
    ordered_toolpaths, layer_z_height,
    cfg_feedrate_primary,
    cfg_feedrate_boundary,
    cfg_feedrate_intra_conn,
    cfg_feedrate_travel,
    cfg_feedrate_wall,
    cfg_coord_epsilon=0.01):
    """
    Generate G-code lines for a single layer.
    All moves use G1 for proper interpolation and timing control.
    Laser is controlled only at layer boundaries (G99/G98), not within a layer.
    No Z movements are included - G97 will handle layer transitions.
    """
    gcode_lines = []
    current_x, current_y = None, None
    
    # No Z movement - G97 handles this
    # No laser on/off commands - handled at layer boundaries
    
    first_xy_move_in_layer_sequence = True
    
    for toolpath_idx, toolpath_item in enumerate(ordered_toolpaths):
        if not isinstance(toolpath_item, tuple) or len(toolpath_item) != 2:
            continue
            
        line_string_obj, segment_type = toolpath_item
        
        if not line_string_obj or not hasattr(line_string_obj, 'coords') or not list(line_string_obj.coords):
            continue
            
        points = list(line_string_obj.coords)
        if len(points) < 2:
            continue
            
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
            
        # Always use G1 for all moves
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
                gcode_lines.append(f"G1 X{start_x_val_initial} Y{start_y_val_initial} F{int(cfg_feedrate_travel)}")
                current_x, current_y = start_x_val_initial, start_y_val_initial
                first_xy_move_in_layer_sequence = False
            elif current_x is not None and current_y is not None and \
                 not gcode_parser.are_points_close((current_x, current_y), p_start, cfg_coord_epsilon * 2.0):
                # Discontinuity detected - need to move to new start position
                start_x_val_jump, start_y_val_jump = round(p_start[0], 3), round(p_start[1], 3)
                gcode_lines.append(f"G1 X{start_x_val_jump} Y{start_y_val_jump} F{int(cfg_feedrate_travel)}")
                current_x, current_y = start_x_val_jump, start_y_val_jump
            
            # Generate the move command
            gcode_command = f"{gcode_prefix} X{target_x_val} Y{target_y_val} F{int(current_feedrate)}"
            gcode_lines.append(gcode_command)
            
            current_x, current_y = target_x_val, target_y_val
    
    return gcode_lines
