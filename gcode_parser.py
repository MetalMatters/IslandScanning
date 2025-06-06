# gcode_parser.py
import re
from collections import defaultdict
from pathlib import Path

# Regexes and constants specific to G-code parsing
COMMENT_ANY_TYPE_RE = re.compile(r";\s*TYPE\s*:\s*(\S+)")
COMMENT_LAYER_RE = re.compile(r";\s*LAYER\s*:\s*(\d+)")
GCOMMAND_RE = re.compile(r"^(G[0123])\s*(.*)")
PARAM_RE = re.compile(r"([XYZEF])([-\d.]+)")
WALL_TYPES = {"WALL-OUTER", "WALL-INNER"}
INFILL_FEATURE_TYPES = {"FILL", "SKIN", "TOP-SURFACE-SKIN", "BOTTOM-SURFACE-SKIN"}

# Utility functions related to points and segments
def dist_sq(p1, p2):
    if p1 is None or p2 is None or p1[0] is None or p1[1] is None or p2[0] is None or p2[1] is None:
        return float('inf')
    return (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2

def segment_length_sq(p1, p2):
    return (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2

def are_points_close(p1, p2, epsilon_val): # Takes epsilon as an argument
    if p1 is None or p2 is None: return False
    return dist_sq(p1, p2) < epsilon_val**2

def extract_gcode_features(
    gcode_filepath: Path,
    cfg_abs_e_change_retraction,
    cfg_min_e_delta_printing,
    cfg_moved_xy_thresh,
    cfg_min_segment_len_eps_sq,
    cfg_coord_epsilon # For stitching
    ):
    current_pos = {'X': None, 'Y': None, 'Z': None, 'E': None}
    current_feature_type_str = "NONE"
    current_gcode_layer_num = -1
    is_retracted = False
    layer_wall_segments = defaultdict(list)
    layer_travel_segments = defaultdict(list)
    layer_original_infill_segments = defaultdict(list)
    layer_z_values = defaultdict(lambda: None)

    print(f"Parsing G-code file: {gcode_filepath}")
    with open(gcode_filepath, 'r') as f:
        for line_num, line_orig in enumerate(f, 1):
            line = line_orig.strip()
            type_match_for_line = COMMENT_ANY_TYPE_RE.search(line_orig)
            if type_match_for_line:
                current_feature_type_str = type_match_for_line.group(1)

            layer_match = COMMENT_LAYER_RE.search(line_orig)
            if layer_match:
                new_layer_num = int(layer_match.group(1))
                if new_layer_num != current_gcode_layer_num:
                    current_gcode_layer_num = new_layer_num
                    if current_pos['Z'] is not None and layer_z_values[current_gcode_layer_num] is None:
                         layer_z_values[current_gcode_layer_num] = current_pos['Z']

            line_content_for_gcode = line.split(';', 1)[0].strip()
            if not line_content_for_gcode: continue
            gcmd_match = GCOMMAND_RE.match(line_content_for_gcode)
            if not gcmd_match: continue
            command = gcmd_match.group(1)
            params_str = gcmd_match.group(2)
            params = dict(PARAM_RE.findall(params_str))

            pos_before_move = current_pos.copy()
            new_pos_temp = current_pos.copy()
            moved_xy = False
            for axis, val_str in params.items():
                try:
                    val = float(val_str)
                    if axis in new_pos_temp: new_pos_temp[axis] = val
                    if axis in ('X', 'Y'):
                        if current_pos[axis] is not None and abs(val - current_pos[axis]) > cfg_moved_xy_thresh:
                            moved_xy = True
                        elif current_pos[axis] is None and val is not None:
                            moved_xy = True
                    if axis == 'Z' and current_gcode_layer_num != -1 and layer_z_values[current_gcode_layer_num] is None:
                        layer_z_values[current_gcode_layer_num] = val
                except ValueError: pass

            for axis_init in ('X', 'Y', 'Z', 'E'):
                if current_pos[axis_init] is None and axis_init in params:
                    try:
                        val = float(params[axis_init])
                        current_pos[axis_init] = val
                        if axis_init == 'Z' and current_gcode_layer_num != -1 and layer_z_values[current_gcode_layer_num] is None:
                            layer_z_values[current_gcode_layer_num] = val
                    except ValueError: pass

            if command == 'G0':
                if moved_xy and \
                   pos_before_move['X'] is not None and pos_before_move['Y'] is not None and \
                   current_gcode_layer_num != -1:
                    p_start_travel = (pos_before_move['X'], pos_before_move['Y'])
                    p_end_travel = (new_pos_temp['X'], new_pos_temp['Y'])
                    if dist_sq(p_start_travel, p_end_travel) > cfg_min_segment_len_eps_sq:
                        layer_travel_segments[current_gcode_layer_num].append((p_start_travel, p_end_travel))
                current_pos = new_pos_temp
            elif command == 'G1':
                delta_e = 0.0
                has_e_param = 'E' in params
                if has_e_param and pos_before_move['E'] is not None and new_pos_temp.get('E') is not None:
                    delta_e = new_pos_temp['E'] - pos_before_move['E']

                is_retraction_cmd_move = False
                is_deretraction_cmd_move = False
                if has_e_param and delta_e < -cfg_abs_e_change_retraction:
                    is_retracted = True
                    is_retraction_cmd_move = True
                elif has_e_param and is_retracted and delta_e > cfg_abs_e_change_retraction:
                    is_retracted = False
                    is_deretraction_cmd_move = True

                if is_retraction_cmd_move or is_deretraction_cmd_move:
                    current_pos = new_pos_temp
                    continue

                if moved_xy:
                    p_start_g1 = (pos_before_move['X'], pos_before_move['Y'])
                    p_end_g1 = (new_pos_temp['X'], new_pos_temp['Y'])
                    can_create_segment = (pos_before_move['X'] is not None and
                                          pos_before_move['Y'] is not None and
                                          current_gcode_layer_num != -1 and
                                          dist_sq(p_start_g1, p_end_g1) > cfg_min_segment_len_eps_sq)
                    if can_create_segment:
                        is_a_printing_g1_move = (not is_retracted and
                                                 has_e_param and
                                                 delta_e > cfg_min_e_delta_printing)
                        is_current_type_wall = current_feature_type_str in WALL_TYPES
                        is_current_type_infill = current_feature_type_str in INFILL_FEATURE_TYPES

                        if is_current_type_wall and is_a_printing_g1_move:
                            layer_wall_segments[current_gcode_layer_num].append((p_start_g1, p_end_g1))
                        elif is_current_type_infill and is_a_printing_g1_move:
                            layer_original_infill_segments[current_gcode_layer_num].append(((p_start_g1), (p_end_g1)))
                        elif not is_a_printing_g1_move:
                            layer_travel_segments[current_gcode_layer_num].append((p_start_g1, p_end_g1))
                current_pos = new_pos_temp
            elif command == 'G92':
                if 'E' in params:
                    try:
                        if float(params['E']) == 0: is_retracted = False
                    except ValueError: pass
                for axis_g92 in ['X', 'Y', 'Z', 'E']:
                    if axis_g92 in params:
                        try: current_pos[axis_g92] = float(params[axis_g92])
                        except ValueError: pass

    stitched_wall_loops = defaultdict(list)
    print("Stitching wall segments into manifold loops...")
    for layer_num, segments in sorted(layer_wall_segments.items()):
        if not segments: continue
        num_segments = len(segments)
        visited = [False] * num_segments
        for i in range(num_segments):
            if visited[i]: continue
            current_loop_points = [segments[i][0], segments[i][1]]
            visited[i] = True
            while True:
                extended = False
                last_point_in_loop = current_loop_points[-1]
                best_candidate_idx, min_dist_sq_cand, candidate_reversed = -1, float('inf'), False
                for j in range(num_segments):
                    if visited[j]: continue
                    segment_j_start, segment_j_end = segments[j]
                    d_sq = dist_sq(last_point_in_loop, segment_j_start)
                    if d_sq < cfg_coord_epsilon**2 and d_sq < min_dist_sq_cand:
                        min_dist_sq_cand, best_candidate_idx, candidate_reversed = d_sq, j, False
                    d_sq_rev = dist_sq(last_point_in_loop, segment_j_end)
                    if d_sq_rev < cfg_coord_epsilon**2 and d_sq_rev < min_dist_sq_cand:
                        min_dist_sq_cand, best_candidate_idx, candidate_reversed = d_sq_rev, j, True
                if best_candidate_idx != -1:
                    chosen_segment = segments[best_candidate_idx]
                    visited[best_candidate_idx] = True
                    current_loop_points.append(chosen_segment[1] if not candidate_reversed else chosen_segment[0])
                    extended = True
                if not extended or (len(current_loop_points) > 1 and are_points_close(current_loop_points[0], current_loop_points[-1], cfg_coord_epsilon)):
                    break
            if len(current_loop_points) > 1:
                if are_points_close(current_loop_points[0], current_loop_points[-1], cfg_coord_epsilon) and len(current_loop_points) > 2:
                    stitched_wall_loops[layer_num].append(current_loop_points[:-1])
                else:
                    stitched_wall_loops[layer_num].append(current_loop_points)
    print("Stitching complete.")
    output_data = defaultdict(lambda: {
        'wall_loops': [], 'travel_segments': [], 'original_infill_segments': []
    })
    all_layer_nums = set(stitched_wall_loops.keys()) | \
                     set(layer_travel_segments.keys()) | \
                     set(layer_original_infill_segments.keys())
    for layer_num in sorted(list(all_layer_nums)):
        output_data[layer_num]['wall_loops'] = stitched_wall_loops.get(layer_num, [])
        output_data[layer_num]['travel_segments'] = layer_travel_segments.get(layer_num, [])
        output_data[layer_num]['original_infill_segments'] = layer_original_infill_segments.get(layer_num, [])
    return output_data, layer_z_values