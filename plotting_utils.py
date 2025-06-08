# plotting_utils.py
import time
try:
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches
    from matplotlib.path import Path as MplPath
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    # Functions in this module will check this flag or main script will.

from . import gcode_parser # For segment_length_sq if used in plotting logic

def setup_plot(figsize=(12, 10)):
    if not MATPLOTLIB_AVAILABLE: return None, None
    fig = plt.figure(figsize=figsize, facecolor='white')
    ax = fig.gca()
    ax.set_facecolor('white')
    return fig, ax

def plot_classified_regions_on_ax(ax, classified_regions_to_plot, cfg_min_significant_track_length_sq):
    if not MATPLOTLIB_AVAILABLE or ax is None: return
    if not classified_regions_to_plot:
        print("Plotting: No classified regions to plot.")
        return
    print(f"Plotting {len(classified_regions_to_plot)} classified regions (Mode: Classified Regions).")
    for i, region_item in enumerate(classified_regions_to_plot):
        path_float = region_item['path_float']
        interiors_float = region_item['interiors_float']
        centroid_coords_for_number = region_item['centroid']
        has_infill = region_item['contains_infill']
        longest_track = region_item.get('longest_track')

        if not path_float or len(path_float) < 3: continue

        path_vertices = list(path_float)
        path_codes = [MplPath.MOVETO] + [MplPath.LINETO]*(len(path_float)-1)
        if not gcode_parser.are_points_close(path_vertices[0], path_vertices[-1], 0.0001): # Ensure closed
            path_vertices.append(path_vertices[0])
            path_codes.append(MplPath.LINETO)
        path_codes[-1] = MplPath.CLOSEPOLY

        for interior_pts in interiors_float:
            if interior_pts and len(interior_pts) >=3:
                # Close interior loop for MplPath
                closed_interior_pts = list(interior_pts)
                if not gcode_parser.are_points_close(closed_interior_pts[0], closed_interior_pts[-1], 0.0001):
                    closed_interior_pts.append(closed_interior_pts[0])
                
                path_vertices.extend(closed_interior_pts)
                path_codes.extend([MplPath.MOVETO] + [MplPath.LINETO]*(len(closed_interior_pts)-1) + [MplPath.CLOSEPOLY])

        mpl_path_obj = MplPath(path_vertices, path_codes)
        face_color, edge_color, num_text_color = ('', '', '') # Pyright fix
        if has_infill:
            face_color, edge_color, num_text_color = (0.2, 0.9, 0.2, 0.5), (0.0, 0.5, 0.0, 0.8), 'darkgreen'
        else:
            face_color, edge_color, num_text_color = (1.0, 0.55, 0.0, 0.6), (0.7, 0.3, 0.0, 0.9), (0.5, 0.1, 0.0, 1.0)

        patch = patches.PathPatch(mpl_path_obj, edgecolor=edge_color, facecolor=face_color,
                                  linestyle='-', linewidth=0.7, alpha=0.65, zorder=2)
        ax.add_patch(patch)
        num_place_coord = centroid_coords_for_number
        if has_infill and longest_track:
            # Check significance based on original classification, not re-filtering here
            ls_start, ls_end = longest_track
            ax.plot([ls_start[0], ls_end[0]], [ls_start[1], ls_end[1]],
                    color='mediumblue', linewidth=1.2, linestyle='--', zorder=3, alpha=0.9)
            num_place_coord = ((ls_start[0] + ls_end[0]) / 2, (ls_start[1] + ls_end[1]) / 2)

        ax.text(num_place_coord[0], num_place_coord[1], str(i + 1), color=num_text_color,
                ha='center', va='center', fontsize=9, fontweight='bold',
                bbox=dict(boxstyle="circle,pad=0.2", fc="white", ec=edge_color, lw=0.5, alpha=0.9), zorder=11)


def plot_masked_grid_on_ax(ax, masked_grid_cells_data, all_toolpaths_ordered):
    if not MATPLOTLIB_AVAILABLE or ax is None: return
    if masked_grid_cells_data:
        print(f"Plotting {len(masked_grid_cells_data)} clipped grid cells and their toolpaths.")
        for i, cell_data in enumerate(masked_grid_cells_data):
            clipped_poly = cell_data['shapely_poly_clipped']
            if not clipped_poly or not clipped_poly.exterior: continue

            path_vertices = list(clipped_poly.exterior.coords)
            path_codes = [MplPath.MOVETO] + [MplPath.LINETO]*(len(path_vertices)-1)
            if not gcode_parser.are_points_close(path_vertices[0], path_vertices[-1], 0.0001):
                path_vertices.append(path_vertices[0])
                path_codes.append(MplPath.LINETO)
            path_codes[-1] = MplPath.CLOSEPOLY

            if hasattr(clipped_poly, 'interiors'):
                for interior_ring in clipped_poly.interiors: # Iterate through interior LinearRings
                    interior_coords = list(interior_ring.coords)
                    if interior_coords and len(interior_coords) >= 3:
                        closed_interior_coords = list(interior_coords)
                        if not gcode_parser.are_points_close(closed_interior_coords[0], closed_interior_coords[-1], 0.0001):
                             closed_interior_coords.append(closed_interior_coords[0])

                        path_vertices.extend(closed_interior_coords)
                        path_codes.extend([MplPath.MOVETO] + [MplPath.LINETO]*(len(closed_interior_coords)-1) + [MplPath.CLOSEPOLY])

            mpl_path_obj = MplPath(path_vertices, path_codes)
            patch = patches.PathPatch(mpl_path_obj, edgecolor='dimgray', facecolor='none',
                                      linestyle='-', linewidth=0.8, alpha=0.8, zorder=2)
            ax.add_patch(patch)

            centroid_x, centroid_y = clipped_poly.centroid.x, clipped_poly.centroid.y
            ax.text(centroid_x, centroid_y, str(cell_data['execution_order']),
                    color='black', ha='center', va='center', fontsize=5, fontweight='bold',
                    bbox=dict(boxstyle="square,pad=0.1", fc="white", ec="none", alpha=0.7), zorder=12)

            if cell_data.get('cell_infill_start_pt'):
                ax.plot(cell_data['cell_infill_start_pt'][0], cell_data['cell_infill_start_pt'][1],
                        'o', color='lime', markersize=1.5, zorder=13, markeredgecolor='green', markeredgewidth=0.3)
            if cell_data.get('cell_infill_end_pt'):
                ax.plot(cell_data['cell_infill_end_pt'][0], cell_data['cell_infill_end_pt'][1],
                        'o', color='red', markersize=1.5, zorder=13, markeredgecolor='darkred', markeredgewidth=0.3)

        if all_toolpaths_ordered:
            # print(f"Plotting {len(all_toolpaths_ordered)} toolpath elements.") # Can be verbose
            for segment_line, segment_type in all_toolpaths_ordered:
                if not segment_line or not hasattr(segment_line, 'coords') or not list(segment_line.coords): continue
                xs, ys = segment_line.xy
                if segment_type == "primary":
                    ax.plot(xs, ys, color='#4169E1', linewidth=0.08, alpha=1.0, zorder=3)
                elif segment_type == "connection":
                    ax.plot(xs, ys, color='#32CD32', linewidth=0.06, alpha=1.0, zorder=4)
                elif segment_type == "retrace":
                    ax.plot(xs, ys, color='purple', linewidth=0.04, alpha=0.5, linestyle=':', zorder=3)
                elif segment_type == "inter_cell_boundary_connection":
                    ax.plot(xs, ys, color='orange', linewidth=0.1, alpha=0.7, linestyle='--', zorder=6)
                elif segment_type == "inter_cell_direct_jump":
                    ax.plot(xs, ys, color='magenta', linewidth=0.12, alpha=0.8, linestyle=':', zorder=7)
                elif segment_type == "inter_cell_direct_jump_part":
                    ax.plot(xs, ys, color='#FF69B4', linewidth=0.08, alpha=0.7, linestyle=':', zorder=5)
    else:
        print("Plotting: No clipped grid cells to plot.")


def plot_common_elements_on_ax(ax, wall_loops_for_plot_layer, travel_segments_to_plot):
    if not MATPLOTLIB_AVAILABLE or ax is None: return
    if travel_segments_to_plot:
        first_travel = True
        for t_start, t_end in travel_segments_to_plot:
            ax.plot([t_start[0], t_end[0]], [t_start[1], t_end[1]],
                     color='dodgerblue', linestyle='-', linewidth=0.5, alpha=0.3, zorder=1,
                     label="Original G0/G1 Travel" if first_travel else None)
            first_travel = False

    if wall_loops_for_plot_layer:
        first_wall = True
        for loop in wall_loops_for_plot_layer:
            if not loop: continue
            # Ensure loop is closed for plotting
            plot_loop = list(loop)
            if not gcode_parser.are_points_close(plot_loop[0], plot_loop[-1], 0.0001):
                plot_loop.append(plot_loop[0])
            
            x_coords = [p[0] for p in plot_loop]
            y_coords = [p[1] for p in plot_loop]
            ax.plot(x_coords, y_coords, color='red',
                     linewidth=0.8, label="Original Wall Loops" if first_wall else None, zorder=10)
            first_wall = False

def finalize_plot(fig, ax, title_str, output_plot_path_str):
    if not MATPLOTLIB_AVAILABLE or fig is None or ax is None: return
    ax.set_title(title_str)
    plt.xlabel("X (mm)")
    plt.ylabel("Y (mm)")
    ax.set_aspect('equal', adjustable='box')
    handles, labels = ax.get_legend_handles_labels()
    if handles: # Create legend with unique labels
        unique_labels = {}
        for handle, label in zip(handles, labels):
            if label not in unique_labels:
                unique_labels[label] = handle
        if unique_labels: # Check if there's anything to make a legend for
             ax.legend(unique_labels.values(), unique_labels.keys(), loc='upper right', prop={'size': 7})
    plt.grid(True, linestyle='--', alpha=0.7)

    if output_plot_path_str:
        save_start_time = time.time()
        print(f"Saving plot to {output_plot_path_str}...")
        try:
            fig.savefig(output_plot_path_str, bbox_inches='tight', format='svg', dpi=300)
            print("Plot saved.")
        except Exception as e:
            print(f"Error saving plot: {e}")
        # print(f"Time for saving plot: {time.time() - save_start_time:.3f} seconds") # Optional timing

    show_start_time = time.time()
    plt.show() # This is a blocking call
    print(f"Time for plt.show() (blocking): {time.time() - show_start_time:.3f} seconds")