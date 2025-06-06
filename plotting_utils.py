# plotting_utils.py
import time # For timing save if desired
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.path import Path as MplPath
# Import gcode_parser for segment_length_sq if needed for filtering in plot
import gcode_parser

def setup_plot(figsize=(12, 10)):
    fig = plt.figure(figsize=figsize, facecolor='white')
    ax = fig.gca() # Use fig.gca() or fig.add_subplot(111)
    ax.set_facecolor('white')
    return fig, ax

def plot_classified_regions_on_ax(ax, classified_regions_to_plot, cfg_min_significant_track_length_sq):
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
        if path_vertices[0] != path_vertices[-1]: # Ensure closed
            path_vertices.append(path_vertices[0])
            path_codes.append(MplPath.LINETO)
        path_codes[-1] = MplPath.CLOSEPOLY

        for interior_pts in interiors_float:
            if interior_pts and len(interior_pts) >=3:
                path_vertices.extend(interior_pts + [interior_pts[0]]) # Close interior
                path_codes.extend([MplPath.MOVETO] + [MplPath.LINETO]*(len(interior_pts)-1) + [MplPath.CLOSEPOLY])

        mpl_path_obj = MplPath(path_vertices, path_codes)
        face_color, edge_color, num_text_color = ('', '', '')
        if has_infill:
            face_color, edge_color, num_text_color = (0.2, 0.9, 0.2, 0.5), (0.0, 0.5, 0.0, 0.8), 'darkgreen'
        else:
            face_color, edge_color, num_text_color = (1.0, 0.55, 0.0, 0.6), (0.7, 0.3, 0.0, 0.9), (0.5, 0.1, 0.0, 1.0)

        patch = patches.PathPatch(mpl_path_obj, edgecolor=edge_color, facecolor=face_color,
                                  linestyle='-', linewidth=0.7, alpha=0.65, zorder=2)
        ax.add_patch(patch)
        num_place_coord = centroid_coords_for_number
        if has_infill and longest_track:
            # Re-check or assume classification already filtered significant tracks
            # For simplicity, plotting it if present.
            ls_start, ls_end = longest_track
            # Example check: if gcode_parser.segment_length_sq(ls_start, ls_end) >= cfg_min_significant_track_length_sq:
            ax.plot([ls_start[0], ls_end[0]], [ls_start[1], ls_end[1]],
                    color='mediumblue', linewidth=1.2, linestyle='--', zorder=3, alpha=0.9)
            num_place_coord = ((ls_start[0] + ls_end[0]) / 2, (ls_start[1] + ls_end[1]) / 2)

        ax.text(num_place_coord[0], num_place_coord[1], str(i + 1), color=num_text_color,
                ha='center', va='center', fontsize=9, fontweight='bold',
                bbox=dict(boxstyle="circle,pad=0.2", fc="white", ec=edge_color, lw=0.5, alpha=0.9), zorder=11)


def plot_masked_grid_on_ax(ax, masked_grid_cells_data, all_toolpaths_ordered):
    # Clearer background for plot - can be added if needed
    # plt.draw()
    # min_x_data, max_x_data = ax.get_xlim()
    # min_y_data, max_y_data = ax.get_ylim()
    # if min_x_data < max_x_data and min_y_data < max_y_data:
    #     rect = patches.Rectangle((min_x_data, min_y_data),
    #                                 max_x_data - min_x_data,
    #                                 max_y_data - min_y_data,
    #                                 facecolor='white', edgecolor='none', zorder=0)
    #     ax.add_patch(rect)

    if masked_grid_cells_data:
        print(f"Plotting {len(masked_grid_cells_data)} clipped grid cells and their toolpaths.")
        for i, cell_data in enumerate(masked_grid_cells_data):
            clipped_poly = cell_data['shapely_poly_clipped']
            if not clipped_poly.exterior: continue

            path_vertices = list(clipped_poly.exterior.coords)
            path_codes = [MplPath.MOVETO] + [MplPath.LINETO]*(len(path_vertices)-1)
            if path_vertices[0] != path_vertices[-1]:
                path_vertices.append(path_vertices[0])
                path_codes.append(MplPath.LINETO)
            path_codes[-1] = MplPath.CLOSEPOLY

            if hasattr(clipped_poly, 'interiors'):
                for interior_pts in clipped_poly.interiors: # Iterate through interior LinearRings
                    interior_coords = list(interior_pts.coords)
                    if interior_coords and len(interior_coords) >= 3:
                        path_vertices.extend(interior_coords + [interior_coords[0]]) # Close interior
                        path_codes.extend([MplPath.MOVETO] + [MplPath.LINETO]*(len(interior_coords)-1) + [MplPath.CLOSEPOLY])

            mpl_path_obj = MplPath(path_vertices, path_codes)
            patch = patches.PathPatch(mpl_path_obj, edgecolor='dimgray', facecolor='none',
                                      linestyle='-', linewidth=0.8, alpha=0.8, zorder=2)
            ax.add_patch(patch)

            centroid_x, centroid_y = clipped_poly.centroid.x, clipped_poly.centroid.y
            ax.text(centroid_x, centroid_y, str(cell_data['execution_order']),
                    color='black', ha='center', va='center', fontsize=5, fontweight='bold',
                    bbox=dict(boxstyle="square,pad=0.1", fc="white", ec="none", alpha=0.7), zorder=12)

            if cell_data['cell_infill_start_pt']:
                ax.plot(cell_data['cell_infill_start_pt'][0], cell_data['cell_infill_start_pt'][1],
                        'o', color='lime', markersize=1.5, zorder=13, markeredgecolor='green', markeredgewidth=0.3)
            if cell_data['cell_infill_end_pt']:
                ax.plot(cell_data['cell_infill_end_pt'][0], cell_data['cell_infill_end_pt'][1],
                        'o', color='red', markersize=1.5, zorder=13, markeredgecolor='darkred', markeredgewidth=0.3)

        if all_toolpaths_ordered:
            print(f"Plotting {len(all_toolpaths_ordered)} toolpath elements.")
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
                elif segment_type == "inter_cell_direct_jump_part": # For parts of a boundary jump
                    ax.plot(xs, ys, color='#FF69B4', linewidth=0.08, alpha=0.7, linestyle=':', zorder=5) # HotPink
    else:
        print("Plotting: No clipped grid cells to plot.")


def plot_common_elements_on_ax(ax, wall_loops_for_plot_layer, travel_segments_to_plot):
    if travel_segments_to_plot:
        first_travel = True
        for t_start, t_end in travel_segments_to_plot:
            ax.plot([t_start[0], t_end[0]], [t_start[1], t_end[1]],
                     color='dodgerblue', linestyle='-', linewidth=0.5, alpha=0.3, zorder=1, # Lower zorder
                     label="Original G0/G1 Travel" if first_travel else None)
            first_travel = False

    if wall_loops_for_plot_layer:
        first_wall = True
        for loop in wall_loops_for_plot_layer:
            if not loop: continue
            x_coords = [p[0] for p in loop] + [loop[0][0]]
            y_coords = [p[1] for p in loop] + [loop[0][1]]
            ax.plot(x_coords, y_coords, color='red',
                     linewidth=0.8, label="Original Wall Loops" if first_wall else None, zorder=10) # Higher zorder
            first_wall = False

def finalize_plot(fig, ax, title_str, output_plot_path_str): # Pass fig for saving
    ax.set_title(title_str)
    plt.xlabel("X (mm)")
    plt.ylabel("Y (mm)")
    ax.set_aspect('equal', adjustable='box')
    handles, labels = ax.get_legend_handles_labels()
    if handles:
        unique_labels_map = dict(zip(labels, handles))
        ax.legend(unique_labels_map.values(), unique_labels_map.keys(), loc='upper right', prop={'size': 7})
    plt.grid(True, linestyle='--', alpha=0.7)

    if output_plot_path_str:
        save_start_time = time.time()
        print(f"Saving plot to {output_plot_path_str}...")
        try:
            fig.savefig(output_plot_path_str, bbox_inches='tight', format='svg', dpi=300) # Use fig.savefig
            print("Plot saved.")
        except Exception as e:
            print(f"Error saving plot: {e}")
        print(f"Time for saving plot: {time.time() - save_start_time:.3f} seconds")

    show_start_time = time.time()
    plt.show()
    print(f"Time for plt.show() (blocking): {time.time() - show_start_time:.3f} seconds")