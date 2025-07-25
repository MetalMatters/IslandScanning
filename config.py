# config.py

from pathlib import Path

# --- Global Configuration (USER: Set these values) ---
# Define the base working directory
BASE_WORKING_DIRECTORY = Path(r"C:\Users\Jeremy\Desktop\work\\") # <--- !!! UPDATE THIS PATH !!!

# Construct GCODE_FILE_PATH_STR using the base directory
GCODE_FILE_NAME = "CFFFP_8mmCube v1.gcode" # Your input G-code file
GCODE_FILE_PATH_STR = str(BASE_WORKING_DIRECTORY / GCODE_FILE_NAME)

# --- Plotting Configuration ---
ENABLE_PLOTTING = True  # Set to False to disable all SVG generation and plotting
PLOT_LAYER_NUMBER = 0  # Only generate SVG for this specific layer number

# Set to "classified_regions" to see the original classification with numbers.
# Set to "masked_grid" to see the grid clipped by infillable regions and generate G-code.
PLOT_MODE = "masked_grid"

# Construct OUTPUT_PLOT_FILE_PATH_STR using the base directory and .svg format
OUTPUT_PLOT_FILE_NAME = f"infill_layer_{PLOT_LAYER_NUMBER}_plot_v_final.svg"

# --- Output Directory Configuration ---
OUTPUT_DIRECTORY = BASE_WORKING_DIRECTORY / "output"  # Directory for all output files

# Fix the output file paths to use the output directory
OUTPUT_PLOT_FILE_PATH_STR = str(OUTPUT_DIRECTORY / OUTPUT_PLOT_FILE_NAME)

# --- Masked Grid Configuration ---
GRID_CELL_SIZE = 1.0 # Size of the grid cells in mm
GRID_BOUNDS_BUFFER = 5.0 # Extra buffer around the print for grid generation, in mm

# --- Infill Generation Configuration ---
INFILL_LINE_SPACING = 0.08 # Distance between infill lines in mm
INFILL_ANGLES_CYCLE = [0, 45, 90, 135] # Cycle through these angles for infill.
INFILL_REGION_SHRINK_OFFSET = 0.01 # In mm. Shrinks the infill mask inwards from wall loops.

# --- Execution Order Configuration ---
X_ORDER_LEFT_TO_RIGHT = True  # True for Left-to-Right X traversal, False for Right-to-Left
Y_ORDER_TOP_TO_BOTTOM = True # True for Top-to-Bottom Y traversal, False for Bottom-to-Top

# --- G-code Generation Configuration (NEW) ---
ENABLE_GCODE_GENERATION = True  # Set to True to generate a G-code file (if PLOT_MODE is "masked_grid")
GENERATED_GCODE_FILE_NAME = f"generated_infill_layer_{PLOT_LAYER_NUMBER}.gcode"
GENERATED_GCODE_FILE_PATH_STR = str(OUTPUT_DIRECTORY / GENERATED_GCODE_FILE_NAME)

FEEDRATE_PRIMARY_INFILL = 1800      # mm/min (e.g., 30 mm/s)
FEEDRATE_BOUNDARY_CONNECTION = 1200 # mm/min (e.g., 20 mm/s, for paths along cell boundaries)
FEEDRATE_INTRA_CELL_CONNECTION = 2400 # mm/min (e.g., 40 mm/s, for quick internal connections within a cell)
FEEDRATE_TRAVEL = 6000              # mm/min (e.g., 100 mm/s, for G0 moves like retrace, jumps)
FEEDRATE_WALL = 2400               # mm/min (e.g., 40 mm/s, walls are often printed faster than infill)
EXTRUSION_MULTIPLIER = 0.033        # mm of filament per mm of XY travel. CRITICAL: TUNE THIS VALUE.
                                    # Example: For 0.4mm nozzle, 0.2mm layer height, extrusion width ~0.4mm
                                    # Extrusion Area = 0.2 * 0.4 = 0.08 mm^2.
                                    # Filament Area (1.75mm dia) = pi*(1.75/2)^2 = ~2.405 mm^2
                                    # Volumetric multiplier for G-code E value = Extrusion Area / Filament Area
                                    # = 0.08 / 2.405 = ~0.03326. Adjust empirically.
INITIAL_E_VALUE_FOR_GENERATED_LAYER = 0.0 # Starting E value for this generated layer's G-code

# --- G-code Multi-layer Output Configuration ---
SINGLE_GCODE_FILE_OUTPUT = True  # When True, generates one file with all layers instead of separate files
SINGLE_GCODE_OUTPUT_FILENAME = "complete_infill_model.gcode"  # Name of the combined output file

# G-code stub templates (can contain custom commands)
# G-code stub templates for laser-based LPBF printer
GCODE_STUB_START = """G94 P20000
G99 P2400
"""

GCODE_STUB_LAYER_TRANSITION = """G98
G94 P10000
G97
G96
G94 P20000
G99 P2400
"""

GCODE_STUB_END = """G94 P10000
G98
"""

# --- Advanced Laser Control Configuration ---
LASER_POWER = 2400                # Power level in deciamps (2400 = 24.00A)
LASER_RAMP_FILE = str(BASE_WORKING_DIRECTORY / "LaserRamp.gcode")  # Path to laser ramp file
G98_BOUNDARY = 23.5               # Safe boundary for laser off position in mm

# --- Script Technical Configuration (Constants) ---
RETRACTION_DISTANCE_SETTING = 6.5 # Assumed from typical slicer settings for retraction detection
RETRACTION_DETECTION_THRESHOLD_FACTOR = 0.8
ABS_E_CHANGE_FOR_RETRACTION_CMD = RETRACTION_DISTANCE_SETTING * RETRACTION_DETECTION_THRESHOLD_FACTOR
MIN_E_DELTA_FOR_PRINTING_WHEN_NOT_RETRACTED = 0.00001 # Smallest E change considered extrusion
COORD_EPSILON = 0.01          # Base epsilon for general floating point comparisons
NODE_MERGE_EPSILON = COORD_EPSILON * 0.1 # Very tight epsilon for merging graph nodes
ON_SEGMENT_EPSILON = COORD_EPSILON * 3.0 # Generous epsilon for point on segment check
ENDPOINT_DISTINCTION_EPSILON = COORD_EPSILON * 1.5 # Epsilon for distinguishing endpoints from projection

MIN_SIGNIFICANT_TRACK_LENGTH = 0.5 # Minimum length (mm) for an infill track to be "significant"
MIN_SIGNIFICANT_TRACK_LENGTH_SQ = MIN_SIGNIFICANT_TRACK_LENGTH**2
MIN_AREA_THRESH_FLOAT = 0.001     # Minimum area (mm^2) for a polygon to be considered

SMALL_DIST_FACTOR = 10
min_segment_dist_threshold = COORD_EPSILON / SMALL_DIST_FACTOR # For original infill classification
min_segment_length_epsilon_sq = min_segment_dist_threshold**2  # For filtering tiny G-code segments
moved_xy_threshold = min_segment_dist_threshold / 10.0         # For detecting XY movement in G-code parser

DRASTIC_CHANGE_THRESHOLD = 0.5  # 50% of cells changed triggers full recompute

# Output control flags
MINIMAL_OUTPUT_MODE = True  # When True, only shows progress percentage