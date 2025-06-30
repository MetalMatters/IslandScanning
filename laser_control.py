import math
import random
from pathlib import Path

# Global cache for laser ramp file content
laser_ramp = ""

def power_cycle(power_up=True, deciamps=2400, next_segment_target=None, g98_boundary=23.5):
    """
    Generates G-code for laser power up/down sequence.
    
    Args:
        power_up: Whether to power up (True) or down (False) the laser
        deciamps: Laser power in deciamps (2400 = 24.00A)
        next_segment_target: Optional target position after power up (tuple of X,Y)
        g98_boundary: Boundary value for G98 positioning
        
    Returns:
        String of G-code commands for the power cycle
    """
    global laser_ramp
    stub = ""
    
    # Power up sequence
    if power_up:
        stub += f"G99 P{int(deciamps)}\n"
    # Power down sequence
    else:
        # Generate random angle for safe laser-off position
        angle = random.uniform(0, 2 * math.pi)
        x = -1 * abs(g98_boundary * math.cos(angle))
        y = g98_boundary * math.sin(angle)
        stub += f"G1 X{x:.2f} Y{y:.2f}\n"  # Using G1 for interpolated movement
        stub += "G98\n"
    
    # Load laser ramp file if not already loaded
    if not laser_ramp:
        try:
            # Try to load the laser ramp file
            laser_ramp_file = Path(next_segment_target[2]) if isinstance(next_segment_target, tuple) and len(next_segment_target) > 2 else None
            
            if not laser_ramp_file or not laser_ramp_file.is_file():
                print(f"Warning: Laser ramp file not found or not specified.")
                laser_ramp = ""
            else:
                with open(laser_ramp_file, 'r') as ramp:
                    laser_ramp = ramp.read()
                    print(f"Loaded laser ramp file: {laser_ramp_file}")
        except Exception as e:
             print(f"ERROR: Could not read laser ramp file: {e}")
             laser_ramp = ""
    
    # Add ramp commands if available
    if laser_ramp:
        stub += laser_ramp
    
    # Add movement to target position after power up if provided
    if power_up and next_segment_target:
        if isinstance(next_segment_target, tuple) and len(next_segment_target) >= 2:
            stub += f"G1 X{next_segment_target[0]:.3f} Y{next_segment_target[1]:.3f}\n"
    
    return stub

def create_start_gcode(power=2400, laser_ramp_file=None):
    """
    Generates the machine start G-code sequence.
    
    Args:
        power: Laser power in deciamps
        laser_ramp_file: Path to laser ramp file
        
    Returns:
        String of G-code commands for start of file
    """
    stub = "G94 P20000\n"  # Set feedrate interpretation
    stub += power_cycle(power_up=True, deciamps=power, next_segment_target=(0, 0, laser_ramp_file))
    return stub

def create_end_gcode(g98_boundary=23.5):
    """
    Generates the machine end G-code sequence.
    
    Args:
        g98_boundary: Boundary value for G98 positioning
        
    Returns:
        String of G-code commands for end of file
    """
    stub = "G94 P10000\n"  # Reset feedrate interpretation
    stub += power_cycle(power_up=False, g98_boundary=g98_boundary)
    return stub

def create_layer_transition(next_xy_pos=None, power=2400, g98_boundary=23.5):
    """
    Generates the layer change G-code sequence.
    
    Args:
        next_xy_pos: Next position to move to (X,Y tuple)
        power: Laser power in deciamps
        g98_boundary: Boundary value for G98 positioning
        
    Returns:
        String of G-code commands for layer transition
    """
    stub = power_cycle(power_up=False, g98_boundary=g98_boundary)
    stub += "G94 P10000\n"  # Lower feedrate during transition
    stub += "G97\n"         # Custom command for your system
    stub += "G96\n"         # Custom command for your system
    stub += "G94 P20000\n"  # Restore high feedrate
    stub += power_cycle(power_up=True, deciamps=power, next_segment_target=next_xy_pos)
    return stub