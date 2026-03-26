import random
import os
import argparse

# --- Configuration ---
parser = argparse.ArgumentParser(description="Generate a maze SDF world")
parser.add_argument("--cells-x", type=int, default=4, help="Maze width in cells")
args = parser.parse_args()

cells_x = args.cells_x
cells_y = cells_x

# --- DIMENSIONS CONFIGURATION ---
desired_path_width = 1.8  # The clear space for the robot to drive
wall_thickness = 0.5      # The thickness of the wall obstacles
wall_height = 1

# Calculate cell_size (center-to-center distance) to guarantee the path width
cell_size = desired_path_width + wall_thickness  # Equals 2.3m

maze_width = cells_x * cell_size
maze_height = cells_y * cell_size

output_dir = os.path.join(os.getcwd(), "worlds")
os.makedirs(output_dir, exist_ok=True)

# Global maze state variables (re-initialized inside generate_sdf)
visited = []
removed_h_walls = set()
removed_v_walls = set()

def carve(start_r, start_c):
    """Carves out paths using the Recursive Backtracker algorithm."""
    # Ensure global state is used inside the function
    global visited, removed_h_walls, removed_v_walls
    
    stack = [(start_r, start_c)]
    visited[start_r][start_c] = True
    
    while stack:
        r, c = stack[-1]
        neighbors = []
        
        # Check unvisited neighbors
        if r > 0 and not visited[r - 1][c]:  # North
            neighbors.append((r - 1, c, 'north'))
        if r < cells_y - 1 and not visited[r + 1][c]:  # South
            neighbors.append((r + 1, c, 'south'))
        if c > 0 and not visited[r][c - 1]:  # West
            neighbors.append((r, c - 1, 'west'))
        if c < cells_x - 1 and not visited[r][c + 1]:  # East
            neighbors.append((r, c + 1, 'east'))
        
        if neighbors:
            nr, nc, direction = random.choice(neighbors)
            visited[nr][nc] = True
            
            # Record which wall was removed
            if direction == 'north':
                removed_h_walls.add((r, c))  # Wall between (r,c) and (r-1, c)
            elif direction == 'south':
                removed_h_walls.add((r + 1, c))  # Wall between (r,c) and (r+1, c)
            elif direction == 'west':
                removed_v_walls.add((r, c))  # Wall between (r,c) and (r, c-1)
            elif direction == 'east':
                removed_v_walls.add((r, c + 1))  # Wall between (r,c) and (r, c+1)
            
            stack.append((nr, nc))
        else:
            stack.pop()

def get_walls_to_keep():
    """Get only the walls that should remain (not carved paths)."""
    h_walls_keep = set()
    v_walls_keep = set()
    
    # Internal horizontal walls (r=1 to cells_y-1)
    for r in range(1, cells_y):
        for c in range(cells_x):
            if (r, c) not in removed_h_walls:
                h_walls_keep.add((r, c))
    
    # Internal vertical walls (c=1 to cells_x-1)
    for r in range(cells_y):
        for c in range(1, cells_x):
            if (r, c) not in removed_v_walls:
                v_walls_keep.add((r, c))
    
    # Add all border walls (ensuring they are not accidentally removed)
    # Top border (r=0)
    for c in range(cells_x):
        h_walls_keep.add((0, c))
    # Bottom border (r=cells_y)
    for c in range(cells_x):
        h_walls_keep.add((cells_y, c))
    # Left border (c=0)
    for r in range(cells_y):
        v_walls_keep.add((r, 0))
    # Right border (c=cells_x)
    for r in range(cells_y):
        v_walls_keep.add((r, cells_x))
    
    return h_walls_keep, v_walls_keep

def wall_to_sdf_coords(r, c, is_horizontal):
    """Convert grid coordinates to SDF world coordinates."""
    # Center the maze at origin
    offset_x = -maze_width / 2
    offset_y = -maze_height / 2
    
    if is_horizontal:
        # Horizontal wall spans along X axis (Y position is exactly on the grid line r)
        x = offset_x + (c + 0.5) * cell_size
        y = offset_y + r * cell_size
        size_x = cell_size
        size_y = wall_thickness
    else:
        # Vertical wall spans along Y axis (X position is exactly on the grid line c)
        x = offset_x + c * cell_size
        y = offset_y + (r + 0.5) * cell_size
        size_x = wall_thickness
        size_y = cell_size
    
    return x, y, size_x, size_y

def generate_sdf():
    """Generate the complete SDF file."""
    # Reset maze state before generation
    global visited, removed_h_walls, removed_v_walls
    visited = [[False] * cells_x for _ in range(cells_y)]
    removed_h_walls = set()
    removed_v_walls = set()
    
    # --- CAMERA POSE CALCULATION ---
    # Centered (0, 0) above the maze
    CAM_HEIGHT = maze_width * 1.5  # 150% of the maze width for a good overhead view
    CAM_PITCH = 1.2  # ~68 degrees looking down (0 is horizontal, 1.57 is straight down)
    
    # Format the calculated pose values into the SDF pose string
    camera_pose = f"0.0 0.0 {CAM_HEIGHT:.2f} 0 {CAM_PITCH:.2f} 0"
    
    # Run maze generation
    carve(0, 0)
    
    sdf_content = f'''<?xml version="1.0"?>
<sdf version='1.8'>
    <world name='maze_{cells_x}x{cells_y}'>
        <physics name='1ms' type='ignored'>
            <max_step_size>0.003</max_step_size>
            <real_time_factor>1</real_time_factor>
            <real_time_update_rate>1000</real_time_update_rate>
        </physics>

        <!-- Plugins -->
        <plugin name="gz::sim::systems::Physics" filename="gz-sim-physics-system" />
        <plugin name="gz::sim::systems::UserCommands" filename="gz-sim-user-commands-system" />
        <plugin name="gz::sim::systems::SceneBroadcaster" filename="gz-sim-scene-broadcaster-system" />
        <plugin name="gz::sim::systems::Contact" filename="gz-sim-contact-system" />
        <plugin name="gz::sim::systems::Sensors" filename="gz-sim-sensors-system">
            <render_engine>ogre2</render_engine>
        </plugin>

        <gui fullscreen="0">
            <camera name="user_camera">
                <pose>{camera_pose}</pose>
                <view_controller>orbit</view_controller>
            </camera>
        </gui>

        <light name='sun' type='directional'>
            <cast_shadows>1</cast_shadows>
            <pose>0 0 40 0 -0 0</pose>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
            <attenuation>
                <range>4000</range>
                <constant>0.90000000000000002</constant>
                <linear>0.01</linear>
                <quadratic>0.001</quadratic>
            </attenuation>
            <direction>-0.5 0.1 -0.9</direction>
            <spot>
                <inner_angle>0</inner_angle>
                <outer_angle>0</outer_angle>
                <falloff>0</falloff>
            </spot>
        </light>

        <gravity>0 0 -9.8</gravity>
        <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
        <atmosphere type='adiabatic' />

        <scene>
            <ambient>0.4 0.4 0.4 1</ambient>
            <background>0.7 0.7 0.7 1</background>
            <shadows>1</shadows>
        </scene>

        <model name='ground_plane'>
            <static>1</static>
            <link name='link'>
                <collision name='collision'>
                    <geometry>
                        <plane>
                            <normal>0 0 1</normal>
                            <size>{maze_width * 2:.2f} {maze_height * 2:.2f}</size>
                        </plane>
                    </geometry>
                    <surface>
                        <friction>
                            <ode />
                        </friction>
                        <bounce />
                        <contact />
                    </surface>
                </collision>
                <visual name='visual'>
                    <geometry>
                        <plane>
                            <normal>0 0 1</normal>
                            <size>{maze_width * 2:.2f} {maze_height * 2:.2f}</size>
                        </plane>
                    </geometry>
                    <material>
                        <ambient>0.8 0.8 0.8 1</ambient>
                        <diffuse>0.8 0.8 0.8 1</diffuse>
                        <specular>0.8 0.8 0.8 1</specular>
                    </material>
                    <plugin name='__default__' filename='__default__' />
                </visual>
            </link>
            <plugin name='__default__' filename='__default__' />
            <pose>0 0 0 0 -0 0</pose>
        </model>

        <model name='obstacles'>
            <pose>0 0 {wall_height / 2:.2f} 0 0 0</pose>
            <static>true</static>
'''
    
    offset_x = -maze_width / 2
    offset_y = -maze_height / 2
    wall_id = 0
    
    # --- 1. ADD CORNER BLOCKS TO SEAL GAPS ---
    # Iterate over all grid intersections (r=0 to cells_y, c=0 to cells_x)
    for r in range(cells_y + 1):
        for c in range(cells_x + 1):
            wall_id += 1
            x = offset_x + c * cell_size
            y = offset_y + r * cell_size
            
            # The corner block is a small cube of size wall_thickness
            corner_size = wall_thickness
            
            sdf_content += f'''        <link name='corner_block_{wall_id}'>
            <pose>{x:.2f} {y:.2f} 0 0 0 0</pose>
            <static>true</static>
            <collision name='corner_block_{wall_id}_collision'>
                <geometry>
                    <box>
                        <size>{corner_size:.2f} {corner_size:.2f} {wall_height:.2f}</size>
                    </box>
                </geometry>
            </collision>
            <visual name='corner_block_{wall_id}_visual'>
                <geometry>
                    <box>
                        <size>{corner_size:.2f} {corner_size:.2f} {wall_height:.2f}</size>
                    </box>
                </geometry>
                <material>
                    <ambient>0.1 0.1 0.1 1</ambient>
                    <diffuse>0 0.1 0.2 1</diffuse>
                    <specular>0 0.01 0.05 1</specular>
                </material>
            </visual>
        </link>
'''

    # --- 2. ADD HORIZONTAL AND VERTICAL WALL SEGMENTS ---
    h_walls, v_walls = get_walls_to_keep()
    
    # Add horizontal walls
    for r, c in sorted(h_walls):
        x, y, sx, sy = wall_to_sdf_coords(r, c, True)
        wall_id += 1
        sdf_content += f'''        <link name='wall{wall_id}'>
            <pose>{x:.2f} {y:.2f} 0 0 0 0</pose>
            <static>true</static>
            <collision name='wall{wall_id}_collision'>
                <geometry>
                    <box>
                        <size>{sx:.2f} {sy:.2f} {wall_height:.2f}</size>
                    </box>
                </geometry>
            </collision>
            <visual name='wall{wall_id}_visual'>
                <geometry>
                    <box>
                        <size>{sx:.2f} {sy:.2f} {wall_height:.2f}</size>
                    </box>
                </geometry>
                <material>
                    <ambient>0.1 0.1 0.1 1</ambient>
                    <diffuse>0 0.1 0.2 1</diffuse>
                    <specular>0 0.01 0.05 1</specular>
                </material>
            </visual>
        </link>
'''
    
    # Add vertical walls
    for r, c in sorted(v_walls):
        x, y, sx, sy = wall_to_sdf_coords(r, c, False)
        wall_id += 1
        sdf_content += f'''        <link name='wall{wall_id}'>
            <pose>{x:.2f} {y:.2f} 0 0 0 0</pose>
            <static>true</static>
            <collision name='wall{wall_id}_collision'>
                <geometry>
                    <box>
                        <size>{sx:.2f} {sy:.2f} {wall_height:.2f}</size>
                    </box>
                </geometry>
            </collision>
            <visual name='wall{wall_id}_visual'>
                <geometry>
                    <box>
                        <size>{sx:.2f} {sy:.2f} {wall_height:.2f}</size>
                    </box>
                </geometry>
                <material>
                    <ambient>0.1 0.1 0.1 1</ambient>
                    <diffuse>0 0.1 0.2 1</diffuse>
                    <specular>0 0.01 0.05 1</specular>
                </material>
            </visual>
        </link>
'''
    
    sdf_content += '''    </model>
</world>
</sdf>'''
    
    return sdf_content, wall_id

# Generate the SDF
sdf_content, num_walls = generate_sdf()

# Save to file
output_path = os.path.join(output_dir, "generated_maze.sdf")
with open(output_path, 'w') as f:
    f.write(sdf_content)

print(f"Generated maze with {num_walls} links (walls + corners)")
print(f"Maze size: {cells_x}x{cells_y} cells ({maze_width}m x {maze_height}m)")
print(f"Path Width: {desired_path_width}m (Cell Size: {cell_size}m)")
print(f"Saved to: {output_path}")