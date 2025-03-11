import math
import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
import cv2

def plot_map_graph(graph, figsize=(10, 10), node_size=50, edge_width=1):
    """
    Plot the map graph using NetworkX and Matplotlib.
    
    Args:
        graph (networkx.Graph): The map graph
        figsize (tuple): Figure size
        node_size (int): Size of nodes
        edge_width (int): Width of edges
        
    Returns:
        matplotlib.figure.Figure: Figure object
    """
    fig, ax = plt.subplots(figsize=figsize)
    
    # Get node positions
    pos = {}
    for node in graph.nodes():
        pos[node] = (float(graph.nodes[node]['x']), float(graph.nodes[node]['y']))
    
    # Draw the graph
    nx.draw(graph, pos, with_labels=True, node_size=node_size,
            node_color='skyblue', font_size=8, width=edge_width,
            arrows=True, arrowsize=10, ax=ax)
    
    ax.set_title('Map Graph')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    
    return fig

def plot_path(graph, path, current_pos=None, figsize=(10, 10), node_size=50):
    """
    Plot a path on the map graph.
    
    Args:
        graph (networkx.Graph): The map graph
        path (list): List of node IDs in the path
        current_pos (tuple): Current position (x, y, yaw) of the vehicle
        figsize (tuple): Figure size
        node_size (int): Size of nodes
        
    Returns:
        matplotlib.figure.Figure: Figure object
    """
    fig, ax = plt.subplots(figsize=figsize)
    
    # Get node positions
    pos = {}
    for node in graph.nodes():
        pos[node] = (float(graph.nodes[node]['x']), float(graph.nodes[node]['y']))
    
    # Draw all nodes and edges
    nx.draw_networkx_nodes(graph, pos, node_size=node_size, node_color='lightgray', ax=ax)
    nx.draw_networkx_edges(graph, pos, width=1, alpha=0.3, arrows=True, arrowsize=10, ax=ax)
    
    # Draw the path
    path_edges = [(path[i], path[i+1]) for i in range(len(path)-1)]
    nx.draw_networkx_nodes(graph, pos, nodelist=path, node_size=node_size, 
                            node_color='skyblue', ax=ax)
    nx.draw_networkx_edges(graph, pos, edgelist=path_edges, width=2,
                            edge_color='blue', arrows=True, arrowsize=15, ax=ax)
    
    # Mark start and end nodes
    if path:
        nx.draw_networkx_nodes(graph, pos, nodelist=[path[0]], node_size=node_size*1.5,
                               node_color='green', ax=ax)
        nx.draw_networkx_nodes(graph, pos, nodelist=[path[-1]], node_size=node_size*1.5,
                               node_color='red', ax=ax)
    
    # Draw the current position
    if current_pos:
        x, y, yaw = current_pos
        ax.plot(x, y, 'ro', markersize=10)
        
        # Draw direction indicator
        arrow_len = 0.2
        dx = arrow_len * math.cos(math.radians(yaw))
        dy = arrow_len * math.sin(math.radians(yaw))
        ax.arrow(x, y, dx, dy, head_width=0.05, head_length=0.1, fc='r', ec='r')
    
    ax.set_title('Planned Path')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    
    return fig

def visualize_path_on_image(frame, graph, path, current_pos, transform_params):
    """
    Visualize path on camera frame by projecting map coordinates to image coordinates.
    
    Args:
        frame (numpy.ndarray): Camera frame
        graph (networkx.Graph): The map graph
        path (list): List of node IDs in the path
        current_pos (tuple): Current position (x, y, yaw)
        transform_params (dict): Transform parameters from map to image coordinates
            - 'scale_x': Scale factor for x coordinates
            - 'scale_y': Scale factor for y coordinates
            - 'offset_x': Offset for x coordinates
            - 'offset_y': Offset for y coordinates
            
    Returns:
        numpy.ndarray: Frame with path overlay
    """
    if frame is None or not path:
        return frame
    
    # Create a copy of the frame
    overlay = frame.copy()
    
    # Extract transform parameters
    scale_x = transform_params.get('scale_x', 100)
    scale_y = transform_params.get('scale_y', -100)  # Y is flipped in image coordinates
    offset_x = transform_params.get('offset_x', frame.shape[1] // 2)
    offset_y = transform_params.get('offset_y', frame.shape[0] // 2)
    
    # Function to convert map coordinates to image coordinates
    def map_to_image(x, y):
        img_x = int(x * scale_x + offset_x)
        img_y = int(y * scale_y + offset_y)
        return (img_x, img_y)
    
    # Draw path edges
    for i in range(len(path) - 1):
        node1 = path[i]
        node2 = path[i+1]
        
        x1 = float(graph.nodes[node1]['x'])
        y1 = float(graph.nodes[node1]['y'])
        x2 = float(graph.nodes[node2]['x'])
        y2 = float(graph.nodes[node2]['y'])
        
        pt1 = map_to_image(x1, y1)
        pt2 = map_to_image(x2, y2)
        
        cv2.line(overlay, pt1, pt2, (0, 255, 0), 2)
    
    # Draw nodes
    for node in path:
        x = float(graph.nodes[node]['x'])
        y = float(graph.nodes[node]['y'])
        
        pt = map_to_image(x, y)
        
        # Different colors for special nodes
        if node == path[0]:
            color = (0, 255, 0)  # Green for start
        elif node == path[-1]:
            color = (0, 0, 255)  # Red for end
        elif node in graph.nodes:
            color = (255, 0, 0)  # Blue for regular nodes
        
        cv2.circle(overlay, pt, 5, color, -1)
    
    # Draw current position
    if current_pos:
        x, y, yaw = current_pos
        pt = map_to_image(x, y)
        
        # Draw position marker
        cv2.circle(overlay, pt, 8, (0, 165, 255), -1)
        
        # Draw direction indicator
        arrow_len = 20
        dx = int(arrow_len * math.cos(math.radians(yaw)))
        dy = int(arrow_len * math.sin(math.radians(yaw)))
        cv2.arrowedLine(overlay, pt, (pt[0] + dx, pt[1] - dy), (0, 165, 255), 2)
    
    # Blend the overlay with the original frame
    alpha = 0.6
    output = cv2.addWeighted(frame, 1 - alpha, overlay, alpha, 0)
    
    return output

def estimate_transform_params(graph, frame_width, frame_height):
    """
    Estimate transform parameters to map graph coordinates to image coordinates.
    
    Args:
        graph (networkx.Graph): The map graph
        frame_width (int): Width of the camera frame
        frame_height (int): Height of the camera frame
        
    Returns:
        dict: Transform parameters
    """
    # Get min and max coordinates of the graph
    min_x = float('inf')
    max_x = float('-inf')
    min_y = float('inf')
    max_y = float('-inf')
    
    for node in graph.nodes():
        x = float(graph.nodes[node]['x'])
        y = float(graph.nodes[node]['y'])
        
        min_x = min(min_x, x)
        max_x = max(max_x, x)
        min_y = min(min_y, y)
        max_y = max(max_y, y)
    
    # Calculate scale factors
    graph_width = max_x - min_x
    graph_height = max_y - min_y
    
    # Add margin
    margin = 0.1
    graph_width *= (1 + margin)
    graph_height *= (1 + margin)
    
    # Calculate scale factors
    scale_x = (frame_width * 0.8) / graph_width
    scale_y = (frame_height * 0.8) / graph_height
    
    # Use the smaller scale factor to maintain aspect ratio
    scale = min(scale_x, scale_y)
    
    # Calculate offsets to center the graph
    graph_center_x = (min_x + max_x) / 2
    graph_center_y = (min_y + max_y) / 2
    
    offset_x = frame_width // 2 - graph_center_x * scale
    offset_y = frame_height // 2 + graph_center_y * scale  # Y is flipped in image coordinates
    
    return {
        'scale_x': scale,
        'scale_y': -scale,  # Negative because y-axis is flipped in image coordinates
        'offset_x': offset_x,
        'offset_y': offset_y
    }

def draw_info_panel(frame, info, position=(10, 30), font_scale=0.5, thickness=1, color=(255, 255, 255)):
    """
    Draw an information panel on the frame.
    
    Args:
        frame (numpy.ndarray): Frame to draw on
        info (dict): Dictionary of information to display
        position (tuple): Position to start drawing
        font_scale (float): Font scale
        thickness (int): Line thickness
        color (tuple): Text color
        
    Returns:
        numpy.ndarray: Frame with info panel
    """
    x, y = position
    line_height = 25
    
    # Create a semi-transparent overlay
    overlay = frame.copy()
    cv2.rectangle(overlay, (x-10, y-20), (x+300, y+line_height*len(info)), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)
    
    # Draw the information
    for i, (key, value) in enumerate(info.items()):
        text = f"{key}: {value}"
        cv2.putText(frame, text, (x, y + i*line_height), 
                    cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, thickness)
    
    return frame