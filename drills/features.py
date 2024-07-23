import os
import subprocess
import networkx as nx
import numpy as np
from sklearn.preprocessing import LabelEncoder, MinMaxScaler

def run_command(command):
    try:
        subprocess.run(command, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except subprocess.CalledProcessError as e:
        print(f"Command '{command}' failed with error: {e.stderr.decode('utf-8')}")
        raise


def optimize_aig(input_file, optimization):
    blif_output_file = f"{os.path.splitext(input_file)[0]}_{optimization.replace(' ', '_')}.blif"
    ascii_output_file = f"{os.path.splitext(input_file)[0]}_{optimization.replace(' ', '_')}.aag"
    
    # Run yosys-abc for optimization and output BLIF
    abc_command = f"yosys-abc -c 'read_verilog {input_file}; strash; {optimization}; write_blif {blif_output_file}'"
    run_command(abc_command)
    
    # Convert BLIF to ASCII AAG using Yosys with additional synthesis steps
    yosys_command = f"yosys -p 'read_blif {blif_output_file}; techmap; aigmap; write_aiger -ascii {ascii_output_file}'"
    run_command(yosys_command)
    
    # Remove the intermediate BLIF file
    os.remove(blif_output_file)
    
    return ascii_output_file

def handle_inverter(node):
    if node % 2 == 1:
        return node - 1, True
    else:
        return node, False

def parse_aag(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()

    header = lines[0].strip().split()
    num_inputs = int(header[2])
    num_outputs = int(header[4])
    num_ands = int(header[5])

    G = nx.DiGraph()

    current_line = 1

    input_nodes = []
    for i in range(num_inputs):
        node = int(lines[current_line].strip())
        G.add_node(node, label='input')
        input_nodes.append(node)
        current_line += 1

    output_nodes = []
    for i in range(num_outputs):
        output = int(lines[current_line].strip())
        output_nodes.append(output)
        current_line += 1

    for i in range(num_ands):
        parts = lines[current_line].strip().split()
        and_node = int(parts[0])
        fanin1, inverted1 = handle_inverter(int(parts[1]))
        fanin2, inverted2 = handle_inverter(int(parts[2]))
        if G.has_node(and_node) and G.nodes[and_node]['label'] != 'output':
            G.nodes[and_node]['label'] = 'gate'
        else:
            G.add_node(and_node, label='gate')
        G.add_edge(fanin1, and_node, inverter=inverted1)
        G.add_edge(fanin2, and_node, inverter=inverted2)
        current_line += 1

    for output in output_nodes:
        if G.has_node(output):
            G.nodes[output]['label'] = 'output'
        else:
            G.add_node(output, label='output')
        fanin, inverted = handle_inverter(output)
        G.add_edge(fanin, output, inverter=inverted)

    return G

# def print_graph_info(G):
#     print("\nGraph Information:")
#     print(f"Number of nodes: {G.number_of_nodes()}")
#     print(f"Number of edges: {G.number_of_edges()}")
    
#     node_types = {}
#     for node, data in G.nodes(data=True):
#         node_type = data.get('label', 'unknown')
#         node_types[node_type] = node_types.get(node_type, 0) + 1
    
#     print("\nNode Types:")
#     for node_type, count in node_types.items():
#         print(f"  {node_type}: {count}")
    
#     print("\nGraph Structure:")
#     for node in sorted(G.nodes()):
#         print(f"Node {node} ({G.nodes[node].get('label', 'unknown')}):")
#         in_edges = list(G.in_edges(node, data=True))
#         out_edges = list(G.out_edges(node, data=True))
        
#         if in_edges:
#             print("  In edges:")
#             for u, v, data in in_edges:
#                 inverter = "inverted" if data.get('inverter', False) else "normal"
#                 print(f"    From Node {u} ({inverter})")
        
#         if out_edges:
#             print("  Out edges:")
#             for u, v, data in out_edges:
#                 inverter = "inverted" if data.get('inverter', False) else "normal"
#                 print(f"    To Node {v} ({inverter})")
        
#         print()

def deterministic_node_embedding(G, dimensions=9):
    sorted_nodes = sorted(G.nodes())
    node_to_idx = {node: idx for idx, node in enumerate(sorted_nodes)}
    
    embeddings = np.zeros((len(sorted_nodes), dimensions))
    
    label_encoder = LabelEncoder()
    node_labels = [G.nodes[node].get('label', 'unknown') for node in sorted_nodes]
    encoded_labels = label_encoder.fit_transform(node_labels)
    
    for node in sorted_nodes:
        idx = node_to_idx[node]
        
        # 1. Node degree as a feature
        embeddings[idx, 0] = G.degree(node)
        
        # 2. Node label as a feature
        embeddings[idx, 1] = encoded_labels[idx]
        
        # 3. Indices of sorted neighbors as features and 4. Edge type (inverter or normal) as feature
        neighbors = sorted(G.neighbors(node))
        for i, neighbor in enumerate(neighbors[:(dimensions-2)//2]):  # Limit number of neighbors to fit dimensions
            neighbor_idx = node_to_idx[neighbor]
            embeddings[idx, i*2+2] = neighbor_idx
            edge_data = G.get_edge_data(node, neighbor)
            if edge_data and edge_data.get('inverter', False):
                embeddings[idx, i*2+3] = 1  # Inverted edge
            else:
                embeddings[idx, i*2+3] = 0  # Normal edge
    
    # Normalize embeddings
    scaler = MinMaxScaler()
    normalized_embeddings = scaler.fit_transform(embeddings)
    
    return normalized_embeddings, sorted_nodes

def generate_graph_vector(node_embeddings):
    return np.mean(node_embeddings, axis=0)

def extract_features(design_file, optimization=""):
    # Optimize AIG and get ASCII AAG file
    aag_file = optimize_aig(design_file, optimization)
    
    # Parse the AAG file to create a graph
    G = parse_aag(aag_file)
    
    # Generate node embeddings
    node_embeddings, sorted_nodes = deterministic_node_embedding(G)
    
    # Generate and print the final graph vector
    graph_vector = generate_graph_vector(node_embeddings)
    # print("\nFinal Graph Vector:")
    # print(graph_vector)
    
    # Remove the ASCII AAG file after processing
    os.remove(aag_file)
    
    return graph_vector

# # Example usage:
# if __name__ == "__main__":
#     design_file = "/home/xyin014/DRiLLS_test/LFA_OPT/adder_aig.v"
#     graph_vector = extract_features(design_file)
