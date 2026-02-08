import struct
import sys
import os

def convert_stl_to_obj(stl_path, obj_path):
    print(f"Converting {stl_path} to {obj_path}")
    
    with open(stl_path, 'rb') as f:
        header = f.read(80)
        # Check if ASCII - looking for "solid" at start
        if header.strip().startswith(b'solid') and b'COLOR=' not in header:
            # Simple heuristic, SolidWorks binary STLs start with 'solid' sometimes but have binary data
            # Real ASCII check involves reading more lines
            pass

        # Assume Binary for now as confirmed by 'file' and 'xxd'
        count_bytes = f.read(4)
        if len(count_bytes) < 4:
            print("Error: File too short")
            return
            
        num_triangles = struct.unpack('<I', count_bytes)[0]
        print(f"Triangles: {num_triangles}")
        
        vertices = []
        faces = []
        
        # Read all triangles
        # Each triangle is 50 bytes: 12 (normal) + 36 (3 vertices) + 2 (attr)
        data = f.read()
        
        if len(data) < num_triangles * 50:
            print("Warning: Data length mismatch")
            
        with open(obj_path, 'w') as obj:
            obj.write(f"# Converted from {os.path.basename(stl_path)}\n")
            
            for i in range(num_triangles):
                offset = i * 50
                # Skip normal (0-12)
                # Vertex 1
                v1 = struct.unpack('<fff', data[offset+12:offset+24])
                # Vertex 2
                v2 = struct.unpack('<fff', data[offset+24:offset+36])
                # Vertex 3
                v3 = struct.unpack('<fff', data[offset+36:offset+48])
                
                # Write vertices
                obj.write(f"v {v1[0]} {v1[1]} {v1[2]}\n")
                obj.write(f"v {v2[0]} {v2[1]} {v2[2]}\n")
                obj.write(f"v {v3[0]} {v3[1]} {v3[2]}\n")
                
                # Write face (1-indexed)
                base = i * 3 + 1
                obj.write(f"f {base} {base+1} {base+2}\n")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 stl2obj.py <input_dir> <output_dir>")
        sys.exit(1)
        
    input_dir = sys.argv[1]
    output_dir = sys.argv[2] # We will write in place actually if needed, or same dir
    
    # We will just process the specific files for GCR5_910
    files = ["base_link.stl", "link_1.stl", "link_2.stl", "link_3.stl", "link_4.stl", "link_5.stl", "link_6.stl"]
    
    for fname in files:
        stl_path = os.path.join(input_dir, fname)
        obj_name = fname.replace(".stl", ".obj")
        obj_path = os.path.join(input_dir, obj_name) # Save in same directory
        
        if os.path.exists(stl_path):
            try:
                convert_stl_to_obj(stl_path, obj_path)
            except Exception as e:
                print(f"Failed to convert {fname}: {e}")
