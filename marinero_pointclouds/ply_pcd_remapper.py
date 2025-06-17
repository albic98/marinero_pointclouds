#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node

class PLYRemappper(Node):
    def __init__(self):
        super().__init__("ply_remapper")
        
        # self.declare_parameter("ply_file_path", "/home/albert/LIDAR_data/Marina_Punat_zona_C.ply")
        # self.declare_parameter("origin", [352306.109894, 4989164.188812, 1.398855])
        
        self.declare_parameter("ply_file_path", "/home/albert/LIDAR_data/Marina_Punat_zona_B_15M.ply")
        self.declare_parameter("origin", [352484.570892, 4989517.552795, 0.948855])
        
        # self.declare_parameter("ply_file_path", "/home/albert/LIDAR_data/Marina_Punat_zona_C_10M.ply")
        # self.declare_parameter("origin", [352517.591919, 4989821.112793, 0.925855])

        self.ply_file_path = self.get_parameter("ply_file_path").get_parameter_value().string_value
        self.origin = self.get_parameter("origin").get_parameter_value().double_array_value

        self.remap_ply_coordinates()

    def remap_ply_coordinates(self):
        try:
            # Open the PLY file for reading
            with open(self.ply_file_path, "r") as f:
                lines = f.readlines()

            # Find the line where the vertex data starts (after the header)
            header_ended = False
            vertex_start_line = 0
            for i, line in enumerate(lines):
                if line.strip() == "end_header":
                    vertex_start_line = i + 1
                    header_ended = True
                    break
            
            if not header_ended:
                self.get_logger().error(f"Failed to find the end of the header in PLY file: {self.ply_file_path}")
                return
            
            # Prepare the output list to store the modified PLY data
            output_lines = lines[:vertex_start_line]  # Copy the header

            # Define the origin point
            origin_x, origin_y, origin_z = self.origin
            
            # Define lists for PCD data
            self.pcd_data = {
                "x": [],
                "y": [],
                "z": [],
                "r": [],
                "g": [],
                "b": []
            }

            # Process each vertex line after the header
            for line in lines[vertex_start_line:]:
                elements = line.split()

                # Extract and modify the x, y, z coordinates
                x_new = float(elements[0]) - origin_x
                y_new = float(elements[1]) - origin_y
                z_new = float(elements[2])
                
                # Extract RGB values (assuming they are in the fourth, fifth, and sixth columns)
                r = int(elements[3])
                g = int(elements[4])
                b = int(elements[5])

                # Store the modified coordinates and RGB values
                self.pcd_data["x"].append(x_new)
                self.pcd_data["y"].append(y_new)
                self.pcd_data["z"].append(z_new)
                self.pcd_data["r"].append(r)
                self.pcd_data["g"].append(g)
                self.pcd_data["b"].append(b)                

                # Reconstruct the line with modified coordinates and the remaining values unchanged
                new_line = f"{x_new:.6f} {y_new:.6f} {z_new:.6f} " + " ".join(elements[3:]) + "\n"
                output_lines.append(new_line)

            # Write the modified data to a new PLY file
            self.output_ply_file_path = self.ply_file_path.replace(".ply", "_remapped.ply")
            with open(self.output_ply_file_path, "w") as f_out:
                f_out.writelines(output_lines)

            # Log success message
            self.get_logger().info(f"New PLY file with updated coordinates saved at: {self.output_ply_file_path}")
            
            # Now convert to PCD
            self.ply_to_pcd()

        except Exception as e:
            self.get_logger().error(f"Failed to read or process PLY file: {e}")
            return None

    def ply_to_pcd(self):
        try:
            output_pcd_file_path = self.output_ply_file_path.replace(".ply", ".pcd")

            # Open the .ply file
            
            with open(output_pcd_file_path, "w") as pcd_file:
                num_points = len(self.pcd_data["x"])
                header = (
                        "# .PCD v0.7 - Point Cloud Data file format\n"
                        "VERSION 0.7\n"
                        "FIELDS x y z r g b\n"
                        "SIZE 4 4 4 4 4 4\n"
                        "TYPE F F F U U U\n"
                        "COUNT 1 1 1 1 1 1\n"
                        f"WIDTH {num_points}\n"
                        "HEIGHT 1\n"
                        "VIEWPOINT 0 0 0 1 0 0 0\n"
                        f"POINTS {num_points}\n"
                        "DATA ascii\n"
                )
                        
                # Write all to PCD header
                pcd_file.write(header)
                
                
                # Write point data (x, y, z, r, g, b)
                for i in range(num_points):
                    pcd_file.write(f"{self.pcd_data['x'][i]:.6f} {self.pcd_data['y'][i]:.6f} {self.pcd_data['z'][i]:.6f} {self.pcd_data['r'][i]} {self.pcd_data['g'][i]} {self.pcd_data['b'][i]}\n")
                    # pcd_file.write(f"{self.pcd_data["x"][i]:.6f} {self.pcd_data["y"][i]:.6f} {self.pcd_data["z"][i]:.6f} {self.pcd_data["r"][i]} {self.pcd_data["g"][i]} {self.pcd_data["b"][i]}\n")
    
            self.get_logger().info(f"New PCD file saved at: {output_pcd_file_path}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to convert PLY to PCD: {e}")
            return None 
        
        
def main(args=None):
    
    rclpy.init(args=args)
    node = PLYRemappper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
