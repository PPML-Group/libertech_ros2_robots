import os
import trimesh
import numpy as np

def convert_stl_to_convex(input_path, output_path):
    # 创建输出文件夹
    os.makedirs(output_path, exist_ok=True)
    
    # 遍历输入文件夹中的所有STL文件
    for filename in os.listdir(input_path):
        if filename.endswith(".STL"):
            # 加载STL文件
            mesh = trimesh.load_mesh(os.path.join(input_path, filename))
            
            if isinstance(mesh, trimesh.Trimesh):
                convex_mesh = mesh.convex_hull
            elif isinstance(mesh, np.ndarray) and mesh.shape[1] == 3:
                convex_mesh = trimesh.convex.convex_hull(mesh)
            else:
                print("Object must be Trimesh or (n,3) points! Skipping...")
                continue
    
            # # 将几何体转换为凸包
            # convex_mesh = mesh.convex_hull
            
            # 构造输出文件路径
            output_filename = os.path.join(output_path, filename)
            
            # 保存凸包为STL文件
            convex_mesh.export(output_filename)
            print(f"Converted {filename} to convex hull and saved to {output_filename}")

if __name__ == "__main__":
    input_path = "libertech_hand_description/meshes"
    output_path = "libertech_hand_description/meshes/collision"
    convert_stl_to_convex(input_path, output_path)

