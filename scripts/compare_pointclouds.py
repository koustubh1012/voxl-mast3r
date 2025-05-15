# import open3d as o3d
# import trimesh
# import numpy as np
# from scipy.spatial import cKDTree

# def glb_to_point_cloud(glb_path, num_points=100000):
#     mesh = trimesh.load(glb_path)
#     if not isinstance(mesh, trimesh.Trimesh):
#         mesh = mesh.dump(concatenate=True)

#     points, _ = trimesh.sample.sample_surface(mesh, num_points)
#     pcd = o3d.geometry.PointCloud()
#     pcd.points = o3d.utility.Vector3dVector(points)
#     return pcd

# def align_point_clouds(source, target):
#     threshold = 0.05
#     trans_init = np.identity(4)
#     reg_p2p = o3d.pipelines.registration.registration_icp(
#         source, target, threshold, trans_init,
#         o3d.pipelines.registration.TransformationEstimationPointToPoint())
#     source.transform(reg_p2p.transformation)
#     return source

# def chamfer_distance(pcd1, pcd2):
#     p1 = np.asarray(pcd1.points)
#     p2 = np.asarray(pcd2.points)
#     tree1 = cKDTree(p1)
#     tree2 = cKDTree(p2)
#     dist1, _ = tree1.query(p2)
#     dist2, _ = tree2.query(p1)
#     chamfer = np.mean(dist1**2) + np.mean(dist2**2)
#     return chamfer

# def hausdorff_distance(pcd1, pcd2):
#     p1 = np.asarray(pcd1.points)
#     p2 = np.asarray(pcd2.points)
#     tree1 = cKDTree(p1)
#     tree2 = cKDTree(p2)
#     dist1, _ = tree1.query(p2)
#     dist2, _ = tree2.query(p1)
#     hausdorff = max(np.max(dist1), np.max(dist2))
#     return hausdorff

# def visualize(pcd1, pcd2):
#     pcd1.paint_uniform_color([1, 0, 0])  # Red
#     pcd2.paint_uniform_color([0, 1, 0])  # Green
#     o3d.visualization.draw_geometries([pcd1, pcd2])


# def main():
#     model_a_path = "/home/koustubh/Downloads/lab_aprroach1.glb"
#     model_b_path = "/home/koustubh/Downloads/lab_approach2.glb"

#     print("Loading models and converting to point clouds...")
#     pcd_A = glb_to_point_cloud(model_a_path)
#     pcd_B = glb_to_point_cloud(model_b_path)

#     # Optional: align one model to the other
#     pcd_A = align_point_clouds(pcd_A, pcd_B)

#     print("Computing metrics...")
#     cd = chamfer_distance(pcd_A, pcd_B)
#     hd = hausdorff_distance(pcd_A, pcd_B)

#     print(f"Chamfer Distance: {cd:.6f}")
#     print(f"Hausdorff Distance: {hd:.6f}")

#     print("Visualizing point clouds (red = model A, green = model B)...")
#     visualize(pcd_A, pcd_B)

# if __name__ == "__main__":
#     main()


# import open3d as o3d
# import trimesh
# import numpy as np
# from scipy.spatial import cKDTree

# def glb_to_point_cloud(glb_path, num_points=100000):
#     mesh = trimesh.load(glb_path)
#     if not isinstance(mesh, trimesh.Trimesh):
#         mesh = mesh.to_geometry()  # Updated per deprecation warning

#     points, _ = trimesh.sample.sample_surface(mesh, num_points)
#     pcd = o3d.geometry.PointCloud()
#     pcd.points = o3d.utility.Vector3dVector(points)
#     return pcd

# def downsample_pcd(pcd, voxel_size=0.01):
#     return pcd.voxel_down_sample(voxel_size)

# def estimate_normals(pcd, radius=0.05):
#     pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=30))

# def align_point_clouds_icp(source, target, voxel_size=0.01):
#     source_down = downsample_pcd(source, voxel_size)
#     target_down = downsample_pcd(target, voxel_size)

#     estimate_normals(source_down)
#     estimate_normals(target_down)

#     threshold = 0.05
#     trans_init = np.identity(4)

#     reg_result = o3d.pipelines.registration.registration_icp(
#         source_down, target_down, threshold, trans_init,
#         o3d.pipelines.registration.TransformationEstimationPointToPoint())

#     print("ICP Fitness:", reg_result.fitness)
#     print("ICP Inlier RMSE:", reg_result.inlier_rmse)

#     # Apply transformation to original point cloud
#     source.transform(reg_result.transformation)
#     return source

# def chamfer_distance(pcd1, pcd2):
#     p1 = np.asarray(pcd1.points)
#     p2 = np.asarray(pcd2.points)
#     tree1 = cKDTree(p1)
#     tree2 = cKDTree(p2)
#     dist1, _ = tree1.query(p2)
#     dist2, _ = tree2.query(p1)
#     chamfer = np.mean(dist1**2) + np.mean(dist2**2)
#     return chamfer

# def hausdorff_distance(pcd1, pcd2):
#     p1 = np.asarray(pcd1.points)
#     p2 = np.asarray(pcd2.points)
#     tree1 = cKDTree(p1)
#     tree2 = cKDTree(p2)
#     dist1, _ = tree1.query(p2)
#     dist2, _ = tree2.query(p1)
#     hausdorff = max(np.max(dist1), np.max(dist2))
#     return hausdorff

# def visualize(pcd1, pcd2, filename="comparison.png"):
#     vis = o3d.visualization.Visualizer()
#     vis.create_window(visible=False)
#     pcd1.paint_uniform_color([1, 0, 0])  # Red
#     pcd2.paint_uniform_color([0, 1, 0])  # Green
#     vis.add_geometry(pcd1)
#     vis.add_geometry(pcd2)
#     vis.poll_events()
#     vis.update_renderer()
#     vis.capture_screen_image(filename)
#     vis.destroy_window()
#     print(f"Saved point cloud comparison to {filename}")
#     o3d.visualization.draw_geometries([pcd1, pcd2])

# # def visualize(pcd1, pcd2):
# #     pcd1.paint_uniform_color([1, 0, 0])  # Red
# #     pcd2.paint_uniform_color([0, 1, 0])  # Green
# #     o3d.visualization.draw_geometries([pcd1, pcd2])

# def main():
#     model_a_path = "/home/koustubh/Downloads/lab_aprroach1.glb"
#     model_b_path = "/home/koustubh/Downloads/lab_approach2.glb"

#     print("Loading models and converting to point clouds...")
#     pcd_A = glb_to_point_cloud(model_a_path)
#     pcd_B = glb_to_point_cloud(model_b_path)

#     print("Aligning point clouds with ICP...")
#     pcd_A = align_point_clouds_icp(pcd_A, pcd_B)

#     print("Computing metrics...")
#     cd = chamfer_distance(pcd_A, pcd_B)
#     hd = hausdorff_distance(pcd_A, pcd_B)

#     print(f"Chamfer Distance: {cd:.6f}")
#     print(f"Hausdorff Distance: {hd:.6f}")

#     print("Saving point cloud visualization to image...")
#     visualize(pcd_A, pcd_B)

# if __name__ == "__main__":
#     main()


import open3d as o3d
import trimesh
import numpy as np
from scipy.spatial import cKDTree


def glb_to_point_cloud(glb_path, num_points=100000):
    mesh = trimesh.load(glb_path)
    if not isinstance(mesh, trimesh.Trimesh):
        mesh = mesh.to_geometry()  # updated for deprecation
    points, _ = trimesh.sample.sample_surface(mesh, num_points)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd


def downsample_pcd(pcd, voxel_size):
    pcd_down = pcd.voxel_down_sample(voxel_size)
    pcd_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
    return pcd_down


def compute_fpfh(pcd_down, voxel_size):
    return o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=100)
    )


def global_registration(source, target, voxel_size):
    src_down = downsample_pcd(source, voxel_size)
    tgt_down = downsample_pcd(target, voxel_size)
    src_fpfh = compute_fpfh(src_down, voxel_size)
    tgt_fpfh = compute_fpfh(tgt_down, voxel_size)

    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        src_down, tgt_down, src_fpfh, tgt_fpfh, mutual_filter=True,
        max_correspondence_distance=voxel_size * 2,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        ransac_n=4,
        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(voxel_size * 2)
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(400000, 500)
    )
    return result.transformation


def refine_with_icp(source, target, initial_transform, voxel_size):
    source.transform(initial_transform)
    result = o3d.pipelines.registration.registration_icp(
        source, target, voxel_size,
        np.identity(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )
    print("ICP Fitness:", result.fitness)
    print("ICP Inlier RMSE:", result.inlier_rmse)
    source.transform(result.transformation)
    return source


def chamfer_distance(pcd1, pcd2):
    p1 = np.asarray(pcd1.points)
    p2 = np.asarray(pcd2.points)
    tree1 = cKDTree(p1)
    tree2 = cKDTree(p2)
    dist1, _ = tree1.query(p2)
    dist2, _ = tree2.query(p1)
    return np.mean(dist1**2) + np.mean(dist2**2)


def hausdorff_distance(pcd1, pcd2):
    p1 = np.asarray(pcd1.points)
    p2 = np.asarray(pcd2.points)
    tree1 = cKDTree(p1)
    tree2 = cKDTree(p2)
    dist1, _ = tree1.query(p2)
    dist2, _ = tree2.query(p1)
    return max(np.max(dist1), np.max(dist2))


def visualize(pcd1, pcd2, filename="comparison.png"):
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False)
    pcd1.paint_uniform_color([1, 0, 0])  # Red
    pcd2.paint_uniform_color([0, 1, 0])  # Green
    vis.add_geometry(pcd1)
    vis.add_geometry(pcd2)
    vis.poll_events()
    vis.update_renderer()
    vis.capture_screen_image(filename)
    vis.destroy_window()
    print(f"Saved visualization to {filename}")
    o3d.visualization.draw_geometries([pcd1, pcd2])


def main():

    model_a_path = "/home/koustubh/Downloads/lab_aprroach1.glb"
    model_b_path = "/home/koustubh/Downloads/lab_approach2.glb"
    voxel_size = 0.02  # tune this if needed

    print("Loading and sampling point clouds...")
    pcd_A = glb_to_point_cloud(model_a_path)
    pcd_B = glb_to_point_cloud(model_b_path)

    print("Registering point clouds...")
    transform = global_registration(pcd_A, pcd_B, voxel_size)
    pcd_A = refine_with_icp(pcd_A, pcd_B, transform, voxel_size)

    print("Computing metrics...")
    cd = chamfer_distance(pcd_A, pcd_B)
    hd = hausdorff_distance(pcd_A, pcd_B)
    print(f"Chamfer Distance: {cd:.6f}")
    print(f"Hausdorff Distance: {hd:.6f}")

    print("Saving comparison image...")
    visualize(pcd_A, pcd_B)


if __name__ == "__main__":
    main()
