import open3d as o3d
import os
from ament_index_python import get_package_share_directory
import numpy as np
from sklearn.mixture import GaussianMixture


# 从 STL 文件加载 3D 模型
def get_convex_hull(path):
    mesh = o3d.io.read_triangle_mesh(path)
    convex_hull,convex_hull_vertex_indices = mesh.compute_convex_hull()
    ps = np.asarray(convex_hull.vertices)

    n_components_scores = []
    means = []
    for n_components in range(1, 11):
        gmm = GaussianMixture(n_components=n_components)
        gmm.fit(ps)
        n_components_scores.append(gmm.score(ps))

    
    max_index = n_components_scores.index(max(n_components_scores))
    gmm = GaussianMixture(n_components=max_index)
    gmm.fit(ps)
    print("means = gmm.means_",gmm.means_)


    return gmm.means_





def main():
    path_pos = os.path.join(
                get_package_share_directory("med7_dock_description"),
                "meshes",
                "EndEffector.STL",
            )

    mesh = o3d.io.read_triangle_mesh(path_pos)

    # 提取 3D 凸包
    convex_hull,convex_hull_vertex_indices = mesh.compute_convex_hull()

    # 可视化原始模型和提取的凸包
    ps = np.asarray(convex_hull.vertices)



    n_components_scores = []
    means = []
    for n_components in range(1, 11):
        gmm = GaussianMixture(n_components=n_components)
        gmm.fit(ps)
        n_components_scores.append(gmm.score(ps))

    max_index = n_components_scores.index(max(n_components_scores))

    gmm = GaussianMixture(n_components=max_index)
    gmm.fit(ps)
    print("means = gmm.means_",gmm.means_)


    # print("")
    point_cloud_o3d = o3d.geometry.PointCloud()
    point_cloud_o3d.points = o3d.utility.Vector3dVector(ps)


    point_cloud_o3d_GMM = o3d.geometry.PointCloud()
    point_cloud_o3d_GMM.points = o3d.utility.Vector3dVector(np.asarray(gmm.means_))

    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(mesh)
    vis.add_geometry(point_cloud_o3d_GMM)
    vis.run()
    vis.destroy_window()


if __name__ == "__main__":
    main()