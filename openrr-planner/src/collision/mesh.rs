use std::path::Path;

#[cfg(feature = "assimp")]
use k::nalgebra as na;
use k::RealField;
use ncollide3d::shape::TriMesh;

use crate::errors::*;

#[cfg(feature = "assimp")]
pub(crate) fn load_mesh<P, T>(filename: P, scale: &[f64]) -> Result<TriMesh<T>>
where
    P: AsRef<Path> + std::fmt::Debug,
    T: RealField,
{
    let mut importer = assimp::Importer::new();
    importer.pre_transform_vertices(|x| x.enable = true);
    importer.collada_ignore_up_direction(true);
    if let Some(file_string) = filename.as_ref().to_str() {
        match importer.read_file(file_string) {
            Ok(assimp_scene) => Ok(assimp_scene_to_ncollide_mesh(assimp_scene, scale)),
            Err(err) => Err(Error::MeshError(err.to_owned())),
        }
    } else {
        Err(Error::MeshError(format!("failed to parse {:?}", filename)))
    }
}

#[cfg(not(feature = "assimp"))]
pub(crate) fn load_mesh<P, T>(filename: P, _scale: &[f64]) -> Result<TriMesh<T>>
where
    P: AsRef<Path> + std::fmt::Debug,
    T: RealField,
{
    Err(Error::MeshError(format!(
        "assimp feature is disabled: could not parse {:?}",
        filename
    )))
}

#[cfg(feature = "assimp")]
pub(crate) fn assimp_scene_to_ncollide_mesh<T>(scene: assimp::Scene, scale: &[f64]) -> TriMesh<T>
where
    T: RealField,
{
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    let mut last_index: usize = 0;
    for mesh in scene.mesh_iter() {
        vertices.extend(mesh.vertex_iter().map(|v| {
            na::Point3::<T>::new(
                na::convert(v.x as f64 * scale[0]),
                na::convert(v.y as f64 * scale[1]),
                na::convert(v.z as f64 * scale[2]),
            )
        }));
        indices.extend(mesh.face_iter().filter_map(|f| {
            if f.num_indices == 3 {
                Some(na::Point3::<usize>::new(
                    f[0] as usize + last_index,
                    f[1] as usize + last_index,
                    f[2] as usize + last_index,
                ))
            } else {
                None
            }
        }));
        last_index = vertices.len() as usize;
    }
    TriMesh::new(vertices, indices, None)
}
