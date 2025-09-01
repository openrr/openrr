use std::{io, path::Path};

use k::{RealField, nalgebra as na};
use ncollide3d::shape::TriMesh;

use crate::errors::*;

#[cfg(feature = "assimp")]
pub(crate) fn load_mesh<P, T>(filename: P, scale: &[f64; 3]) -> Result<TriMesh<T>>
where
    P: AsRef<Path>,
    T: RealField + Copy,
{
    let filename = filename.as_ref();
    let mut importer = assimp::Importer::new();
    importer.pre_transform_vertices(|x| x.enable = true);
    importer.collada_ignore_up_direction(true);
    importer.triangulate(true);
    if let Some(file_string) = filename.to_str() {
        match importer.read_file(file_string) {
            Ok(assimp_scene) => Ok(assimp_scene_to_ncollide_mesh(assimp_scene, scale)),
            Err(err) => Err(Error::MeshError(err.to_owned())),
        }
    } else {
        // assimp crate only supports UTF-8 path
        load_mesh_with_mesh_loader(filename, scale)
    }
}

#[cfg(not(feature = "assimp"))]
pub(crate) use load_mesh_with_mesh_loader as load_mesh;
pub(crate) fn load_mesh_with_mesh_loader<P, T>(filename: P, scale: &[f64; 3]) -> Result<TriMesh<T>>
where
    P: AsRef<Path>,
    T: RealField + Copy,
{
    let filename = filename.as_ref();
    let mesh = mesh_loader::Loader::default()
        .merge_meshes(true)
        .load(filename)
        .map_err(|e| {
            if e.kind() == io::ErrorKind::Unsupported {
                if cfg!(feature = "assimp") {
                    Error::MeshError(format!(
                        "assimp feature is enabled but filename is not UTF-8: could not parse {filename:?}"
                    ))
                } else {
                    Error::MeshError(format!(
                        "assimp feature is disabled: could not parse {filename:?}"
                    ))
                }
            } else {
                e.into()
            }
        })?
        .meshes
        .pop()
        .unwrap(); // merge_meshes(true) merges all meshes into one

    let vertices = mesh
        .vertices
        .iter()
        .map(|v| {
            na::Point3::<T>::new(
                na::convert(v[0] as f64 * scale[0]),
                na::convert(v[1] as f64 * scale[1]),
                na::convert(v[2] as f64 * scale[2]),
            )
        })
        .collect();
    let indices = mesh
        .faces
        .iter()
        .map(|face| na::Point3::new(face[0] as usize, face[1] as usize, face[2] as usize))
        .collect();

    Ok(TriMesh::new(vertices, indices, None))
}

#[cfg(feature = "assimp")]
fn assimp_scene_to_ncollide_mesh<T>(scene: assimp::Scene<'_>, scale: &[f64; 3]) -> TriMesh<T>
where
    T: RealField + Copy,
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
                Some(na::Point3::new(
                    f[0] as usize + last_index,
                    f[1] as usize + last_index,
                    f[2] as usize + last_index,
                ))
            } else {
                None
            }
        }));
        last_index = vertices.len();
    }
    TriMesh::new(vertices, indices, None)
}
