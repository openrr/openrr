use std::path::Path;

use k::{nalgebra as na, RealField};
use ncollide3d::shape::TriMesh;

use crate::errors::*;

pub(crate) fn load_mesh<P, T>(filename: P, scale: &[f64; 3]) -> Result<TriMesh<T>>
where
    P: AsRef<Path>,
    T: RealField + Copy,
{
    let filename = filename.as_ref();
    let mesh = mesh_loader::Loader::default()
        .merge_meshes(true)
        .load(filename)?
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
