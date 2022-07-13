use std::path::Path;

use k::{nalgebra as na, RealField, Vector3};
use parry3d::shape::{Ball, Cuboid, Cylinder, SharedShape, TriMesh};
use tracing::*;

use super::mesh::load_mesh;

pub(crate) fn urdf_geometry_to_shape_handle<T>(
    collision_geometry: &urdf_rs::Geometry,
    base_dir: Option<&Path>,
) -> Option<SharedShape>
where
    T: RealField + Copy,
{
    match *collision_geometry {
        urdf_rs::Geometry::Box { ref size } => {
            let cube = Cuboid::new(Vector3::new(
                na::convert(size[0] * 0.5),
                na::convert(size[1] * 0.5),
                na::convert(size[2] * 0.5),
            ));
            Some(SharedShape::new(cube))
        }
        urdf_rs::Geometry::Cylinder { radius, length } => {
            let y_cylinder = Cylinder::new(na::convert(length * 0.5), na::convert(radius));
            let components = y_cylinder.to_trimesh(30);
            Some(SharedShape::new(TriMesh::new(components.0, components.1)))
        }
        urdf_rs::Geometry::Capsule { .. } => {
            todo!()
        }
        urdf_rs::Geometry::Sphere { radius } => {
            Some(SharedShape::new(Ball::new(na::convert(radius))))
        }
        urdf_rs::Geometry::Mesh {
            ref filename,
            scale,
        } => {
            let scale = scale.unwrap_or(DEFAULT_MESH_SCALE);
            let replaced_filename = urdf_rs::utils::expand_package_path(filename, base_dir);
            let path = Path::new(&replaced_filename);
            if !path.exists() {
                error!("{replaced_filename} not found");
                return None;
            }
            match load_mesh(path, &scale) {
                Ok(mesh) => Some(SharedShape::new(mesh)),
                Err(err) => {
                    error!("load_mesh {path:?} failed: {err}");
                    None
                }
            }
        }
    }
}

// https://github.com/openrr/urdf-rs/pull/3/files#diff-0fb2eeea3273a4c9b3de69ee949567f546dc8c06b1e190336870d00b54ea0979L36-L38
const DEFAULT_MESH_SCALE: [f64; 3] = [1.0f64; 3];
