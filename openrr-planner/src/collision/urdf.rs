use super::mesh::load_mesh;
use k::{RealField, Vector3};
use log::*;
use nalgebra as na;
use ncollide3d::procedural::IndexBuffer::{Split, Unified};
use ncollide3d::shape::{Ball, Cuboid, Cylinder, ShapeHandle, TriMesh};
use ncollide3d::transformation::ToTriMesh;
use std::path::Path;

pub(crate) fn urdf_geometry_to_shape_handle<T>(
    collision_geometry: &urdf_rs::Geometry,
    base_dir: Option<&Path>,
) -> Option<ShapeHandle<T>>
where
    T: RealField,
{
    match *collision_geometry {
        urdf_rs::Geometry::Box { ref size } => {
            let cube = Cuboid::new(Vector3::new(
                na::convert(size[0] * 0.5),
                na::convert(size[1] * 0.5),
                na::convert(size[2] * 0.5),
            ));
            Some(ShapeHandle::new(cube))
        }
        urdf_rs::Geometry::Cylinder { radius, length } => {
            let y_cylinder = Cylinder::new(na::convert(length * 0.5), na::convert(radius));
            let tri_mesh = ncollide3d::transformation::convex_hull(
                &y_cylinder
                    .to_trimesh(30)
                    .coords
                    .iter()
                    .map(|point| point.xzy())
                    .collect::<Vec<_>>(),
            );
            let ind = match tri_mesh.indices {
                Unified(ind) => ind
                    .into_iter()
                    .map(|p| na::Point3::new(p[0] as usize, p[1] as usize, p[2] as usize))
                    .collect(),
                Split(_) => {
                    panic!("convex_hull implemenataion has been changed by ncollide3d update?");
                }
            };
            Some(ShapeHandle::new(TriMesh::new(
                tri_mesh.coords,
                ind,
                tri_mesh.uvs,
            )))
        }
        urdf_rs::Geometry::Sphere { radius } => {
            Some(ShapeHandle::new(Ball::new(na::convert(radius))))
        }
        urdf_rs::Geometry::Mesh {
            ref filename,
            scale,
        } => {
            let replaced_filename = urdf_rs::utils::expand_package_path(filename, base_dir);
            let path = Path::new(&replaced_filename);
            if !path.exists() {
                error!("{} not found", replaced_filename);
                return None;
            }
            match load_mesh(path, &scale) {
                Ok(mesh) => Some(ShapeHandle::new(mesh)),
                Err(err) => {
                    error!("load_mesh {:?} failed: {}", path, err);
                    None
                }
            }
        }
    }
}
