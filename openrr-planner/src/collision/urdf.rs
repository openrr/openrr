use std::path::Path;

use k::{nalgebra as na, RealField, Vector3};
use ncollide3d::{
    procedural::IndexBuffer::{Split, Unified},
    shape::{Ball, Cuboid, Cylinder, ShapeHandle, TriMesh},
    transformation::ToTriMesh,
};
use tracing::*;

use super::mesh::load_mesh;

pub(crate) fn urdf_geometry_to_shape_handle<T>(
    collision_geometry: &urdf_rs::Geometry,
    base_dir: Option<&Path>,
) -> Option<ShapeHandle<T>>
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
                    panic!("convex_hull implementation has been changed by ncollide3d update?");
                }
            };
            Some(ShapeHandle::new(TriMesh::new(
                tri_mesh.coords,
                ind,
                tri_mesh.uvs,
            )))
        }
        urdf_rs::Geometry::Capsule { .. } => {
            todo!()
        }
        urdf_rs::Geometry::Sphere { radius } => {
            Some(ShapeHandle::new(Ball::new(na::convert(radius))))
        }
        urdf_rs::Geometry::Mesh {
            ref filename,
            scale,
        } => {
            let scale = scale.unwrap_or(DEFAULT_MESH_SCALE);
            let replaced_filename = match urdf_rs::utils::expand_package_path(filename, base_dir) {
                Ok(replaced_filename) => replaced_filename,
                Err(e) => {
                    error!("{e}");
                    return None;
                }
            };
            let path = Path::new(&*replaced_filename);
            if !path.exists() {
                error!("{replaced_filename} not found");
                return None;
            }
            match load_mesh(path, &scale) {
                Ok(mesh) => Some(ShapeHandle::new(mesh)),
                Err(err) => {
                    error!("load_mesh {path:?} failed: {err}");
                    None
                }
            }
        }
    }
}

pub(crate) fn k_link_geometry_to_shape_handle<T>(
    collision_geometry: &k::link::Geometry<T>,
) -> Option<ShapeHandle<T>>
where
    T: RealField + Copy + k::SubsetOf<f64>,
{
    let converted_geometry = match collision_geometry {
        k::link::Geometry::Box {
            depth,
            width,
            height,
        } => urdf_rs::Geometry::Box {
            size: urdf_rs::Vec3([
                na::convert(*depth),
                na::convert(*width),
                na::convert(*height),
            ]),
        },
        k::link::Geometry::Cylinder { radius, length } => urdf_rs::Geometry::Cylinder {
            radius: na::convert(*radius),
            length: na::convert(*length),
        },
        k::link::Geometry::Capsule { .. } => {
            todo!()
        }
        k::link::Geometry::Sphere { radius } => urdf_rs::Geometry::Sphere {
            radius: na::convert(*radius),
        },
        k::link::Geometry::Mesh { filename, scale } => urdf_rs::Geometry::Mesh {
            filename: filename.to_string(),
            scale: Some(urdf_rs::Vec3([
                na::convert(scale[0]),
                na::convert(scale[1]),
                na::convert(scale[2]),
            ])),
        },
    };
    urdf_geometry_to_shape_handle(&converted_geometry, None)
}

// https://github.com/openrr/urdf-rs/pull/3/files#diff-0fb2eeea3273a4c9b3de69ee949567f546dc8c06b1e190336870d00b54ea0979L36-L38
const DEFAULT_MESH_SCALE: urdf_rs::Vec3 = urdf_rs::Vec3([1.0f64; 3]);
