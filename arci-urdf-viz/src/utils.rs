use std::fmt;

use arci::nalgebra as na;
use serde::{de::DeserializeOwned, Deserialize, Serialize};
use url::Url;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub(crate) struct JointState {
    pub names: Vec<String>,
    pub positions: Vec<f64>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub(crate) struct BasePose {
    pub position: [f64; 3],
    pub quaternion: [f64; 4],
}

impl From<na::Isometry2<f64>> for BasePose {
    fn from(nav_pose: na::Isometry2<f64>) -> Self {
        let mut position = [0.0; 3];
        position[0] = nav_pose.translation.x;
        position[1] = nav_pose.translation.y;
        let quaternion = quaternion_from_euler_angles(0.0, 0.0, nav_pose.rotation.angle());
        Self {
            position,
            quaternion,
        }
    }
}

#[derive(Serialize, Deserialize)]
pub(crate) struct RpcResult {
    pub is_ok: bool,
    pub reason: String,
}

fn map_connection_error<E: fmt::Display>(url: &Url) -> impl FnOnce(E) -> arci::Error + '_ {
    move |e: E| arci::Error::Connection {
        message: format!("url:{}: {}", url, e),
    }
}

fn get(url: Url) -> Result<ureq::Response, arci::Error> {
    ureq::get(url.as_str())
        .call()
        .map_err(map_connection_error(&url))
}

fn post<T: Serialize, U: DeserializeOwned>(url: Url, msg: T) -> Result<U, arci::Error> {
    ureq::post(url.as_str())
        .send_json(serde_json::to_value(msg).unwrap())
        .map_err(map_connection_error(&url))?
        .into_json()
        .map_err(map_connection_error(&url))
}

pub(crate) fn get_joint_positions(base_url: &Url) -> Result<JointState, arci::Error> {
    get(base_url.join("get_joint_positions").unwrap())?
        .into_json()
        .map_err(map_connection_error(base_url))
}

pub(crate) fn send_joint_positions(
    base_url: &Url,
    joint_state: JointState,
) -> Result<(), arci::Error> {
    let res: RpcResult = post(base_url.join("set_joint_positions").unwrap(), joint_state)?;
    if !res.is_ok {
        return Err(arci::Error::Connection {
            message: res.reason,
        });
    }
    Ok(())
}

pub(crate) fn get_robot_origin(base_url: &Url) -> Result<BasePose, arci::Error> {
    get(base_url.join("get_robot_origin").unwrap())?
        .into_json()
        .map_err(map_connection_error(base_url))
}

pub(crate) fn send_robot_origin(base_url: &Url, base_pose: BasePose) -> Result<(), arci::Error> {
    let res: RpcResult = post(base_url.join("set_robot_origin").unwrap(), base_pose)?;
    if !res.is_ok {
        return Err(arci::Error::Connection {
            message: res.reason,
        });
    }
    Ok(())
}

pub(crate) fn get_urdf(base_url: &Url) -> Result<urdf_rs::Robot, arci::Error> {
    let s = get(base_url.join("get_urdf_text").unwrap())?
        .into_string()
        .map_err(map_connection_error(base_url))?;
    Ok(urdf_rs::read_from_string(&s)?)
}

pub(crate) fn euler_angles_from_quaternion(q: &[f64; 4]) -> (f64, f64, f64) {
    to_nalgebra(q).euler_angles()
}

pub(crate) fn quaternion_from_euler_angles(r: f64, p: f64, y: f64) -> [f64; 4] {
    let na_q = na::UnitQuaternion::from_euler_angles(r, p, y);
    from_nalgebra(&na_q)
}

fn from_nalgebra(na_q: &na::UnitQuaternion<f64>) -> [f64; 4] {
    let mut q = [0.0; 4];
    q[0] = na_q.w;
    q[1] = na_q.i;
    q[2] = na_q.j;
    q[3] = na_q.k;
    q
}

fn to_nalgebra(q: &[f64; 4]) -> na::UnitQuaternion<f64> {
    na::UnitQuaternion::from_quaternion(na::Quaternion::new(q[0], q[1], q[2], q[3]))
}

#[cfg(test)]
mod tests {
    use assert_approx_eq::assert_approx_eq;

    use super::*;
    #[test]
    fn test_euler_quaternion() {
        const R: f64 = 0.5;
        const P: f64 = -0.2;
        const Y: f64 = 1.0;
        let q = quaternion_from_euler_angles(R, P, Y);
        let angles = euler_angles_from_quaternion(&q);
        assert_approx_eq!(angles.0, R);
        assert_approx_eq!(angles.1, P);
        assert_approx_eq!(angles.2, Y);
        let q2 = quaternion_from_euler_angles(angles.0, angles.1, angles.2);
        assert_approx_eq!(q[0], q2[0]);
        assert_approx_eq!(q[1], q2[1]);
        assert_approx_eq!(q[2], q2[2]);
        assert_approx_eq!(q[3], q2[3]);
    }
}
