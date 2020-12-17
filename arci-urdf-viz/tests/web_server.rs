// This is copy of urdf-viz/web_server.rs
// version = "0.21.1"
// https://github.com/OpenRR/urdf-viz/blob/v0.21.1/src/web_server.rs
// It is difficult to test with urdf-viz with `RUSTFLAGS=-Cpanic=abort`
// because of wayland-client.
// That flag is needed to get the test coverage.
//
// https://github.com/smilerobotics/sorriso/runs/1416786487
//
use rouille::*;
use serde_derive::*;
use std::sync::{Arc, Mutex};

#[derive(Deserialize, Serialize, Debug, Clone, Default)]
pub struct JointNamesAndPositions {
    pub names: Vec<String>,
    pub positions: Vec<f32>,
}

#[derive(Debug, Clone)]
pub struct JointNamesAndPositionsRequest {
    pub joint_positions: JointNamesAndPositions,
    pub requested: bool,
}

#[derive(Deserialize, Serialize, Debug, Clone, Default)]
pub struct RobotOrigin {
    pub position: [f32; 3],
    pub quaternion: [f32; 4],
}

#[derive(Debug, Clone)]
pub struct RobotOriginRequest {
    pub origin: RobotOrigin,
    pub requested: bool,
}

#[derive(Deserialize, Serialize, Debug, Clone)]
struct ResultResponse {
    is_ok: bool,
    reason: String,
}

#[derive(Debug)]
pub struct WebServer {
    pub port: u16,
    pub target_joint_positions: Arc<Mutex<JointNamesAndPositionsRequest>>,
    pub current_joint_positions: Arc<Mutex<JointNamesAndPositions>>,
    pub target_robot_origin: Arc<Mutex<RobotOriginRequest>>,
    pub current_robot_origin: Arc<Mutex<RobotOrigin>>,
}

impl WebServer {
    pub fn new(port: u16) -> Self {
        Self {
            port,
            target_joint_positions: Arc::new(Mutex::new(JointNamesAndPositionsRequest {
                joint_positions: JointNamesAndPositions::default(),
                requested: false,
            })),
            current_joint_positions: Arc::new(Mutex::new(JointNamesAndPositions::default())),
            target_robot_origin: Arc::new(Mutex::new(RobotOriginRequest {
                origin: RobotOrigin::default(),
                requested: false,
            })),
            current_robot_origin: Arc::new(Mutex::new(RobotOrigin::default())),
        }
    }
    pub fn start(self) {
        rouille::start_server(("0.0.0.0", self.port), move |request| {
            router!(request,
                (POST) (/set_joint_positions) => {
                    let json: JointNamesAndPositions = try_or_400!(rouille::input::json_input(request));
                    let mut jp_request = try_or_404!(self.target_joint_positions.lock());
                    jp_request.joint_positions = json;
                    let jp = jp_request.joint_positions.clone();
                    if jp.names.len() != jp.positions.len() {
                        Response::json(
                            &ResultResponse {
                                is_ok: false,
                                reason: format!("names and positions size mismatch ({} != {})",
                                jp.names.len(), jp.positions.len()),
                            })
                    } else {
                        jp_request.requested = true;
                        Response::json(&ResultResponse { is_ok: true, reason: "".to_string() })
                    }
                },
                (POST) (/set_robot_origin) => {
                    let json: RobotOrigin = try_or_400!(rouille::input::json_input(request));
                    let mut pose_request = try_or_404!(self.target_robot_origin.lock());
                    pose_request.origin = json;
                    pose_request.requested = true;
                    Response::json(&ResultResponse { is_ok: true, reason: "".to_string() })
                },
                (OPTIONS) (/set_joint_positions) => {
                    Response::empty_204()
                        .with_additional_header("Allow", "OPTIONS, POST")
                        .with_additional_header("Access-Control-Allow-Methods", "POST")
                        .with_additional_header("Access-Control-Allow-Origin", "*")
                        .with_additional_header("Access-Control-Allow-Headers", "authorization,content-type")
                        .with_additional_header("Access-Control-Max-Age", "86400")
                },
                (OPTIONS) (/set_robot_origin) => {
                    Response::empty_204()
                        .with_additional_header("Allow", "OPTIONS, POST")
                        .with_additional_header("Access-Control-Allow-Methods", "POST")
                        .with_additional_header("Access-Control-Allow-Origin", "*")
                        .with_additional_header("Access-Control-Allow-Headers", "authorization,content-type")
                        .with_additional_header("Access-Control-Max-Age", "86400")
                },
                (GET) (/get_joint_positions) => {
                    let ja = try_or_404!(self.current_joint_positions.lock());
                    Response::json(&*ja)
                },
                (GET) (/get_robot_origin) => {
                    let origin = try_or_404!(self.current_robot_origin.lock());
                    Response::json(&*origin)
                },
                (OPTIONS) (/get_joint_positions) => {
                    Response::empty_204()
                        .with_additional_header("Allow", "OPTIONS, GET")
                        .with_additional_header("Access-Control-Allow-Methods", "GET")
                        .with_additional_header("Access-Control-Allow-Origin", "*")
                        .with_additional_header("Access-Control-Allow-Headers", "authorization")
                        .with_additional_header("Access-Control-Max-Age", "86400")
                },
                (OPTIONS) (/get_robot_origin) => {
                    Response::empty_204()
                        .with_additional_header("Allow", "OPTIONS, GET")
                        .with_additional_header("Access-Control-Allow-Methods", "GET")
                        .with_additional_header("Access-Control-Allow-Origin", "*")
                        .with_additional_header("Access-Control-Allow-Headers", "authorization")
                        .with_additional_header("Access-Control-Max-Age", "86400")
                },

                _ => Response::empty_404(),
            )
        });
    }
}
