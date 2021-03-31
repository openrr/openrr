// This is copy of urdf-viz/web_server.rs
// version = "0.25.0"
// https://github.com/OpenRR/urdf-viz/blob/v0.25.0/src/web_server.rs
// It is difficult to test with urdf-viz with `RUSTFLAGS=-Cpanic=abort`
// because of wayland-client.
// That flag is needed to get the test coverage.

use actix_web::*;
use serde::{Deserialize, Serialize};
use std::{
    io,
    sync::{Arc, Mutex},
};

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

#[derive(Debug, Clone)]
struct Data {
    target_joint_positions: Arc<Mutex<JointNamesAndPositionsRequest>>,
    current_joint_positions: Arc<Mutex<JointNamesAndPositions>>,
    target_robot_origin: Arc<Mutex<RobotOriginRequest>>,
    current_robot_origin: Arc<Mutex<RobotOrigin>>,
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

    pub fn clone_in_out(
        &self,
    ) -> (
        Arc<Mutex<JointNamesAndPositionsRequest>>,
        Arc<Mutex<JointNamesAndPositions>>,
        Arc<Mutex<RobotOriginRequest>>,
        Arc<Mutex<RobotOrigin>>,
    ) {
        (
            self.target_joint_positions.clone(),
            self.current_joint_positions.clone(),
            self.target_robot_origin.clone(),
            self.current_robot_origin.clone(),
        )
    }

    #[actix_web::main]
    pub async fn start(self) -> io::Result<()> {
        let data = Data {
            target_joint_positions: self.target_joint_positions,
            current_joint_positions: self.current_joint_positions,
            target_robot_origin: self.target_robot_origin,
            current_robot_origin: self.current_robot_origin,
        };

        HttpServer::new(move || {
            App::new()
                .data(data.clone())
                .service(set_joint_positions)
                .service(set_robot_origin)
                .service(options_set_joint_positions)
                .service(options_set_robot_origin)
                .service(get_joint_positions)
                .service(get_robot_origin)
                .service(options_get_joint_positions)
                .service(options_get_robot_origin)
        })
        .bind(("0.0.0.0", self.port))?
        .run()
        .await
    }
}

#[post("set_joint_positions")]
async fn set_joint_positions(
    json: web::Json<JointNamesAndPositions>,
    data: web::Data<Data>,
) -> HttpResponse {
    let mut jp_request = data.target_joint_positions.lock().unwrap();
    jp_request.joint_positions = json.into_inner();
    let jp = jp_request.joint_positions.clone();
    if jp.names.len() != jp.positions.len() {
        HttpResponse::Ok().json(&ResultResponse {
            is_ok: false,
            reason: format!(
                "names and positions size mismatch ({} != {})",
                jp.names.len(),
                jp.positions.len()
            ),
        })
    } else {
        jp_request.requested = true;
        HttpResponse::Ok().json(&ResultResponse {
            is_ok: true,
            reason: "".to_string(),
        })
    }
}

#[post("set_robot_origin")]
async fn set_robot_origin(json: web::Json<RobotOrigin>, data: web::Data<Data>) -> HttpResponse {
    let mut pose_request = data.target_robot_origin.lock().unwrap();
    pose_request.origin = json.into_inner();
    pose_request.requested = true;
    HttpResponse::Ok().json(&ResultResponse {
        is_ok: true,
        reason: "".to_string(),
    })
}

#[options("set_joint_positions")]
async fn options_set_joint_positions() -> HttpResponse {
    HttpResponse::NoContent()
        .header("Allow", "OPTIONS, POST")
        .header("Access-Control-Allow-Methods", "POST")
        .header("Access-Control-Allow-Origin", "*")
        .header("Access-Control-Allow-Headers", "authorization,content-type")
        .header("Access-Control-Max-Age", "86400")
        .finish()
}

#[options("set_robot_origin")]
async fn options_set_robot_origin() -> HttpResponse {
    HttpResponse::NoContent()
        .header("Allow", "OPTIONS, POST")
        .header("Access-Control-Allow-Methods", "POST")
        .header("Access-Control-Allow-Origin", "*")
        .header("Access-Control-Allow-Headers", "authorization,content-type")
        .header("Access-Control-Max-Age", "86400")
        .finish()
}

#[get("get_joint_positions")]
async fn get_joint_positions(server: web::Data<Data>) -> HttpResponse {
    let json = server.current_joint_positions.lock().unwrap();
    HttpResponse::Ok().json(&*json)
}

#[get("get_robot_origin")]
async fn get_robot_origin(server: web::Data<Data>) -> HttpResponse {
    let origin = server.current_robot_origin.lock().unwrap();
    HttpResponse::Ok().json(&*origin)
}

#[options("get_joint_positions")]
async fn options_get_joint_positions() -> HttpResponse {
    HttpResponse::NoContent()
        .header("Allow", "OPTIONS, GET")
        .header("Access-Control-Allow-Methods", "GET")
        .header("Access-Control-Allow-Origin", "*")
        .header("Access-Control-Allow-Headers", "authorization")
        .header("Access-Control-Max-Age", "86400")
        .finish()
}

#[options("get_robot_origin")]
async fn options_get_robot_origin() -> HttpResponse {
    HttpResponse::NoContent()
        .header("Allow", "OPTIONS, GET")
        .header("Access-Control-Allow-Methods", "GET")
        .header("Access-Control-Allow-Origin", "*")
        .header("Access-Control-Allow-Headers", "authorization")
        .header("Access-Control-Max-Age", "86400")
        .finish()
}
