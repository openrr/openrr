// This is based on urdf-viz/web_server.rs
// version = "0.25.0"
// https://github.com/OpenRR/urdf-viz/blob/v0.25.0/src/web_server.rs
// It is difficult to test with urdf-viz with `RUSTFLAGS=-Cpanic=abort`
// because of wayland-client.
// That flag is needed to get the test coverage.

use std::{
    io,
    sync::{Arc, Mutex},
};

use actix_web::*;
use serde::{Deserialize, Serialize};

#[derive(Deserialize, Serialize, Debug, Clone, Default)]
pub struct JointNamesAndPositions {
    pub names: Vec<String>,
    pub positions: Vec<f32>,
}

#[derive(Deserialize, Serialize, Debug, Clone, Default)]
pub struct RobotOrigin {
    pub position: [f32; 3],
    pub quaternion: [f32; 4],
}

#[derive(Deserialize, Serialize, Debug, Clone)]
struct ResultResponse {
    is_ok: bool,
    reason: String,
}

#[derive(Debug)]
pub struct WebServer {
    pub port: u16,
    pub current_joint_positions: Arc<Mutex<JointNamesAndPositions>>,
    pub current_robot_origin: Arc<Mutex<RobotOrigin>>,
}

#[derive(Debug, Clone)]
struct Data {
    current_joint_positions: Arc<Mutex<JointNamesAndPositions>>,
    current_robot_origin: Arc<Mutex<RobotOrigin>>,
}

impl WebServer {
    pub fn new(port: u16) -> Self {
        Self {
            port,
            current_joint_positions: Arc::new(Mutex::new(JointNamesAndPositions::default())),
            current_robot_origin: Arc::new(Mutex::new(RobotOrigin::default())),
        }
    }

    #[actix_web::main]
    async fn start(self) -> io::Result<()> {
        let data = Data {
            current_joint_positions: self.current_joint_positions,
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

    pub fn start_background(self) {
        std::thread::spawn(move || self.start().unwrap());
        std::thread::sleep(std::time::Duration::from_secs(1)); // Wait for web server to start.
    }
}

#[post("set_joint_positions")]
async fn set_joint_positions(
    json: web::Json<JointNamesAndPositions>,
    data: web::Data<Data>,
) -> HttpResponse {
    let mut joint_positions = data.current_joint_positions.lock().unwrap();
    *joint_positions = json.into_inner();
    let jp = joint_positions.clone();
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
        HttpResponse::Ok().json(&ResultResponse {
            is_ok: true,
            reason: "".to_string(),
        })
    }
}

#[post("set_robot_origin")]
async fn set_robot_origin(json: web::Json<RobotOrigin>, data: web::Data<Data>) -> HttpResponse {
    let mut robot_origin = data.current_robot_origin.lock().unwrap();
    *robot_origin = json.into_inner();
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
