use std::time::Duration;

use urdf_viz::{JointNamesAndPositions, WebServer};
use url::Url;

pub(crate) fn port_and_url() -> (u16, Url) {
    let port = portpicker::pick_unused_port().expect("No ports free");
    let url = Url::parse(&format!("http://127.0.0.1:{port}")).unwrap();
    (port, url)
}

#[easy_ext::ext]
pub(crate) impl WebServer {
    fn start_background(self) {
        let handle = self.handle();
        std::thread::spawn(move || {
            tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
                .unwrap()
                .block_on(async move { self.bind().unwrap().await.unwrap() })
        });
        std::thread::spawn(move || loop {
            if let Some(positions) = handle.take_target_joint_positions() {
                *handle.current_joint_positions() = positions;
            }
            if let Some(origin) = handle.pop_target_object_origin() {
                if origin.id == urdf_viz::ROBOT_OBJECT_ID {
                    *handle.current_robot_origin() = urdf_viz::RobotOrigin {
                        position: origin.position,
                        quaternion: origin.quaternion,
                    };
                }
            }
        });
        std::thread::sleep(Duration::from_secs(1)); // Wait for web server to start.
    }

    fn set_current_joint_positions(&self, joint_positions: JointNamesAndPositions) {
        *self.handle().current_joint_positions() = joint_positions;
    }
}
