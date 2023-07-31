use arci::{BaseVelocity, MoveBase};
use eframe::{egui, IconData};
use egui_extras::RetainedImage;
use tracing::{debug, error};

const DEFAULT_ACTIVE_VELOCITY_VALUE: f64 = 0.25;

/// Launches GUI that send base velocity from GUI to the given `move_base`.
#[cfg(not(target_family = "wasm"))]
pub fn velocity_sender<M>(move_base: M) -> Result<(), crate::Error>
where
    M: MoveBase + 'static,
{
    let native_options = eframe::NativeOptions {
        initial_window_size: Some(egui::vec2(400.0, 400.0)),
        icon_data: Some(
            IconData::try_from_png_bytes(include_bytes!("../assets/icon/openrr.png")).unwrap(),
        ),
        ..eframe::NativeOptions::default()
    };
    eframe::run_native(
        "Velocity Sender",
        native_options,
        Box::new(|_cc| Box::new(VelocitySender::new(move_base))),
    )
    .map_err(|e| crate::Error::Other(e.to_string()))?; // eframe::Error is not Send
    Ok(())
}

pub struct VelocitySender<M>
where
    M: MoveBase + 'static,
{
    move_base: M,

    velocity: BaseVelocity,
    show_velocity: bool,

    up_button: RetainedImage,
    down_button: RetainedImage,
    left_button: RetainedImage,
    right_button: RetainedImage,
    counterclockwise_button: RetainedImage,
    clockwise_button: RetainedImage,
}

impl<M> VelocitySender<M>
where
    M: MoveBase + 'static,
{
    fn new(move_base: M) -> Self {
        macro_rules! button_image {
            ($alt:expr, $img_path:expr $(,)?) => {
                RetainedImage::from_image_bytes(
                    $alt,
                    include_bytes!(concat!("../assets/material-design-icons/", $img_path)),
                )
                .unwrap()
            };
        }

        Self {
            move_base,
            velocity: BaseVelocity {
                x: 0.0,
                y: 0.0,
                theta: 0.0,
            },
            up_button: button_image!("↑", "baseline_arrow_upward_white_24dp.png"),
            down_button: button_image!("↓", "baseline_arrow_downward_white_24dp.png"),
            left_button: button_image!("←", "baseline_arrow_back_white_24dp.png"),
            right_button: button_image!("→", "baseline_arrow_forward_white_24dp.png"),
            // ↺
            counterclockwise_button: button_image!("CCW", "baseline_undo_white_24dp_rotate90.png"),
            // ↻
            clockwise_button: button_image!("CW", "baseline_redo_white_24dp_rotate270.png"),
            show_velocity: true,
        }
    }
}

impl<M> eframe::App for VelocitySender<M>
where
    M: MoveBase + 'static,
{
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| {
            egui::warn_if_debug_build(ui);

            ui.add_space(20.0);
            ui.horizontal(|ui| {
                ui.add_space(180.0);
                if ui
                    .add(
                        egui::ImageButton::new(
                            self.up_button.texture_id(ctx),
                            self.up_button.size_vec2(),
                        )
                        .selected(self.velocity.x > 0.0),
                    )
                    .clicked()
                {
                    if self.velocity.x > 0.0 {
                        self.velocity.x = 0.0;
                    } else {
                        self.velocity.x = DEFAULT_ACTIVE_VELOCITY_VALUE;
                    }
                }
            });
            ui.horizontal(|ui| {
                ui.add_space(100.0);
                if ui
                    .add(
                        egui::ImageButton::new(
                            self.clockwise_button.texture_id(ctx),
                            self.clockwise_button.size_vec2(),
                        )
                        .selected(self.velocity.theta < 0.0),
                    )
                    .clicked()
                {
                    if self.velocity.theta < 0.0 {
                        self.velocity.theta = 0.0;
                    } else {
                        self.velocity.theta = -DEFAULT_ACTIVE_VELOCITY_VALUE;
                    }
                }
                if ui
                    .add(
                        egui::ImageButton::new(
                            self.left_button.texture_id(ctx),
                            self.left_button.size_vec2(),
                        )
                        .selected(self.velocity.y > 0.0),
                    )
                    .clicked()
                {
                    if self.velocity.y > 0.0 {
                        self.velocity.y = 0.0;
                    } else {
                        self.velocity.y = DEFAULT_ACTIVE_VELOCITY_VALUE;
                    }
                }
                if ui.button("Stop").clicked() {
                    self.velocity.x = 0.0;
                    self.velocity.y = 0.0;
                    self.velocity.theta = 0.0;
                }
                if ui
                    .add(
                        egui::ImageButton::new(
                            self.right_button.texture_id(ctx),
                            self.right_button.size_vec2(),
                        )
                        .selected(self.velocity.y < 0.0),
                    )
                    .clicked()
                {
                    if self.velocity.y < 0.0 {
                        self.velocity.y = 0.0;
                    } else {
                        self.velocity.y = -DEFAULT_ACTIVE_VELOCITY_VALUE;
                    }
                }
                if ui
                    .add(
                        egui::ImageButton::new(
                            self.counterclockwise_button.texture_id(ctx),
                            self.counterclockwise_button.size_vec2(),
                        )
                        .selected(self.velocity.theta > 0.0),
                    )
                    .clicked()
                {
                    if self.velocity.theta > 0.0 {
                        self.velocity.theta = 0.0;
                    } else {
                        self.velocity.theta = DEFAULT_ACTIVE_VELOCITY_VALUE;
                    }
                }
            });
            ui.horizontal(|ui| {
                ui.add_space(180.0);
                if ui
                    .add(
                        egui::ImageButton::new(
                            self.down_button.texture_id(ctx),
                            self.down_button.size_vec2(),
                        )
                        .selected(self.velocity.x < 0.0),
                    )
                    .clicked()
                {
                    if self.velocity.x < 0.0 {
                        self.velocity.x = 0.0;
                    } else {
                        self.velocity.x = -DEFAULT_ACTIVE_VELOCITY_VALUE;
                    }
                }
            });

            ui.add_space(100.0);
            ui.horizontal(|ui| {
                ui.add_space(140.0);
                ui.vertical(|ui| {
                    if self.show_velocity {
                        ui.add(egui::Slider::new(&mut self.velocity.x, -1.0..=1.0).text("x"));
                        ui.add(egui::Slider::new(&mut self.velocity.y, -1.0..=1.0).text("y"));
                        ui.add(egui::Slider::new(&mut self.velocity.theta, -1.0..=1.0).text("θ"));
                    }
                });
            });

            ui.add_space(20.0);
            ui.horizontal(|ui| {
                ui.add_space(240.0);
                ui.checkbox(&mut self.show_velocity, "Show details");
            });

            // TODO: Regularly wake up Ui threads.
            debug!(?self.velocity, "send_velocity");
            if let Err(e) = self.move_base.send_velocity(&self.velocity) {
                error!("{e}");
                ui.colored_label(ui.visuals().error_fg_color, format!("Error: {e:#}"));
            }
        });
    }
}

#[cfg(test)]
mod test {
    use arci::DummyMoveBase;
    use assert_approx_eq::assert_approx_eq;

    use super::*;

    #[test]
    fn test_velocity_sender() {
        let move_base = DummyMoveBase::new();

        let velocity_sender = VelocitySender::new(move_base);

        assert_approx_eq!(velocity_sender.velocity.x, 0.0);
        assert_approx_eq!(velocity_sender.velocity.y, 0.0);
        assert_approx_eq!(velocity_sender.velocity.theta, 0.0);
    }
}
