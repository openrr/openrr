use crate::error::Error;
use crate::traits::{JointTrajectoryClient, TrajectoryPoint};
use crate::waits::WaitFuture;
use futures::stream::FuturesOrdered;

pub struct JointTrajectoryClientsContainer<T: JointTrajectoryClient> {
    joint_names: Vec<String>,
    clients: Vec<T>,
}

impl<T> JointTrajectoryClientsContainer<T>
where
    T: JointTrajectoryClient,
{
    pub fn new(clients: Vec<T>) -> Self {
        let mut joint_names = vec![];
        for c in &clients {
            joint_names.append(&mut c.joint_names().to_vec());
        }
        Self {
            joint_names,
            clients,
        }
    }
}

impl<T> JointTrajectoryClient for JointTrajectoryClientsContainer<T>
where
    T: JointTrajectoryClient + Sync,
{
    fn joint_names(&self) -> &[String] {
        &self.joint_names
    }
    fn current_joint_positions(&self) -> Result<Vec<f64>, Error> {
        let mut ret = vec![];
        for c in &self.clients {
            let mut positions = c.current_joint_positions()?;
            ret.append(&mut positions);
        }
        Ok(ret)
    }
    fn send_joint_positions(
        &self,
        positions: Vec<f64>,
        duration: std::time::Duration,
    ) -> Result<WaitFuture, Error> {
        let mut offset = 0;
        let mut waits = FuturesOrdered::new();
        for c in &self.clients {
            let mut current_positions = c.current_joint_positions()?;
            for i in 0..current_positions.len() {
                if positions.len() > (offset + i) {
                    current_positions[i] = positions[offset + i];
                }
            }
            offset += current_positions.len();
            waits.push(c.send_joint_positions(current_positions, duration)?);
        }
        Ok(WaitFuture::from_stream(waits))
    }
    fn send_joint_trajectory(
        &self,
        full_trajectory: Vec<TrajectoryPoint>,
    ) -> Result<WaitFuture, Error> {
        let mut offset = 0;
        let full_dof = self.joint_names().len();
        let mut waits = FuturesOrdered::new();
        for client in &self.clients {
            let mut current_positions = client.current_joint_positions()?;
            let partial_dof = current_positions.len();
            let mut partial_trajectory: Vec<TrajectoryPoint> = vec![];
            for full_point in &full_trajectory {
                for (i, current) in current_positions.iter_mut().enumerate().take(partial_dof) {
                    if full_dof > (offset + i) {
                        *current = full_point.positions[offset + i];
                    }
                }
                let partial_velocities = if let Some(full_velocities) = &full_point.velocities {
                    let mut partial_velocities = vec![0.0; partial_dof];
                    for (i, partial_velocity) in
                        partial_velocities.iter_mut().enumerate().take(partial_dof)
                    {
                        if full_dof > (offset + i) {
                            *partial_velocity = full_velocities[offset + i];
                        }
                    }
                    Some(partial_velocities)
                } else {
                    None
                };
                partial_trajectory.push(TrajectoryPoint {
                    positions: current_positions.clone(),
                    velocities: partial_velocities,
                    time_from_start: full_point.time_from_start,
                });
            }
            waits.push(client.send_joint_trajectory(partial_trajectory)?);
            offset += partial_dof;
        }
        Ok(WaitFuture::from_stream(waits))
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn test_container_new() {
        use super::*;
        #[derive(Debug, Clone)]
        struct Dummy {
            name: Vec<String>,
        }
        impl JointTrajectoryClient for Dummy {
            fn joint_names(&self) -> &[String] {
                &self.name
            }
            fn current_joint_positions(&self) -> Result<Vec<f64>, Error> {
                unimplemented!();
            }
            fn send_joint_positions(
                &self,
                _positions: Vec<f64>,
                _duration: std::time::Duration,
            ) -> Result<WaitFuture, Error> {
                unimplemented!();
            }
            fn send_joint_trajectory(
                &self,
                _trajectory: Vec<TrajectoryPoint>,
            ) -> Result<WaitFuture, Error> {
                unimplemented!()
            }
        }

        let clients = vec![
            Dummy {
                name: vec![String::from("part1"), String::from("high")],
            },
            Dummy {
                name: vec![String::from("part2"), String::from("low")],
            },
        ];

        let container = JointTrajectoryClientsContainer::new(clients.clone());

        assert_eq!(
            format!("{:?}", container.joint_names),
            "[\"part1\", \"high\", \"part2\", \"low\"]"
        );
        assert_eq!(
            format!("{:?}", clients[0]),
            format!("{:?}", container.clients[0])
        );
        assert_eq!(
            format!("{:?}", clients[1]),
            format!("{:?}", container.clients[1])
        );
    }

    #[test]
    fn test_container_jointname() {
        use super::*;
        struct Dummy {
            name: Vec<String>,
        }
        impl JointTrajectoryClient for Dummy {
            fn joint_names(&self) -> &[String] {
                &self.name
            }
            fn current_joint_positions(&self) -> Result<Vec<f64>, Error> {
                unimplemented!();
            }
            fn send_joint_positions(
                &self,
                _positions: Vec<f64>,
                _duration: std::time::Duration,
            ) -> Result<WaitFuture, Error> {
                unimplemented!();
            }
            fn send_joint_trajectory(
                &self,
                _trajectory: Vec<TrajectoryPoint>,
            ) -> Result<WaitFuture, Error> {
                unimplemented!()
            }
        }

        let clients = vec![
            Dummy {
                name: vec![String::from("part1"), String::from("high")],
            },
            Dummy {
                name: vec![String::from("part2"), String::from("low")],
            },
            Dummy {
                name: vec![String::from("part3"), String::from("high")],
            },
            Dummy {
                name: vec![String::from("part4"), String::from("middle")],
            },
        ];

        let container = JointTrajectoryClientsContainer::new(clients);

        assert_eq!(
            format!("{:?}", container.joint_names()),
            "[\"part1\", \"high\", \"part2\", \"low\", \"part3\", \"high\", \"part4\", \"middle\"]"
        );
    }

    #[test]
    fn test_container_current_pos_ok() {
        use super::*;
        use assert_approx_eq::assert_approx_eq;
        #[derive(Debug, Clone)]
        struct Dummy {
            name: Vec<String>,
            pos: Vec<f64>,
        }
        impl JointTrajectoryClient for Dummy {
            fn joint_names(&self) -> &[String] {
                &self.name
            }
            fn current_joint_positions(&self) -> Result<Vec<f64>, Error> {
                Ok(self.pos.clone())
            }
            fn send_joint_positions(
                &self,
                _positions: Vec<f64>,
                _duration: std::time::Duration,
            ) -> Result<WaitFuture, Error> {
                unimplemented!();
            }
            fn send_joint_trajectory(
                &self,
                _trajectory: Vec<TrajectoryPoint>,
            ) -> Result<WaitFuture, Error> {
                unimplemented!()
            }
        }
        let clients = vec![
            Dummy {
                name: vec![String::from("part1"), String::from("high")],
                pos: vec![1.0_f64, 3.0_f64],
            },
            Dummy {
                name: vec![String::from("part2"), String::from("low")],
                pos: vec![2.2_f64, 4.0_f64],
            },
        ];

        let container = JointTrajectoryClientsContainer::new(clients);
        let current = container.current_joint_positions();
        let expect = vec![1.0_f64, 3.0_f64, 2.2_f64, 4.0_f64];

        let positions = current.unwrap();

        positions
            .iter()
            .zip(expect)
            .for_each(|(pos, correct)| assert_approx_eq!(*pos, correct));
    }

    #[test]
    #[should_panic]
    fn test_container_current_pos_err() {
        use super::*;
        use assert_approx_eq::assert_approx_eq;

        #[derive(Debug, Clone)]
        struct Dummy {
            name: Vec<String>,
            pos: Vec<f64>,
        }
        impl JointTrajectoryClient for Dummy {
            fn joint_names(&self) -> &[String] {
                &self.name
            }
            fn current_joint_positions(&self) -> Result<Vec<f64>, Error> {
                Err(Error::Uninitialized {
                    message: String::from("current pos is not exist."),
                })
            }
            fn send_joint_positions(
                &self,
                _positions: Vec<f64>,
                _duration: std::time::Duration,
            ) -> Result<WaitFuture, Error> {
                unimplemented!();
            }
            fn send_joint_trajectory(
                &self,
                _trajectory: Vec<TrajectoryPoint>,
            ) -> Result<WaitFuture, Error> {
                unimplemented!()
            }
        }
        let clients = vec![
            Dummy {
                name: vec![String::from("part1"), String::from("high")],
                pos: vec![1.0_f64, 3.0_f64],
            },
            Dummy {
                name: vec![String::from("part2"), String::from("low")],
                pos: vec![2.2_f64, 4.0_f64],
            },
        ];

        let container = JointTrajectoryClientsContainer::new(clients);
        let current = container.current_joint_positions();
        let expect = vec![1.0_f64, 3.0_f64, 2.2_f64, 4.0_f64];

        let positions = current.unwrap();

        positions
            .iter()
            .zip(expect)
            .for_each(|(pos, correct)| assert_approx_eq!(*pos, correct));
    }

    #[test]
    fn test_container_send_pos_ok() {
        use super::*;

        #[derive(Debug, Clone)]
        struct Dummy {
            name: Vec<String>,
            pos: Vec<f64>,
        }
        impl JointTrajectoryClient for Dummy {
            fn joint_names(&self) -> &[String] {
                &self.name
            }
            fn current_joint_positions(&self) -> Result<Vec<f64>, Error> {
                Ok(self.pos.clone())
            }
            fn send_joint_positions(
                &self,
                _positions: Vec<f64>,
                _duration: std::time::Duration,
            ) -> Result<WaitFuture, Error> {
                Ok(WaitFuture::ready())
            }
            fn send_joint_trajectory(
                &self,
                _trajectory: Vec<TrajectoryPoint>,
            ) -> Result<WaitFuture, Error> {
                unimplemented!()
            }
        }
        let clients = vec![
            Dummy {
                name: vec![String::from("part1"), String::from("high")],
                pos: vec![1.0_f64, 3.0_f64],
            },
            Dummy {
                name: vec![String::from("part2"), String::from("low")],
                pos: vec![2.2_f64, 4.0_f64],
            },
        ];

        let container = JointTrajectoryClientsContainer::new(clients);
        let pos = vec![3.4_f64, 5.8_f64, 0.1_f64, 2.5_f64];
        let duration = std::time::Duration::from_secs(5);
        let _ = container.send_joint_positions(pos, duration).unwrap();
    }

    #[test]
    #[should_panic]
    fn test_container_send_pos_err() {
        use super::*;

        #[derive(Debug, Clone)]
        struct Dummy {
            name: Vec<String>,
            pos: Vec<f64>,
        }
        impl JointTrajectoryClient for Dummy {
            fn joint_names(&self) -> &[String] {
                &self.name
            }
            fn current_joint_positions(&self) -> Result<Vec<f64>, Error> {
                Ok(self.pos.clone())
            }
            fn send_joint_positions(
                &self,
                _positions: Vec<f64>,
                _duration: std::time::Duration,
            ) -> Result<WaitFuture, Error> {
                Err(Error::Uninitialized {
                    message: String::from("error pattern."),
                })
            }
            fn send_joint_trajectory(
                &self,
                _trajectory: Vec<TrajectoryPoint>,
            ) -> Result<WaitFuture, Error> {
                unimplemented!()
            }
        }
        let clients = vec![
            Dummy {
                name: vec![String::from("part1"), String::from("high")],
                pos: vec![1.0_f64, 3.0_f64],
            },
            Dummy {
                name: vec![String::from("part2"), String::from("low")],
                pos: vec![2.2_f64, 4.0_f64],
            },
        ];

        let container = JointTrajectoryClientsContainer::new(clients);
        let pos = vec![3.4_f64, 5.8_f64, 0.1_f64, 2.5_f64];
        let duration = std::time::Duration::from_secs(5);
        let _ = container.send_joint_positions(pos, duration).unwrap();
    }
}
