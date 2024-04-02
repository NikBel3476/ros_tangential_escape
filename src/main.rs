use once_cell::sync::Lazy;
use rosrust::ros_info;
use rosrust_msg::geometry_msgs;
use rosrust_msg::sensor_msgs::LaserScan;
use rosrust_msg::{geometry_msgs::PoseWithCovariance, nav_msgs::Odometry};
use std::env;
use std::f64::consts::PI;
use std::sync::Mutex;

const LIDAR_POINT_COUNT: usize = 1147;
const U_MAX: f64 = 0.1;
const K_W: f64 = 0.5;
const DELTA: f64 = 0.05;
const D_MIN: f64 = 0.25;

// static mut current_pose: Box<PoseWithCovariance> = Box::new(PoseWithCovariance::default());
static CURRENT_POSE: Lazy<Mutex<Option<PoseWithCovariance>>> = Lazy::new(|| Mutex::new(None));
static LASER_SCAN: Lazy<Mutex<Option<LaserScan>>> = Lazy::new(|| Mutex::new(None));

fn main() {
    rosrust::init("to_point");

    // ros_info!("{:#?}", env::args());
    let x_arg = env::args().nth(1);
    let y_arg = env::args().nth(2);
    let (x, y): (f64, f64) = match (x_arg, y_arg) {
        (Some(x_str), Some(y_str)) => (
            x_str.parse().expect("Incorrect x coordinate"),
            y_str.parse().expect("Incorrect y coordinate"),
        ),
        _ => panic!("Expected x and y coordinates. Using: `cargo run <x> <y>`"),
    };

    ros_info!("x: {x}, y: {y}");

    let cmd_vel_pub = rosrust::publish("cmd_vel", 10).unwrap();
    cmd_vel_pub.wait_for_subscribers(None).unwrap();

    let pose_sub = rosrust::subscribe("odom", 10, |odometry: Odometry| {
        let mut pose = CURRENT_POSE.lock().unwrap();
        *pose = Some(odometry.pose);
    })
    .unwrap();

    let lidar_sub = rosrust::subscribe("scan", 10, |scan: LaserScan| {
        let mut laser_scan = LASER_SCAN.lock().unwrap();
        *laser_scan = Some(scan);
    })
    .unwrap();

    loop {
        if CURRENT_POSE.lock().unwrap().is_some() && LASER_SCAN.lock().unwrap().is_some() {
            break;
        }
    }

    let mut p_current = p(x, y).unwrap();
    let mut a_current = a(x, y).unwrap();
    while p(x, y).unwrap() > DELTA {
        // let (goal_x, goal_y) = (x, y);
        let (goal_x, goal_y) = match b() {
            Some(beta) => {
                let phi = beta.signum() * PI / 2.0 - (beta - a(x, y).unwrap());
                ros_info!("beta: {beta}, phi: {phi}, a: {a_current}");
                rotate_point(x, y, phi)
            }
            None => (x, y),
        };

        ros_info!("x: {goal_x}, y: {goal_y}");

        let u = U_MAX * p_current.tanh() * a_current.cos();
        let w = K_W * a_current
            + U_MAX * p_current.tanh() * a_current.sin() * a_current.cos() / p_current;

        ros_info!("p: {p_current}, a: {a_current}, u: {u}, w: {w}");

        cmd_vel_pub
            .send(geometry_msgs::Twist {
                linear: geometry_msgs::Vector3 {
                    x: u,
                    y: 0.0,
                    z: 0.0,
                },
                angular: geometry_msgs::Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: w,
                },
            })
            .unwrap();

        rosrust::sleep(rosrust::Duration::from_nanos(100_000_000));
        p_current = p(goal_x, goal_y).unwrap();
        a_current = a(goal_x, goal_y).unwrap();
    }

    cmd_vel_pub
        .send(geometry_msgs::Twist {
            linear: Default::default(),
            angular: Default::default(),
        })
        .unwrap();

    ros_info!("Destination point is reached");
}

fn get_robot_angle() -> Option<f64> {
    let pose = CURRENT_POSE.lock().unwrap();
    match (*pose).clone() {
        Some(pose) => Some(get_angle_from_pose(&pose)),
        _ => None,
    }
}

fn get_angle_from_pose(pose: &PoseWithCovariance) -> f64 {
    // (pose.pose.orientation.z.asin() * 2.0 * 180.0 / PI).to_radians()
    let angle =
        (pose.pose.orientation.z.atan2(pose.pose.orientation.w) * 2.0 * 180.0 / PI).to_radians();
    if angle < -PI {
        2.0 * PI + angle
    } else {
        angle
    }
}

/// Distance between current robot position and point
fn p(x: f64, y: f64) -> Result<f64, &'static str> {
    let pose = (*CURRENT_POSE.lock().unwrap()).clone();
    match pose {
        Some(pose) => {
            Ok(((x - pose.pose.position.x).powi(2) + (y - pose.pose.position.y).powi(2)).sqrt())
        }
        _ => Err("Cannot get robot position"),
    }
}

/// The orientation error, regarding the goal position
fn a(goal_x: f64, goal_y: f64) -> Result<f64, &'static str> {
    let pose = (*CURRENT_POSE.lock().unwrap()).clone();
    match pose {
        Some(pose) => {
            let angle_to_goal =
                (goal_y - pose.pose.position.y).atan2(goal_x - pose.pose.position.x);
            let robot_current_angle = get_robot_angle().unwrap();

            ros_info!("goal: {angle_to_goal}, robot_angle: {robot_current_angle}");
            Ok(angle_to_goal - robot_current_angle)
        }
        _ => Err("Cannot get robot position"),
    }
}

fn b() -> Option<f64> {
    let laser_scan = LASER_SCAN.lock().unwrap().clone().unwrap();
    let points_at_left_side = &laser_scan.ranges[..(LIDAR_POINT_COUNT / 4)];
    let points_at_right_side =
        &laser_scan.ranges[(laser_scan.ranges.len() - LIDAR_POINT_COUNT / 4 - 1)..];

    let ranges = [points_at_right_side, points_at_left_side].concat();
    let mut min_range = ranges[0] as f64;
    let mut index_of_min_range = None;
    for (i, range) in ranges.iter().enumerate() {
        if *range as f64 <= D_MIN && (*range as f64) < min_range {
            min_range = *range as f64;
            index_of_min_range = Some(i);
        }
    }
    if let Some(i) = index_of_min_range {
        let angle = i as f64 * 2.0 * PI / LIDAR_POINT_COUNT as f64;
        return match angle > PI / 2.0 {
            true => Some(angle - PI / 2.0),
            false => Some(-PI / 2.0 + angle),
        };
    }
    None
}

// fn b() -> Option<f64> {
//     let laser_scan = LASER_SCAN.lock().unwrap().clone().unwrap();
//     let mut min_range = laser_scan.ranges[0] as f64;
//     let mut index_of_min_range = None;
//     for (i, range) in laser_scan.ranges.iter().enumerate() {
//         if *range as f64 <= D_MIN && (*range as f64) < min_range {
//             min_range = *range as f64;
//             index_of_min_range = Some(i);
//         }
//     }
//     if let Some(i) = index_of_min_range {
//         let angle = i as f64 * PI / 720.0;
//         return match angle > PI / 2.0 {
//             true => Some(angle - PI / 2.0),
//             false => Some(-PI / 2.0 + angle),
//         };
//     }
//     None
// }

fn rotate_point(x: f64, y: f64, angle: f64) -> (f64, f64) {
    (
        angle.cos() * x + angle.sin() * y,
        -angle.sin() * x + angle.cos() * y,
    )
}
