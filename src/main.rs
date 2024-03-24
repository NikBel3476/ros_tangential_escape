use once_cell::sync::Lazy;
use rosrust::ros_info;
use rosrust_msg::geometry_msgs;
use rosrust_msg::{geometry_msgs::PoseWithCovariance, nav_msgs::Odometry};
use std::cmp::Ordering;
use std::env;
use std::f64::consts::PI;
use std::sync::Mutex;

const U_MAX: f64 = 0.2;
const K_W: f64 = 0.5;
const DELTA: f64 = 0.1;

// static mut current_pose: Box<PoseWithCovariance> = Box::new(PoseWithCovariance::default());
static CURRENT_POSE: Lazy<Mutex<Option<PoseWithCovariance>>> = Lazy::new(|| Mutex::new(None));

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

    loop {
        if CURRENT_POSE.lock().unwrap().is_some() {
            break;
        }
    }

    let mut p_current = p(x, y).unwrap();
    let mut a_current = a(x, y).unwrap();
    while p_current > DELTA {
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
        p_current = p(x, y).unwrap();
        a_current = a(x, y).unwrap();
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
    (pose.pose.orientation.z.atan2(pose.pose.orientation.w) * 2.0 * 180.0 / PI).to_radians()
    // pose.pose.orientation.z.atan2(pose.pose.orientation.w)
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
            // angle between vector to point and <0, 1>
            let goal_vector_length = ((goal_x - pose.pose.position.x).powi(2)
                + (goal_y - pose.pose.position.y).powi(2))
            .sqrt();
            // let angle_to_goal = ((pose.pose.position.x * goal_y) / goal_vector_length).acos();
            let angle_to_goal =
                (goal_y - pose.pose.position.y).atan2(goal_x - pose.pose.position.x);
            let robot_current_angle = get_robot_angle().unwrap();
            Ok(angle_to_goal - robot_current_angle)
        }
        _ => Err("Cannot get robot position"),
    }
}
