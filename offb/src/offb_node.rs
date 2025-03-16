use geometry_msgs::msg::{PoseStamped, TwistStamped};
use mavros_msgs::{
    msg::State,
    srv::{CommandBool, CommandBool_Request, SetMode, SetMode_Request},
};
use rclrs::*;
use ros2_tools_rs::*;
use std::{
    sync::{Arc, Mutex},
    thread::sleep,
    time::Duration,
};
use std_msgs::msg::Bool;
use vision::msg::Vision;

fn main() -> anyhow::Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("offb_node")?;

    let current_state = Arc::new(Mutex::new(State::default()));
    let current_state_clone = Arc::clone(&current_state);

    let current_pose = Arc::new(Mutex::new(EulerAnglesPose::default()));
    let current_pose_clone = Arc::clone(&current_pose);

    let vision_result = Arc::new(Mutex::new(Vision::default()));
    let vision_result_clone = Arc::clone(&vision_result);

    let _state_sub = node.create_subscription("mavros/state", move |msg: State| {
        *current_state_clone.lock().unwrap() = msg;
    })?;
    let _pose_sub = node.create_subscription(
        "mavros/local_position/pose".qos(QoSProfile::default().best_effort()),
        move |msg: PoseStamped| {
            let w = msg.pose.orientation.w;
            let x = msg.pose.orientation.x;
            let y = msg.pose.orientation.y;
            let z = msg.pose.orientation.z;
            let roll = (x * y + z * w) * 2.0;
            let pitch = (y * z - x * w).asin();
            let yaw = (x * z + y * w) * 2.0;
            let x = msg.pose.position.x;
            let y = msg.pose.position.y;
            let z = msg.pose.position.z;
            *current_pose_clone.lock().unwrap() = EulerAnglesPose {
                x,
                y,
                z,
                roll,
                pitch,
                yaw,
            };
        },
    )?;
    let _vision_sub = node.create_subscription("vision", move |msg: Vision| {
        *vision_result_clone.lock().unwrap() = msg;
    })?;
    let local_pos_pub = node.create_publisher("mavros/setpoint_position/local")?;
    let local_vel_pub = node.create_publisher("mavros/setpoint_velocity/cmd_vel")?;
    let servo_pub = node.create_publisher("servo_flag")?;
    let arming_client = node.create_client::<CommandBool>("mavros/cmd/arming")?;
    let set_mode_client = node.create_client::<SetMode>("mavros/set_mode")?;

    while context.ok() && !current_state.lock().unwrap().connected {
        executor.spin(SpinOptions::default().timeout(Duration::ZERO));
        sleep(Duration::from_secs(1));
    }

    let mut offb_set_mode = SetMode_Request::default();
    offb_set_mode.custom_mode = "OFFBOARD".to_string();
    let mut arm_cmd = CommandBool_Request::default();
    arm_cmd.value = true;

    while context.ok() {
        if current_state.lock().unwrap().mode != "OFFBOARD" {
            set_mode_client.async_send_request_with_callback(&offb_set_mode, |res| {
                if res.mode_sent {
                    println!("Offboard enabled sent");
                }
            })?;
        } else if !current_state.lock().unwrap().armed {
            arming_client.async_send_request_with_callback(&arm_cmd, |res| {
                if res.success {
                    println!("Vehicle armed sent");
                }
            })?;
        } else {
            break;
        }
        local_pos_pub.publish(PoseStamped::default())?;
        executor.spin(SpinOptions::default().timeout(Duration::ZERO));
        sleep(Duration::from_secs(1));
    }

    let mut mode = 0;

    let points = vec![create_target(0.0, 0.0, 1.5, 0.0)];

    let mut points_index = 0;
    let mut temp_point = EulerAnglesPose::default();
    let mut has_dropped = false;

    println!("Mode 0");
    while context.ok() {
        match mode {
            0 => {
                local_pos_pub.publish(&gen_pose(points[points_index]))?;
                if current_pose.lock().unwrap().dist(points[points_index]) < 0.1 {
                    points_index += 1;
                    if points_index == 1 {
                        mode = 1;
                        println!("Mode 1");
                    }
                }
            }
            1 => {
                let mut vel_msg = TwistStamped::default();
                vel_msg.twist.linear.x = 0.1;
                vel_msg.twist.linear.z = 1.5 - current_pose.lock().unwrap().z;
                if vision_result.lock().unwrap().is_line_detected
                    && vision_result.lock().unwrap().lateral_error.abs() < 100
                {
                    vel_msg.twist.linear.y =
                        (-vision_result.lock().unwrap().lateral_error / 50) as f64;
                    vel_msg.twist.angular.z =
                        (vision_result.lock().unwrap().angle_error / 10.0) as f64;
                }
                local_vel_pub.publish(&vel_msg)?;
                if !has_dropped {
                    if vision_result.lock().unwrap().is_square_detected
                        && vision_result.lock().unwrap().center_x1_error.abs() < 20
                    {
                        temp_point = current_pose.lock().unwrap().clone();
                        mode = 2;
                        println!("Mode 2");
                    }
                } else {
                    if vision_result.lock().unwrap().is_circle_detected
                        && vision_result.lock().unwrap().center_x2_error.abs() < 20
                    {
                        mode = 3;
                        println!("Mode 3");
                    }
                }
            }
            2 => {
                let mut vel_msg = TwistStamped::default();
                vel_msg.twist.linear.z = 1.5 - current_pose.lock().unwrap().z;
                vel_msg.twist.linear.x =
                    -vision_result.lock().unwrap().center_x1_error as f64 / 800.0;
                vel_msg.twist.linear.y =
                    -vision_result.lock().unwrap().center_y1_error as f64 / 800.0;
                local_vel_pub.publish(&vel_msg)?;
                let x2 = vision_result.lock().unwrap().center_x1_error.pow(2);
                let y2 = vision_result.lock().unwrap().center_y1_error.pow(2);
                if vision_result.lock().unwrap().is_square_detected && x2 + y2 < 300 {
                    servo_pub.publish(Bool { data: true })?;
                    has_dropped = true;
                    println!("脱钩");
                    vel_msg.twist.linear.x = 0.0;
                    vel_msg.twist.linear.y = 0.0;
                    local_vel_pub.publish(&vel_msg)?;
                    sleep(Duration::from_secs(1));
                    local_vel_pub.publish(&vel_msg)?;
                    sleep(Duration::from_secs(1));
                    mode = 4;
                    println!("Mode 4");
                }
            }
            3 => {
                let mut vel_msg = TwistStamped::default();
                vel_msg.twist.linear.z = 1.5 - current_pose.lock().unwrap().z;
                vel_msg.twist.linear.x =
                    -vision_result.lock().unwrap().center_x2_error as f64 / 1000.0;
                vel_msg.twist.linear.y =
                    -vision_result.lock().unwrap().center_y2_error as f64 / 1000.0;
                local_vel_pub.publish(&vel_msg)?;
                let x2 = vision_result.lock().unwrap().center_x2_error.pow(2);
                let y2 = vision_result.lock().unwrap().center_y2_error.pow(2);
                if vision_result.lock().unwrap().is_circle_detected && x2 + y2 < 300 {
                    temp_point = current_pose.lock().unwrap().clone();
                    temp_point.z = 0.2;
                    mode = 4;
                    println!("Mode 4");
                }
            }
            4 => {
                local_pos_pub.publish(&gen_pose(temp_point))?;
                if current_pose.lock().unwrap().dist(temp_point) < 0.05 {
                    points_index += 1;
                    if points_index == 2 {
                        mode = 1;
                        println!("Mode 1");
                    } else if points_index == 3 {
                        mode = 5;
                        println!("Finish");
                    }
                }
            }
            _ => break,
        }

        executor.spin(SpinOptions::default().timeout(Duration::ZERO));
        sleep(Duration::from_millis(50));
    }

    offb_set_mode.custom_mode = "AUTO.LAND".to_string();
    set_mode_client.async_send_request_with_callback(&offb_set_mode, |res| {
        if res.mode_sent {
            println!("Landing sent");
        }
    })?;

    Ok(())
}
