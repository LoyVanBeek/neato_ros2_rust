use rclrs;
use std_msgs;
use sensor_msgs;
use geometry_msgs;

use neato_driver::{DSeries, NeatoRobot, Toggle};
use serialport::SerialPortSettings;

use std::cmp;

fn main() -> rclrs::RclResult {
    let context = rclrs::Context::default();

    let mut node = context.create_node("neato")?;

    let scan_publisher =
        node.create_publisher::<sensor_msgs::msg::LaserScan>("scan", rclrs::QOS_PROFILE_DEFAULT)?;

    let s = SerialPortSettings {
        baud_rate: 115200,
        timeout: std::time::Duration::from_secs(1),

        ..Default::default()
    };

    println!("Opening serial port");
    let comms = serialport::open_with_settings("/dev/ttyACM0", &s).expect("Failed to open port");
    println!("Opened serial port");

    println!("Creating robot");
    let mut robot = DSeries::new(comms);
    println!("Create robot");

    let base_width = 0.245; // 24-25 cm roughly as wheelbase. Where to easure this anyways? center of the wheels?
    let max_speed = 300.0; // mm/s

    let _cmd_vel_sub = node.create_subscription::<geometry_msgs::msg::Twist, _>(
        "cmd_vel",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: &geometry_msgs::msg::Twist| {
            println!("I heard: 'lin.x={:?}, lin.y={:?}, ang.z={:?}'", msg.linear.x, msg.linear.y, msg.angular.z);

            let mut x = (msg.linear.x * 1000.0);
            let mut th = (msg.angular.z * (base_width/2.0));

            let mut v: [f64; 2] = [(x-th).abs(), (x+th).abs()];
            v.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let k = v[1];
            println!("x={}, th={}, k={}", x, th, k);

            if(k > max_speed) {
                x = x*max_speed/k;
                th = th*max_speed/k;
                println!("k > max_speed: x={}, th={}", x, th);
            };

            let mut v: [f64; 2] = [(x-th).abs(), (x+th).abs()];
            v.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let speed = v[1];

            println!("x-th={}, x+th={}, speed={}", x-th, x+th, speed);

            robot.set_motors((x-th) as i32, 
                             (x+th) as i32,
                             speed as i32
                            );
        },
    )?;

    // robot
    //     .set_testmode(Toggle::On)
    //     .expect("Failed to enable testmode");
    
    // robot
    //     .set_ldsrotation(Toggle::On)
    //     .expect("Failed to enable LDS rotation");

    rclrs::spin(&node);
    
    // while context.ok() {
    //     // robot.request_scan().expect("Failed to request a scan");
    //     // match robot.get_scan_ranges() {
    //     //     Ok(scanned_ranges) => {
    //     //         // println!("Got ranges: {:?}", scanned_ranges);
    //     //         let message = sensor_msgs::msg::LaserScan {
    //     //             angle_min: 0.0,
    //     //             angle_max: 6.28,  //2pi
    //     //             angle_increment: 0.01745329252, //2pi rad / 360 points
    //     //             time_increment: 0.0005414771496642842, // = 1 / (5.13 RPM * 360 points)
    //     //             scan_time: 0.19493177387914232, // = 1 / 5.13
    //     //             range_min: 0.0,
    //     //             range_max: 10.0, // Guess
    //     //             ranges: scanned_ranges,
    //     //             ..Default::default()
    //     //         };
    //     //         scan_publisher.publish(&message)?;
    //     //     }
    //     //     Err(err) => {
    //     //         eprintln!("Could not get_scan_ranges: {:?}", err);
    //     //     }
    //     // }
    //     std::thread::sleep(std::time::Duration::from_millis(500));
    // }
    println!("Exiting...");

    // robot.exit().expect("Failed to exit robot");

    Ok(())
}

