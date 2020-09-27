use rclrs;
use std_msgs;
use sensor_msgs;

use neato_driver::{DSeries, NeatoRobot, Toggle};
use serialport::SerialPortSettings;

fn main() -> rclrs::RclResult {
    let context = rclrs::Context::default();

    let node = context.create_node("neato")?;

    let scan_publisher =
        node.create_publisher::<sensor_msgs::msg::LaserScan>("scan", rclrs::QOS_PROFILE_DEFAULT)?;

    let int8_array_publisher =
        node.create_publisher::<std_msgs::msg::Int8MultiArray>("multiarray", rclrs::QOS_PROFILE_DEFAULT)?;

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

    robot
        .set_testmode(Toggle::On)
        .expect("Failed to enable testmode");
    
    robot
        .set_ldsrotation(Toggle::On)
        .expect("Failed to enable LDS rotation");


    let mut publish_count: u32 = 1;
    let mut ints = vec![];
    let mut array = std_msgs::msg::Int8MultiArray{
        data: vec![1, 2, 3],
        ..Default::default()
    };

    while context.ok() {
        array.data.push(publish_count as i8);
        int8_array_publisher.publish(&array)?;


        robot.request_scan().expect("Failed to request a scan");
        ints.push(publish_count as f32);

        match robot.get_scan_ranges() {
            Ok(scanned_ranges) => {
                println!("Got ranges: {:?}", scanned_ranges);
                let message = sensor_msgs::msg::LaserScan {
                    angle_min: publish_count as f32,
                    angle_max: 6.28,  //2pi
                    angle_increment: 0.01745329252, //2pi rad / 360 points
                    time_increment: 0.0005414771496642842, // = 1 / (5.13 RPM * 360 points)
                    scan_time: 0.19493177387914232, // = 1 / 5.13
                    range_min: 0.0,
                    range_max: 10.0, // Guess
                    ranges: scanned_ranges,
                    intensities: vec![publish_count as f32],
                    ..Default::default()
                };
                println!("Publishing...");
                scan_publisher.publish(&message)?;
            }
            Err(err) => {
                eprintln!("Could not get_scan_ranges: {:?}", err);
                // robot.exit().expect("Failed to exit robot while handling err");
            }
        }

        publish_count += 1;
        std::thread::sleep(std::time::Duration::from_millis(500));
    }
    println!("Exiting...");

    robot.exit().expect("Failed to exit robot");

    Ok(())
}

