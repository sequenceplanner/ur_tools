use r2r;
use chrono::prelude::*;
use r2r::ur_script_generator_msgs::srv::GenerateURScript;
use r2r::ur_script_generator_msgs::msg::JointPositions;
use r2r::ur_script_generator_msgs::msg::Payload;

const TIME_FORMAT_STR: &'static str = "%Y-%m-%d %H:%M:%S";

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "ur_script_generator_test", "")?;

    let client = node.create_client::<GenerateURScript::Service>("ur_script_generator")?;

    let waiting_for_server = node.is_available(&client)?;

    let handle = std::thread::spawn(move || loop {
        &node.spin_once(std::time::Duration::from_millis(100));
    });

    r2r::log_warn!(
        &format!("URSG TEST {}", Local::now().format(TIME_FORMAT_STR).to_string()),
        "Waiting for UR Script Generator Service..."
    );
    waiting_for_server.await?;
    r2r::log_info!(
        &format!("URSG TEST {}", Local::now().format(TIME_FORMAT_STR).to_string()),
        "UR Script Generator Service available."
    );
    r2r::log_info!(
        &format!("URSG TEST {}", Local::now().format(TIME_FORMAT_STR).to_string()),
        "UR Script Generator Test Client Node started."
    );

    ursg_test(
        &client,
    ).await?;

    handle.join().unwrap();

    Ok(())
}

async fn ursg_test(
    client: &r2r::Client<GenerateURScript::Service>,
) -> Result<(), Box<dyn std::error::Error>> {
    
    let mut request = GenerateURScript::Request::default();
    request.use_joint_positions = true;
    request.command = "move_j".to_string();
    request.joint_positions = JointPositions {
        j0: 1.5707,
        j1: 1.5707,
        j2: 1.5707,
        j3: 1.5707,
        j4: 1.5707,
        j5: 1.5707,
    };

    let response = client
        .request(&request)
        .expect("Could not send URSG request.")
        .await
        .expect("Cancelled.");

    r2r::log_info!(
        &format!("URSG TEST {}", Local::now().format(TIME_FORMAT_STR).to_string()),
        "Request to UR Script Generator sent."
    );

    match response.success {
        true => {
            r2r::log_info!(
                &format!("URSG TEST {}", Local::now().format(TIME_FORMAT_STR).to_string()),
                "Got generated UR Script: \n{}",
                response.script
            );
        },
        false => {
            r2r::log_error!(
                &format!("URSG TEST {}", Local::now().format(TIME_FORMAT_STR).to_string()),
                "Couldn't generate UR Script for command '{}'.",
                request.command
            );
        }
    }

    Ok(())
}
