use r2r;
// use chrono::prelude::*;
use r2r::ur_tools_msgs::srv::GenerateURScript;
use r2r::ur_tools_msgs::msg::JointPositions;
use r2r::ur_tools_msgs::msg::Payload;

// const TIME_FORMAT_STR: &'static str = "%Y-%m-%d %H:%M:%S";

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
        "ur_script_generator_test",
        "Waiting for UR Script Generator Service..."
    );
    waiting_for_server.await?;
    r2r::log_info!(
        "ur_script_generator_test",
        "UR Script Generator Service available."
    );
    r2r::log_info!(
        "ur_script_generator_test",
        "UR Script Generator Test Client Node started."
    );

    let commands = vec!("safe_move_j");

    let mut messages = vec!();
    let bools = vec!(true, false);

    // Test all combinations
    let mut message = GenerateURScript::Request::default();
    message.acceleration = 0.3;
    message.velocity = 0.5;
    message.goal_feature_name = "pos1".to_string();
    message.tcp_name = "svt_tcp".to_string();
    for b1 in &bools {
        let mut message = message.clone();
        message.use_execution_time = *b1;
        message.execution_time = 1.75;
        for b2 in &bools {
            message.use_blend_radius = *b2;
            let mut message = message.clone();
            message.blend_radius = 0.375;
            for b3 in &bools {
                let mut message = message.clone();
                message.use_joint_positions = *b3;
                message.joint_positions = JointPositions {
                    j0: 1.5707,
                    j1: 1.5707,
                    j2: 1.5707,
                    j3: 1.5707,
                    j4: 1.5707,
                    j5: 1.5707,
                };
                for b4 in &bools {
                    let mut message = message.clone();
                    message.use_preferred_joint_config = *b4;
                    message.preferred_joint_config = JointPositions {
                        j0: 0.56789,
                        j1: 0.56789,
                        j2: 0.56789,
                        j3: 0.56789,
                        j4: 0.56789,
                        j5: 0.56789,
                    };
                    for b5 in &bools {
                        let mut message = message.clone();
                        message.use_payload = *b5;
                        message.payload = Payload::default();
                        for command in &commands {
                            let mut message = message.clone();
                            message.command = command.to_string();
                            messages.push(message);
                        }
                    }
                }
            }
        }
    }

    for message in messages {
        ursg_test(
            &client,
            message
        ).await?;
    }
    

    handle.join().unwrap();

    Ok(())
}

async fn ursg_test(
    client: &r2r::Client<GenerateURScript::Service>,
    message: GenerateURScript::Request
) -> Result<(), Box<dyn std::error::Error>> {

    let response = client
        .request(&message)
        .expect("Could not send URSG request.")
        .await
        .expect("Cancelled.");

    r2r::log_info!(
        "ur_script_generator_test",
        "Request to UR Script Generator sent."
    );

    match response.success {
        true => {
            r2r::log_info!(
                "ur_script_generator_test",
                "Got generated UR Script: \n{}",
                response.script
            );
        },
        false => {
            r2r::log_error!(
                "ur_script_generator_test",
                "Couldn't generate UR Script for command '{}'.",
                message.command
            );
        }
    }

    Ok(())
}
