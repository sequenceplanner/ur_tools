use futures::stream::Stream;
use futures::StreamExt;
use r2r::ur_script_msgs::action::ExecuteScript;
use r2r::ur_tools_msgs::action::URScriptControl;
use r2r::ur_tools_msgs::srv::GenerateURScript;
use r2r::ActionServerGoal;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "ur_script_controller", "")?;

    let action = node.create_action_server::<URScriptControl::Action>("ur_script_controller")?;
    let ursg_client = node.create_client::<GenerateURScript::Service>("ur_script_generator")?;
    let urc_client = node.create_action_client::<ExecuteScript::Action>("ur_script")?;
    let waiting_for_ursg_server = node.is_available(&ursg_client)?;
    let waiting_for_urc_server = node.is_available(&urc_client)?;

    let handle = std::thread::spawn(move || loop {
        node.spin_once(std::time::Duration::from_millis(100));
    });

    r2r::log_warn!(
        "ur_script_controller",
        "Waiting for the UR Script Generator Service..."
    );
    waiting_for_ursg_server.await?;
    r2r::log_info!(
        "ur_script_controller",
        "UR Script Generator Service available."
    );

    r2r::log_warn!(
        "ur_script_controller",
        "Waiting for the UR Script Driver..."
    );
    waiting_for_urc_server.await?;
    r2r::log_info!("ur_script_controller", "UR Script Driver available.");

    r2r::log_info!("ur_script_controller", "UR Script Controller node started.");

    tokio::task::spawn(async move {
        let result = ur_script_controller_server(action, ursg_client, urc_client).await;
        match result {
            Ok(()) => r2r::log_info!(
                "ur_script_controller",
                "UR Script Controller Service call succeeded."
            ),
            Err(e) => r2r::log_error!(
                "ur_script_controller",
                "UR Script Controller Service call failed with: {}.",
                e
            ),
        };
    });

    handle.join().unwrap();

    Ok(())
}

async fn ur_script_controller_server(
    mut requests: impl Stream<Item = r2r::ActionServerGoalRequest<URScriptControl::Action>> + Unpin,
    ursg_client: r2r::Client<GenerateURScript::Service>,
    urc_client: r2r::ActionClient<ExecuteScript::Action>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        match requests.next().await {
            Some(request) => {
                let (mut g, mut _cancel) =
                    request.accept().expect("Could not accept goal request.");
                let g_clone = g.clone();
                match generate_script(g_clone, &ursg_client).await {
                    Some(script) => {
                        let g_clone = g.clone();
                        match execute_script(g_clone, &script, &urc_client).await {
                            true => {
                                g.succeed(URScriptControl::Result { success: true })
                                    .expect("could not send result");
                                continue;
                            }
                            false => {
                                let _ = g.abort(URScriptControl::Result { success: false });
                                continue;
                            }
                        }
                    }
                    None => {
                        let _ = g.abort(URScriptControl::Result { success: false });
                        continue;
                    }
                }
            }
            None => (),
        }
    }
}

async fn generate_script(
    g: ActionServerGoal<URScriptControl::Action>,
    ursg_client: &r2r::Client<GenerateURScript::Service>,
) -> Option<String> {
    r2r::log_info!(
        "ur_script_controller",
        "Sending request to UR Script Generator."
    );
    let _ = g.publish_feedback(URScriptControl::Feedback {
        current_state: "Sending request to UR Script Generator.".into(),
    });

    let ursg_req = GenerateURScript::Request {
        command: g.goal.command.to_string(),
        acceleration: g.goal.acceleration * g.goal.acceleration_scaling,
        velocity: g.goal.velocity * g.goal.velocity_scaling,
        goal_feature_name: g.goal.goal_feature_name.to_string(),
        tcp_name: g.goal.tcp_name.to_string(),
        use_execution_time: g.goal.use_execution_time,
        execution_time: g.goal.execution_time,
        use_blend_radius: g.goal.use_blend_radius,
        blend_radius: g.goal.blend_radius,
        use_joint_positions: g.goal.use_joint_positions,
        joint_positions: g.goal.joint_positions.to_owned(),
        use_preferred_joint_config: g.goal.use_preferred_joint_config,
        preferred_joint_config: g.goal.preferred_joint_config.to_owned(),
        use_payload: g.goal.use_payload,
        payload: g.goal.payload.to_owned(),
        tf_lookup_deadline: g.goal.tf_lookup_deadline,
    };

    let ursg_response = ursg_client
        .request(&ursg_req)
        .expect("Could not send UR Script Generator request.")
        .await
        .expect("Cancelled.");

    match ursg_response.success {
        true => {
            r2r::log_info!("ur_script_controller", "Got generated UR Script.");
            let _ = g.publish_feedback(URScriptControl::Feedback {
                current_state: "Got generated UR Script.".into(),
            });
            Some(ursg_response.script)
        }
        false => {
            r2r::log_error!("ur_script_controller", "Failed to get generated UR Script.");
            let _ = g.publish_feedback(URScriptControl::Feedback {
                current_state: "Failed to get generated UR Script.".into(),
            });
            None
        }
    }
}

async fn execute_script(
    g: ActionServerGoal<URScriptControl::Action>,
    script: &str,
    urc_client: &r2r::ActionClient<ExecuteScript::Action>,
) -> bool {
    let goal = ExecuteScript::Goal {
        script: script.to_string(),
    };

    r2r::log_info!(
        "ur_script_controller",
        "Sending request to UR Script Driver."
    );
    let _ = g.publish_feedback(URScriptControl::Feedback {
        current_state: "Sending request to UR Script Driver.".into(),
    });

    let (_goal, result, _feedback) = match urc_client.send_goal_request(goal) {
        Ok(x) => match x.await {
            Ok(y) => y,
            Err(_) => {
                r2r::log_info!("ur_script_controller", "Could not send goal request.");
                return false;
            }
        },
        Err(_) => {
            r2r::log_info!("ur_script_controller", "Did not get goal.");
            return false;
        }
    };

    match result.await {
        Ok((status, msg)) => match status {
            r2r::GoalStatus::Aborted => {
                r2r::log_info!(
                    "ur_script_controller",
                    "Goal succesfully aborted with: {:?}",
                    msg
                );
                let _ = g.publish_feedback(URScriptControl::Feedback {
                    current_state: "Goal succesfully aborted.".into(),
                });
                true
            }
            _ => {
                r2r::log_info!("ur_script_controller", "Executing the UR Script succeeded.");
                let _ = g.publish_feedback(URScriptControl::Feedback {
                    current_state: "Executing the UR Script succeeded.".into(),
                });
                true
            }
        },
        Err(e) => {
            r2r::log_error!(
                "ur_script_controller",
                "UR Script Driver Action failed with: {:?}",
                e,
            );
            let _ = g.publish_feedback(URScriptControl::Feedback {
                current_state: "UR Script Driver Action failed. Aborting.".into(),
            });
            false
        }
    }
}
