use chrono::prelude::*;
use futures::stream::Stream;
use futures::StreamExt;
use lazy_static::lazy_static;
use r2r::geometry_msgs::msg::TransformStamped;
use r2r::tf_tools_msgs::srv::LookupTransform;
use r2r::ur_script_generator_msgs::msg::JointPositions;
use r2r::ur_script_generator_msgs::msg::Payload;
use r2r::ur_script_generator_msgs::srv::GenerateURScript;
use r2r::ServiceRequest;
use serde::{Deserialize, Serialize};
use tera;

pub static BASEFRAME_ID: &'static str = "base";
pub static FACEPLATE_ID: &'static str = "tool0";
const TIME_FORMAT_STR: &'static str = "%Y-%m-%d %H:%M:%S";

#[derive(Serialize, Deserialize)]
pub struct Interpretation {
    pub valid: bool,
    pub command: String,
    pub acceleration: f64,
    pub velocity: f64,
    pub use_execution_time: bool,
    pub execution_time: f32,
    pub use_blend_radius: bool,
    pub blend_radius: f32,
    pub use_joint_positions: bool,
    pub joint_positions: JointPositions,
    pub use_preferred_joint_config: bool,
    pub preferred_joint_config: JointPositions,
    pub use_payload: bool,
    pub payload: Payload,
    pub target_in_base: String,
    pub tcp_in_faceplate: String,
}

lazy_static! {
    pub static ref TEMPLATES: tera::Tera = {
        // let tera = match tera::Tera::new("src/templates/*.script") {
        let tera = match tera::Tera::new("/home/endre/Desktop/templates/*.script") {
            Ok(t) => {
                r2r::log_warn!(
                    &format!("URSG {}", Local::now().format(TIME_FORMAT_STR).to_string()),
                    "Searching for Tera templates, wait...",
                );
                t
            },
            Err(e) => {
                r2r::log_error!(
                    &format!("URSG {}", Local::now().format(TIME_FORMAT_STR).to_string()),
                    "UR Script template parsing error(s): {}", e
                );
                ::std::process::exit(1); // don't exit but warn or don't collect?
            }
        };
        tera
    };
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "ur_script_generator", "")?;

    let service = node.create_service::<GenerateURScript::Service>("ur_script_generator")?;
    let tf_lookup_client = node.create_client::<LookupTransform::Service>("tf_lookup")?;
    let waiting_for_tf_lookup_server = node.is_available(&tf_lookup_client)?;

    let handle = std::thread::spawn(move || loop {
        node.spin_once(std::time::Duration::from_millis(100));
    });

    r2r::log_warn!(
        &format!("URSG {}", Local::now().format(TIME_FORMAT_STR).to_string()),
        "Waiting for TF Lookup service..."
    );
    waiting_for_tf_lookup_server.await?;
    r2r::log_info!(
        &format!("URSG {}", Local::now().format(TIME_FORMAT_STR).to_string()),
        "TF Lookup Service available."
    );
    r2r::log_info!(
        &format!("URSG {}", Local::now().format(TIME_FORMAT_STR).to_string()),
        "UR Script Generator Service node started."
    );

    tokio::task::spawn(async move {
        let result = ur_script_generator_server(service, tf_lookup_client).await;
        match result {
            Ok(()) => r2r::log_info!(
                &format!("URSG {}", Local::now().format(TIME_FORMAT_STR).to_string()),
                "UR Script Generator Service call succeeded."
            ),
            Err(e) => r2r::log_error!(
                &format!("URSG {}", Local::now().format(TIME_FORMAT_STR).to_string()),
                "UR Script Generator Service call failed with: {}.",
                e
            ),
        };
    });

    handle.join().unwrap();

    Ok(())
}

async fn ur_script_generator_server(
    mut requests: impl Stream<Item = ServiceRequest<GenerateURScript::Service>> + Unpin,
    tf_lookup_client: r2r::Client<LookupTransform::Service>,
) -> Result<(), Box<dyn std::error::Error>> {
    let template_names = TEMPLATES
        .get_template_names()
        .map(|x| x.to_string())
        .collect::<Vec<String>>();
    for template in &template_names {
        r2r::log_info!(
            &format!("URSG {}", Local::now().format(TIME_FORMAT_STR).to_string()),
            "Found template: {:?}",
            template
        );
    }

    loop {
        match requests.next().await {
            Some(request) => {
                match template_names.contains(&format!("{}.script", &request.message.command)) {
                    true => match generate_script(&request.message, &tf_lookup_client).await {
                        Some(script) => {
                            r2r::log_warn!(
                                &format!("URSG {}", Local::now().format(TIME_FORMAT_STR).to_string()),
                                "Generated UR Script: \n{}",
                                script
                            );
                            request
                                .respond(GenerateURScript::Response {
                                    success: true,
                                    script,
                                })
                                .expect("Could not send service response.")
                        }
                        None => {
                            r2r::log_error!(
                                &format!("URSG {}", Local::now().format(TIME_FORMAT_STR).to_string()),
                                "Failed to generate UR Script.",
                            );
                            request
                                .respond(GenerateURScript::Response {
                                    success: false,
                                    script: String::default(),
                                })
                                .expect("Could not send service response.")
                        }
                    },
                    false => {
                        r2r::log_warn!(
                            &format!("URSG {}", Local::now().format(TIME_FORMAT_STR).to_string()),
                            "Template doesn't exist for command: {}.",
                            request.message.command
                        );
                        request
                            .respond(GenerateURScript::Response {
                                success: false,
                                script: String::default(),
                            })
                            .expect("Could not send service response.")
                    }
                };
            }
            None => break,
        }
    }

    Ok(())
}

async fn generate_script(
    message: &r2r::ur_script_generator_msgs::srv::GenerateURScript::Request,
    tf_lookup_client: &r2r::Client<LookupTransform::Service>,
) -> Option<String> {
    let empty_context = tera::Context::new();
    match TEMPLATES.render(
        &format!("{}.script", message.command),
        match &tera::Context::from_serialize(interpret_message(message, &tf_lookup_client).await) {
            Ok(context) => context,
            Err(e) => {
                r2r::log_error!(
                    &format!("URSG {}", Local::now().format(TIME_FORMAT_STR).to_string()),
                    "Creating a Tera Context from a serialized Interpretation failed with: {}.",
                    e
                );
                r2r::log_warn!(
                    &format!("URSG {}", Local::now().format(TIME_FORMAT_STR).to_string()),
                    "An empty Tera Context will be used instead."
                );
                &empty_context
            }
        },
    ) {
        Ok(script) => Some(script),
        Err(e) => {
            r2r::log_error!(
                &format!("URSG {}", Local::now().format(TIME_FORMAT_STR).to_string()),
                "Rendering the {}.script Tera Template failed with: {}.",
                message.command,
                e
            );
            None
        }
    }
}

async fn interpret_message(
    message: &r2r::ur_script_generator_msgs::srv::GenerateURScript::Request,
    tf_lookup_client: &r2r::Client<LookupTransform::Service>,
) -> Interpretation {
    let target_in_base = match message.use_joint_positions {
        true => pose_to_string(&TransformStamped::default()),
        false => {
            match lookup_tf(
                BASEFRAME_ID,
                &message.goal_feature_name,
                message.tf_lookup_deadline,
                tf_lookup_client,
            )
            .await
            {
                Some(transform) => pose_to_string(&transform),
                None => "failed".to_string(),
            }
        }
    };

    let tcp_in_faceplate = match message.use_joint_positions {
        true => pose_to_string(&TransformStamped::default()),
        false => {
            match lookup_tf(
                FACEPLATE_ID,
                &message.tcp_name,
                message.tf_lookup_deadline,
                tf_lookup_client,
            )
            .await
            {
                Some(transform) => pose_to_string(&transform),
                None => "failed".to_string(),
            }
        }
    };

    let valid: bool = (target_in_base != "falied" && tcp_in_faceplate != "falied")
        || (message.command == "move_j" && message.use_joint_positions);

    Interpretation {
        valid,
        command: message.command.to_string(),
        acceleration: message.acceleration,
        velocity: message.velocity,
        use_execution_time: message.use_execution_time,
        execution_time: message.execution_time,
        use_blend_radius: message.use_blend_radius,
        blend_radius: message.blend_radius,
        use_joint_positions: message.use_joint_positions,
        joint_positions: message.joint_positions.clone(),
        use_preferred_joint_config: message.use_preferred_joint_config,
        preferred_joint_config: message.preferred_joint_config.clone(),
        use_payload: message.use_payload,
        payload: message.payload.clone(),
        target_in_base,
        tcp_in_faceplate,
    }
}

fn pose_to_string(tf_stamped: &TransformStamped) -> String {
    let x = tf_stamped.transform.translation.x;
    let y = tf_stamped.transform.translation.y;
    let z = tf_stamped.transform.translation.z;
    let rot = tf_stamped.transform.rotation.clone();
    let angle = 2.0 * rot.w.acos();
    let den = (1.0 - rot.w.powi(2)).sqrt();
    let (rx, ry, rz) = match den < 0.001 {
        true => (rot.x * angle, rot.y * angle, rot.z * angle),
        false => (
            (rot.x / den) * angle,
            (rot.y / den) * angle,
            (rot.z / den) * angle,
        ),
    };
    format!("p[{},{},{},{},{},{}]", x, y, z, rx, ry, rz)
}

async fn lookup_tf(
    parent_id: &str,
    child_id: &str,
    deadline: i32,
    tf_lookup_client: &r2r::Client<LookupTransform::Service>,
) -> Option<TransformStamped> {
    let request = LookupTransform::Request {
        parent_id: parent_id.to_string(),
        child_id: child_id.to_string(),
        deadline,
    };

    let response = tf_lookup_client
        .request(&request)
        .expect("Could not send TF Lookup request.")
        .await
        .expect("Cancelled.");

    r2r::log_info!(
        &format!("URSG {}", Local::now().format(TIME_FORMAT_STR).to_string()),
        "Request to TF Lookup sent."
    );

    match response.success {
        true => Some(response.transform),
        false => {
            r2r::log_error!(
                &format!("URSG {}", Local::now().format(TIME_FORMAT_STR).to_string()),
                "Couldn't lookup TF for parent '{}' and child '{}'.",
                parent_id,
                child_id
            );
            None
        }
    }
}
