use futures::future;
use futures::stream::Stream;
use futures::Future;
use futures::StreamExt;
use lazy_static::lazy_static;
use r2r::geometry_msgs::msg::Transform;
use r2r::geometry_msgs::msg::TransformStamped;
use r2r::tf_tools_msgs::srv::LookupTransform;
use r2r::ur_script_generator_msgs::srv::GenerateURScript;
use r2r::ur_script_msgs::action::ExecuteScript;
use r2r::Context;
use r2r::ServiceRequest;
use r2r::{self, ActionServerGoal};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use serde::{Serialize, Deserialize};
use tera;

pub static BASEFRAME_ID: &'static str = "base";
pub static FACEPLATE_ID: &'static str = "tool0";

#[derive(Serialize, Deserialize)]
pub struct Interpretation {
    pub valid: bool,
    pub target_in_base: String,
    pub tcp_in_faceplate: String,
    pub velocity: f64,
    pub acceleration: f64,
    pub consider_pwc: bool,
    // pub pwc: PreferredJointConfig
}

lazy_static! {
    pub static ref TEMPLATES: tera::Tera = {
        let tera = match tera::Tera::new("templates/**/*.script") {
            Ok(t) => t,
            Err(e) => {
                r2r::log_error!(
                    &format!("{:?}", std::time::SystemTime::now()),
                    "UR Script template parsing error(s): {}", e
                );
                ::std::process::exit(1); // don't exit but warn or don't collect
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
    let tf_lookup_client = node.create_client::<LookupTransform::Service>("lookup_tf")?;
    let waiting_for_tf_lookup_server = node.is_available(&tf_lookup_client)?;

    r2r::log_warn!(
        &format!("{:?}", std::time::SystemTime::now()),
        "Waiting for TF Lookup service..."
    );
    waiting_for_tf_lookup_server.await?;
    r2r::log_info!(
        &format!("{:?}", std::time::SystemTime::now()),
        "TF Lookup Service available."
    );
    r2r::log_info!(
        &format!("{:?}", std::time::SystemTime::now()),
        "UR Script Generator Service node started."
    );

    tokio::task::spawn(async move {
        let result = ur_script_generator_server(service, tf_lookup_client).await;
        match result {
            Ok(()) => r2r::log_info!(
                &format!("{:?}", std::time::SystemTime::now()),
                "UR Script Generator Service call succeeded."
            ),
            Err(e) => r2r::log_error!(
                &format!("{:?}", std::time::SystemTime::now()),
                "UR Script Generator Service call failed with: {}.",
                e
            ),
        };
    });

    let handle = std::thread::spawn(move || loop {
        node.spin_once(std::time::Duration::from_millis(100));
    });

    handle.join().unwrap();

    Ok(())
}

async fn ur_script_generator_server(
    mut requests: impl Stream<Item = ServiceRequest<GenerateURScript::Service>> + Unpin,
    tf_lookup_client: r2r::Client<LookupTransform::Service>,
) -> Result<(), Box<dyn std::error::Error>> {
    let template_names = TEMPLATES.get_template_names().collect::<String>();

    loop {
        match requests.next().await {
            Some(request) => {
                let generate = match template_names.contains(&request.message.command) {
                    true => generate_script(&request.message).await.unwrap(),
                    // true => match generate_script(&request.message) {
                    //     Some(sctipt) =>
                    // }
                    false => {
                        r2r::log_warn!(
                            &format!("{:?}", std::time::SystemTime::now()),
                            "Template doesn't exist for command: {}.",
                            request.message.command
                        );
                        String::default()
                    }
                };
                let response = GenerateURScript::Response {
                    gantry_pose: 1,
                    ur_script: generate,
                };
                request
                    .respond(response)
                    .expect("could not send service response");
            }
            None => break,
        }
    }

    Ok(())
}

async fn generate_script(
    message: &r2r::ur_script_generator_msgs::srv::GenerateURScript::Request,
) -> Option<String> {
    let empty_context = tera::Context::new();
    match TEMPLATES.render(
        &format!("{}.script", message.command),
        // match &tera::Context::from_serialize(interpret_message(message)) {
            match &tera::Context::from_serialize(message) {
            Ok(context) => context,
            Err(e) => {
                r2r::log_error!(
                    &format!("{:?}", std::time::SystemTime::now()),
                    "Creating a Tera Context from a serialized ROS message failed with: {}.",
                    e
                );
                r2r::log_warn!(
                    &format!("{:?}", std::time::SystemTime::now()),
                    "An empty Tera Context will be used instead."
                );
                &empty_context
            }
        },
    ) {
        Ok(script) => Some(script),
        Err(e) => {
            r2r::log_error!(
                &format!("{:?}", std::time::SystemTime::now()),
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

    let target_in_base = match lookup_tf(
        BASEFRAME_ID, &message.goal_feature_name, message.tf_lookup_deadline, tf_lookup_client
    ).await {
        Some(transform) => pose_to_string(&transform),
        None => "failed".to_string()
    };

    let tcp_in_faceplate = match lookup_tf(
        FACEPLATE_ID, &message.tcp_name, message.tf_lookup_deadline, tf_lookup_client
    ).await {
        Some(transform) => pose_to_string(&transform),
        None => "failed".to_string()
    };
    
    Interpretation {
        valid: target_in_base != "falied" && tcp_in_faceplate != "falied",
        target_in_base,
        tcp_in_faceplate,
        velocity: message.velocity,
        acceleration: message.acceleration,
        consider_pwc: false
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
        false => ((rot.x / den) * angle, (rot.y / den) * angle, (rot.z / den) * angle)
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
        &format!("{:?}", std::time::SystemTime::now()),
        "Request to TF Lookup sent."
    );

    match response.success {
        true => Some(response.transform),
        false => {
            r2r::log_error!(
                &format!("{:?}", std::time::SystemTime::now()),
                "Couldn't lookup TF for parent '{}' and child '{}'.",
                parent_id,
                child_id
            );
            None
        }
    }
}
