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
use tera;

pub struct Interpretation {

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

// async fn interpret_message(
//     message: &r2r::ur_script_generator_msgs::srv::GenerateURScript::Request,
// ) ->

// def pose_to_string(self, tf_stamped):
//         x = tf_stamped.transform.translation.x
//         y = tf_stamped.transform.translation.y
//         z = tf_stamped.transform.translation.z
//         q = tf_stamped.transform.rotation
//         angle = 2 * math.acos(q.w)
//         den = math.sqrt(1 - q.w * q.w)
//         if den < 0.001:
//             Rx = q.x * angle
//             Ry = q.y * angle
//             Rz = q.z * angle
//         else:
//             Rx = (q.x / den) * angle
//             Ry = (q.y / den) * angle
//             Rz = (q.z / den) * angle

//         return (
//             "p["
//             + str(x)
//             + ", "
//             + str(y)
//             + ", "
//             + str(z)
//             + ", "
//             + str(Rx)
//             + ", "
//             + str(Ry)
//             + ", "
//             + str(Rz)
//             + "]"
//         )

fn pose_to_string(tf_stamped: &TransformStamped) -> String {
    let x = tf_stamped.transform.translation.x;
    let y = tf_stamped.transform.translation.y;
    "".to_string()
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
