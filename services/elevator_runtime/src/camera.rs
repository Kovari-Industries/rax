use anyhow::{Result, bail, ensure};
use serde_json::{Value, json};
pub fn ray(frame: &Value, pixel: [f64; 2]) -> Result<Value> {
    ensure!(
        frame["geometry_validated"] == true,
        "metric camera geometry unavailable"
    );
    ensure!(
        pixel.iter().all(|v| v.is_finite())
            && pixel[0] >= 0.0
            && pixel[1] >= 0.0
            && pixel[0] < frame["width"].as_f64().unwrap_or(0.0)
            && pixel[1] < frame["height"].as_f64().unwrap_or(0.0),
        "pixel outside image"
    );
    let k: Vec<f64> = serde_json::from_value(frame["intrinsics"]["k"].clone())?;
    let d: Vec<f64> = serde_json::from_value(frame["intrinsics"]["d"].clone())?;
    validate_intrinsics(
        &k,
        &d,
        frame["intrinsics"]["distortion_model"]
            .as_str()
            .unwrap_or(""),
    )?;
    let xd = (pixel[0] - k[2]) / k[0];
    let yd = (pixel[1] - k[5]) / k[4];
    let (mut x, mut y) = (xd, yd);
    if frame["intrinsics"]["distortion_model"] == "equidistant" {
        let radius = xd.hypot(yd);
        let mut theta = radius;
        for _ in 0..20 {
            let t2 = theta * theta;
            let polynomial =
                1.0 + d[0] * t2 + d[1] * t2.powi(2) + d[2] * t2.powi(3) + d[3] * t2.powi(4);
            let derivative = 1.0
                + 3.0 * d[0] * t2
                + 5.0 * d[1] * t2.powi(2)
                + 7.0 * d[2] * t2.powi(3)
                + 9.0 * d[3] * t2.powi(4);
            ensure!(derivative.abs() > 1e-10, "singular camera ray");
            theta -= (theta * polynomial - radius) / derivative;
        }
        let t2 = theta * theta;
        let residual = theta
            * (1.0 + d[0] * t2 + d[1] * t2.powi(2) + d[2] * t2.powi(3) + d[3] * t2.powi(4))
            - radius;
        ensure!(residual.abs() < 1e-9, "camera ray did not converge");
        ensure!(
            (0.0..std::f64::consts::FRAC_PI_2).contains(&theta),
            "pixel ray is outside supported hemisphere"
        );
        if radius > 1e-12 {
            x = xd * theta.tan() / radius;
            y = yd * theta.tan() / radius;
        }
    } else {
        for _ in 0..30 {
            let r2 = x * x + y * y;
            let radial = 1.0 + d[0] * r2 + d[1] * r2 * r2 + d[4] * r2.powi(3);
            ensure!(radial.abs() > 1e-10, "singular camera ray");
            let next_x = (xd - 2.0 * d[2] * x * y - d[3] * (r2 + 2.0 * x * x)) / radial;
            let next_y = (yd - d[2] * (r2 + 2.0 * y * y) - 2.0 * d[3] * x * y) / radial;
            (x, y) = (next_x, next_y);
        }
    }
    if frame["intrinsics"]["distortion_model"] == "plumb_bob" {
        let r2 = x * x + y * y;
        let radial = 1.0 + d[0] * r2 + d[1] * r2 * r2 + d[4] * r2.powi(3);
        let projected_x = x * radial + 2.0 * d[2] * x * y + d[3] * (r2 + 2.0 * x * x);
        let projected_y = y * radial + d[2] * (r2 + 2.0 * y * y) + 2.0 * d[3] * x * y;
        ensure!(
            (projected_x - xd).hypot(projected_y - yd) < 1e-9,
            "camera ray did not converge"
        );
    }
    let norm = (x * x + y * y + 1.0).sqrt();
    let v = [x / norm, y / norm, 1.0 / norm];
    let q: [f64; 4] = serde_json::from_value(frame["camera_to_base"]["quaternion_xyzw"].clone())?;
    ensure!(
        q.iter().all(|v| v.is_finite())
            && (q.iter().map(|v| v * v).sum::<f64>() - 1.0).abs() < 1e-6,
        "invalid camera orientation"
    );
    let origin: [f64; 3] = serde_json::from_value(frame["camera_to_base"]["position"].clone())?;
    ensure!(
        origin.iter().all(|v| v.is_finite()),
        "invalid camera origin"
    );
    let t = [
        2.0 * (q[1] * v[2] - q[2] * v[1]),
        2.0 * (q[2] * v[0] - q[0] * v[2]),
        2.0 * (q[0] * v[1] - q[1] * v[0]),
    ];
    let direction = [
        v[0] + q[3] * t[0] + q[1] * t[2] - q[2] * t[1],
        v[1] + q[3] * t[1] + q[2] * t[0] - q[0] * t[2],
        v[2] + q[3] * t[2] + q[0] * t[1] - q[1] * t[0],
    ];
    if !direction.iter().all(|v| v.is_finite()) {
        bail!("invalid camera ray");
    }
    Ok(
        json!({"frame":"base_link","origin":frame["camera_to_base"]["position"],"direction":direction,"capture_us":frame["capture_us"],"depth_available":false}),
    )
}

fn validate_intrinsics(k: &[f64], d: &[f64], distortion: &str) -> Result<()> {
    ensure!(
        k.len() == 9 && k.iter().chain(d).all(|v| v.is_finite()),
        "invalid camera intrinsics"
    );
    ensure!(
        k[0] > 0.0
            && k[4] > 0.0
            && k[1] == 0.0
            && k[3] == 0.0
            && k[6] == 0.0
            && k[7] == 0.0
            && k[8] == 1.0,
        "unsupported camera matrix"
    );
    ensure!(
        (distortion == "plumb_bob" && d.len() == 5)
            || (distortion == "equidistant" && d.len() == 4),
        "unsupported distortion model"
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    fn frame() -> Value {
        json!({"geometry_validated":true,"width":640,"height":480,"capture_us":123,
        "intrinsics":{"k":[200,0,320,0,200,240,0,0,1],"d":[0,0,0,0,0],"distortion_model":"plumb_bob"},
        "camera_to_base":{"position":[1,2,3],"quaternion_xyzw":[0,0,0,1]}})
    }
    #[test]
    fn tangential_distortion_round_trip_and_invalid_calibration() {
        let mut f = frame();
        f["intrinsics"]["d"] = json!([0.05, -0.01, 0.02, -0.01, 0.001]);
        let (x, y): (f64, f64) = (0.3, -0.2);
        let r2 = x * x + y * y;
        let radial = 1.0 + 0.05 * r2 - 0.01 * r2 * r2 + 0.001 * r2.powi(3);
        let pixel = [
            320.0 + 200.0 * (x * radial + 0.04 * x * y - 0.01 * (r2 + 2.0 * x * x)),
            240.0 + 200.0 * (y * radial + 0.02 * (r2 + 2.0 * y * y) - 0.02 * x * y),
        ];
        let result = ray(&f, pixel).unwrap();
        let direction: [f64; 3] = serde_json::from_value(result["direction"].clone()).unwrap();
        assert!((direction[0] / direction[2] - x).abs() < 1e-9);
        assert!((direction[1] / direction[2] - y).abs() < 1e-9);
        f["intrinsics"]["d"] = json!([]);
        assert!(ray(&f, pixel).is_err());
        f = frame();
        f["intrinsics"]["k"][1] = json!(1.0);
        assert!(ray(&f, pixel).is_err());
    }
    #[test]
    fn ray_is_calibrated_and_has_no_invented_depth() {
        let result = ray(&frame(), [320.0, 240.0]).unwrap();
        assert_eq!(result["direction"], json!([0.0, 0.0, 1.0]));
        assert_eq!(result["origin"], json!([1, 2, 3]));
        assert_eq!(result["depth_available"], false);
        assert!(ray(&frame(), [640.0, 240.0]).is_err());
        let mut missing = frame();
        missing["geometry_validated"] = json!(false);
        assert!(ray(&missing, [320.0, 240.0]).is_err());
    }
}
