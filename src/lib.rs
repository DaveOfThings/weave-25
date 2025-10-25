mod split;

// use vector3d::Vector3d;
use vecmath::{Vector3, vec3_normalized};
use quaternion::{rotation_from_to, rotate_vector, axis_angle, mul};

pub fn add(left: u64, right: u64) -> u64 {
    left + right
}

// Split arc from a to b into a-c-d-b, returning new coordinates, c, d.
pub fn midpoint(a: Vector3<f64>, b: Vector3<f64>) -> Vector3<f64> {
    let q_ab = rotation_from_to(a, b);

    // Calculate quaternion with same axis, half the rotation of q_ab.
    let axis = vec3_normalized(q_ab.1);
    let half_theta = q_ab.0.acos();
    let half_q_ab = axis_angle(axis, half_theta);

    // Get midpoint from a to b by applying half_q_ab.
    let mid = rotate_vector(half_q_ab, a);

    mid
}

const THETA_ACCURACY: f64 = 0.0001_f64.to_radians();

// We need to find a point, c, to the left of a->b that we can reach in some distance, theta,
// then turn 60 degrees right, and then theta/2 forward to reach b.
pub fn z_point(a: Vector3<f64>, b: Vector3<f64>) -> Vector3<f64> {
    // First task: figure out theta_ac.
    // Let theta_ab be the distance from a to b.
    // Strategy: Do a binary search between theta_ac and theta_ac/2
    let q_ab = rotation_from_to(a, b);
    let theta_ab = 2.0 * q_ab.0.acos();
    let mut theta_low = theta_ab/2.0;
    let mut theta_high = theta_ab;
    while theta_high-theta_low > THETA_ACCURACY {
        let theta_test = (theta_high+theta_low)/2.0;

        // Rotate by theta_test about Z.
        let q1 = axis_angle([0.0, 0.0, 1.0], theta_test);
        // Turn -60 degrees about X.
        let q2 = axis_angle([1.0, 0.0, 0.0], (60.0_f64).to_radians());
        // Rotate by theta_test/2 about Z.
        let q3 = axis_angle([0.0, 0.0, 1.0], theta_test/2.0);
        
        let q = mul(q3, mul(q2, q1));
        let theta_result = q.0.acos()/2.0;

        if theta_result > theta_ab {
            theta_high = theta_test;
        }
        else {
            theta_low = theta_test;
        }
    }
    let theta_ac = (theta_high+theta_low)/2.0;

    println!("Theta ac found as {theta_ac}");


    [0.0, 0.0, 0.0]
}

pub fn z_split(a: Vector3<f64>, b: Vector3<f64>) -> (Vector3<f64>, Vector3<f64>) {
    let mid = midpoint(a, b);

    let c = z_point(a, mid);
    let d = z_point(b, mid);

    (c, d)

}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let result = add(2, 2);
        assert_eq!(result, 4);
    }

    #[test]
    fn test_mid() {
        let a: Vector3::<f64> = [1.0, 0.0, 0.0];
        let b: Vector3::<f64> = [0.0, 1.0, 0.0];
        let c = midpoint(a, b);

        println!("Mid: {:?}", c);
    }

    #[test]
    fn test_split() {
        let a: Vector3::<f64> = [1.0, 0.0, 0.0];
        let b: Vector3::<f64> = [1.0, 1.0, 0.0];
        let (c, d) = z_split(a, b);

        println!("Mid: {:?}", c);
    }
}
