mod split;

// use vector3d::Vector3d;
use vecmath::{Vector3, vec3_normalized, vec3_dot, vec3_cross};
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

const THETA_ACCURACY: f64 = 0.0000001_f64.to_radians();

// We need to find a point, c, to the left of a->b that we can reach in some distance, theta,
// then turn 60 degrees right, and then theta/2 forward to reach b.
pub fn z_point(a: Vector3<f64>, b: Vector3<f64>) -> Vector3<f64> {
    // First task: figure out theta_ac, the distance to point c.
    // Let theta_ab be the full distance from a to b.
    // Strategy: Do a binary search between theta_ab and theta_ab/4
    // For each test angle, use the law of cosines to find the length of
    // the resulting steps, then refine the search.

    // let q_ab = rotation_from_to(a, b);
    let cos_theta_ab = vec3_dot(a, b);
    let theta_ab = cos_theta_ab.acos();

    // Bracket the theta_ac value we need.
    let mut theta_low = theta_ab/4.0;
    let mut theta_high = theta_ab;

    // Hone in on theta_ac with a binary search until we meet desired accuracy.
    while theta_high-theta_low > THETA_ACCURACY {
        // Pick a theta to test from the middle of the range
        let theta_test = (theta_high+theta_low)/2.0;

        // from law of cosines on a sphere.
        let cos_theta_result = theta_test.cos()*(theta_test/2.0).cos()
                             + (-0.5_f64)*theta_test.sin()*(theta_test/2.0).sin();

        let theta_result = cos_theta_result.acos();

        // Decide whether to search higher or lower than theta_test.
        if theta_result > theta_ab {
            theta_high = theta_test;
        }
        else {
            theta_low = theta_test;
        }
        
    }

    // We have theta_ac, the distance for the a-c segment.
    let theta_ac = (theta_high+theta_low)/2.0;

    // Next step, find phi, using the law of cosines, now that we know the sides of this triangle.
    let cos_phi = (((theta_ac/2.0).cos() - theta_ac.cos()*theta_ab.cos())) / (theta_ac.sin()*theta_ab.sin());
    let phi = cos_phi.acos();

    let axis_ab = vec3_normalized(vec3_cross(a, b));
    let q1 = axis_angle(axis_ab, theta_ac);

    // Now, to get to c, we start from a, rotate phi then q1
    let q_phi = axis_angle(a, phi);
    let q_c = mul(q_phi, q1); // q1 * q_phi?
    let c = rotate_vector(q_c, a);

    // Point c is the value we needed to compute
    c
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
    use vecmath::{Vector3, vec3_normalized, vec3_dot};

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
        let a: Vector3::<f64> = vec3_normalized([1.0, 0.0, 0.0]);
        let b: Vector3::<f64> = vec3_normalized([0.0, 1.0, 0.0]);
        let (c, d) = z_split(a, b);

        let theta_ac = vec3_dot(a, c).acos();
        let theta_cd = vec3_dot(c, d).acos();
        let theta_db = vec3_dot(d, b).acos();

        assert!((theta_ac-theta_cd).abs() <= THETA_ACCURACY);
        assert!((theta_cd-theta_db).abs() <= THETA_ACCURACY);
        assert!((theta_db-theta_ac).abs() <= THETA_ACCURACY);
    }

    #[test]
    fn test_q_mul() {
        let x90 = axis_angle([1.0, 0.0, 0.0], 90.0_f64.to_radians());
        let y90 = axis_angle([0.0, 1.0, 0.0], 90.0_f64.to_radians());

        let _q1q2 = mul(x90, y90); // computes y90 * x90, rot about x, then y
        let _q2q1 = mul(y90, x90);

        // We see here that mul(q1, q2) is computing q2 * q1.  In other words it's left multiplying
        // q1 by q2.  The arguments seem backward to me, but I think that's what we're seeing.
        // println!("q1 {x90:?} * q2 {y90:?} = {q1q2:?}");
        // println!("q2 {y90:?} * q1 {x90:?} = {q2q1:?}");   

        // TODO-DW : Add asserts.
    }
}
