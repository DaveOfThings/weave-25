mod split;

// use vector3d::Vector3d;
use vecmath::{Vector3, vec3_normalized, vec3_dot};
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
    // First task: figure out theta_ac.
    // Let theta_ab be the distance from a to b.
    // Strategy: Do a binary search between theta_ac and theta_ac/2
    let q_ab = rotation_from_to(a, b);
    let theta_ab = vec3_dot(a, b).acos();
    let axis_ab = vec3_normalized(q_ab.1);
    // println!("axis_ab: {:?}", axis_ab);
    let axis_a = vec3_normalized(a);
    // println!("axis_a: {:?}", axis_a);

    let mut theta_low = theta_ab/4.0;
    let mut theta_high = theta_ab;
    while theta_high-theta_low > THETA_ACCURACY {
        // println!("Range: {} deg - {} deg", theta_low.to_degrees(), theta_high.to_degrees());

        let theta_test = (theta_high+theta_low)/2.0;

        // Rotate by theta_test about axb axis
        let q1 = axis_angle(axis_ab, theta_test);
        // Turn -60 degrees about a axis
        let q2 = axis_angle(axis_a, (-60.0_f64).to_radians());
        // Rotate by theta_test/2 about axb axis
        let q3 = axis_angle(axis_ab, theta_test/2.0);
        
        let q = mul(mul(q1, q2), q3);
        let result = rotate_vector(q, a);

        let theta_result = vec3_dot(a, result).acos();
        // println!("theta_test {} -> Theta_result = {} deg", 
        //    theta_test.to_degrees(), 
        //    theta_result.to_degrees());
        // println!("rotation 1: {} deg, rotation 2: {} deg, rotation 3: {} deg",
        //    2.0*q1.0.acos().to_degrees(), 2.0*q2.0.acos().to_degrees(), 2.0*q3.0.acos().to_degrees());

        if theta_result > theta_ab {
            theta_high = theta_test;
        }
        else {
            theta_low = theta_test;
        }
        
    }
    let theta_ac = (theta_high+theta_low)/2.0;

    // println!("Theta ac found as {} degrees", theta_ac.to_degrees());

    // Rotate by theta_test about axb
    let q1 = axis_angle(axis_ab, theta_ac);
    // Turn -60 degrees about a
    let q2 = axis_angle(axis_a, (-60.0_f64).to_radians());
    // Rotate by theta_test/2 about axb
    let q3 = axis_angle(axis_ab, theta_ac/2.0);
    
    let q = mul(mul(q1, q2), q3);
    let result = rotate_vector(q, a);
    let q_result = rotation_from_to(a, result);

    // Find angle between current a->b and a->result
    let phi = vec3_dot(vec3_normalized(q_ab.1), vec3_normalized(q_result.1)).acos();
    // println!("  Phi is {phi} ({} degrees)", phi.to_degrees());

    // Now, to get to c, we need to turn by phi around axis a, then q1.
    let q_phi = axis_angle(a, phi);
    let q_c = mul(q_phi, q1); // q1 * q_phi?
    let c = rotate_vector(q_c, a);

    // let q_full = mul(q_phi, q_result);
    // let end = rotate_vector(q_full, a);
    // println!("Full operation got to: {:?}", end);

    // Check angles from a to c, b to c
    // let ac = vec3_dot(a, c).acos();
    // let bc = vec3_dot(b, c).acos();
    // println!("ac : {} deg, bc : {} deg", ac.to_degrees(), bc.to_degrees());

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

        println!("thetas ac: {}, cd: {}, db: {}", 
            theta_ac.to_degrees(), theta_cd.to_degrees(), theta_db.to_degrees());

        assert!((theta_ac-theta_cd).abs() <= THETA_ACCURACY);
        assert!((theta_cd-theta_db).abs() <= THETA_ACCURACY);
        assert!((theta_db-theta_ac).abs() <= THETA_ACCURACY);


        println!("c: {c:?}");
        println!("d: {d:?}");
    }

    #[test]
    fn test_q_mul() {
        let x90 = axis_angle([1.0, 0.0, 0.0], 90.0_f64.to_radians());
        let y90 = axis_angle([0.0, 1.0, 0.0], 90.0_f64.to_radians());

        let q1q2 = mul(x90, y90); // computes y90 * x90, rot about x, then y
        let q2q1 = mul(y90, x90);

        println!("q1 {x90:?} * q2 {y90:?} = {q1q2:?}");
        println!("q2 {y90:?} * q1 {x90:?} = {q2q1:?}");   
    }
}
