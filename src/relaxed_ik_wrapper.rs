use crate::relaxed_ik::{RelaxedIK, Opt};
use std::sync::{Arc, Mutex};
use nalgebra::{Vector3, Vector6, UnitQuaternion, Quaternion,Translation3, Isometry3};
use std::os::raw::{*};
use std::str;
use crate::utils_rust::file_utils::{*};
use crate::groove::objective_master::ObjectiveMaster;

// http://jakegoulding.com/rust-ffi-omnibus/objects/
#[no_mangle]
pub unsafe extern "C" fn relaxed_ik_new(path_to_setting: *const c_char) -> *mut RelaxedIK {
    if path_to_setting.is_null() 
    { 
        let path_to_src = get_path_to_src();
        let default_path_to_setting = path_to_src +  "configs/settings.yaml";
        return Box::into_raw(Box::new(RelaxedIK::load_settings(default_path_to_setting.as_str())))
    }
    let c_str = std::ffi::CStr::from_ptr(path_to_setting);
    let path_to_setting_str = c_str.to_str().expect("Not a valid UTF-8 string");

    Box::into_raw(Box::new(RelaxedIK::load_settings(path_to_setting_str)))
}

#[no_mangle] 
pub unsafe extern "C" fn relaxed_ik_free(ptr: *mut RelaxedIK) {
    if ptr.is_null() { return }
    Box::from_raw(ptr);
}

#[no_mangle]
pub unsafe extern "C" fn reset(ptr: *mut RelaxedIK, prev_state3: *const c_double, prev_state3_length: c_int, 
    prev_state2: *const c_double, prev_state2_length: c_int, prev_state: *const c_double, prev_state_length: c_int, 
    init_state: *const c_double, init_state_length: c_int, 
) {
    let relaxed_ik = unsafe {
        assert!(!ptr.is_null());
        &mut *ptr
    };

    let prev_state3_slice: &[c_double] = std::slice::from_raw_parts(prev_state3, prev_state3_length as usize);
    let prev_state3_vec = prev_state3_slice.to_vec();

    let prev_state2_slice: &[c_double] = std::slice::from_raw_parts(prev_state2, prev_state2_length as usize);
    let prev_state2_vec = prev_state2_slice.to_vec();

    let prev_state_slice: &[c_double] = std::slice::from_raw_parts(prev_state, prev_state_length as usize);
    let prev_state_vec = prev_state_slice.to_vec();

    let init_state_slice: &[c_double] = std::slice::from_raw_parts(init_state, init_state_length as usize);
    let init_state_vec = init_state_slice.to_vec();
    
    relaxed_ik.reset(prev_state3_vec, prev_state2_vec, prev_state_vec, init_state_vec);
}

#[no_mangle]
pub unsafe extern "C" fn set_ee_only(ptr: *mut RelaxedIK, ee_only: c_int) {
    let relaxed_ik = unsafe {
        assert!(!ptr.is_null());
        &mut *ptr
    };

    relaxed_ik.set_ee_only(ee_only != 0);
}


#[no_mangle]
pub unsafe extern "C" fn set_valid_chains(ptr: *mut RelaxedIK, valid_chains: *const c_int, valid_chain_length: c_int) {
    let relaxed_ik = unsafe {
        assert!(!ptr.is_null());
        &mut *ptr
    };

    let valid_chain_slice: &[c_int] = std::slice::from_raw_parts(valid_chains, valid_chain_length as usize);
    let valid_chains_usize: Vec<usize> = valid_chain_slice.iter().map(|&x| x as usize).collect();

    relaxed_ik.set_valid_chains(&valid_chains_usize);
}

#[no_mangle]
pub unsafe extern "C" fn solve_position(ptr: *mut RelaxedIK, pos_goals: *const c_double, pos_length: c_int, 
    quat_goals: *const c_double, quat_length: c_int,
    tolerance: *const c_double, tolerance_length: c_int) -> Opt {

    let relaxed_ik = unsafe {
        assert!(!ptr.is_null());
        &mut *ptr
    };
    
    assert!(!pos_goals.is_null(), "Null pointer for pos goals!");
    assert!(!quat_goals.is_null(), "Null pointer for quat goals!");
    assert!(!tolerance.is_null(), "Null pointer for tolerance!"); 

    // assert!(pos_length as usize == relaxed_ik.vars.robot.num_chains * 3 , 
    //     "Pos vels are expected to have {} numbers, but got {}", 
    //     relaxed_ik.vars.robot.num_chains * 3, pos_length);
    // assert!(quat_length as usize == relaxed_ik.vars.robot.num_chains * 4,
    //     "Rot vels are expected to have {} numbers, but got {}", 
    //     relaxed_ik.vars.robot.num_chains * 4, quat_length);
    // assert!(tolerance_length as usize == relaxed_ik.vars.robot.num_chains * 6, 
    //     "Tolerance are expected to have {} numbers, but got {}", 
    //     relaxed_ik.vars.robot.num_chains * 6, tolerance_length);

    let pos_slice: &[c_double] = std::slice::from_raw_parts(pos_goals, pos_length as usize);
    let quat_slice: &[c_double] = std::slice::from_raw_parts(quat_goals, quat_length as usize);
    let tolerance_slice: &[c_double] = std::slice::from_raw_parts(tolerance, tolerance_length as usize);

    let pos_vec = pos_slice.to_vec();
    let quat_vec = quat_slice.to_vec();
    let tolerance_vec = tolerance_slice.to_vec();

    let ja = solve_position_helper(relaxed_ik, pos_vec, quat_vec, tolerance_vec);

    let ptr = ja.as_ptr();
    let len = ja.len();
    std::mem::forget(ja);
    Opt {data: ptr, length: len as c_int}
}


#[no_mangle]
pub unsafe extern "C" fn solve_velocity(ptr: *mut RelaxedIK, pos_vels: *const c_double, pos_length: c_int, 
    rot_vels: *const c_double, rot_length: c_int,
    tolerance: *const c_double, tolerance_length: c_int) -> Opt {

    let relaxed_ik = unsafe {
        assert!(!ptr.is_null());
        &mut *ptr
    };
    
    assert!(!pos_vels.is_null(), "Null pointer for pos vels!");
    assert!(!rot_vels.is_null(), "Null pointer for rot vels!");
    assert!(!tolerance.is_null(), "Null pointer for tolerance!"); 

    // assert!(pos_length as usize == relaxed_ik.vars.robot.num_chains * 3 , 
    //     "Pos vels are expected to have {} numbers, but got {}", 
    //     relaxed_ik.vars.robot.num_chains * 3, pos_length);
    // assert!(rot_length as usize == relaxed_ik.vars.robot.num_chains * 3,
    //     "Rot vels are expected to have {} numbers, but got {}", 
    //     relaxed_ik.vars.robot.num_chains * 3, rot_length);
    // assert!(tolerance_length as usize == relaxed_ik.vars.robot.num_chains * 6, 
    //     "Tolerance are expected to have {} numbers, but got {}", 
    //     relaxed_ik.vars.robot.num_chains * 6, tolerance_length);

    let pos_slice: &[c_double] = std::slice::from_raw_parts(pos_vels, pos_length as usize);
    let rot_slice: &[c_double] = std::slice::from_raw_parts(rot_vels, rot_length as usize);
    let tolerance_slice: &[c_double] = std::slice::from_raw_parts(tolerance, tolerance_length as usize);

    let pos_vec = pos_slice.to_vec();
    let rot_vec = rot_slice.to_vec();
    let tolerance_vec = tolerance_slice.to_vec();

    let ja = solve_velocity_helper(relaxed_ik, pos_vec, rot_vec, tolerance_vec);

    let ptr = ja.as_ptr();
    let len = ja.len();
    std::mem::forget(ja);
    Opt {data: ptr, length: len as c_int}
}

#[no_mangle]
pub unsafe extern "C" fn get_ee_poses(ptr: *mut RelaxedIK) -> Opt {
    let relaxed_ik = unsafe {
        assert!(!ptr.is_null());
        &mut *ptr
    };

    let mut poses = Vec::new();
    let ee_pose = relaxed_ik.vars.robot.get_ee_pos_and_quat_immutable(&relaxed_ik.vars.xopt);
    for i in 0..ee_pose.len() {
        poses.push(ee_pose[i].0.x);
        poses.push(ee_pose[i].0.y);
        poses.push(ee_pose[i].0.z);
        poses.push(ee_pose[i].1.coords.w);
        poses.push(ee_pose[i].1.coords.x);
        poses.push(ee_pose[i].1.coords.y);
        poses.push(ee_pose[i].1.coords.z);
    }
    let ptr = poses.as_ptr();
    let len = poses.len();
    std::mem::forget(poses);
    Opt {data: ptr, length: len as c_int}
}

#[no_mangle]
pub unsafe extern "C" fn get_wrist_poses(ptr: *mut RelaxedIK) -> Opt {
    let relaxed_ik = unsafe {
        assert!(!ptr.is_null());
        &mut *ptr
    };
    let robot = &relaxed_ik.vars.robot;

    let mut poses = Vec::new();
    let frames = robot.get_frames_immutable(&relaxed_ik.vars.xopt);
    for i in vec![0, 5] { // 0, 5 are the base indices
        poses.push(frames[i].0[7].x);
        poses.push(frames[i].0[7].y);
        poses.push(frames[i].0[7].z);
        poses.push(frames[i].1[7].coords.w);
        poses.push(frames[i].1[7].coords.x);
        poses.push(frames[i].1[7].coords.y);
        poses.push(frames[i].1[7].coords.z);
    }
    let ptr = poses.as_ptr();
    let len = poses.len();
    std::mem::forget(poses);
    Opt {data: ptr, length: len as c_int}
}

#[no_mangle]
pub unsafe extern "C" fn get_frames(ptr: *mut RelaxedIK) -> Opt {
    let relaxed_ik = unsafe {
        assert!(!ptr.is_null());
        &mut *ptr
    };
    let robot = &relaxed_ik.vars.robot;

    let mut poses = Vec::new();
    let frames = robot.get_frames_immutable(&relaxed_ik.vars.xopt);
    for joint_idx in 0..robot.num_dofs {
        'outer: for i in 0..robot.chain_indices.len() {
            for j in 0..robot.chain_indices[i].len() {    
                if joint_idx == robot.chain_indices[i][j] {
                    let mut ctr = j + 1;
                    let mut k = 0;
                    while ctr > 0 {
                        if robot.arms[i].joint_types[k] != "fixed" {
                            ctr -= 1;
                        }
                        k += 1;
                    }
                    poses.push(frames[i].0[k-1].x);
                    poses.push(frames[i].0[k-1].y);
                    poses.push(frames[i].0[k-1].z);
                    poses.push(frames[i].1[k-1].coords.w);
                    poses.push(frames[i].1[k-1].coords.x);
                    poses.push(frames[i].1[k-1].coords.y);
                    poses.push(frames[i].1[k-1].coords.z);
                    break 'outer;
                }
            }
        }
    }
    let ptr = poses.as_ptr();
    let len = poses.len();
    std::mem::forget(poses);
    Opt {data: ptr, length: len as c_int}
}

// This is mainly for backward compatibility
#[no_mangle]
pub unsafe extern "C" fn solve(ptr: *mut RelaxedIK, pos_goals: *const c_double, pos_length: c_int, 
    quat_goals: *const c_double, quat_length: c_int,
    tolerance: *const c_double, tolerance_length: c_int) -> Opt {

    let relaxed_ik = unsafe {
        assert!(!ptr.is_null());
        &mut *ptr
    };
    
    assert!(!pos_goals.is_null(), "Null pointer for pos goals!");
    assert!(!quat_goals.is_null(), "Null pointer for quat goals!");
    assert!(!tolerance.is_null(), "Null pointer for tolerance!"); 

    let pos_slice: &[c_double] = std::slice::from_raw_parts(pos_goals, pos_length as usize);
    let quat_slice: &[c_double] = std::slice::from_raw_parts(quat_goals, quat_length as usize);
    let tolerance_slice: &[c_double] = std::slice::from_raw_parts(tolerance, tolerance_length as usize);

    let pos_vec = pos_slice.to_vec();
    let quat_vec = quat_slice.to_vec();
    let tolerance_vec = tolerance_slice.to_vec();

    let ja = solve_position_helper(relaxed_ik, pos_vec, quat_vec, tolerance_vec);

    let ptr = ja.as_ptr();
    let len = ja.len();
    std::mem::forget(ja);
    Opt {data: ptr, length: len as c_int}
}

fn solve_position_helper(relaxed_ik: &mut RelaxedIK, pos_goals: Vec<f64>, quat_goals: Vec<f64>,
                tolerance: Vec<f64>) -> Vec<f64> {

    for i in 0..relaxed_ik.vars.goal_positions.len()  {
        relaxed_ik.vars.goal_positions[i] = Vector3::new(pos_goals[3*i], pos_goals[3*i+1], pos_goals[3*i+2]);
        relaxed_ik.vars.goal_quats[i] =  UnitQuaternion::from_quaternion(Quaternion::new(quat_goals[4*i+3], quat_goals[4*i], quat_goals[4*i+1], quat_goals[4*i+2]));
        relaxed_ik.vars.tolerances[i] = Vector6::new( 
            tolerance[6*i], tolerance[6*i+1], tolerance[6*i+2],
            tolerance[6*i+3], tolerance[6*i+4], tolerance[6*i+5]
        );
    }
                    
    let x = relaxed_ik.solve();
    return x;
}

fn solve_velocity_helper(relaxed_ik: &mut RelaxedIK, pos_vels: Vec<f64>, rot_vels: Vec<f64>,
    tolerance: Vec<f64>) -> Vec<f64> {

    let mut i = 0;
    for i in 0..relaxed_ik.vars.goal_positions.len()  {
        relaxed_ik.vars.goal_positions[i] += Vector3::new(pos_vels[3*i], pos_vels[3*i+1], pos_vels[3*i+2]);
        let axisangle = Vector3::new(rot_vels[3*i], rot_vels[3*i+1], rot_vels[3*i+2]);
        let tmp_q = UnitQuaternion::from_scaled_axis(axisangle);
        let org_q = relaxed_ik.vars.goal_quats[i].clone();
        relaxed_ik.vars.goal_quats[i] = tmp_q * org_q;
        relaxed_ik.vars.tolerances[i] = Vector6::new(
            tolerance[6*i], tolerance[6*i+1], tolerance[6*i+2],
            tolerance[6*i+3], tolerance[6*i+4], tolerance[6*i+5]
        );
    }

    let x = relaxed_ik.solve();
    return x;
}
