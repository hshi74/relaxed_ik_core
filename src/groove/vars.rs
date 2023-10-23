use nalgebra::{UnitQuaternion, Vector3, Vector6, Quaternion, Point3};
use crate::spacetime::robot::Robot;
use crate::utils_rust::file_utils::{*};
use time::PreciseTime;
use std::ops::Deref;
use yaml_rust::{YamlLoader, Yaml};
use std::fs::File;
use std::io::prelude::*;

use wasm_bindgen::prelude::*;
use serde::{Serialize, Deserialize};

#[derive(Serialize, Deserialize)]
pub struct VarsConstructorData {
    // pub urdf: String,
    pub link_radius:f64,
    pub base_links: Vec<String>,
    pub ee_links: Vec<String>,
    pub joint_ordering: Vec<String>,
    starting_config: Vec<f64>,
    pub ee_only: bool
}

pub struct RelaxedIKVars {
    pub robot: Robot,
    pub init_state: Vec<f64>,
    pub xopt: Vec<f64>,
    pub prev_state: Vec<f64>,
    pub prev_state2: Vec<f64>,
    pub prev_state3: Vec<f64>,
    pub goal_positions: Vec<Vector3<f64>>,
    pub goal_quats: Vec<UnitQuaternion<f64>>,
    pub tolerances: Vec<Vector6<f64>>,
    pub init_ee_positions: Vec<Vector3<f64>>,
    pub init_ee_quats: Vec<UnitQuaternion<f64>>,
    pub ee_only: bool,
    pub valid_chains: Vec<usize>
}
impl RelaxedIKVars {
    pub fn from_local_settings(path_to_setting: &str) -> Self {
        let path_to_src = get_path_to_src();
        let mut file = File::open(path_to_setting).unwrap();
        let mut contents = String::new();
        let res = file.read_to_string(&mut contents).unwrap();
        let docs = YamlLoader::load_from_str(contents.as_str()).unwrap();
        let settings = &docs[0];

        let path_to_urdf = path_to_src + "configs/urdfs/" + settings["urdf"].as_str().unwrap();
        println!("RelaxedIK is using below URDF file: {}", path_to_urdf);
        let chain = k::Chain::<f64>::from_urdf_file(path_to_urdf.clone()).unwrap();

        let base_links_arr = settings["base_links"].as_vec().unwrap();
        let ee_links_arr = settings["ee_links"].as_vec().unwrap();
        let num_chains = base_links_arr.len();

        let mut base_links: Vec<String> = Vec::new();
        let mut ee_links = Vec::new();
        for i in 0..num_chains {
            base_links.push(base_links_arr[i].as_str().unwrap().to_string());
            ee_links.push(ee_links_arr[i].as_str().unwrap().to_string());
        }

        let joint_ordering = if let Some(joint_ordering_arr) = settings["joint_ordering"].as_vec() {
            if joint_ordering_arr.is_empty() {
                None
            } else {
                Some(joint_ordering_arr.iter().filter_map(|item| item.as_str()).map(String::from).collect::<Vec<String>>())
            }
        } else {
            None
        };

        let urdf = &std::fs::read_to_string(path_to_urdf).unwrap();
        let robot = Robot::from_urdf(urdf, &base_links, &ee_links, joint_ordering);

        let mut starting_config = Vec::new();
        if settings["starting_config"].is_badvalue() {
            println!("No starting config provided, using all zeros");
            for i in 0..robot.num_dofs {
                starting_config.push(0.0);
            }
        } else {
            let starting_config_arr = settings["starting_config"].as_vec().unwrap();
            for i in 0..starting_config_arr.len() {
                starting_config.push(starting_config_arr[i].as_f64().unwrap());
            }
        }

        let mut init_ee_positions: Vec<Vector3<f64>> = Vec::new();
        let mut init_ee_quats: Vec<UnitQuaternion<f64>> = Vec::new();
        let pose = robot.get_ee_pos_and_quat_immutable(&starting_config);
        assert!(pose.len() == num_chains);

        for i in 0..pose.len() {
            init_ee_positions.push(pose[i].0);
            init_ee_quats.push(pose[i].1);
        }

        let ee_only = true;
        let valid_chains: Vec<usize> = (0..num_chains).collect();
        
        let mut goal_positions: Vec<Vector3<f64>> = Vec::new();
        let mut goal_quats: Vec<UnitQuaternion<f64>> = Vec::new();
        let mut tolerances: Vec<Vector6<f64>> = Vec::new();
        for i in 0..pose.len() {
            goal_positions.push(pose[i].0);
            goal_quats.push(pose[i].1);
            tolerances.push(Vector6::new(0., 0., 0., 0., 0., 0.));
        }

        RelaxedIKVars{robot, init_state: starting_config.clone(), xopt: starting_config.clone(),
            prev_state: starting_config.clone(), prev_state2: starting_config.clone(), prev_state3: starting_config.clone(),
            goal_positions: goal_positions, goal_quats: goal_quats, tolerances: tolerances, 
            init_ee_positions, init_ee_quats, ee_only,valid_chains}
    }
    
    // for webassembly
    pub fn from_jsvalue( configs: VarsConstructorData, urdf: &str) -> Self  {

        let num_chains = configs.base_links.len();

        let mut tolerances: Vec<Vector6<f64>> = Vec::new();
        for i in 0..num_chains {
            tolerances.push(Vector6::new(0., 0., 0., 0., 0., 0.));
        }

        let robot = Robot::from_urdf(urdf, &configs.base_links, &configs.ee_links, Some(configs.joint_ordering));

        let starting_config = configs.starting_config;

        let mut init_ee_positions: Vec<Vector3<f64>> = Vec::new();
        let mut init_ee_quats: Vec<UnitQuaternion<f64>> = Vec::new();
        let pose = robot.get_ee_pos_and_quat_immutable(&starting_config);
        assert!(pose.len() == num_chains);

        for i in 0..pose.len() {
            init_ee_positions.push(pose[i].0);
            init_ee_quats.push(pose[i].1);
        }
        
        let ee_only = true;
        let valid_chains: Vec<usize> = (0..num_chains).collect();
        
        let mut goal_positions: Vec<Vector3<f64>> = Vec::new();
        let mut goal_quats: Vec<UnitQuaternion<f64>> = Vec::new();
        let mut tolerances: Vec<Vector6<f64>> = Vec::new();
        for i in 0..pose.len() {
            goal_positions.push(pose[i].0);
            goal_quats.push(pose[i].1);
            tolerances.push(Vector6::new(0., 0., 0., 0., 0., 0.));
        }

        RelaxedIKVars{robot, init_state: starting_config.clone(), xopt: starting_config.clone(),
            prev_state: starting_config.clone(), prev_state2: starting_config.clone(), prev_state3: starting_config.clone(),
            goal_positions: goal_positions, goal_quats: goal_quats, tolerances: tolerances, 
            init_ee_positions, init_ee_quats, ee_only, valid_chains}
    }

    pub fn update(&mut self, xopt: Vec<f64>) {
        self.prev_state3 = self.prev_state2.clone();
        self.prev_state2 = self.prev_state.clone();
        self.prev_state = self.xopt.clone();
        self.xopt = xopt.clone();
    }

    pub fn reset(&mut self, prev_state3: Vec<f64>, prev_state2: Vec<f64>, prev_state: Vec<f64>, init_state: Vec<f64>) {
        self.prev_state3 = prev_state3.clone();
        self.prev_state2 = prev_state2.clone();
        self.prev_state = prev_state.clone();
        self.xopt = init_state.clone();
        self.init_state = init_state.clone();

        let mut init_ee_positions: Vec<Vector3<f64>> = Vec::new();
        let mut init_ee_quats: Vec<UnitQuaternion<f64>> = Vec::new();
        let pose = self.robot.get_ee_pos_and_quat_immutable(&init_state);

        for i in 0..pose.len() {
            init_ee_positions.push(pose[i].0);
            init_ee_quats.push(pose[i].1);
        }

        self.init_ee_positions = init_ee_positions.clone();
        self.init_ee_quats = init_ee_quats.clone();
    }

    pub fn set_ee_only(&mut self, ee_only: bool) {
        self.ee_only = ee_only;

        let pose = self.robot.get_ee_pos_and_quat_immutable(&self.init_state);
        let mut goal_positions: Vec<Vector3<f64>> = Vec::new();
        let mut goal_quats: Vec<UnitQuaternion<f64>> = Vec::new();
        let mut tolerances: Vec<Vector6<f64>> = Vec::new();
        if ee_only {
            for i in 0..pose.len() {
                goal_positions.push(pose[i].0);
                goal_quats.push(pose[i].1);
                tolerances.push(Vector6::new(0., 0., 0., 0., 0., 0.));
            }
        }
        else {
            for i in 0..self.robot.num_dofs {
                goal_positions.push(Vector3::new(0., 0., 0.));
                goal_quats.push(UnitQuaternion::identity());
                tolerances.push(Vector6::new(0., 0., 0., 0., 0., 0.));
            }
        }

        self.goal_positions = goal_positions;
        self.goal_quats = goal_quats;
        self.tolerances = tolerances;
    }

    pub fn set_valid_chains(&mut self, valid_chains: &[usize]) {
        self.valid_chains = valid_chains.to_vec();
    }
}
