
use crate::groove::vars::{RelaxedIKVars, VarsConstructorData};
use crate::groove::groove::{OptimizationEngineOpen};
use crate::groove::objective_master::ObjectiveMaster;
use crate::utils_rust::transformations::{*};
use wasm_bindgen::prelude::*;
use js_sys::Array;
extern crate serde_json;
use web_sys;
extern crate console_error_panic_hook;
use nalgebra::{UnitQuaternion, Vector3, Vector6, Quaternion, Point3};

#[wasm_bindgen]
pub struct RelaxedIK {
    pub(crate) vars: RelaxedIKVars,
    pub(crate) om: ObjectiveMaster,
    pub(crate) groove: OptimizationEngineOpen
}

#[wasm_bindgen]
impl RelaxedIK {
    #[wasm_bindgen(constructor)]
    pub fn new( configs:  JsValue, urdf: String) -> Self {
        console_error_panic_hook::set_once();

        let cfg: VarsConstructorData = serde_wasm_bindgen::from_value(configs).unwrap();

        let vars = RelaxedIKVars::from_jsvalue(cfg, &urdf);

        let om = ObjectiveMaster::relaxed_ik(&vars.robot.chain_indices, vars.ee_only);
        let groove = OptimizationEngineOpen::new(vars.robot.num_dofs.clone());
        Self{vars, om, groove}
    }

    pub fn reset(&mut self, init_state:  JsValue) {
        let starting_config = if init_state.is_null() || init_state.is_undefined() {
            self.vars.init_state.clone()
        } else {
            let tmp: Vec<f64> = serde_wasm_bindgen::from_value(init_state).unwrap();
            if tmp.len() != self.vars.robot.num_dofs {
                self.vars.init_state.clone()
            } else {
                tmp
            }
        };

        self.vars.reset( starting_config.clone());
    }

    pub fn solve_position(&mut self, pos_goal:  JsValue,  quat_goal:  JsValue, tolerance: JsValue) -> Array{
        self.solve_position_helper(pos_goal, quat_goal, tolerance, false)
    }

    pub fn solve_position_relative(&mut self, pos_goal:  JsValue,  quat_goal:  JsValue, tolerance: JsValue) -> Array{
        self.solve_position_helper(pos_goal, quat_goal, tolerance, true)
    }

    pub fn solve(&mut self, pos_goal:  JsValue,  quat_goal:  JsValue) -> Array{
        self.solve_position_relative(pos_goal, quat_goal, JsValue::undefined())
    }
}

impl RelaxedIK {

    pub fn solve_position_helper(&mut self, pos_goal:  JsValue,  quat_goal:  JsValue, tolerance: JsValue, relative: bool) -> Array{

        let pos_vec: Vec<f64> = serde_wasm_bindgen::from_value(pos_goal).unwrap();
        let quat_vec: Vec<f64> = serde_wasm_bindgen::from_value(quat_goal).unwrap();

        let tole_vec = if tolerance.is_null() || tolerance.is_undefined() {
            vec![0.0; self.vars.robot.num_chains * 6]
        } else {
            serde_wasm_bindgen::from_value(tolerance).unwrap()
        };

        let mut ctr = 0;
        for i in 0..self.vars.robot.num_chains  {
            for j in 0..self.vars.robot.chain_indices[i].len() {
                if self.vars.ee_only && j > 0 { continue;}
                let pos = Vector3::new(pos_vec[3*ctr], pos_vec[3*ctr+1], pos_vec[3*ctr+2]);
                let tmp_q = Quaternion::new(quat_vec[4*ctr+3], quat_vec[4*ctr], quat_vec[4*ctr+1], quat_vec[4*ctr+2]);
                let quat =  UnitQuaternion::from_quaternion(tmp_q);
                let tole = Vector6::new( 
                    tole_vec[6*ctr], tole_vec[6*ctr+1], tole_vec[6*ctr+2],
                    tole_vec[6*ctr+3], tole_vec[6*ctr+4], tole_vec[6*ctr+5]
                );
                if relative {
                    self.vars.goal_positions[i][j] = self.vars.init_ee_positions[i] + pos;
                    self.vars.goal_quats[i][j] = quat * self.vars.init_ee_quats[i];
                } else {
                    self.vars.goal_positions[i][j] = pos.clone();
                    self.vars.goal_quats[i][j] = quat.clone();
                }
                self.vars.tolerances[i][j] = tole.clone();
                ctr += 1;
            }
        }

        let mut out_x = self.vars.xopt.clone();
        self.groove.optimize(&mut out_x, &self.vars, &self.om, 100);
        self.vars.update(out_x.clone());

        out_x.into_iter().map(JsValue::from).collect()
    }

}