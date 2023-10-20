use crate::groove::objective::*;
use crate::groove::vars::RelaxedIKVars;

pub struct ObjectiveMaster {
    pub objectives: Vec<Box<dyn ObjectiveTrait + Send>>,
    pub num_chains: usize,
    pub weight_priors: Vec<f64>,
    pub lite: bool,
    pub finite_diff_grad: bool
}

impl ObjectiveMaster {
    pub fn standard_ik(num_chains: usize) -> Self {
        let mut objectives: Vec<Box<dyn ObjectiveTrait + Send>> = Vec::new();
        let mut weight_priors: Vec<f64> = Vec::new();
        for i in 0..num_chains {
            objectives.push(Box::new(MatchEEPosGoals::new(i)));
            weight_priors.push(1.0);
            objectives.push(Box::new(MatchEEQuatGoals::new(i)));
            weight_priors.push(1.0);
        }
        Self{objectives, num_chains, weight_priors, lite: true, finite_diff_grad: true}
    }


    pub fn relaxed_ik(chain_indices: &[Vec<usize>]) -> Self {
        let mut objectives: Vec<Box<dyn ObjectiveTrait + Send>> = Vec::new();
        let mut weight_priors: Vec<f64> = Vec::new();
        let num_chains = chain_indices.len();
        for i in 0..num_chains {
            objectives.push(Box::new(MatchEEPosiDoF::new(i, 0)));
            weight_priors.push(50.0);
            objectives.push(Box::new(MatchEEPosiDoF::new(i, 1)));
            weight_priors.push(50.0);
            objectives.push(Box::new(MatchEEPosiDoF::new(i, 2)));
            weight_priors.push(50.0);
            // objectives.push(Box::new(MatchEERotaDoF::new(i, 0)));
            // weight_priors.push(10.0);
            // objectives.push(Box::new(MatchEERotaDoF::new(i, 1)));
            // weight_priors.push(10.0);
            // objectives.push(Box::new(MatchEERotaDoF::new(i, 2)));
            // weight_priors.push(10.0);
            // objectives.push(Box::new(EnvCollision::new(i)));
            // weight_priors.push(1.0);
        }

        let num_dofs = chain_indices.iter().flat_map(|v| v.iter()).cloned().max().unwrap() + 1;
        for j in 0..num_dofs {
            objectives.push(Box::new(EachJointLimits::new(j)));
            weight_priors.push(0.1);
        }

        // objectives.push(Box::new(MinimizeVelocity));
        // weight_priors.push(0.7);
        // objectives.push(Box::new(MinimizeAcceleration));
        // weight_priors.push(0.5);
        // objectives.push(Box::new(MinimizeJerk));
        // weight_priors.push(0.3);
        // objectives.push(Box::new(MaximizeManipulability));
        // weight_priors.push(1.0);

        // Hard code the contact pairs of the shadow hand based on 
        // https://github.com/NVIDIA-Omniverse/IsaacGymEnvs/blob/main/assets/mjcf/open_ai_assets/hand/shared.xml
        // plus some pairs that are not in the above file
        // let contact_pairs: std::collections::HashMap<&str, (usize, usize, usize, usize)> = [
        //     ("rh_ffdistal-rh_thdistal", ((0, 6, 4, 7))),
        //     ("rh_ffmiddle-rh_thdistal", ((0, 5, 4, 7))),
        //     ("rh_ffproximal-rh_thdistal", ((0, 4, 4, 7))),
        //     ("rh_mfproximal-rh_thdistal", ((1, 4, 4, 7))),
        //     ("rh_mfdistal-rh_thdistal", ((1, 6, 4, 7))),
        //     ("rh_rfdistal-rh_thdistal", ((2, 6, 4, 7))),
        //     ("rh_lfdistal-rh_thdistal", ((3, 7, 4, 7))),
        //     ("rh_palm-rh_thdistal", ((2, 2, 4, 7))),
        //     ("rh_mfdistal-rh_ffdistal", ((1, 6, 0, 6))),
        //     ("rh_rfdistal-rh_mfdistal", ((2, 6, 1, 6))),
        //     ("rh_lfdistal-rh_rfdistal", ((3, 7, 2, 6))),
        //     ("rh_mfproximal-rh_ffproximal", ((1, 4, 0, 4))),
        //     ("rh_rfproximal-rh_mfproximal", ((2, 4, 1, 4))),
        //     ("rh_lfproximal-rh_rfproximal", ((3, 5, 2, 4))),
        //     ("rh_lfdistal-rh_rfdistal", ((3, 7, 2, 6))),
        //     ("rh_lfdistal-rh_mfdistal", ((3, 7, 1, 6))),
        //     ("rh_lfdistal-rh_rfmiddle", ((3, 7, 2, 5))),
        //     ("rh_lfmiddle-rh_rfdistal", ((3, 6, 2, 6))),
        //     ("rh_lfmiddle-rh_rfmiddle", ((3, 6, 2, 5))),
        //     ("rh_lfmiddle-rh_rfmiddle", ((3, 6, 2, 5)))
        // ].iter().cloned().collect();

        // for (pair, indices) in &contact_pairs {
        //     println!("Pair: {}, Indices: {:?}", pair, indices);
        //     objectives.push(Box::new(SelfCollision::new(indices.0, indices.2, indices.1-1, indices.3-1)));
        //     weight_priors.push(self_collision_weight);
        // }

        // for i in 0..num_chains {
        //     for j in 0..chain_indices[i].len()-2 {  
        //         for k in j+2..chain_indices[i].len() {
        //             objectives.push(Box::new(SelfCollision::new(i, i, j, k)));
        //             weight_priors.push(self_collision_weight);
        //             pairs.push((chain_indices[i][j], chain_indices[i][k]));
        //         }
        //     }
        // }
        // println!("Num self-collision pairs: {}", pairs.len());

        // let self_collision_weight = 0.01;
        // let mut pairs = Vec::new();

        // for i in 0..num_chains {
        //     let next_chain = (i + 1) % num_chains;
        //     for j in 0..chain_indices[i].len() {  
        //         for k in 0..chain_indices[next_chain].len() {
        //             if !chain_indices[i].contains(&chain_indices[next_chain][k]) && !chain_indices[next_chain].contains(&chain_indices[i][j]) {
        //                 objectives.push(Box::new(SelfCollision::new(i, next_chain, j, k)));
        //                 weight_priors.push(self_collision_weight);
        //                 pairs.push((chain_indices[i][j], chain_indices[next_chain][k]));
        //                 // println!("{} {}", chain_indices[i][ii], chain_indices[j][jj]);
        //             }
        //         }
        //     }
        // }
        // println!("Num self-collision pairs: {}", pairs.len());
        
        Self{objectives, num_chains, weight_priors, lite: false, finite_diff_grad: false}
    }

    pub fn call(&self, x: &[f64], vars: &RelaxedIKVars) -> f64 {
        if self.lite {
            self.__call_lite(x, vars)
        } else {
            self.__call(x, vars)
        }
    }

    pub fn gradient(&self, x: &[f64], vars: &RelaxedIKVars) -> (f64, Vec<f64>) {
        if self.lite {
            if self.finite_diff_grad {
                self.__gradient_finite_diff_lite(x, vars)
            } else {
                self.__gradient_lite(x, vars)
            }
        } else {
            if self.finite_diff_grad {
                self.__gradient_finite_diff(x, vars)
            } else {
                self.__gradient(x, vars)
            }
        }
    }

    pub fn gradient_finite_diff(&self, x: &[f64], vars: &RelaxedIKVars) -> (f64, Vec<f64>) {
        if self.lite {
            self.__gradient_finite_diff_lite(x, vars)
        } else {
            self.__gradient_finite_diff(x, vars)
        }
    }

    fn __call(&self, x: &[f64], vars: &RelaxedIKVars) -> f64 {
        let mut out = 0.0;
        let frames = vars.robot.get_frames_immutable(x);
        for i in 0..self.objectives.len() {
            out += self.weight_priors[i] * self.objectives[i].call(x, vars, &frames);
        }
        out
    }

    fn __call_lite(&self, x: &[f64], vars: &RelaxedIKVars) -> f64 {
        let mut out = 0.0;
        let poses = vars.robot.get_ee_pos_and_quat_immutable(x);
        for i in 0..self.objectives.len() {
            out += self.weight_priors[i] * self.objectives[i].call_lite(x, vars, &poses);
        }
        out
    }

    fn __gradient(&self, x: &[f64], vars: &RelaxedIKVars) -> (f64, Vec<f64>) {
        let mut grad: Vec<f64> = vec![0. ; x.len()];
        let mut obj = 0.0;

        let mut finite_diff_list: Vec<usize> = Vec::new();
        let mut f_0s: Vec<f64> = Vec::new();
        let frames_0 = vars.robot.get_frames_immutable(x);
        for i in 0..self.objectives.len() {
            if self.objectives[i].gradient_type() == 0 {
                let (local_obj, local_grad) = self.objectives[i].gradient(x, vars, &frames_0);
                f_0s.push(local_obj);
                obj += self.weight_priors[i] * local_obj;
                for j in 0..local_grad.len() {
                    grad[j] += self.weight_priors[i] * local_grad[j];
                }
            } else if self.objectives[i].gradient_type() == 1 {
                finite_diff_list.push(i);
                let local_obj = self.objectives[i].call(x, vars, &frames_0);
                obj += self.weight_priors[i] * local_obj;
                f_0s.push(local_obj);
            }
        }

        if finite_diff_list.len() > 0 {
            for i in 0..x.len() {
                let mut x_h = x.to_vec();
                x_h[i] += 0.0000001;
                let frames_h = vars.robot.get_frames_immutable(x_h.as_slice());
                for j in &finite_diff_list {
                    let f_h = self.objectives[*j].call(&x_h, vars, &frames_h);
                    grad[i] += self.weight_priors[*j] * ((-f_0s[*j] + f_h) /  0.0000001);
                }
            }
        }

        (obj, grad)
    }

    fn __gradient_lite(&self, x: &[f64], vars: &RelaxedIKVars) -> (f64, Vec<f64>) {
        let mut grad: Vec<f64> = vec![0. ; x.len()];
        let mut obj = 0.0;

        let mut finite_diff_list: Vec<usize> = Vec::new();
        let mut f_0s: Vec<f64> = Vec::new();
        let poses_0 = vars.robot.get_ee_pos_and_quat_immutable(x);
        for i in 0..self.objectives.len() {
            if self.objectives[i].gradient_type() == 1 {
                let (local_obj, local_grad) = self.objectives[i].gradient_lite(x, vars, &poses_0);
                f_0s.push(local_obj);
                obj += self.weight_priors[i] * local_obj;
                for j in 0..local_grad.len() {
                    grad[j] += self.weight_priors[i] * local_grad[j];
                }
            } else if self.objectives[i].gradient_type() == 0 {
                finite_diff_list.push(i);
                let local_obj = self.objectives[i].call_lite(x, vars, &poses_0);
                obj += self.weight_priors[i] * local_obj;
                f_0s.push(local_obj);
            }
        }

        if finite_diff_list.len() > 0 {
            for i in 0..x.len() {
                let mut x_h = x.to_vec();
                x_h[i] += 0.0000001;
                let poses_h = vars.robot.get_ee_pos_and_quat_immutable(x_h.as_slice());
                for j in &finite_diff_list {
                    let f_h = self.objectives[*j].call_lite(x, vars, &poses_h);
                    grad[i] += self.weight_priors[*j] * ((-f_0s[*j] + f_h) /  0.0000001);
                }
            }
        }

        (obj, grad)
    }

    fn __gradient_finite_diff(&self, x: &[f64], vars: &RelaxedIKVars) -> (f64, Vec<f64>)  {
        let mut grad: Vec<f64> = vec![0. ; x.len()];
        let mut f_0 = self.call(x, vars);

        for i in 0..x.len() {
            let mut x_h = x.to_vec();
            x_h[i] += 0.000001;
            let f_h = self.call(x_h.as_slice(), vars);
            grad[i] = (-f_0 + f_h) / 0.000001;
        }

        (f_0, grad)
    }

    fn __gradient_finite_diff_lite(&self, x: &[f64], vars: &RelaxedIKVars) -> (f64, Vec<f64>) {
        let mut grad: Vec<f64> = vec![0. ; x.len()];
        let mut f_0 = self.call(x, vars);

        for i in 0..x.len() {
            let mut x_h = x.to_vec();
            x_h[i] += 0.000001;
            let f_h = self.__call_lite(x_h.as_slice(), vars);
            grad[i] = (-f_0 + f_h) / 0.000001;
        }

        (f_0, grad)
    }
}