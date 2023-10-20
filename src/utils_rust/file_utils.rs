use std::path::{Path, PathBuf};

pub fn get_path_to_src() -> String {
    // Getting the directory of the Cargo.toml (crate root)
    let crate_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    
    // Getting the full path to the current file
    let absolute_file_path = crate_dir.join(file!());

    // Navigate up two directories from the file's location to get to relaxed_ik_core
    let target_dir = absolute_file_path.parent().unwrap().parent().unwrap().parent().unwrap();
                               
    target_dir.to_str().unwrap().to_string() + "/"
}