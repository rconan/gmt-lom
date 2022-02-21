use gmt_lom::{OpticalSensitivities, OpticalSensitivity};
use serde_generate::SourceInstaller;
use serde_reflection::{Tracer, TracerConfig};
use std::path::Path;

fn main() {
    // Start the tracing session.
    let mut tracer = Tracer::new(TracerConfig::default());

    // Trace the desired top-level type(s).
    tracer.trace_simple_type::<OpticalSensitivities>().unwrap();

    // Also trace each enum type separately to fix any `MissingVariants` error.
    tracer.trace_simple_type::<OpticalSensitivity>().unwrap();

    // Obtain the registry of Serde formats and serialize it in YAML (for instance).
    let registry = tracer.registry().unwrap();
    let data = serde_yaml::to_string(&registry).unwrap();
    println!("{data}");

    // Create Python class definitions.
    let mut source = Vec::new();
    let config = serde_generate::CodeGeneratorConfig::new("optical_sensitivities".to_string())
        .with_encodings(vec![serde_generate::Encoding::Bincode]);
    let generator = serde_generate::python3::CodeGenerator::new(&config);
    generator.output(&mut source, &registry).unwrap();

    let path = Path::new("lompy");
    let install = serde_generate::python3::Installer::new(path.to_path_buf(), None);
    install.install_module(&config, &registry).unwrap();
    install.install_bincode_runtime().unwrap();
    install.install_serde_runtime().unwrap();
}
