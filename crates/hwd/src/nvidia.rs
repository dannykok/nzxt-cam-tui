use nvml_wrapper::{Nvml, enum_wrappers::device::TemperatureSensor};

/// The only injectable boundary in host telemetry. Production uses NVML; tests
/// provide deterministic outcomes without loading the NVML shared library.
pub(crate) trait NvidiaSource: Send {
    fn temperature_celsius(&mut self, uuid: &str, expected_pci_bus_id: Option<&str>)
    -> Option<f64>;
}

/// Lazily owns an NVML handle. Construction does not load a library or query a
/// device. Any operational failure drops the handle so a later poll can retry
/// initialization after driver/library/device recovery.
pub(crate) struct NvmlSource {
    nvml: Option<Nvml>,
}

impl NvmlSource {
    pub(crate) const fn new() -> Self {
        Self { nvml: None }
    }

    fn try_temperature(
        &mut self,
        uuid: &str,
        expected_pci_bus_id: Option<&str>,
    ) -> Result<f64, ()> {
        if self.nvml.is_none() {
            self.nvml = Some(Nvml::init().map_err(|_| ())?);
        }

        let nvml = self.nvml.as_ref().ok_or(())?;
        // UUID is deliberately the sole device selector. NVML indices and names
        // are unstable and must never be used here.
        let device = nvml.device_by_uuid(uuid).map_err(|_| ())?;
        if let Some(expected) = expected_pci_bus_id {
            let actual = device.pci_info().map_err(|_| ())?.bus_id;
            if actual != expected {
                return Err(());
            }
        }
        device
            .temperature(TemperatureSensor::Gpu)
            .map(f64::from)
            .map_err(|_| ())
    }
}

impl NvidiaSource for NvmlSource {
    fn temperature_celsius(
        &mut self,
        uuid: &str,
        expected_pci_bus_id: Option<&str>,
    ) -> Option<f64> {
        match self.try_temperature(uuid, expected_pci_bus_id) {
            Ok(temperature) => Some(temperature),
            Err(()) => {
                self.nvml = None;
                None
            }
        }
    }
}
