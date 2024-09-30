#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

extern crate alloc;

use alloc::vec;
use core::mem::MaybeUninit;
use embassy_executor::Spawner;
use embassy_time::{Delay, Instant};
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    gpio::{Input, Io, Level, Output, Pull, NO_PIN},
    peripherals::Peripherals,
    prelude::*,
    spi::{master::Spi, SpiMode},
    system::SystemControl,
    timer::timg::TimerGroup,
};
mod mulsc3;
mod spi_adapter;
use a121_rs::config::profile::RadarProfile::AccProfile5;
use a121_rs::detector::distance::{config::*, RadarDistanceDetector};
use a121_rs::radar::Radar;

extern crate tinyrlibc; // this provides malloc and free via the global allocator

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    init_heap();
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    esp_hal_embassy::init(&clocks, timg0.timer0);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    // setup logger
    // To change the log_level change the env section in .cargo/config.toml
    // or remove it and set ESP_LOGLEVEL manually before running cargo run
    // this requires a clean rebuild because of https://github.com/rust-lang/cargo/issues/10358
    esp_println::logger::init_logger_from_env();

    log::info!("A121 library version: {}", a121_rs::radar::rss_version());

    let radar_en = Output::new(io.pins.gpio22, Level::Low);
    let radar_int = Input::new(io.pins.gpio23, Pull::Down);

    let sclk = io.pins.gpio18;
    let miso = io.pins.gpio10;
    let mosi = io.pins.gpio5;
    let cs = io.pins.gpio2;

    let cs = Output::new(cs, Level::High);

    let spi_bus = Spi::new(peripherals.SPI2, 1u32.MHz(), SpiMode::Mode0, &clocks).with_pins(Some(sclk), Some(mosi), Some(miso), NO_PIN);
    let spi_device = ExclusiveDevice::new(spi_bus, cs, Delay).expect("SPI device initialize error");
    let spi_adater = spi_adapter::SpiAdapter::new(spi_device);
    let spi_adapter = static_cell::make_static!(spi_adater);

    let mut radar = Radar::new(1, spi_adapter, radar_int, radar_en, Delay).await;

    log::info!("Radar enabled.");
    log::info!("Starting calibration...");
    let mut calibration = radar.calibrate().await.unwrap();
    let mut radar = radar.prepare_sensor(&mut calibration).unwrap();
    log::info!("Radar calibrated and prepared.");

    let mut distance_config = RadarDistanceConfig::default();
    distance_config.set_interval(0.2..=3.0);
    distance_config.set_max_step_length(MaxStepLenght::ProfileBased);
    distance_config.set_max_profile(AccProfile5);
    distance_config.set_reflector_shape(ReflectorShape::Generic);
    distance_config.set_peak_sorting_method(PeakSortingMethod::Strength);
    distance_config.set_threshold_method(ThresholdMethod::Cfar);
    distance_config.set_threshold_sensitivity(0.5);
    distance_config.set_signal_quality(15.0);
    distance_config.set_close_range_leakage_cancelation(false);

    let mut distance = RadarDistanceDetector::with_config(&mut radar, distance_config);
    let mut buffer = vec![0u8; distance.get_distance_buffer_size()];
    let mut static_cal_result = vec![0u8; distance.get_static_result_buffer_size()];
    log::info!("Starting detector calibration...");
    let mut dynamic_cal_result = distance
        .calibrate_detector(&calibration, &mut buffer, &mut static_cal_result)
        .await
        .unwrap();

    let mut frames = 0;
    let mut measurements = 0;
    let mut distances = 0;
    let mut last_print = Instant::now();

    loop {
        distance
            .prepare_detector(&calibration, &mut buffer)
            .unwrap();
        distance.measure(&mut buffer).await.unwrap();

        match distance.process_data(&mut buffer, &mut static_cal_result, &mut dynamic_cal_result) {
            Ok(res) => {
                frames += 1;
                if res.num_distances() > 0 {
                    measurements += 1;
                    distances += res.num_distances();
                    log::info!(
                        "{} Distances found:\n{:?}",
                        res.num_distances(),
                        res.distances()
                    );
                }
                if res.calibration_needed() {
                    log::info!("Calibration needed.");
                    let calibration = distance.calibrate().await.unwrap();
                    dynamic_cal_result = distance
                        .update_calibration(&calibration, &mut buffer)
                        .await
                        .unwrap();
                }
            }
            Err(_) => log::error!("Failed to process data."),
        }

        if Instant::now() - last_print >= embassy_time::Duration::from_secs(1) {
            log::info!(
                "[Measurement frames]:[Frames with at least 1 distance]:[Total distances] per second: \n {}:{}:{}",
                frames, measurements, distances
            );
            frames = 0;
            measurements = 0;
            distances = 0;
            last_print = Instant::now();
        }
    }
}
