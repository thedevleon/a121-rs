//! Presence Detection Module
//!
//! Provides an API for presence detection using radar technology. This module includes
//! functionality to create and configure presence detectors, prepare for
//! measurements, detect presence, and more.
//!
//! For a detailed description of the algorithm and its parameters, see the Acconeer documentation.

#![warn(missing_docs)]

use crate::config::profile::RadarProfile;
use crate::config::RadarIdleState;
use a121_sys::*;
use core::ops::RangeInclusive;

/// Type alias for the signal quality
pub type SignalQuality = f32;

/// Configuration for the radar presence detection.
pub struct PresenceConfig {
    /// Pointer to the inner presence detector configuration.
    pub inner: *mut acc_detector_presence_config,
}

impl Drop for PresenceConfig {
    fn drop(&mut self) {
        unsafe { acc_detector_presence_config_destroy(self.inner) }
    }
}

impl Default for PresenceConfig {
    fn default() -> Self {
        Self {
            inner: unsafe { acc_detector_presence_config_create() },
        }
    }
}

impl PresenceConfig {
    /// Sets the sensor ID.
    pub fn sensor_set(&mut self, sensor_id: u32) {
        unsafe { acc_detector_presence_config_sensor_set(self.inner, sensor_id) }
    }

    /// Sets the measurement range in meters.
    pub fn set_range(&mut self, range: RangeInclusive<f32>) {
        unsafe {
            acc_detector_presence_config_start_set(self.inner, *range.start());
            acc_detector_presence_config_end_set(self.inner, *range.end());
        }
    }

    /// Set automatic subsweeps
    pub fn set_automatic_subsweeps(&mut self, enable: bool) {
        unsafe { acc_detector_presence_config_automatic_subsweeps_set(self.inner, enable) }
    }

    /// Set signal quality
    pub fn set_signal_quality(&mut self, signal_quality: SignalQuality) {
        unsafe { acc_detector_presence_config_signal_quality_set(self.inner, signal_quality) }
    }

    /// Sets the inter-frame idle state.
    pub fn set_inter_frame_idle_state(&mut self, idle_state: RadarIdleState) {
        unsafe {
            acc_detector_presence_config_inter_frame_idle_state_set(self.inner, idle_state as u32)
        }
    }

    /// Sets the number of sweeps per frame.
    pub fn set_sweeps_per_frame(&mut self, sweeps: u16) {
        unsafe { acc_detector_presence_config_sweeps_per_frame_set(self.inner, sweeps) }
    }

    /// Sets the frame rate.
    pub fn set_frame_rate(&mut self, frame_rate: f32) {
        unsafe { acc_detector_presence_config_frame_rate_set(self.inner, frame_rate) }
    }

    /// Sets whether the frame rate is application-driven.
    pub fn set_frame_rate_app_driven(&mut self, app_driven: bool) {
        unsafe { acc_detector_presence_config_frame_rate_app_driven_set(self.inner, app_driven) }
    }

    /// Sets whether to reset filters on prepare.
    pub fn set_reset_filters_on_prepare(&mut self, reset: bool) {
        unsafe { acc_detector_presence_config_reset_filters_on_prepare_set(self.inner, reset) }
    }

    /// Sets whether intra-detection is enabled.
    pub fn set_intra_detection(&mut self, enabled: bool) {
        unsafe { acc_detector_presence_config_intra_detection_set(self.inner, enabled) }
    }

    /// Sets the intra-detection threshold.
    pub fn set_intra_detection_threshold(&mut self, threshold: f32) {
        unsafe { acc_detector_presence_config_intra_detection_threshold_set(self.inner, threshold) }
    }

    /// Sets the intra-frame time constant.
    pub fn set_intra_frame_time_const(&mut self, time_const: f32) {
        unsafe { acc_detector_presence_config_intra_frame_time_const_set(self.inner, time_const) }
    }

    /// Sets the intra-output time constant.
    pub fn set_intra_output_time_const(&mut self, time_const: f32) {
        unsafe { acc_detector_presence_config_intra_output_time_const_set(self.inner, time_const) }
    }

    /// Sets whether inter-detection is enabled.
    pub fn set_inter_detection(&mut self, enabled: bool) {
        unsafe { acc_detector_presence_config_inter_detection_set(self.inner, enabled) }
    }

    /// Sets the inter-detection threshold.
    pub fn set_inter_detection_threshold(&mut self, threshold: f32) {
        unsafe { acc_detector_presence_config_inter_detection_threshold_set(self.inner, threshold) }
    }

    /// Sets the inter-frame deviation time constant.
    pub fn set_inter_frame_deviation_time_const(&mut self, time_const: f32) {
        unsafe {
            acc_detector_presence_config_inter_frame_deviation_time_const_set(
                self.inner, time_const,
            )
        }
    }

    /// Sets the inter-frame fast cutoff.
    pub fn set_inter_frame_fast_cutoff(&mut self, cutoff: f32) {
        unsafe { acc_detector_presence_config_inter_frame_fast_cutoff_set(self.inner, cutoff) }
    }

    /// Sets the inter-frame slow cutoff.
    pub fn set_inter_frame_slow_cutoff(&mut self, cutoff: f32) {
        unsafe { acc_detector_presence_config_inter_frame_slow_cutoff_set(self.inner, cutoff) }
    }

    /// Sets the inter-output time constant.
    pub fn set_inter_output_time_const(&mut self, time_const: f32) {
        unsafe { acc_detector_presence_config_inter_output_time_const_set(self.inner, time_const) }
    }

    /// Sets the inter-frame presence timeout.
    pub fn set_inter_frame_presence_timeout(&mut self, timeout: u16) {
        unsafe {
            acc_detector_presence_config_inter_frame_presence_timeout_set(self.inner, timeout)
        }
    }

    /// Sets whether inter-phase boost is enabled.
    pub fn set_inter_phase_boost(&mut self, enabled: bool) {
        unsafe { acc_detector_presence_config_inter_phase_boost_set(self.inner, enabled) }
    }

    /// Set the auto step length
    pub fn set_auto_step_length(&mut self, enable: bool) {
        unsafe { acc_detector_presence_config_auto_step_length_set(self.inner, enable) }
    }

    /// Enables or disables automatic profile selection.
    pub fn set_auto_profile(&mut self, enable: bool) {
        unsafe { acc_detector_presence_config_auto_profile_set(self.inner, enable) }
    }

    /// Sets the profile for presence detection.
    pub fn profile_set(&mut self, profile: RadarProfile) {
        unsafe { acc_detector_presence_config_profile_set(self.inner, profile as u32) }
    }

    /// Set hwaas
    pub fn set_hwaas(&mut self, hwaas: u16) {
        unsafe { acc_detector_presence_config_hwaas_set(self.inner, hwaas) }
    }

    /// Print out the complete configuration.
    pub fn log_config(&mut self) {
        unsafe {
            acc_detector_presence_config_log(self.inner);
        }
    }

    // Presets
    /// Short Range Preset as found in C Sample (example_detector_presence.c)
    pub fn preset_short_range(config: &mut PresenceConfig) {
        config.set_range(0.06..=1.0);
        config.set_automatic_subsweeps(true);
        config.set_signal_quality(30.0);
        config.set_inter_frame_idle_state(RadarIdleState::Ready);
        config.set_sweeps_per_frame(16);

        config.set_frame_rate(10.0);
        config.set_frame_rate_app_driven(false);
        config.set_reset_filters_on_prepare(true);

        config.set_intra_detection(true);
        config.set_intra_detection_threshold(1.4);
        config.set_intra_frame_time_const(0.15);
        config.set_intra_output_time_const(0.3);

        config.set_inter_detection(true);
        config.set_inter_detection_threshold(1.0);
        config.set_inter_frame_deviation_time_const(0.5);
        config.set_inter_frame_fast_cutoff(5.0);
        config.set_inter_frame_slow_cutoff(0.2);
        config.set_inter_output_time_const(2.0);
        config.set_inter_frame_presence_timeout(3);
        config.set_inter_phase_boost(false);
    }

    /// Medium Range Preset as found in C Sample (example_detector_presence.c)
    pub fn preset_medium_range(config: &mut PresenceConfig) {
        config.set_range(0.3..=2.5);
        config.set_automatic_subsweeps(true);
        config.set_signal_quality(30.0);
        config.set_inter_frame_idle_state(RadarIdleState::Ready);
        config.set_sweeps_per_frame(16);

        config.set_frame_rate(10.0);
        config.set_frame_rate_app_driven(false);
        config.set_reset_filters_on_prepare(true);

        config.set_intra_detection(true);
        config.set_intra_detection_threshold(1.3);
        config.set_intra_frame_time_const(0.15);
        config.set_intra_output_time_const(0.3);

        config.set_inter_detection(true);
        config.set_inter_detection_threshold(1.0);
        config.set_inter_frame_deviation_time_const(0.5);
        config.set_inter_frame_fast_cutoff(6.0);
        config.set_inter_frame_slow_cutoff(0.2);
        config.set_inter_output_time_const(2.0);
        config.set_inter_frame_presence_timeout(3);
        config.set_inter_phase_boost(false);
    }

    /// Long Range Preset as found in C Sample (example_detector_presence.c)
    pub fn preset_long_range(config: &mut PresenceConfig) {
        config.set_range(5.0..=7.5);
        config.set_automatic_subsweeps(true);
        config.set_signal_quality(10.0);
        config.set_inter_frame_idle_state(RadarIdleState::Ready);
        config.set_sweeps_per_frame(16);

        config.set_frame_rate(12.0);
        config.set_frame_rate_app_driven(false);
        config.set_reset_filters_on_prepare(true);

        config.set_intra_detection(true);
        config.set_intra_detection_threshold(1.2);
        config.set_intra_frame_time_const(0.15);
        config.set_intra_output_time_const(0.3);

        config.set_inter_detection(true);
        config.set_inter_detection_threshold(0.8);
        config.set_inter_frame_deviation_time_const(0.5);
        config.set_inter_frame_fast_cutoff(6.0);
        config.set_inter_frame_slow_cutoff(0.2);
        config.set_inter_output_time_const(2.0);
        config.set_inter_frame_presence_timeout(3);
        config.set_inter_phase_boost(false);
    }

    /// Preset for a ceiling mounted radar.
    pub fn preset_ceiling(config: &mut PresenceConfig) {
        config.set_range(4.0..=7.0);
        config.set_auto_profile(true);
        config.set_auto_step_length(true);
        config.set_sweeps_per_frame(16);
        config.set_hwaas(32);

        config.set_frame_rate(5.0);
        config.set_inter_frame_idle_state(RadarIdleState::Ready);

        config.set_intra_detection(true);
        config.set_intra_detection_threshold(0.13);
        config.set_intra_frame_time_const(0.15);
        config.set_intra_output_time_const(0.3);

        config.set_inter_detection(true);
        config.set_inter_detection_threshold(1.0);
        config.set_inter_frame_fast_cutoff(6.0);
        config.set_inter_frame_slow_cutoff(0.2);
        config.set_inter_frame_deviation_time_const(0.5);
        config.set_inter_output_time_const(2.0);
        config.set_inter_phase_boost(true);
        config.set_inter_frame_presence_timeout(10);
    }
}
