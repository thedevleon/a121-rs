use core::ffi::{c_char, c_void, CStr};
use core::mem::MaybeUninit;
use core::marker::PhantomData;
use embedded_hal::spi::SpiDevice;
use a121_sys::{acc_hal_a121_t, acc_hal_optimization_t, acc_rss_hal_register, acc_sensor_id_t};

/// Global instance of a Mutex, wrapping a raw C pointer contains a mutable reference to a `SpiBus`.
///
/// `SPI_INSTANCE` is used to store and provide controlled access to the SPI device required by the radar sensor.
/// The `Mutex` ensures thread-safe access in environments where multi-threading is possible, while the `RefCell`
/// allows for mutable access to the SPI device. This setup is crucial for enabling SPI communications in a safe
/// and controlled manner within the radar sensor's hardware abstraction layer.
///
/// # Safety
///
/// The access to the `SPI_INSTANCE` is controlled via a mutex to prevent concurrent access issues.
/// However, care must be taken to ensure that the SPI device is properly initialized before use
/// and is not accessed after it has been freed or gone out of scope.
static mut SPI_INSTANCE: MaybeUninit<*mut c_void> = MaybeUninit::uninit();

/// Represents the hardware abstraction layer implementation for the radar sensor.
///
/// This struct encapsulates the necessary functionality to interface with the radar sensor
/// using the SPI communication protocol and provides methods for memory management and logging.
pub struct AccHalImpl<SPI> {
    inner: acc_hal_a121_t,
    _spi: PhantomData<SPI>
}

impl<SPI: SpiDevice + Send + 'static> AccHalImpl<SPI> {
    /// Constructs a new `AccHalImpl` instance, registering the SPI device and initializing
    /// the radar hardware abstraction layer.
    ///
    /// # Arguments
    ///
    /// * `spi` - A reference to an SPI device that implements the `SpiBus` trait.
    ///
    /// # Panics
    ///
    /// Panics if the HAL registration fails.
    pub fn new(spi: &'static mut SPI) -> Self
    {
        let inner = acc_hal_a121_t {
            max_spi_transfer_size: u16::MAX,
            mem_alloc: Some(mem_alloc),
            mem_free: Some(mem_free),
            transfer: Some(Self::transfer8_function),
            log: Some(a121_sys::c_log_stub), // TODO replace with logger once va are correctly parsed
            optimization: acc_hal_optimization_t { transfer16: None },
        };

        #[allow(static_mut_refs)]
        unsafe {
            SPI_INSTANCE.write(spi as *mut SPI as *mut c_void);  
        }
  
        Self { inner, _spi: PhantomData::default() }
    }

    /// Transfer function for 16-bit data used by the radar SDK.
    ///
    /// This function is registered as part of the HAL and is called by the radar SDK to
    /// perform SPI transfers.
    ///
    /// # Safety
    ///
    /// This function is unsafe as it involves raw pointers and direct hardware access.
    #[allow(dead_code)]
    extern "C" fn transfer16_function(
        _sensor_id: acc_sensor_id_t,
        _buffer: *mut u16,
        _buffer_length: usize,
    ) {
        #[cfg(feature = "defmt")]
        {
            let tmp_buf = unsafe { core::slice::from_raw_parts_mut(_buffer, _buffer_length) };
            defmt::trace!(
                "Transfer16 function called: buffer={:#X} (size:{})",
                tmp_buf,
                _buffer_length
            );
        }
        todo!("Perform the SPI 16 transfer");
    }

    extern "C" fn transfer8_function(
        _sensor_id: acc_sensor_id_t,
        buffer: *mut u8,
        buffer_length: usize,
    ) {
        let tmp_buf = unsafe { core::slice::from_raw_parts_mut(buffer, buffer_length) };
        #[allow(static_mut_refs)]
        let spi = unsafe { &mut *((*SPI_INSTANCE.as_mut_ptr()) as *mut SPI)};
        spi.transfer_in_place(tmp_buf).unwrap();
    }

    /// Registers the HAL implementation with the radar SDK.
    ///
    /// This method should be called to register the HAL implementation, allowing the
    /// radar sensor to communicate using the provided SPI interface.
    ///
    /// # Panics
    ///
    /// Panics if the HAL registration fails.
    #[inline(always)]
    pub fn register(&self) {
        #[cfg(feature = "defmt")]
        defmt::trace!("Registering HAL");
        let result = unsafe { acc_rss_hal_register(&self.inner) };
        assert!(result, "Failed to register HAL");
    }
}

extern "C" {
    fn malloc(size: usize) -> *mut c_void;
    fn free(ptr: *mut c_void);
}

/// Allocates memory for use by the radar SDK.
///
/// # Safety
///
/// This function is unsafe as it performs raw pointer manipulation.
unsafe extern "C" fn mem_alloc(size: usize) -> *mut c_void {
    malloc(size)
}

/// Frees memory previously allocated for the radar SDK.
///
/// # Safety
///
/// This function is unsafe as it performs raw pointer manipulation.
unsafe extern "C" fn mem_free(ptr: *mut c_void) {
    free(ptr);
}

// unsafe extern "C" fn logger(
//     level: a121_sys::acc_log_level_t,
//     module: *const c_char,
//     format: *const c_char,
//     mut _va: ...
// ) {
//     let module = unsafe { CStr::from_ptr(module) };
//     let format = unsafe { CStr::from_ptr(format) };
//     let message = format.to_str().unwrap_or("");

//     // TODO parse _va to get the actual message

//     #[cfg(feature = "defmt")]
//     match level {
//         0 => defmt::error!("{}: {}", module.to_str().unwrap_or(""), message),
//         1 => defmt::warn!("{}: {}", module.to_str().unwrap_or(""), message),
//         2 => defmt::info!("{}: {}", module.to_str().unwrap_or(""), message),
//         3 => defmt::debug!("{}: {}", module.to_str().unwrap_or(""), message),
//         4 => defmt::trace!("{}: {}", module.to_str().unwrap_or(""), message),
//         _ => defmt::error!("Unknown log level: {}", level),
//     }

//     #[cfg(feature = "log")]
//     match level {
//         0 => log::error!("{}: {}", module.to_str().unwrap_or(""), message),
//         1 => log::warn!("{}: {}", module.to_str().unwrap_or(""), message),
//         2 => log::info!("{}: {}", module.to_str().unwrap_or(""), message),
//         3 => log::debug!("{}: {}", module.to_str().unwrap_or(""), message),
//         4 => log::trace!("{}: {}", module.to_str().unwrap_or(""), message),
//         _ => log::error!("Unknown log level: {}", level),
//     }
// }

// This is called by the c_log_stub function from a121-sys, to better parse the va_list
// Ultimately, this function will be replaced by the logger function
#[no_mangle]
pub unsafe extern "C" fn rust_log(_level: u32, _message: *const c_char) {
    #[cfg(feature = "defmt")]
    {
        let c_str = unsafe { CStr::from_ptr(_message) };
        let str_slice = c_str.to_str().unwrap_or("");
        match _level {
            0 => defmt::error!("{}", str_slice),
            1 => defmt::warn!("{}", str_slice),
            2 => defmt::info!("{}", str_slice),
            3 => defmt::debug!("{}", str_slice),
            4 => defmt::trace!("{}", str_slice),
            _ => defmt::error!("Unknown log level: {}", _level),
        }
    }

    #[cfg(feature = "log")]
    {
        let c_str = unsafe { CStr::from_ptr(_message) };
        let str_slice = c_str.to_str().unwrap_or("");
        match _level {
            0 => log::error!("{}", str_slice),
            1 => log::warn!("{}", str_slice),
            2 => log::info!("{}", str_slice),
            3 => log::debug!("{}", str_slice),
            4 => log::trace!("{}", str_slice),
            _ => log::error!("Unknown log level: {}", _level),
        }
    }
}