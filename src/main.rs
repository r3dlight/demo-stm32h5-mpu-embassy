#![no_std]
#![no_main]

use core::str::from_utf8;
//use cortex_m::peripheral::Peripherals;
use cortex_m::peripheral::MPU;
//use cortex_m::peripheral::SCB;
use cortex_m::register::control;
//use cortex_m::register::control::Control;
use cortex_m::register::control::Npriv;
use defmt::*;
use embassy_executor::Spawner;
//use embassy_futures::join;
use embassy_stm32::rcc::*;
use embassy_stm32::{
    exti::{AnyChannel, Channel, ExtiInput},
    gpio::{AnyPin, Level, Output, Pin, Pull, Speed},
    Config,
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer, WithTimeout};
use {defmt_rtt as _, panic_probe as _};

// User button status
#[derive(Copy, Clone)]
enum ButtonStatus {
    On,
    Off,
}

// Signal to communicate between tasks
static SIGNAL: Signal<CriticalSectionRawMutex, ButtonStatus> = Signal::new();
// Mutable shared buffer between privileged and unprivileged tasks
#[link_section = ".shared_buffer"]
static mut SHARED_BUFFER: [u8; 512] = [0; 512];

// Configure the MPU for the STM32H573
fn configure_mpu() {
    unsafe {
        // Take a reference to the MPU
        let mpu = &*MPU::PTR;

        // Disable the MPU before configuration
        mpu.ctrl.write(0);
        info!("MPU disabled before configuration");

        // Data Synchronization Barrier
        // Force write of registers
        cortex_m::asm::dsb();

        // Instruction Synchronization Barrier
        // Flush the instruction pipeline and apply the new configuration
        cortex_m::asm::isb();
        info!("DSB & ISB done");

        // Configure MAIR (Memory Attribute Indirection Register)
        // Indice 0 → Normal Memory, Indice 1 → Device Memory, Indice 2 → SRAM
        // 0x44 = 0b01000100 : Flash Normal, Write-back
        // 0x04 = 0b00000100 : Device Memory
        // 0x40 = 0b01000000 : Normal, Write-back, non transient (SRAM)

        // Defining 3 memory attributes for 3 different regions
        // 4 attributes are possible (0-3) in MAIR0 register
        // 4 atrributes are possible (4-7) in MAIR1 register
        // 8 types of memory are possible (0-7) in the MPU_RLAR register
        // Each attribute is 8 bits long
        mpu.mair[0].write(
            (0xFF << 0)  // ATTRINDEX 0 → Flash (Normal, Write-back)
            | (0x04 << 8)  // ATTRINDEX 1 → Device Memory (SPI)
            | (0x40 << 16), // ATTRINDEX 2 → RAM (Write-back, cachable)
        );
        /*  Bits	Value	        Signification
        01	    Normal Memory	Optimized for performance
        00	    Write-Through	Direct write to memory, no cache delay
        0000	Non-Transient	No auto cache cleaning */
        info!("MAIR register configured");

        // Select region 0 via MPU_RNR (Region NumbeR) for flash memory and remap zone
        mpu.rnr.write(0);
        // MPU_RBAR definition
        // MPU_RBAR is splitted into 4 parts:
        // |31 (BASE ADDR) 5|4 (SHARABILITY) 3|2 (ACCESS PERMISSION) 1|0 (XN)|
        // Base address for flash is remapped to 0x00000000 (see DDI0553B_y_armv8m_arm-1.pdf)
        // SHARABILITY:       0b00: Non-shareable
        //                    0b01: Reserved
        //                    0b10: Outer Shareable
        //                    0b11: Inner Shareable
        // ACCESS PERMISSION: 0b00: Read/write by privileged code only,
        //                    0b01: Read/write by any privilege level.
        //                    0b10: Read-only by privileged code only
        //                    0b11: Read-only by any privilege level
        // XN (Execute-Never):
        //                    0b0: Executable
        //                    0b1: Non-executable
        // In our case, Embassy needs the flash to be r/w & executable
        // to be able to run the priviledged code.
        mpu.rbar.write(0x00000000 | (1 << 1) | 0);
        // MPU_RLAR definition
        // MPU_RLAR is splitted into 4 parts:
        // |31 (LIMIT ADDR) 5|4 PXN|3 (ATTR INDEX) 1|0 (EN)|
        // Limit address for flash is 0x1FFFFFF0 (must be aligned to 32 bytes)
        // PXN (Privileged eXecute Never):
        //                    0b0: Execution only permitted if read permitted
        //                    0b1: Execution from a privileged mode is not permitted
        // ATTR INDEX:        Select the index previously defined in MAIR
        // EN (Enable):       Enable the region:
        //                    0b0: Region is disabled
        //                    0b1: Region is enabled
        mpu.rlar.write(0x1FFFFFF0 | (0 << 1) | 1); // Attr index 0 (Flash Memory)
        info!("Region 0 defined for flash memory");

        // Select region 1 via MPU_RNR for SRAM1
        mpu.rnr.write(1);
        // SRAM1 begins at 0x20000200 now (the shared buffer is mapped before) 
        // and can be accessed by any privilege level
        // It is not executable (XN+PXN) for security reasons
        // Next step could be to separate each task in a specific Flash region
        mpu.rbar.write(0x20000200 | (1 << 1) | 1);
        // SRAM1 ends at 0x3FFFFFF0 (must be aligned to 32 bytes)
        mpu.rlar.write(0x3FFFFFF0 | (1 << 4) | (2 << 1) | 1); // Attr index 1 (SRAM Memory)
        info!("Region 1 defined for SRAM1");

        // To be more restrictive, we could also define a specific region for the shared buffer
        // defined at the beginning of the SRAM1.
        // Embassy stack will be mapped after in a specific SRAM1 region
        // This region starts at 0x20000200 and ends at 0x20000200 + 512 - 1 = 0x200003FF
        mpu.rnr.write(2);
        mpu.rbar
            .write(((&raw const SHARED_BUFFER as *const _ as u32) & 0xFFFFFFF0) | (1 << 1) | 1);
        // Buffer size in RAM = 512 bytes -1 (inclusive)
        // XN & PXN set to 1 to avoid execution from the buffer
        mpu.rlar.write(
            ((&raw const SHARED_BUFFER as *const _ as u32 + 512 - 1) & 0xFFFFFFF0)
                | (1 << 4)
                | (2 << 1)
                | 1,
        );
        info!("Region 2 defined for shared buffer");

        // Select region 3 via MPU_RNR for Device Memory (Peripherals)
        // This code demo uses GPIO pins (button+LED) and write to USART with info!()
        mpu.rnr.write(3);
        // Device memory starts at 0x40000000 and can be accessed by any privilege level
        // This should be forbidden for unprivileged tasks but for this demo
        // we need to allow it for the USART peripheral for info!() function
        mpu.rbar.write(0x40000000 | (1 << 1) | 1);
        // Device memory ends at 0x5FFFFFF0 (must be aligned to 32 bytes)
        // Devices are NX and PXN!
        mpu.rlar.write(0x5FFFFFF0 | (1 << 4) | (0 << 1) | 1); // Attr index 1 (Device Memory)
        info!("Region 3 defined for peripherals");

        // Optionaly define hardfaults IRQs to handle MPU faults
        // Needs to be tested !
        //let mut peripherals =  Peripherals::steal();
        //let scb = &mut peripherals.SCB;
        //scb.shcsr.modify(|r| r | (1 << 16) | (1 << 17) | (1 << 18));

        // Enable the MPU
        mpu.ctrl.write(1);

        // Data Synchronization Barrier
        cortex_m::asm::dsb();
        // Instruction Synchronization Barrier
        cortex_m::asm::isb();
        info!("DSB & ISB done");
    }
    info!("Congrats, MPU is re-enabled :)");
}

// Main privileged function
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Configure the MPU is blocking the execution
    // We want the MPU to be configured before anything else
    configure_mpu();

    // Doing some RCC configuration
    let mut config = Config::default();
    {
        config.rcc.hsi = Some(HSIPrescaler::DIV1);
        config.rcc.csi = true;
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL10,
            divp: Some(PllDiv::DIV2),
            divq: Some(PllDiv::DIV8), // used by SPI1. 100Mhz.
            divr: None,
        });
        config.rcc.sys = Sysclk::PLL1_P; // 400 Mhz
        config.rcc.ahb_pre = AHBPrescaler::DIV2; // 200 Mhz
        config.rcc.apb1_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb2_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb3_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.voltage_scale = VoltageScale::Scale1;
    }

    let p = embassy_stm32::init(config);

    info!("Press the USER button...");

    // Launch the GPIO tasks
    spawner.spawn(blink(p.PF4.degrade())).unwrap();
    spawner
        .spawn(button_pressed(p.PC13.degrade(), p.EXTI13.degrade()))
        .unwrap();

    unsafe {
        // Read the part of the buffer where the data has been written
        // The buffer is not written yet, so it should be empty
        let buffer_slice = &SHARED_BUFFER[..12]; // "Hello SPI !" is 12 bytes

        // Convert to UTF-8 str if valid
        if let Ok(text) = from_utf8(buffer_slice) {
            info!(
                "Content of the buffer from the privileged task (before): {}",
                text
            );
        } else {
            info!("Error: No valid UTF-8 found in the buffer !");
        }
    }
    // Launch the user task to write to the shared buffer
    spawner.spawn(user_task()).unwrap();
    // Wait a bit to for the buffer to be written by the user task
    Timer::after_millis(400).await;
    unsafe {
        // Read the part of the buffer where the data has been written
        let buffer_slice = &SHARED_BUFFER[..12]; // "Hello SPI !" is 12 bytes

        // Convert to UTF-8 str if valid
        if let Ok(text) = from_utf8(buffer_slice) {
            info!("Content of the buffer from the privileged task: {}", text);
        } else {
            info!("Error: No valid UTF-8 found in the buffer !");
        }
    }
}

// Blink the LED task
#[embassy_executor::task(pool_size = 1)]
async fn blink(led: AnyPin) {
    // As a user task can access peripherials, we can set the current task as unprivileged
    set_unpriviliged();
    let mut led = Output::new(led, Level::High, Speed::Low);
    let delay = Duration::from_millis(100 as u64);

    loop {
        if let Some(v) = SIGNAL.wait().with_timeout(delay).await.ok() {
            info!("SIGNAL received !");
            match v {
                ButtonStatus::On => {
                    info!("High detected");
                    led.set_high();
                    Timer::after_millis(100).await;
                }
                ButtonStatus::Off => {
                    info!("Low detected");
                    led.set_low();
                    Timer::after_millis(100).await;
                }
            };
        }
    }
}

// Button pressed task
#[embassy_executor::task(pool_size = 2)]
async fn button_pressed(pc13: AnyPin, exti13: AnyChannel) {
    // As a user task can access peripherials, we can also set the current task as unprivileged
    set_unpriviliged();
    let mut button = ExtiInput::new(pc13, exti13, Pull::Down);

    loop {
        button.wait_for_rising_edge().await;
        SIGNAL.signal(ButtonStatus::Off);

        button.wait_for_falling_edge().await;
        SIGNAL.signal(ButtonStatus::On);
    }
}

#[embassy_executor::task]
async fn user_task() {
    // Set the current task as unprivileged
    set_unpriviliged();
    unsafe {
        // Write some data to the shared buffer
        let data = b"Hello USER !";
        SHARED_BUFFER[..data.len()].copy_from_slice(data);
    }
    // Removing this info() to remove the need of the peripheral in the MPU
    // for the user tasks. Periperals will not be allowed for unprivileged tasks.
    info!("SHARED_BUFFER written from the user task ! ");
}

// Blocking & unsafe function, not a task
// Set the current task as unprivileged
fn set_unpriviliged() {
    unsafe {
        // ARM: Check if the task is in Handler or Thread mode
        // via ipsr register
        let ipsr: u32;
        core::arch::asm!("mrs {}, ipsr", out(reg) ipsr);

        if ipsr == 0 {
            info!("Task is running in thread Mode");
        } else {
            info!("Task is running in handler Mode");
        }
        // Unsafe code to modify the CONTROL register
        // Read the current value of the CONTROL register
        let mut control_reg = control::read();
        // Set the nPRIV bit to unprivileged
        control_reg.set_npriv(Npriv::Unprivileged);
        // Write the new value
        control::write(control_reg);
        // Instruction & Data Synchronization Barrier
        cortex_m::asm::isb();
        cortex_m::asm::dsb();

        // Check if the task is unprivileged
        // Access to peripherls like usart should be forbidden
        // Remove any info() function to remove the need of the peripheral in the MPU
        if let Npriv::Unprivileged = control::read().npriv() {
            info!("User task is now unprivileged !");
        } else {
            info!("Error: User task is still privileged !");
            // Further actions required to handle the error
        }
        // Unsafe code to modify the CONTROL register
        // Try to set the task as privileged and see what happens
        let mut control_reg = control::read();
        // Set the nPRIV bit to unprivileged
        control_reg.set_npriv(Npriv::Unprivileged);
        // Write the new value
        control::write(control_reg);
        // Instruction & Data Synchronization Barrier
        cortex_m::asm::isb();
        cortex_m::asm::dsb();
        // Check if the task now privileged
        if let Npriv::Unprivileged = control::read().npriv() {
            info!("User task is still unprivileged !");
        } else {
            info!("Error: User task is now privileged !");
            // Further actions required to handle the error
        }
    }
}
