//! DM-MC02（STM32H723VG）开发板配置：时钟、CAN1、DR16 输入、看门狗和 panic 处理。
//!
//! CPU 使用 24 MHz 晶振运行在 520 MHz；TIM2 用作 Embassy 时间驱动。

use core::fmt::Write as _;
use core::num::{NonZeroU8, NonZeroU16};
use core::panic::PanicInfo;
use core::sync::atomic::{AtomicBool, Ordering};
use embassy_stm32::can::config::NominalBitTiming;
use embassy_stm32::can::{self, BufferedCan, CanConfigurator, RxBuf, TxBuf};
use embassy_stm32::dma;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::mode::Async;
use embassy_stm32::rcc::mux::Fdcansel;
use embassy_stm32::rcc::{
    AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllDiv, PllMul, PllPreDiv, PllSource, Sysclk,
    VoltageScale,
};
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{self, Parity, StopBits, UartRx};
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_stm32::{bind_interrupts, pac};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};

use static_cell::StaticCell;

pub type RemoteRx = UartRx<'static, Async>;
pub static REMOTE: Signal<CriticalSectionRawMutex, Option<Dr16>> = Signal::new();

pub type Can1 = BufferedCan<'static, 4, 16>;
static CAN_TX: StaticCell<TxBuf<4>> = StaticCell::new();
static CAN_RX: StaticCell<RxBuf<16>> = StaticCell::new();

pub type Watchdog = IndependentWatchdog<'static, embassy_stm32::peripherals::IWDG1>;

/// 标称值；LSI 时钟会随电压和温度变化。
const WATCHDOG_TIMEOUT_US: u32 = 100_000;

bind_interrupts!(struct Irqs {
    UART5 => usart::InterruptHandler<embassy_stm32::peripherals::UART5>;
    DMA1_STREAM5 => dma::InterruptHandler<embassy_stm32::peripherals::DMA1_CH5>;
    FDCAN1_IT0 => can::IT0InterruptHandler<embassy_stm32::peripherals::FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<embassy_stm32::peripherals::FDCAN1>;
});

/// 获取 MCU 外设，返回 CAN1、已启动的看门狗和 DR16 接收器。
/// 启动时调用一次。开发板须通过 USB 或正常电源输入供电，
/// 仅靠调试探针供电无法给 CAN 收发器供电。
pub fn init() -> (Can1, Watchdog, RemoteRx) {
    // 看门狗复位后，在启动 CAN 之前停止。先清除复位标志，
    // 使下一次手动复位能够正常运行。
    let watchdog_reset = pac::RCC.rsr().read().iwdg1rstf();
    pac::RCC.rsr().modify(|w| w.set_rmvf(true));
    if watchdog_reset {
        rtt_target::rprintln!("Watchdog reset: halted. Press RESET to run again.");
        cortex_m::interrupt::disable();
        loop {
            core::hint::spin_loop();
        }
    }

    // 24 MHz / 3 * 65 = 520 MHz，是未启用 CPU 加速选项位时的 VOS0 上限。
    // AHB 运行在 260 MHz，各 APB 运行在 130 MHz；CAN 直接使用晶振时钟。
    let mut config = embassy_stm32::Config::default();
    config.rcc.hse = Some(Hse {
        freq: Hertz(24_000_000),
        mode: HseMode::Oscillator,
    });
    config.rcc.pll1 = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::DIV3,
        mul: PllMul::MUL65,
        divp: Some(PllDiv::DIV1),
        divq: None,
        divr: None,
    });
    config.rcc.sys = Sysclk::PLL1_P;
    config.rcc.ahb_pre = AHBPrescaler::DIV2;
    config.rcc.apb1_pre = APBPrescaler::DIV2;
    config.rcc.apb2_pre = APBPrescaler::DIV2;
    config.rcc.apb3_pre = APBPrescaler::DIV2;
    config.rcc.apb4_pre = APBPrescaler::DIV2;
    config.rcc.voltage_scale = VoltageScale::Scale0;
    config.rcc.mux.fdcansel = Fdcansel::HSE;
    let p = embassy_stm32::init(config);

    // 仅由控制任务在每个周期完成后喂狗。
    let mut watchdog = Watchdog::new(p.IWDG1, WATCHDOG_TIMEOUT_US);
    watchdog.unleash();

    // 经典 CAN 使用 PD0（接收）和 PD1（发送）：24 MHz / (1 + 17 + 6) = 1 Mbit/s，
    // 采样点为 75%。
    let mut can = CanConfigurator::new(p.FDCAN1, p.PD0, p.PD1, Irqs);
    can.set_config(can.config().set_nominal_bit_timing(NominalBitTiming {
        prescaler: NonZeroU16::new(1).unwrap(),
        seg1: NonZeroU8::new(17).unwrap(),
        seg2: NonZeroU8::new(6).unwrap(),
        sync_jump_width: NonZeroU8::new(6).unwrap(),
    }));
    let can = can
        .into_normal_mode()
        .buffered(CAN_TX.init(TxBuf::new()), CAN_RX.init(RxBuf::new()));

    // PD2 上的 DBUS：100 kbaud、8E2。开发板会反相信号，因此 UART 不再反相。
    let mut remote_config = usart::Config::default();
    remote_config.baudrate = 100_000;
    remote_config.parity = Parity::ParityEven;
    remote_config.stop_bits = StopBits::STOP2;
    let remote = UartRx::new(p.UART5, p.PD2, p.DMA1_CH5, Irqs, remote_config).unwrap();
    // PC15 控制为接收器供电的 5 V 电源。
    core::mem::forget(Output::new(p.PC15, Level::High, Speed::Low));
    (can, watchdog, remote)
}

/// 一帧有效的 18 字节 DBUS 数据，未进行通道重映射或死区处理。
#[derive(Clone, Copy)]
pub struct Dr16 {
    /// 右摇杆水平、右摇杆垂直、左摇杆水平、左摇杆垂直、滚轮；范围为 -660..=660。
    pub channels: [i16; 5],
    /// 1 为上位，2 为下位，3 为中位。
    pub right_switch: u8,
    pub left_switch: u8,
    pub mouse: [i16; 3],
    pub mouse_left: bool,
    pub mouse_right: bool,
    /// 第 0～15 位对应按键：W S A D Shift Ctrl Q E R F G Z X C V B。
    pub keys: u16,
}

impl Dr16 {
    /// 对不完整的帧、超出范围的通道值或拨杆值返回 `None`。
    pub fn parse(bytes: &[u8]) -> Option<Self> {
        let b: &[u8; 18] = bytes.try_into().ok()?;
        let u = |i: usize| u16::from(b[i]);
        let channels = [
            u(0) | u(1) << 8,
            u(1) >> 3 | u(2) << 5,
            u(2) >> 6 | u(3) << 2 | u(4) << 10,
            u(4) >> 1 | u(5) << 7,
            u(16) | u(17) << 8,
        ]
        .map(|value| (value & 0x7ff) as i16 - 1024);
        let right_switch = b[5] >> 4 & 3;
        let left_switch = b[5] >> 6 & 3;
        if channels.iter().any(|value| !(-660..=660).contains(value))
            || !(1..=3).contains(&right_switch)
            || !(1..=3).contains(&left_switch)
        {
            return None;
        }
        let signed = |i: usize| i16::from_le_bytes([b[i], b[i + 1]]);
        Some(Self {
            channels,
            right_switch,
            left_switch,
            mouse: [signed(6), signed(8), signed(10)],
            mouse_left: b[12] != 0,
            mouse_right: b[13] != 0,
            keys: u16::from_le_bytes([b[14], b[15]]),
        })
    }
}

/// 根据 UART 空闲间隔分割 DBUS 帧，只发布最新的一帧。
#[embassy_executor::task]
pub async fn remote_task(mut rx: RemoteRx) {
    // 缓冲区为两帧长度；如果读取填满缓冲区，说明缺少空闲间隔。
    let mut bytes = [0; 36];
    loop {
        match rx.read_until_idle(&mut bytes).await {
            Ok(n) => REMOTE.signal(Dr16::parse(&bytes[..n])),
            Err(_) => {
                REMOTE.signal(None);
                Timer::after(Duration::from_millis(1)).await;
            }
        }
    }
}

/// 由于控制任务持有 HAL 看门狗，这里直接配置 IWDG1。在标称 32 kHz LSI 下，
/// 重装载值 799 对应 100 ms，重装载值 0 对应 125 us。
fn configure_panic_watchdog(reload: u16) {
    use pac::iwdg::vals::{Key, Pr};

    let wdg = pac::IWDG1;
    wdg.kr().write(|w| w.set_key(Key::START));
    wdg.kr().write(|w| w.set_key(Key::ENABLE));
    while wdg.sr().read().pvu() || wdg.sr().read().rvu() {
        core::hint::spin_loop();
    }
    wdg.pr().write(|w| w.set_pr(Pr::DIVIDE_BY4));
    wdg.rlr().write(|w| w.set_rl(reload));
    while wdg.sr().read().pvu() || wdg.sr().read().rvu() {
        core::hint::spin_loop();
    }
    wdg.kr().write(|w| w.set_key(Key::RESET));
}

/// 关闭中断，最多等待 100 ms 供探针读取消息，然后通过看门狗复位。
/// 下次启动时会在 `init` 中停止，直到手动复位。
#[panic_handler]
fn panic(info: &PanicInfo<'_>) -> ! {
    static PANICKED: AtomicBool = AtomicBool::new(false);

    cortex_m::interrupt::disable();
    if !PANICKED.swap(true, Ordering::Relaxed) {
        configure_panic_watchdog(799);
        rtt_target::with_terminal_channel(|term| {
            term.set_mode(rtt_target::ChannelMode::BlockIfFull);
            let _ = writeln!(term.write(0), "{}", info);
            term.flush();
        });
    }
    configure_panic_watchdog(0);
    loop {
        core::hint::spin_loop();
    }
}
