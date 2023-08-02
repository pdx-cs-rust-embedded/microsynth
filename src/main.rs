#![no_main]
#![no_std]

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};

use cortex_m_rt::entry;
use microbit::hal::gpiote::Gpiote;
use microbit::hal::twim;
use microbit::pac::twim0::frequency::FREQUENCY_A;
use microbit::{
    hal::{
        gpio,
        prelude::OutputPin,
        pwm,
        rtc::{Rtc, RtcInterrupt},
        time::Hertz,
    },
    pac::{self, interrupt},
    Board,
};

use lsm303agr::{AccelOutputDataRate, Lsm303agr};

struct Context {
    gpio: Gpiote,
    rtc: Rtc<pac::RTC0>,
    speaker: pwm::Pwm<pac::PWM0>,
    steps: i32,
    pitch_bend: f32,
    bounce: u8,
}

impl Context {
    fn new(gpio: Gpiote, rtc: Rtc<pac::RTC0>, speaker: pwm::Pwm<pac::PWM0>) -> Self {
        Self { gpio, rtc, speaker, steps: 0, pitch_bend: 0.0, bounce: 0 }
    }
}

static CONTEXT: Mutex<RefCell<Option<Context>>> = Mutex::new(RefCell::new(None));
static USE_MINOR_SCALE: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(true));

// Map MIDI key number to key frequency.
fn key_to_freq(semitone: i32, pitch_bend: f32) -> f32 {
    let semitone_bend = 2.0 * pitch_bend;
    440.0 * libm::powf(2.0, (1.0 / 12.0) * ((semitone - 69) as f32 + semitone_bend))
}

fn steps_to_freq(steps: i32, pitch_bend: f32, is_minor_scale: bool) -> f32 {
    // XXX Excuse the terrible behavior here. Should
    // div_euclid() and mod_euclid() to make everything come
    // out, but that would be a pain in no_std, so so we
    // instead just fudge everything positive and fix later.
    // Means no more than 9 octaves down from A4, which is
    // probably fine…
    let octave = (steps + 63).max(0) / 7 - 4;
    let note = ((steps + 63).max(0) % 7) as usize;
    let scale = if is_minor_scale {
        // minor
        [
            0,  // root
            2,  // 2nd: whole step from root
            3,  // 3rd: half step from 2nd
            5,  // 4th: whole step from 3rd
            7,  // 5th: whole step from 4th
            8,  // 6th: half step from 5th
            10, // 7th: whole step from 6th
        ]
    } else {
        // major
        [
            0,  // root
            2,  // 2nd: whole step from root
            4,  // 3rd: whole step from 2nd
            5,  // 4th: half step from 3rd
            7,  // 5th: whole step from 4th
            9,  // 6th: whole step from 5th
            11, // 7th: whole step from 6th
        ]
    };
    key_to_freq(scale[note] + 12 * octave + 9, pitch_bend)
}

#[interrupt]
fn GPIOTE() {
    cortex_m::interrupt::free(|cs| {
        if let Some(pc) = CONTEXT.borrow(cs).borrow_mut().as_mut() {
            // Check whether in debounce.
            if pc.bounce > 0 {
                pc.gpio.channel0().reset_events();
                pc.gpio.channel1().reset_events();
                return;
            }

            let buttonapressed = pc.gpio.channel0().is_event_triggered();
            let buttonbpressed = pc.gpio.channel1().is_event_triggered();

            // XXX Limit to 9 octaves up and down. See `steps_to_freq()` above.
            match (buttonapressed, buttonbpressed) {
                (false, false) => (),
                (true, false) => {
                    pc.steps = (pc.steps - 1).max(-63);
                    // Start the debounce counter.
                    pc.bounce = 8;
                }
                (false, true) => {
                    pc.steps = (pc.steps + 1).min(63);
                    // Start the debounce counter.
                    pc.bounce = 8;
                }
                (true, true) => (),
            }
            rprintln!("steps: {}", pc.steps);

            /* Clear events */
            pc.gpio.channel0().reset_events();
            pc.gpio.channel1().reset_events();
        }
    });
}

// RTC interrupt, exectued for each RTC tick
#[interrupt]
fn RTC0() {
    /* Enter critical section */
    cortex_m::interrupt::free(|cs| {
        /* Borrow devices */
        if let Some(pc) = CONTEXT.borrow(cs).borrow_mut().as_mut() {
            let is_minor_scale = *USE_MINOR_SCALE.borrow(cs).borrow();
            let freq = steps_to_freq(pc.steps, pc.pitch_bend, is_minor_scale);
            //rprintln!("rtc0: {} {}", pc.steps, freq);
            pc.speaker.set_period(Hertz(freq as u32));

            // Restart the PWM at 33% duty cycle to preserve em ears
            let max_duty = pc.speaker.max_duty();
            pc.speaker.set_duty_on_common(max_duty / 3);

            // Clear the RTC interrupt
            pc.rtc.reset_event(RtcInterrupt::Tick);

            // Bump down the debounce counter.
            if pc.bounce > 0 {
                pc.bounce -= 1;
            }
        }
    });
}

#[entry]
fn main() -> ! {
    rtt_init_print!();

    let mut board = Board::take().unwrap();
    let gpiote = Gpiote::new(board.GPIOTE);

    let channel0 = gpiote.channel0();
    channel0
        .input_pin(&board.buttons.button_a.degrade())
        .hi_to_lo()
        .enable_interrupt();
    channel0.reset_events();

    let channel1 = gpiote.channel1();
    channel1
        .input_pin(&board.buttons.button_b.degrade())
        .hi_to_lo()
        .enable_interrupt();
    channel1.reset_events();

    let i2c = { twim::Twim::new(board.TWIM0, board.i2c_internal.into(), FREQUENCY_A::K100) };
    let mut sensor = Lsm303agr::new_with_i2c(i2c);
    sensor.init().unwrap();

    let acc_id = sensor.accelerometer_id().unwrap();
    rprintln!("The accelerometer chip's id is: {:#b}", acc_id);

    sensor.set_accel_odr(AccelOutputDataRate::Hz50).unwrap();

    cortex_m::interrupt::free(move |cs| {
        /* Enable external GPIO interrupts */
        unsafe {
            pac::NVIC::unmask(pac::Interrupt::GPIOTE);
        }
        pac::NVIC::unpend(pac::Interrupt::GPIOTE);

        let mut rtc = Rtc::new(board.RTC0, 511).unwrap();
        rtc.enable_counter();
        rtc.enable_interrupt(RtcInterrupt::Tick, Some(&mut board.NVIC));
        rtc.enable_event(RtcInterrupt::Tick);

        let mut speaker_pin = board.speaker_pin.into_push_pull_output(gpio::Level::High);
        let _ = speaker_pin.set_low();

        // Use the PWM peripheral to generate a waveform for the speaker
        let speaker = pwm::Pwm::new(board.PWM0);
        speaker
            // output the waveform on the speaker pin
            .set_output_pin(pwm::Channel::C0, speaker_pin.degrade())
            // Use prescale by 16 to achive darker sounds
            .set_prescaler(pwm::Prescaler::Div16)
            // Initial frequency
            .set_period(Hertz(440u32))
            // Configure for up and down counter mode
            .set_counter_mode(pwm::CounterMode::UpAndDown)
            // Set maximum duty cycle
            .set_max_duty(5000)
            // enable PWM
            .enable();

        // Configure 50% duty cycle
        let max_duty = speaker.max_duty();
        speaker.set_duty_on_common(max_duty / 2);

        *CONTEXT.borrow(cs).borrow_mut() = Some(Context::new(gpiote, rtc, speaker));

        unsafe {
            pac::NVIC::unmask(pac::Interrupt::RTC0);
        }
        pac::NVIC::unpend(pac::Interrupt::RTC0);
    });

    loop {
        if sensor.accel_status().unwrap().xyz_new_data {
            let data = sensor.accel_data().unwrap();

            //rprintln!("Acceleration: x {} y {} z {}", data.x, data.y, data.z);
            cortex_m::interrupt::free(move |cs| {
                // Standard midi mapping ish
                let new_freq = data.x as f32 / 8192.0;
                let abs_data = libm::fabsf(data.x as f32);
                // Extremely scientific method to arrive at 100.
                if abs_data > 100.0 {
                    if let Some(pc) = CONTEXT.borrow(cs).borrow_mut().as_mut() {
                        pc.pitch_bend= new_freq;
                    }
                }
            });
        }
    }
}
