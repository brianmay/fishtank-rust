use std::collections::VecDeque;
use std::{env, sync::atomic::*, sync::Arc, thread, time::*};

use anyhow::bail;
use anyhow::Result;

use esp_idf_hal::uart::config::Config;
use esp_idf_hal::uart::UartDriver;
use esp_idf_sys::esp_efuse_mac_get_default;
use log::*;

use one_wire_bus::{Address, OneWire};
use serde::Serialize;

use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::digital::v2::{InputPin, OutputPin};

use embedded_svc::mqtt::client::{Connection, Event, MessageImpl, QoS};
use embedded_svc::utils::mqtt::client::ConnState;
use embedded_svc::wifi::*;

use esp_idf_svc::eventloop::*;
use esp_idf_svc::mqtt::client::*;
use esp_idf_svc::netif::*;
use esp_idf_svc::wifi::*;

use esp_idf_hal::adc::{self, AdcChannelDriver, AdcDriver, Atten0dB};
use esp_idf_hal::delay;
use esp_idf_hal::gpio::{self, PinDriver};
use esp_idf_hal::peripheral;
use esp_idf_hal::prelude::*;

use esp_idf_sys::EspError;

const SSID: &str = env!("RUST_ESP32_STD_DEMO_WIFI_SSID");
const PASS: &str = env!("RUST_ESP32_STD_DEMO_WIFI_PASS");

#[derive(Serialize, Debug)]
struct MqttData {
    distance: u16,
    temperature: f32,
    tds: Option<f32>,
}

fn main() -> Result<()> {
    esp_idf_sys::link_patches();

    test_print();

    test_atomics();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    #[allow(unused)]
    let peripherals = Peripherals::take().unwrap();
    #[allow(unused)]
    let pins = peripherals.pins;

    let mqtt_url = env!("MQTT_URL");

    let sysloop = EspSystemEventLoop::take()?;
    let _wifi = wifi(peripherals.modem, sysloop).unwrap();
    let mut client = get_client(mqtt_url).unwrap();

    let distance_driver = UartDriver::new(
        peripherals.uart1,
        pins.gpio17,
        pins.gpio18,
        Option::<gpio::Gpio21>::None,
        Option::<gpio::Gpio21>::None,
        &Config::new().baudrate(9600.into()),
    )
    .unwrap();

    let temperature_driver = PinDriver::input_output(pins.gpio4).unwrap();
    let mut temperature_bus = OneWire::new(temperature_driver).unwrap();
    let address = find_devices(&mut temperature_bus, &mut delay::Ets).unwrap();

    let mut a2 = adc::AdcChannelDriver::<_, adc::Atten0dB<adc::ADC1>>::new(pins.gpio7)?;

    let mut adc = AdcDriver::new(
        peripherals.adc1,
        &adc::config::Config::new().calibration(true),
    )
    .unwrap();

    let distance: Arc<AtomicU16> = Arc::new(AtomicU16::new(0));

    {
        let distance = distance.clone();
        thread::spawn(move || loop {
            if let Some(value) = read_distance(&distance_driver) {
                distance.store(value, Ordering::Relaxed);
            }
        });
    }

    const COUNT: usize = 30;
    let mut tdc_buffer: VecDeque<u16> = VecDeque::with_capacity(COUNT);
    let mut last_publish = Instant::now() - Duration::from_secs(60);
    loop {
        info!("-----------------------");
        info!("Hello, world!");

        let distance = distance.load(Ordering::Relaxed);
        info!("distance: {}", distance);

        let temperature = read_temperature(&mut temperature_bus, address);
        info!("temperature: {:?}", temperature);

        let tds = read_tds(&mut adc, &mut a2, &mut tdc_buffer, temperature);
        info!("TDS: {:?}", tds);

        let data = MqttData {
            distance,
            temperature,
            tds,
        };

        info!("{data:?}");

        if last_publish.elapsed() > Duration::from_secs(60) {
            last_publish = Instant::now();
            publish_data(data, &mut client);
        }

        thread::sleep(Duration::from_secs(1));
    }
}

fn publish_data(data: MqttData, client: &mut EspMqttClient<ConnState<MessageImpl, EspError>>) {
    let data = serde_json::to_string(&data).unwrap();
    client
        .publish("fishtank/sensors", QoS::AtLeastOnce, false, data.as_bytes())
        .unwrap();
}

fn read_distance(driver: &UartDriver) -> Option<u16> {
    let mut buf: [u8; 4] = [0; 4];
    let x = driver.read(&mut buf, 1000).unwrap();
    // info!("distance: {:x?} {x}", buf);
    if x == 4 && buf[0] == 0xff {
        let value: u16 = (u16::from(buf[1]) << 8) + u16::from(buf[2]);
        Some(value)
    } else {
        None
    }
}

fn read_tds(
    adc: &mut AdcDriver<adc::ADC1>,
    a2: &mut AdcChannelDriver<gpio::Gpio7, Atten0dB<adc::ADC1>>,
    tdc_buffer: &mut VecDeque<u16>,
    temperature: f32,
) -> Option<f32> {
    let value = adc.read(a2).unwrap();
    info!("TDS (raw): {}", value);
    if tdc_buffer.len() >= tdc_buffer.capacity() {
        tdc_buffer.pop_front();
    }
    tdc_buffer.push_back(value);

    // Wait until we have enough samples.
    if tdc_buffer.len() < tdc_buffer.capacity() / 2 {
        return None;
    };

    let medium = median(tdc_buffer.make_contiguous());
    info!("TDS (medium): {tdc_buffer:?} {}", medium);
    let medium = f32::from(medium) * 3.3 / 4096.0;
    info!("TDS (volt medium): {tdc_buffer:?} {}", medium);
    let coefficient = 1.0 + 0.02 * (temperature - 25.0);
    info!("TDS (coefficient): {}", coefficient);
    let compensation = medium / coefficient;
    info!("TDS (compensation): {}", compensation);

    Some(
        133.42 * compensation * compensation * compensation - 255.86 * compensation * compensation
            + 857.39 * compensation,
    )
}

fn read_temperature(
    one_wire_bus: &mut OneWire<PinDriver<gpio::Gpio4, gpio::InputOutput>>,
    address: Address,
) -> f32 {
    let mut buf: [u8; 2] = [0; 2];

    loop {
        one_wire_bus
            .send_command(0x44, Some(&address), &mut delay::Ets)
            .unwrap();
        // thread::sleep(Duration::from_millis(100));
        one_wire_bus
            .send_command(0xBE, Some(&address), &mut delay::Ets)
            .unwrap();
        one_wire_bus.read_bytes(&mut buf, &mut delay::Ets).unwrap();
        one_wire_bus.reset(&mut delay::Ets).unwrap();

        info!("temperature buf: {:x?}", buf);

        if buf != [0xff, 0xff] {
            break;
        }

        thread::sleep(Duration::from_millis(100));
        info!("temperature buf was 0xffff, retrying");
    }

    let temp: i16 = (i16::from(buf[1]) << 8) + i16::from(buf[0]);
    info!("raw temperature: {:?}", temp);
    f32::from(temp) * 0.0625
}

fn median(numbers: &[u16]) -> u16 {
    let mut numbers: Vec<u16> = numbers.to_vec();
    numbers.sort();
    let mid = numbers.len() / 2;
    numbers[mid]
}

#[allow(clippy::vec_init_then_push)]
fn test_print() {
    // Start simple
    println!("Hello from Rust!");

    // Check collections
    let mut children = vec![];

    children.push("foo");
    children.push("bar");
    println!("More complex print {children:?}");
}

#[allow(deprecated)]
fn test_atomics() {
    let a = AtomicUsize::new(0);
    let v1 = a.compare_and_swap(0, 1, Ordering::SeqCst);
    let v2 = a.swap(2, Ordering::SeqCst);

    let (r1, r2) = unsafe {
        // don't optimize our atomics out
        let r1 = core::ptr::read_volatile(&v1);
        let r2 = core::ptr::read_volatile(&v2);

        (r1, r2)
    };

    println!("Result: {r1}, {r2}");
}

#[cfg(not(feature = "qemu"))]
fn wifi(
    modem: impl peripheral::Peripheral<P = esp_idf_hal::modem::Modem> + 'static,
    sysloop: EspSystemEventLoop,
) -> Result<Box<EspWifi<'static>>> {
    use std::net::Ipv4Addr;

    let mut wifi = Box::new(EspWifi::new(modem, sysloop.clone(), None)?);

    info!("Wifi created, about to scan");

    let ap_infos = wifi.scan()?;

    let ours = ap_infos.into_iter().find(|a| a.ssid == SSID);

    let channel = if let Some(ours) = ours {
        info!(
            "Found configured access point {} on channel {}",
            SSID, ours.channel
        );
        Some(ours.channel)
    } else {
        info!(
            "Configured access point {} not found during scanning, will go with unknown channel",
            SSID
        );
        None
    };

    wifi.set_configuration(&Configuration::Client(ClientConfiguration {
        ssid: SSID.into(),
        password: PASS.into(),
        channel,
        ..Default::default()
    }))?;

    wifi.start()?;

    info!("Starting wifi...");

    if !WifiWait::new(&sysloop)?
        .wait_with_timeout(Duration::from_secs(20), || wifi.is_started().unwrap())
    {
        bail!("Wifi did not start");
    }

    info!("Connecting wifi...");

    wifi.connect()?;

    if !EspNetifWait::new::<EspNetif>(wifi.sta_netif(), &sysloop)?.wait_with_timeout(
        Duration::from_secs(20),
        || {
            wifi.is_connected().unwrap()
                && wifi.sta_netif().get_ip_info().unwrap().ip != Ipv4Addr::new(0, 0, 0, 0)
        },
    ) {
        bail!("Wifi did not connect or did not receive a DHCP lease");
    }

    let ip_info = wifi.sta_netif().get_ip_info()?;

    info!("Wifi DHCP info: {:?}", ip_info);

    Ok(wifi)
}

fn find_devices<P, E, D>(one_wire_bus: &mut OneWire<P>, delay: &mut D) -> Option<Address>
where
    P: OutputPin<Error = E> + InputPin<Error = E>,
    E: std::fmt::Debug,
    D: DelayUs<u16>,
{
    loop {
        for device_address in one_wire_bus.devices(false, delay) {
            // The search could fail at any time, so check each result. The iterator automatically
            // ends after an error.
            if let Ok(device_address) = device_address {
                // The family code can be used to identify the type of device
                // If supported, another crate can be used to interact with that device at the given address
                log::info!(
                    "Found device at address {:?} with family code: {:#x?}",
                    device_address,
                    device_address.family_code()
                );

                return Some(device_address);
            } else {
                log::error!("Error searching for devices: {:?}", device_address);
            }
        }

        log::error!("No devices found, retrying");
        thread::sleep(Duration::from_secs(5));
    }
}

fn get_client(url: &str) -> Result<EspMqttClient<ConnState<MessageImpl, EspError>>, EspError> {
    let client_id = format!("fishtank-rust_{}", get_unique_id());
    let conf = MqttClientConfiguration {
        client_id: Some(&client_id),
        keep_alive_interval: Some(std::time::Duration::new(60, 0)),
        lwt: Some(LwtConfiguration {
            topic: "fishtank/status",
            payload: b"offline",
            qos: QoS::AtLeastOnce,
            retain: true,
        }),
        ..Default::default()
    };

    let (mut client, mut connection) = EspMqttClient::new_with_conn(url, &conf)?;

    thread::spawn(move || {
        while let Some(msg) = connection.next() {
            let event = msg.as_ref().unwrap();
            match event {
                Event::Received(_msg) => {}
                Event::Connected(_) => {}
                Event::Disconnected => {}
                Event::Subscribed(_x) => {
                    // Do nothing
                }
                _event => info!("Got unknown MQTT event"),
            }
        }
    });

    client
        .publish("fishtank/status", QoS::AtLeastOnce, true, b"online")
        .unwrap();

    Ok(client)
}

pub fn get_unique_id() -> String {
    let mut mac: [u8; 6] = [0; 6];
    unsafe {
        let ptr = &mut mac as *mut u8;
        esp_efuse_mac_get_default(ptr);
    }
    hex::encode(mac)
}
