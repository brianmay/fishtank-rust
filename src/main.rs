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
use smart_leds::{SmartLedsWrite, RGB8};
use ws2812_esp32_rmt_driver::driver::color::LedPixelColorGrb24;
use ws2812_esp32_rmt_driver::LedPixelEsp32Rmt;

const SSID: &str = env!("WIFI_SSID");
const PASS: &str = env!("WIFI_PASS");

#[derive(Serialize, Debug)]
struct MqttData {
    distance: u16,
    temperature: f32,
    tds: f32,
}

fn main() -> Result<()> {
    esp_idf_sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    #[allow(unused)]
    let peripherals = Peripherals::take().unwrap();
    #[allow(unused)]
    let pins = peripherals.pins;

    let mut ws2812 = LedPixelEsp32Rmt::<RGB8, LedPixelColorGrb24>::new(0, 48).unwrap();
    set_led(&mut ws2812, RGB8::new(0x01, 0, 0));

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

    let mut temperature_buffer: VecDeque<f32> = VecDeque::with_capacity(3);
    let mut tds_buffer: VecDeque<f32> = VecDeque::with_capacity(30);
    let mut last_publish = Instant::now() - Duration::from_secs(60);
    loop {
        let enough_samples = tds_buffer.len() >= 15 - 1;
        let colour = if enough_samples {
            RGB8::new(0, 0x01, 0)
        } else {
            RGB8::new(0, 0, 0x01)
        };

        info!("-----------------------");

        set_led(&mut ws2812, colour);

        {
            let temperature = read_temperature(&mut temperature_bus, address);
            let tds = read_tds(&mut adc, &mut a2, temperature);
            add_sample(&mut temperature_buffer, temperature);
            add_sample(&mut tds_buffer, tds);
        }

        let distance = distance.load(Ordering::Relaxed);
        let temperature = median(&temperature_buffer);
        let tds = median(&tds_buffer);

        if enough_samples {
            let data = MqttData {
                distance,
                temperature,
                tds,
            };

            if last_publish.elapsed() > Duration::from_secs(60) {
                last_publish = Instant::now();
                info!("publishing {data:?}");
                publish_data(data, &mut client);
            } else {
                info!("read {data:?}");
            }
        } else {
            info!("waiting for more tds samples");
        }

        set_led(&mut ws2812, RGB8::new(0, 0, 0));
        thread::sleep(Duration::from_secs(1));
    }
}

fn set_led<T>(ws2812: &mut T, pixel: T::Color)
where
    T: SmartLedsWrite,
    T::Error: std::fmt::Debug,
{
    ws2812.write(vec![pixel].into_iter()).unwrap_or_else(|e| {
        error!("error setting led: {:?}", e);
    });
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
    temperature: f32,
) -> f32 {
    let value_raw = adc.read(a2).unwrap();
    let value_volts = f32::from(value_raw) * 3.3 / 4096.0;
    let coefficient = 1.0 + 0.02 * (temperature - 25.0);
    let compensation = value_volts / coefficient;
    let tds = (133.42 * compensation * compensation * compensation
        - 255.86 * compensation * compensation
        + 857.39 * compensation)
        * 0.5;
    info!("TDS: {value_raw} {value_volts} {coefficient} {compensation} {tds}");

    tds
}

fn read_temperature(
    one_wire_bus: &mut OneWire<PinDriver<gpio::Gpio4, gpio::InputOutput>>,
    address: Address,
) -> f32 {
    loop {
        one_wire_bus
            .send_command(0x44, Some(&address), &mut delay::Ets)
            .unwrap();

        loop {
            let v = one_wire_bus.read_bit(&mut delay::Ets).unwrap();
            if v {
                break;
            }
        }

        let mut buf: [u8; 2] = [0; 2];
        one_wire_bus
            .send_command(0xBE, Some(&address), &mut delay::Ets)
            .unwrap();
        one_wire_bus.read_bytes(&mut buf, &mut delay::Ets).unwrap();
        one_wire_bus.reset(&mut delay::Ets).unwrap();

        let temp: i16 = (i16::from(buf[1]) << 8) + i16::from(buf[0]);
        let temp = f32::from(temp) * 0.0625;
        info!("temperature: {buf:x?} {temp:?}");

        if (0.0..=40.0).contains(&temp) {
            break temp;
        }

        thread::sleep(Duration::from_millis(100));
        info!("temperature was outside reasonable limits, retrying");
    }
}

fn add_sample(buffer: &mut VecDeque<f32>, sample: f32) {
    if buffer.len() >= buffer.capacity() {
        buffer.pop_front();
    }
    buffer.push_back(sample);
}

fn median(buffer: &VecDeque<f32>) -> f32 {
    let mut numbers: Vec<_> = buffer.clone().make_contiguous().to_vec();
    numbers.sort_by(|a, b| a.total_cmp(b));
    let mid = numbers.len() / 2;
    numbers[mid]
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
