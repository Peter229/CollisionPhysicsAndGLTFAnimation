use std::collections::HashMap;
use std::net::{UdpSocket, SocketAddr, IpAddr, Ipv4Addr, SocketAddrV4};
use std::io::{self, Read, Error, stdin, stdout, Write, ErrorKind};


pub struct Dummy {
    pos: cgmath::Vector3<f32>,
}

impl Dummy {
    pub fn new() -> Dummy {

        Dummy { pos: cgmath::Vector3::new(0.0, 0.0, 0.0) }
    }
}

pub struct Server {
    pub players: HashMap<SocketAddr, Dummy>,
    pub players_id: HashMap<u8, SocketAddr>,
}

impl Server {

    pub fn new() -> Server {

        let players: HashMap<SocketAddr, Dummy> = HashMap::new();

        let players_id: HashMap<u8, SocketAddr> = HashMap::new();

        Server { players, players_id }
    }
}

pub struct Client {
    pub server: SocketAddr,
    pub players: HashMap<u8, Dummy>,
    pub id: u8,
}

impl Client {
    pub fn new(is_server: bool) -> Client {

        let players: HashMap<u8, Dummy> = HashMap::new();

        let mut server = "127.0.0.1:9374".parse().unwrap();

        if !is_server {
            let mut sd = String::new();
            print!("Please enter server ip: ");
            let _d = stdout().flush();
            stdin().read_line(&mut sd).expect("Did not enter a correct string");
    
            sd.retain(|c| !c.is_whitespace());
            server = sd.parse().unwrap();
        }
        Client { server, players, id: 0 }
    }
}