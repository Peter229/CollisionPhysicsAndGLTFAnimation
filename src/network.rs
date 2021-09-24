use std::net::{UdpSocket, SocketAddr, IpAddr, Ipv4Addr, SocketAddrV4};
use std::io::{self, Read, Error, stdin, stdout, Write, ErrorKind};

use crate::server;

const ID_SEND: u8 = 1;
const UPDATE_PLAYERS: u8 = 2; 

const CONNECT_REQUEST: u8 = 1;
const SEND_UPDATE: u8 = 2;

pub struct Network {
    socket: UdpSocket,
    pub is_server: bool,
    server: server::Server,
    client: server::Client,
}

impl Network {

    pub fn new() -> Network {

        let mut s = String::new();
        print!("Please enter your ip: ");
        let _ = stdout().flush();

        stdin().read_line(&mut s).expect("Did not enter a correct string");
        s.retain(|c| !c.is_whitespace());
        let socket = UdpSocket::bind(s).expect("couldn't bind to address");
        socket.set_nonblocking(true).expect("Failed to enter non-blocking mode");

        let mut s = String::new();
        print!("Enter Y if server: ");
        let _ = stdout().flush();

        stdin().read_line(&mut s).expect("Did not enter a correct string");
        s.retain(|c| !c.is_whitespace());
        let mut is_server = false;
        if s.chars().nth(0).unwrap() == 'y' {
            println!("You are server");
            is_server = true;
        }

        let server = server::Server::new();

        let client = server::Client::new(is_server);

        Network { socket, is_server, server, client }
    }

    pub fn send(&self) {

        let buf: [u8; 10] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9];
        if self.is_server {
            for ip in self.server.players.keys() {
                self.socket.send_to(&buf, ip).expect("Cant send");
            }
        }
        else {
            self.socket.send_to(&buf, self.client.server).expect("Cant send");
        }
    }

    pub fn recv(&mut self) {

        let mut buf = [0u8; 128];

        loop {
            let result = self.socket.recv_from(&mut buf);
            match result {
                Ok((num_bytes, from)) => {
                    //println!("{}", from);
                    //println!("{:?}", buf);

                    if self.is_server {
                        if self.server.players.contains_key(&from) {
                            //Player exists
                            println!("Player exists");
                        }
                        else {
                            self.s_unknown_connection(&buf, from);
                        }
                    }
                    else {
                        self.c_personal_recieve(&buf);
                    }

                },
                Err(ref err) if err.kind() != ErrorKind::WouldBlock => {
                    println!("Something went wrong: {}", err)
                }
                _ => {
                    break;
                }
            }
        }
    }

    pub fn s_unknown_connection(&mut self, buffer: &[u8; 128], address: SocketAddr) {

        println!("Unknown connection");
        let request_id = buffer[0];
        match request_id {
            CONNECT_REQUEST => {
                println!("Send ID");
                self.server.players.insert(address, server::Dummy::new());
                self.s_send_id(address);
            }
            _ => {

            }
        }
    }

    pub fn s_send_id(&mut self, address: SocketAddr) {

        let mut id = 0;

        for i in 0..u8::MAX {
            id = i;
            if !self.server.players_id.contains_key(&i) {
                break;
            }
        }

        self.server.players_id.insert(id, address);

        let buf: [u8; 10] = [ID_SEND, id, 2, 3, 4, 5, 6, 7, 8, 9];
        self.socket.send_to(&buf, address).expect("Failed to send ID");
    }

    pub fn c_personal_recieve(&mut self, buffer: &[u8; 128]) {

        println!("Personal Message");
        let request_id = buffer[0];
        match request_id {
            ID_SEND => {
                println!("Recieved ID: {}", buffer[1]);
                self.client.id = buffer[1];
                //self.server.players.insert(address, server::Dummy::new());
                //self.s_send_id(address);
            }
            _ => {

            }
        }
    }

    pub fn c_request_connect(&self) {

        let mut id = 0;

        let buf: [u8; 10] = [CONNECT_REQUEST, id, 2, 3, 4, 5, 6, 7, 8, 9];
        self.socket.send_to(&buf, self.client.server).expect("Cant send");
    }
}