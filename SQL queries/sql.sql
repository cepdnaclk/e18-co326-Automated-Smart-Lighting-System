create database smart_lighting_system;

use smart_lighting_system;

CREATE TABLE light_intensity (
    id VARCHAR(36) PRIMARY KEY,
    light_intensity INT NOT NULL,
    sent_time TIMESTAMP NOT NULL,
    received_time TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    rtc_time TIME NOT NULL
);

CREATE TABLE occupancy (
    id VARCHAR(36) PRIMARY KEY,
    occupancy BOOLEAN NOT NULL,
    sent_time TIMESTAMP NOT NULL,
    received_time TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    rtc_time TIME NOT NULL
);





