const fs = require('fs');
const path = require('path');
const reader1 = fs.createReadStream(path.normalize(__dirname + '/AL_3_3.txt'));
const reader2 = fs.createReadStream(path.normalize(__dirname + '/G_3_3.txt'));

const writer1 = fs.createWriteStream(path.normalize(__dirname + '/AL_3_3_0.txt'), {
    flags: 'w+'
});
const writer2 = fs.createWriteStream(path.normalize(__dirname + '/G_3_3_0.txt'), {
    flags: 'w+'
});
let accelData = '';
let gyroData = '';
let pointer = 0;

reader1.on('data', (chunk) => {
    accelData += chunk;
});

reader2.on('data', (chunk) => {
    gyroData += chunk;
});

reader1.on('end', () => {
    console.log('accel end');
    reader1.close();
    handleData();
});

reader2.on('end', () => {
    console.log('gyro end');
    reader2.close();
    handleData();
});

function handleData() {
    pointer++;
    if (pointer != 2) {
        return;
    }
    let accelRaw = [];
    accelRaw = accelData.split('\n');
    let gyroRaw = [];
    gyroRaw = gyroData.split('\n');
    let accelFiltered = [];
    let gyroFiltered = [];
    let item = [];
    all:
    for (let i = 0; i < accelRaw.length; i++) {
        item = accelRaw[i].split(',');
        for (let j = 0; j < item.length; j++) {
            item[j] = Number(item[j]);
            if (Math.abs(item[j]) > 100) {
                continue all;
            }
        }

        item = gyroRaw[i].split(',');
        for (let j = 0; j < item.length; j++) {
            item[j] = Number(item[j]);
            if (Math.abs(item[j]) > 100) {
                continue all;
            }
        }
        
        accelFiltered.push(accelRaw[i]);
        gyroFiltered.push(gyroRaw[i]);
    }
    
    accelFiltered = accelFiltered.join('\n');
    gyroFiltered = gyroFiltered.join('\n');


    writer1.write(accelFiltered, (err) => {
        writer1.close();
    });

    writer2.write(gyroFiltered, (err) => {
        writer2.close();
    });
}