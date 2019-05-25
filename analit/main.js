const fs = require('fs');
const path = require('path')

const reader = fs.createReadStream(path.normalize(__dirname + '/accel.txt'));
const writer = fs.createWriteStream(path.normalize(__dirname + '/accel_1.txt'), {
    flags: 'w+'
});
let data = '';

reader.on('data', function (chunk) {
    data += chunk;
});

reader.on('end', function () {
    reader.close();
    let temp = data.split('\n');
    data = [];
    for (let i = 0; i < temp.length; i++) {
        if (temp[i].length == 0) {
            continue;
        }
        if (temp[i][0] != '[') {
            continue;
        }

        temp[i] = temp[i].replace(/\[|\]/gi, '');
        temp[i] = temp[i].replace(/\|/gi, ',');
        data.push(temp[i]);
    }

    data = data.join('\n');

    writer.write(data, function (err) {
        writer.close();
        console.log('OK');
    });
});