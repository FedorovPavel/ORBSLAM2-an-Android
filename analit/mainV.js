function initPlot(data) {
    var lines = data.split('\n');

    // chart vars
    var accelX = {
        x: [],
        y: [],
        mode: 'lines',
        name: 'AX',
    }

    var accelY = {
        x: [],
        y: [],
        mode: 'lines',
        name: 'AY',
    }

    var accelZ = {
        x: [],
        y: [],
        mode: 'lines',
        name: 'AZ'
    };

    var velocityX = {
        x: [],
        y: [],
        mode: 'lines',
        name: 'VX'
    }

    var velocityX1 = {
        x: [],
        y: [],
        mode: 'lines',
        name: 'VX1'
    }

    var velocityY = {
        x: [],
        y: [],
        mode: 'lines',
        name: 'VY'
    }

    var velocityY1 = {
        x: [],
        y: [],
        mode: 'lines',
        name: 'VY1'
    }

    var velocityZ = {
        x: [],
        y: [],
        mode: 'lines',
        name: 'VZ'
    }

    var velocityY1 = {
        x: [],
        y: [],
        mode: 'lines',
        name: 'VZ1'
    }

    var positionX = {
        x: [],
        y: [],
        mode: 'lines',
        name: 'PX'
    }

    var positionX1 = {
        x: [],
        y: [],
        mode: 'lines',
        name: 'PX1'
    }

    var positionY = {
        x: [],
        y: [],
        mode: 'lines',
        name: 'PY'
    }

    var positionY1 = {
        x: [],
        y: [],
        mode: 'lines',
        name: 'PY1'
    }

    var positionZ = {
        x: [],
        y: [],
        mode: 'lines',
        name: 'PZ'
    }

    var positionZ1 = {
        x: [],
        y: [],
        mode: 'lines',
        name: 'PZ1'
    }

    var avgX = {
        x: [],
        y: [],
        mode: 'lines',
        name: 'avgX'
    }

    var layout = {
        yaxis: {range: [-1, 1]},
        xaxis: {range: [0, 100]}
    };
    var time = 0;

    
    let temp;
    var dt = undefined;
    let temp2;
    let t;
    let prevT = undefined;
    let prevA = 0;
    let K = 0.15;
    let Kalm;
    let prevV = 0;
    let prevP = 0;

    prevT = Number(lines[0].split(',')[0]);
    for (let i = 1; i < lines.length; i++) {
        temp = lines[i].split(',');
        dt = (Number(temp[0]) - prevT)/1000000000.0;
        time += dt;

        accelX.x.push(time);
        accelX.y.push(Number(temp[1]));

        // accelY.x.push(time);
        // accelY.y.push(Number(temp[2]));

        // accelZ.x.push(time);
        // accelZ.y.push(Number(temp[3]));

        velocityX.x.push(time);
        velocityX.y.push(Number(temp[11]));

        positionX.x.push(time);
        positionX.y.push(Number(temp[8]));


        // positionY.x.push(time);
        // positionY.y.push(Number(temp[9]));

        // positionZ.x.push(time);
        // positionZ.y.push(Number(temp[10]));
        avgX.x.push(time);
        
        // Kalm = Number(temp[1]);
        Kalm = K * Number(temp[1]) + (1-K) * prevA;
        avgX.y.push(Kalm);

        if (avgX.y.length >= 2) {
            velocityX1.x.push(time);
            if (Math.sqrt(Kalm * Kalm) > 0.05) {
                velocityX1.y.push(prevV + (Kalm + prevA) / 2 * dt);
            }else {
                velocityX1.y.push(0);
            }
            prevV = velocityX1.y[velocityX1.y.length-1];
        }

        if (velocityX1.x.length >= 2){
            positionX1.x.push(time);
            positionX1.y.push(prevP + (prevV + velocityX1.y[velocityX1.y.length-2])/2*dt);
            prevP = positionX1.y[positionX1.y.length - 1];
        }
        prevA = Kalm;
        // avgX.y.push(Number(temp[20]));
        prevT = Number(temp[0]);
    }

    Plotly.newPlot('plot', [accelX, accelY, accelZ, velocityX, positionX, positionX1, velocityX1, avgX], layout);
}

function mean(values) {
    var capaciter = 0;
    for(var i = 0; i < values.length; i++)
        capaciter += values[i];

    return capaciter / values.length;
}

function std(values) {
    var meanVal = mean(values);
    var quadroVals = values.map(function(val) {
        return Math.pow(val - meanVal, 2);
    });

    var quadroSum = 0;
    for(var i = 0; i < quadroVals.length; i++)
        quadroSum += quadroVals[i];

    return Math.sqrt(quadroSum/values.length);
}

window.onload = function () { 
    var fileSelected = document.getElementById('file-input');
    fileSelected.addEventListener('change', function (e) { 
        var fileTobeRead = fileSelected.files[0];
        var fileReader = new FileReader(); 
        fileReader.onload = function (e) {
            var fileContents = document.getElementById('filecontents'); 
            initPlot(fileReader.result); 
        }
        fileReader.readAsText(fileTobeRead); 
    }, false);
} 

// function initPlot(data) {
//     var lines = data.split('\n');

//     // chart vars
//     var accelX = {
//         x: [],
//         y: [],
//         mode: 'lines',
//         name: 'AX',
//     }

//     var accelAvgX = {
//         x: [],
//         y: [],
//         mode: 'lines',
//         name: 'avgAX',
//     }

//     var accelY = {
//         x: [],
//         y: [],
//         mode: 'lines',
//         name: 'AY',
//     }

//     var accelAvgY = {
//         x: [],
//         y: [],
//         mode: 'lines',
//         name: 'avgAY',
//     }

//     var accelZ = {
//         x: [],
//         y: [],
//         mode: 'lines',
//         name: 'AZ'
//     };

//     var accelAvgZ = {
//         x: [],
//         y: [],
//         mode: 'lines',
//         name: 'avgAZ',
//     }

//     var velocityX = {
//         x: [],
//         y: [],
//         mode: 'lines',
//         name: 'VX'
//     }

//     var velocityAvgX = {
//         x: [],
//         y: [],
//         mode: 'lines',
//         name: 'avgVX'
//     }

//     var velocityAvgY = {
//         x: [],
//         y: [],
//         mode: 'lines',
//         name: 'avgVY'
//     }

//     var velocityAvgZ = {
//         x: [],
//         y: [],
//         mode: 'lines',
//         name: 'avgVZ'
//     }

//     var positionAvgX = {
//         x: [],
//         y: [],
//         mode: 'lines',
//         name: 'avgPX'
//     }

//     var positionAvgY = {
//         x: [],
//         y: [],
//         mode: 'lines',
//         name: 'avgPY'
//     }

//     var positionAvgZ = {
//         x: [],
//         y: [],
//         mode: 'lines',
//         name: 'avgPZ'
//     }

//     var layout = {
//         // yaxis: {range: [-0.2, 0.2]},
//         // xaxis: {range: [0, 1]}
//     };
//     var time = 0;
//     var sample = 2;

    
//     let temp;
//     var dt = undefined;
//     let temp2;
//     let t;
//     let prevT = undefined;
//     let curAvgA = [0,0,0];
//     let prevAvgA = [0,0,0];
//     let prevAvgV = [0,0,0];
//     let curAvgV = [0,0,0];
//     let prevAvgP = [0,0,0];
//     let curAvgP = [0,0,0];
//     for (let i = 0; i < lines.length; i++) {
//         temp = lines[i].split(',');
//         time += Number(temp[3]);
//         dt = (Number(temp[3]) - Number(prevT)) / 1000000000.0;
//         accelX.y.push(Number(temp[0]));
//         accelX.x.push(Number(time));
//         accelY.y.push(Number(temp[1]));
//         accelY.x.push(Number(time));
//         accelZ.y.push(Number(temp[2]));
//         accelZ.x.push(Number(time));

//         //  Avg
//         if (i > sample) {
//             accelAvgX.x.push(time);
//             curAvgA[0] = mean(accelX.y.slice(i - sample, i));
//             accelAvgX.y.push(curAvgA[0]);
//             accelAvgY.x.push(time);
//             curAvgA[1] = mean(accelY.y.slice(i - sample, i));
//             accelAvgY.y.push(curAvgA[1]);
//             accelAvgZ.x.push(time);
//             curAvgA[2] = mean(accelZ.y.slice(i - sample, i));
//             accelAvgZ.y.push(curAvgA[2]);

//             if (i > sample + 1) {
//                 velocityAvgX.x.push(time);
//                 curAvgV[0] = prevAvgV[0] + dt*(prevAvgA[0]+curAvgA[0])/2;
//                 velocityAvgX.y.push(curAvgV[0]);

//                 velocityAvgY.x.push(time);
//                 curAvgV[1] = prevAvgV[1] + dt*(prevAvgA[1]+curAvgA[1])/2;
//                 velocityAvgY.y.push(curAvgV[1]);

//                 velocityAvgZ.x.push(time);
//                 curAvgV[2] = prevAvgV[2] + dt*(prevAvgA[2]+curAvgA[2])/2;
//                 velocityAvgZ.y.push(curAvgV[2]);

//                 if (i > sample + 2) {
//                     positionAvgX.x.push(time);
//                     curAvgP[0] = prevAvgP[0] + (prevAvgV[0] + curAvgV[0]) * dt / 2;
//                     positionAvgX.y.push(curAvgP[0]);
//                     positionAvgY.x.push(time);
//                     curAvgP[1] = prevAvgP[1] + (prevAvgV[1] + curAvgV[1]) * dt / 2;
//                     positionAvgY.y.push(curAvgP[1]);
//                     positionAvgZ.x.push(time);
//                     curAvgP[2] = prevAvgP[2] + (prevAvgV[2] + curAvgV[2]) * dt / 2
//                     positionAvgZ.y.push(curAvgP[2]);
//                     prevAvgP =  curAvgP;
//                 }

//                 prevAvgV = curAvgV;
//             }

//             prevAvgA = curAvgA;  
//         }
    
//         prevT = Number(temp[3]);
//     }

//     // main loop
    
//     // for(var i = lag; i < lines.length; i++) {
//     //     var lineData = lines[i].split('/');

//     //     time = i;// Number(lineData[0].replace(',','.'));
//     //     var diffz = Number(lineData[1].replace(',','.'));
//     //     var speed = Number(lineData[2].replace(',','.'));

//     //     diff.x.push(time);
//     //     diff.y.push(diffz);

//     //     speedChart.x.push(time);
//     //     speedChart.y.push(speed);

//     //     // var koef = 98;
//     //     // var C = 0.02 * koef;
//     //     // if (speed == 0)
//     //     //     C = 0.00155 * koef;
//     //     // else if (speed >= 1 && speed < 10)
//     //     //     C = 0.00425 * koef;
//     //     // else if (speed >= 10 && speed < 20)
//     //     //     C = 0.00575 * koef;
//     //     // else if (speed >= 20 && speed < 30)
//     //     //     C = 0.00675 * koef;
//     //     // else if (speed >= 30 && speed < 40)
//     //     //     C = 0.00775 * koef;
//     //     // else if (speed >= 40 && speed < 50)
//     //     //     C = 0.009 * koef;
//     //     // else if (speed >= 50 && speed < 60)
//     //     //     C = 0.01 * koef;
//     //     // else if (speed >= 60)
//     //     //     C = 0.011 * koef;

//     //     if(speed == 0)
//     //         C = 1;
//     //     else
//     //         C = 0.005977011 * speed + 0.321264395;

//     //     ckoef.x.push(time);
//     //     ckoef.y.push(C);

//     //     if (eventStartFound) {
//     //         if (diffz < C) {
//     //             eventEndFoundCounter++;
//     //             if(eventTempEndTime == -1)
//     //                 eventTempEndTime = time
//     //             if (eventEndFoundCounter >= 24) {
//     //                 eventEndFound = true;
//     //                 eventStops.x.push(eventTempEndTime);
//     //                 eventStops.y.push(1);
//     //                 eventEndFoundCounter = 0;
//     //                 eventTempEndTime = -1;
//     //             }
//     //         } else {
//     //             eventEndFoundCounter = 0;
//     //             eventTempEndTime = -1;
//     //         }
//     //     } else {
//     //         if (diffz > C) {
//     //             eventStartFound = true;
//     //             eventStarts.x.push(time);
//     //             eventStarts.y.push(1);
//     //         }
//     //     }

//     //     if(eventStartFound && eventEndFound) {
//     //         eventEndFound = false;
//     //         eventStartFound = false;
//     //     }

//     //     diffAndSpeed.x.push(speed);
//     //     diffAndSpeed.y.push(diffz);

//     //     if(Math.abs(y[i] - avgFilter[i-1]) > threshold*stdFilter[i-1]) {
//     //         if(y[i] > avgFilter[i-1])
//     //             signals[i]+=0.1;
        
//     //         filteredY[i] = influence*y[i] + (1-influence)*filteredY[i-1];
//     //     } else {
//     //         signals[i] = 1;
//     //         filteredY[i] = y[i];
//     //     }

//     //     avgFilter[i] = mean(filteredY.slice(i-lag, i));
//     //     stdFilter[i] = std(filteredY.slice(i-lag, i));

//     //     signalChart.x.push(time);
//     //     signalChart.y.push(signals[i]);

//     //     porogChart.x.push(time);
//     //     porogChart.y.push(avgFilter[i]+threshold*stdFilter[i]);
//     // }
    
//     Plotly.newPlot('plot', [accelX, accelAvgX, velocityAvgX, positionAvgX], layout);
//     //Plotly.newPlot('plot', [diff, signalChart, porogChart, ckoef, eventStarts, eventStops]);
//     //Plotly.newPlot('plot', [diffAndSpeed, ckoef]);
// }

// function mean(values) {
//     var capaciter = 0;
//     for(var i = 0; i < values.length; i++)
//         capaciter += values[i];

//     return capaciter / values.length;
// }

// function std(values) {
//     var meanVal = mean(values);
//     var quadroVals = values.map(function(val) {
//         return Math.pow(val - meanVal, 2);
//     });

//     var quadroSum = 0;
//     for(var i = 0; i < quadroVals.length; i++)
//         quadroSum += quadroVals[i];

//     return Math.sqrt(quadroSum/values.length);
// }

// window.onload = function () { 
//     var fileSelected = document.getElementById('file-input');
//     fileSelected.addEventListener('change', function (e) { 
//         var fileTobeRead = fileSelected.files[0];
//         var fileReader = new FileReader(); 
//         fileReader.onload = function (e) {
//             var fileContents = document.getElementById('filecontents'); 
//             initPlot(fileReader.result); 
//         }
//         fileReader.readAsText(fileTobeRead); 
//     }, false);
// } 