var opts = {
    angle: 0, // The span of the gauge arc
    lineWidth: 0.2, // The line thickness
    radiusScale: 1, // Relative radius

    staticLabels: {
        font: "15px sans-serif ",  // Specifies font
        
        labels: [0, 50, 100, 150, 200, 300, 500],  // Print labels at these values
        color: "#fff",  // Optional: Label text color
        fractionDigits: 0  // Optional: Numerical precision. 0=round off.
    },

    pointer: {
        length: 0.55, // // Relative to gauge radius
        strokeWidth: 0.055, // The thickness
        color: '#fff' // Fill color

    },
    staticZones: [
        { strokeStyle: "#82D92B", min: 0, max: 50}, // Red from 100 to 130
        { strokeStyle: "#FFDF36", min: 50, max: 100}, // Yellow
        { strokeStyle: "#FF8329", min: 100, max: 150}, // Green
        { strokeStyle: "#F01861", min: 150, max: 200}, // Yellow
        { strokeStyle: "#E40099", min: 200, max: 300} , // Red
        { strokeStyle: "#B614BF", min: 300, max: 500}  // Red
    ],

   

    limitMax: false,     // If false, max value increases automatically if value > maxValue
    limitMin: false,     // If true, the min value of the gauge will be fixed
    colorStart: '#00609a',   // Colors
    colorStop: '#00609a',    // just experiment with them
    strokeColor: '#E0E0E0',  // to see which ones work best for you

    generateGradient: true,
    highDpiSupport: true,     // High resolution support

};
var target = document.getElementById('Gaugetttt'); // your canvas element
var gauge = new Gauge(target).setOptions(opts); // create sexy gauge!

gauge.maxValue = 500; // set max gauge value
gauge.setMinValue(0);  // Prefer setter over gauge.minValue = 0
gauge.animationSpeed = 32; // set animation speed (32 is default value)
gauge.set(60); // set actual value











