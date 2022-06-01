setInterval(function () {
    // Call a function repetatively with 2 Second interval
    getData();
}, 1000); //2000mSeconds update rate



var CO2, VOC, AI, light, temp_C, temp_F;
var CO2_ready = false;
var VOC_ready = false;
function getData() {
    var xhttp_co2 = new XMLHttpRequest();
    xhttp_co2.open("GET", "co2Val", true);
    xhttp_co2.send();

    xhttp_co2.onreadystatechange = function () {
        if (this.readyState == 4 && this.status == 200) {
            CO2_ready = true;
            CO2 = parseInt(this.responseText);
            document.getElementById("co2").innerHTML = CO2;
        }
    };


    var xhttp_voc = new XMLHttpRequest();
    xhttp_voc.open("GET", "vocVal", true);
    xhttp_voc.send();

    xhttp_voc.onreadystatechange = function () {
        if (this.readyState == 4 && this.status == 200) {
            VOC_ready = true;
            VOC = parseInt(this.responseText);
            document.getElementById("voc").innerHTML = VOC;

        }
    };

    var xhttp_hum = new XMLHttpRequest();
    xhttp_hum.open("GET", "humVal", true);
    xhttp_hum.send();

    xhttp_hum.onreadystatechange = function () {
        if (this.readyState == 4 && this.status == 200) {
            document.getElementById("Humidity").innerHTML =
                this.responseText;
        }
    };

    var xhttp_press = new XMLHttpRequest();
    xhttp_press.open("GET", "presVal", true);
    xhttp_press.send();

    xhttp_press.onreadystatechange = function () {
        if (this.readyState == 4 && this.status == 200) {
            document.getElementById("Air-Pressure").innerHTML =
                this.responseText;
        }
    };

    if (CO2_ready && VOC_ready) {
        CO2_ready = false;
        VOC_ready = false;
        AI = (CO2 + VOC) / 10;
        if (AI < 0) {
            AI = 0;
            gauge.set(AI); // set actual value
        }
        else if (AI > 500) {
            gauge.set(500); // set actual value
        }
        else {
            gauge.set(AI); // set actual value
        }
        // console.log(AI);        
        document.getElementById("Air-Quality-value").innerHTML = AI;

        var AI_mood, AI_col;

        if (AI < 50) {
            AI_mood = "Good 😎";
            AI_col = "#82D92B";
        }
        else if (AI < 100) {
            AI_mood = "Moderate 😄";
            AI_col = "#FFDF36";
        }
        else if (AI < 150) {
            AI_mood = "Poor 😔";
            AI_col = "#FF8329";
        }
        else if (AI < 200) {
            AI_mood = "Unhealthy 😟";
            AI_col = "#F01861";
        }
        else if (AI < 300) {
            AI_mood = "Very Unhealthy 😨";
            AI_col = "#E40099";
        }
        else {
            AI_mood = "Hazardous 🤢";
            AI_col = "#B614BF";
        }

        document.getElementById("Air-Quality-type").style.color = AI_col;
        document.getElementById("Air-Quality-type").innerHTML = AI_mood;
    }

    var xhttp_light = new XMLHttpRequest();
    xhttp_light.open("GET", "lightVal", true);
    xhttp_light.send();

    xhttp_light.onreadystatechange = function () {
        if (this.readyState == 4 && this.status == 200) {
            light = parseInt(this.responseText);
            document.getElementById("Ambient-light").innerHTML = light;
            var bright = map_range(light, 0, 1500, 0, 130);
            if (bright < 0) {
                bright = 0;
            }
            else if (bright > 130) {
                bright = 130;
            }
            var g = map_range(bright, 0, 130, 255, 196);
            var b = map_range(bright, 0, 130, 255, 0);
            document.getElementById("Bulb").style.backgroundColor = "rgba(255," + g + "," + b + ",0.85)";
            document.getElementById("Bulb").style.boxShadow = "0px 0px 110px " + bright + "px rgb(255 207 120)";
        }
    };

    var xhttp_temp = new XMLHttpRequest();
    xhttp_temp.open("GET", "tmpVal", true);
    xhttp_temp.send();

    xhttp_temp.onreadystatechange = function () {
        if (this.readyState == 4 && this.status == 200) {
            temp_C = parseInt(this.responseText);
            temp_F = (temp_C * 1.8) + 32;
			temp_F = (Math.round(temp_F * 100) / 100).toFixed(2);
            currentTemp = temp_C;
            if (currentTemp > 90) {
                currentTemp = 90;
            }
            else if (currentTemp < -15) {
                currentTemp = -15;
            }
            tubeFill_top = scale(currentTemp);

            d3.select("#tmpV").remove();
            svg
                .append("rect")
                .attr("id", "tmpV")
                .attr("x", width / 2 - (tubeWidth - 10) / 2)
                .attr("y", tubeFill_top)
                .attr("width", tubeWidth - 10)
                .attr("height", tubeFill_bottom - tubeFill_top)
                .style("shape-rendering", "crispEdges")
                .style("fill", mercuryColor);
            // Main thermometer bulb fill
            svg
                .append("circle")
                .attr("r", bulbRadius - 6)
                .attr("cx", bulb_cx)
                .attr("cy", bulb_cy)
                .style("fill", "url(#bulbGradient)")
                .style("stroke", mercuryColor)
                .style("stroke-width", "2px");

            document.getElementById("temp-in-c").innerHTML = temp_C;
            document.getElementById("temp-in-f").innerHTML = temp_F;
        }
    };
}

function map_range(value, low1, high1, low2, high2) {
    return low2 + (high2 - low2) * (value - low1) / (high1 - low1);
}