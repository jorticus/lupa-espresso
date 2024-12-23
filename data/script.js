//const gateway = `ws://${window.location.hostname}/ws`;
const gateway = `ws://lupa/ws`;
const url = 'http://lupa/sensors';
const interval = 1000; // 1 second
const max_samples = 60 * 2 * 10;

var websocket;

const temperatureCtx = document.getElementById('tempChart').getContext('2d');
const tempChart = new Chart(temperatureCtx, {
type: 'line',
data: {
labels: [],  // Time labels
datasets: [{
label: 'Boiler Temperature (C)',
data: [],
borderColor: 'rgb(75, 192, 192)',
backgroundColor: 'rgba(75, 192, 192, 0.2)',
fill: true
},{
label: 'Hx Temperature (C)',
data: [],
borderColor: 'rgb(255, 99, 132)',
backgroundColor: 'rgba(255, 99, 132, 0.2)',
fill: true
}]
},
options: { scales: {
x: {
	type: 'time',   // Use time scale
	time: { unit: 'second', tooltipFormat: 'yyyy-MM-dd HH:mm:ss' },
	title: { display: true, text: 'Time' }
},
y: {
	beginAtZero: false,
	suggestedMin: 20.0,
	suggestedMax: 150.0,
	title: { display: true, text: 'Temperature (C)' }
},
}
}});

// Pressure Chart setup
const pressureCtx = document.getElementById('pressureChart').getContext('2d');
const pressureChart = new Chart(pressureCtx, {
type: 'line',
data: {
labels: [],
datasets: [{
	label: 'Pressure (hPa)',
	data: [],
	borderColor: 'rgb(255, 99, 132)',
	backgroundColor: 'rgba(255, 99, 132, 0.2)',
	fill: true
}]
},
options: { scales: {
x: {
	type: 'time',
	time: { unit: 'second', tooltipFormat: 'yyyy-MM-dd HH:mm:ss' },
	title: { display: true, text: 'Time' }
},
y: {
	beginAtZero: false,
	suggestedMin: 1.0,
	suggestedMax: 12.0,
	title: { display: true, text: 'Pressure (Bar)' }
}
}}
});

// Update the chart with new data
function updateChart(response) {
	const temp1     = parseFloat(response.t1);
	const temp2     = parseFloat(response.t2);
	const pressure  = parseFloat(response.p1);

	const now = new Date();

	tempChart.data.labels.push(now);
	pressureChart.data.labels.push(now);
	// pidChart.data.labels.push(now);

	tempChart.data.datasets[0].data.push(temp1);
	tempChart.data.datasets[1].data.push(temp2);
	pressureChart.data.datasets[0].data.push(pressure);

	// pidChart.data.datasets[0].data.push(parseFloat(response.tsp));
	// pidChart.data.datasets[1].data.push(parseFloat(response.t1));
	// pidChart.data.datasets[2].data.push(parseFloat(response.pid_i));
	// pidChart.data.datasets[3].data.push(parseFloat(response.pid_d1));
	// pidChart.data.datasets[4].data.push(parseFloat(response.pid_d2));

	if (tempChart.data.labels.length > max_samples) {  // Limit to 60 data points
		tempChart.data.labels.shift();
		tempChart.data.datasets[0].data.shift();
		tempChart.data.datasets[1].data.shift();
	}
	if (pressureChart.data.labels.length > max_samples) {
		pressureChart.data.labels.shift();
		pressureChart.data.datasets[0].data.shift();
	}
	// if (pidChart.data.labels.length > max_samples) {
	// 	pidChart.data.labels.shift();
	// 	for (i = 0; i < pidChart.data.datasets.length; i++) {
	// 		pidChart.data.datasets[i].data.shift();
	// 	}
	// }


	tempChart.update();
	pressureChart.update();
	// pidChart.update();
}

// function getReadings(){
//     websocket.send("getReadings");
// }

function onOpen(event) {
	console.log('Connection opened');
	//getReadings();
}

function onMessage(event) {
	console.log(event.data);
	var json = JSON.parse(event.data);
	updateChart(json);
}

function onClose(event) {
	console.log('Connection closed');
	setTimeout(initWebSocket, 2000);
}

function initWebSocket() {
	websocket = new WebSocket(gateway);
	websocket.onopen = onOpen;
	websocket.onclose = onClose;
	websocket.onmessage = onMessage;
}

function onLoad() {
	initWebSocket();
}


window.addEventListener('load', onLoad);

// Fetch and update data
async function fetchData() {
	try {
		const response = await fetch(url);
		const json = await response.json();
		
		updateChart(json);
	} catch (error) {
		console.error('Error fetching data:', error);
	}
}

//setInterval(fetchData, interval);