var websocket = new WebSocket("ws://localhost:8000");

var data = {
	"write": {
		"Auto Alliance Confidence": "",
		"Auto Starting Pose": ""
	},
	"read": {
		"Match Cycle": ""
		//TODO Set more keys we want to read
	}
}

websocket.addEventListener('open', function(event) {
	console.log('Connected to Server');
});

websocket.addEventListener('message', function(event) {
	var info = JSON.parse(event.data);
	console.log(info.key + ": " + info.value);
	if (data.read[info.key] !== undefined) {
		data.read[info.key] = info.value;
	}

	update();
});

websocket.addEventListener('close', function(event) {
	console.log('Closed Connection to Server');
});

$(document).ready(function() {
	update();

	$("#alliance-auto-slider").change(function() {
		$("#alliance-auto-slider-label").text("Confidence In Our Alliance Partners: " + $(this).val());
		data.write["Auto Alliance Confidence"] = parseInt($(this).val());
		websocket.send(JSON.stringify({
			"type": "number",
			"key": "Auto Alliance Confidence",
			"value": data.write["Auto Alliance Confidence"]
		}));
	});

	$("#starting-pose-slider").change(function() {
		var value;
		switch (parseInt($(this).val())) {
			case 0:
				value = "Left";
				break;
			case 1:
				value = "Center";
				break;
			case 2:
				value = "Right";
				break;
		}
		$("#starting-pose-slider-label").text("Starting Position: " + value);
		data.write["Auto Starting Pose"] = parseInt($(this).val());
		websocket.send(JSON.stringify({
			"type": "number",
			"key": "Auto Starting Pose",
			"value": data.write["Auto Starting Pose"]
		}));
	});
});

function update() {
	$("body").children().hide();
	if (data.read["Match Cycle"] != "") {
		$("#" + data.read["Match Cycle"]).show();

		$("tbody").empty();
		for (var key in data.read) {
			$("tbody").append("<tr><td>" + key + "</td><td>"+ data.read[key] + "</td></tr>");
		}
	}
}