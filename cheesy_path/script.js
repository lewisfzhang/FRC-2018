var waypoints = [];
var ctx;
var width = 1656; //pixels
var height = 823; //pixels
var fieldWidth = 886; // in inches
var fieldHeight = 360; // in inches
var pointRadius = 5;
var kEpsilon = 1E-9;
var image;
var imageFlipped;
var wto;

class Translation2d {
	constructor(x, y) {
		this.x = x;
		this.y = y;
	}

	norm() {
		return Math.sqrt(Translation2d.dot(this, this));
	}

	scale(s) {
		return new Translation2d(this.x * s, this.y * s);
	}

	translate(t) {
		return new Translation2d(this.x + t.x, this.y + t.y);
	}

	invert() {
		return new Translation2d(-this.x, -this.y);
	}

	perp() {
		return new Translation2d(-this.y, this.x);
	}

	draw(color) {
		color = color || "#f72c1c";
		ctx.beginPath();
		ctx.arc(this.drawX, this.drawY, pointRadius, 0, 2 * Math.PI, false);
		ctx.fillStyle = color;
		ctx.strokeStyle = color;
		ctx.fill();
		ctx.lineWidth = 0;
		ctx.stroke();
	}

	get drawX() {
		return this.x*(width/fieldWidth);
	}

	get drawY() {
		return height - this.y*(height/fieldHeight);
	}

	get angle() {
		return Math.atan2(-this.y, this.x);
	}

	static diff(a, b) {
		return new Translation2d(b.x - a.x, b.y - a.y);
	}

	static cross(a, b) {
		return a.x * b.y - a.y * b.x;
	}

	static dot(a, b) {
		return a.x * b.x + a.y * b.y;
	}

	static angle(a, b) {
		return Math.acos(Translation2d.dot(a,b) / (a.norm() * b.norm()));
	}
}

class Waypoint {
	constructor(position, heading, comment) {
		this.position = position;
		this.heading = heading;
		this.comment = comment;
	}

	draw() {
		this.position.draw(null);
		var normalizedHeading = normalizeHeading(this.heading);
		var x = this.position.drawX;
		var y = this.position.drawY;
		ctx.beginPath();
		ctx.moveTo(x, y);
		ctx.lineTo(x + 25 * Math.cos(normalizedHeading), y + 25 * Math.sin(normalizedHeading));
		ctx.lineWidth = 3;
		ctx.stroke();
	}

	toString() {
		var comment = (this.comment.length > 0) ? " //" + this.comment : "";
		return "sWaypoints.add(new Waypoint("+this.position.x+","+this.position.y+","+this.heading+"));" + comment;
	}
}

function normalizeHeading(heading) {
	return -((360 + heading) % 360) * (Math.PI / 180);
}

function init() { 
	$("#field").css("width", (width / 1.5) + "px");
	$("#field").css("height", (height / 1.5) + "px");
	ctx = document.getElementById('field').getContext('2d')
    ctx.canvas.width = width;
    ctx.canvas.height = height;
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle="#FF0000";
    image = new Image();
    image.src = 'field.png';
    image.onload = function(){
        ctx.drawImage(image, 0, 0, width, height);
        update();
    }
    imageFlipped = new Image();
    imageFlipped.src = 'fieldflipped.png';
    $('input').bind("change paste keyup", function() {
		console.log("change");
		clearTimeout(wto);
			wto = setTimeout(function() {
			update();
		}, 500);
	});
}

function update() {
	waypoints = [];
	$('tbody').children('tr').each(function () {
        var x = parseInt( $($($(this).children()).children()[0]).val() );
        console.log(x);
        var y = parseInt( $($($(this).children()).children()[1]).val() );
        var heading = parseInt( $($($(this).children()).children()[2]).val() );
        if(isNaN(heading)) {
        	heading = 0;
        }
        var comment = ( $($($(this).children()).children()[3]).val() )
        waypoints.push(new Waypoint(new Translation2d(x,y), heading, comment));
    });
    drawPoints();
}

function clear() {
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle="#FF0000";
    if(flipped)
    	ctx.drawImage(imageFlipped, 0, 0, width, height);
    else
    	ctx.drawImage(image, 0, 0, width, height);
}

function addPoint() {
	var prev;
	if(waypoints.length > 0)
		prev = waypoints[waypoints.length - 1].position;
	else 
		prev = new Translation2d(50, 50);
	$("tbody").append("<tr>"
		+"<td><input value='"+(prev.x+20)+"'></td>"
		+"<td><input value='"+(prev.y+20)+"'></td>"
		+"<td><input value='0'></td>"
		+"<td class='comments'><input placeholder='Comments'></td>"
		+"<td><button onclick='$(this).parent().parent().remove();update()'>Delete</button></td></tr>"
	);
	update();
	$('input').unbind("change paste keyup");
	$('input').bind("change paste keyup", function() {
		console.log("change");
		clearTimeout(wto);
			wto = setTimeout(function() {
			update();
		}, 500);
	});
}

function drawPoints() {
	clear();
	for (var i = 0; i < waypoints.length; i++) {
		getPoint(i).draw();
	}
}

function getPoint(i) {
	if(i >= waypoints.length)
		return waypoints[waypoints.length - 1];
	else
		return waypoints[i];
}

function showModal() {
	$(".modal, .shade").removeClass("behind");
	$(".modal, .shade").removeClass("hide");
}

function closeModal() {
	$(".modal, .shade").addClass("hide");
	setTimeout(function() {
		$(".modal, .shade").addClass("behind");
	}, 500);
}

var flipped = false;
function flipField() {
	flipped = !flipped;
	if(flipped)
		ctx.drawImage(imageFlipped, 0, 0, width, height);
	else
		ctx.drawImage(image, 0, 0, width, height);
	update();
}

function hexToRGB(hex) {
    var result = /^#?([a-f\d]{2})([a-f\d]{2})([a-f\d]{2})$/i.exec(hex);
    return result ? [
        parseInt(result[1], 16),
        parseInt(result[2], 16),
        parseInt(result[3], 16)
    ] : null;
}

function RGBToHex(rgb) {
    return "#" + ((1 << 24) + (rgb[0] << 16) + (rgb[1] << 8) + rgb[2]).toString(16).slice(1);
}