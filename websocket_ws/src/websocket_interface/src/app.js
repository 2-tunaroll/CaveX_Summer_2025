"use strict";
var __awaiter = (this && this.__awaiter) || function (thisArg, _arguments, P, generator) {
    function adopt(value) { return value instanceof P ? value : new P(function (resolve) { resolve(value); }); }
    return new (P || (P = Promise))(function (resolve, reject) {
        function fulfilled(value) { try { step(generator.next(value)); } catch (e) { reject(e); } }
        function rejected(value) { try { step(generator["throw"](value)); } catch (e) { reject(e); } }
        function step(result) { result.done ? resolve(result.value) : adopt(result.value).then(fulfilled, rejected); }
        step((generator = generator.apply(thisArg, _arguments || [])).next());
    });
};
var __generator = (this && this.__generator) || function (thisArg, body) {
    var _ = { label: 0, sent: function() { if (t[0] & 1) throw t[1]; return t[1]; }, trys: [], ops: [] }, f, y, t, g;
    return g = { next: verb(0), "throw": verb(1), "return": verb(2) }, typeof Symbol === "function" && (g[Symbol.iterator] = function() { return this; }), g;
    function verb(n) { return function (v) { return step([n, v]); }; }
    function step(op) {
        if (f) throw new TypeError("Generator is already executing.");
        while (g && (g = 0, op[0] && (_ = 0)), _) try {
            if (f = 1, y && (t = op[0] & 2 ? y["return"] : op[0] ? y["throw"] || ((t = y["return"]) && t.call(y), 0) : y.next) && !(t = t.call(y, op[1])).done) return t;
            if (y = 0, t) op = [op[0] & 2, t.value];
            switch (op[0]) {
                case 0: case 1: t = op; break;
                case 4: _.label++; return { value: op[1], done: false };
                case 5: _.label++; y = op[1]; op = [0]; continue;
                case 7: op = _.ops.pop(); _.trys.pop(); continue;
                default:
                    if (!(t = _.trys, t = t.length > 0 && t[t.length - 1]) && (op[0] === 6 || op[0] === 2)) { _ = 0; continue; }
                    if (op[0] === 3 && (!t || (op[1] > t[0] && op[1] < t[3]))) { _.label = op[1]; break; }
                    if (op[0] === 6 && _.label < t[1]) { _.label = t[1]; t = op; break; }
                    if (t && _.label < t[2]) { _.label = t[2]; _.ops.push(op); break; }
                    if (t[2]) _.ops.pop();
                    _.trys.pop(); continue;
            }
            op = body.call(thisArg, _);
        } catch (e) { op = [6, e]; y = 0; } finally { f = t = 0; }
        if (op[0] & 5) throw op[1]; return { value: op[0] ? op[1] : void 0, done: true };
    }
};
Object.defineProperty(exports, "__esModule", { value: true });
var express_1 = require("express");
var body_parser_1 = require("body-parser");
var cors_1 = require("cors");
var ws_1 = require("ws");
var rosnodejs_1 = require("rosnodejs");
var node_dgram_1 = require("node:dgram");
var app = (0, express_1.default)();
var port = 4000;
app.use(function (req, res, next) {
    var contentType = req.headers['content-type'];
    if (contentType && contentType === "application/x-www-form-urlencoded") {
        return body_parser_1.default.urlencoded({ extended: true })(req, res, next);
    }
    return body_parser_1.default.json()(req, res, next);
});
var whitelist = [
    'http://localhost:5000',
    'http://localhost:3000',
];
var corsOptions = {
    credentials: true,
    origin: function (origin, callback) {
        if (!origin || whitelist.indexOf(origin) !== -1) {
            callback(null, true);
        }
        else {
            console.log("".concat(origin, " not allowed by CORS"));
        }
    }
};
app.use((0, cors_1.default)(corsOptions));
app.use(function (req, res, next) {
    var allowedMethods = [
        "GET",
        "POST",
        "PUT",
        "DELETE",
        "OPTIONS",
        "PATCH",
        "HEAD",
        "CONNECT",
    ];
    if (!allowedMethods.includes(req.method))
        res.status(405).send("Method not allowed");
    next();
});
function createWSServer(expressServer) {
    return __awaiter(this, void 0, void 0, function () {
        var wss;
        return __generator(this, function (_a) {
            wss = new ws_1.WebSocketServer({
                noServer: true,
                path: "/ws",
            });
            expressServer.on("upgrade", function (request, socket, head) {
                wss.handleUpgrade(request, socket, head, function (ws) {
                    wss.emit("connection", ws, request);
                });
            });
            wss.on("connection", function (wsConnection, connectionRequest) {
                wsConnection.on("message", function (msg) {
                    var msgStr = msg.toString();
                    console.log(msgStr);
                    var parsedMessage = JSON.parse(msgStr);
                    while (true) { // infinite loop which checks if there's data available to send and sends it if there is
                    }
                });
            });
            return [2 /*return*/, wss];
        });
    });
}
;
// START: ROS Stuff
// const arachnidaMsgs = rosnodejs.require("arachnida");
// const rosNH = rosnodejs.nh; // NodeHandler
// END: ROS Stuff
var server = app.listen(port, function () {
    console.log("[websocket_interface] Server running at port ".concat(port));
    var ws = new ws_1.default("wss://api.arachnida.live/ws");
    ws.onopen = function () {
        console.log("[websocket_interface] WebSocket to wss://api.arachnida.live/ws opened");
    };
    var udpSock = node_dgram_1.default.createSocket('udp4'); // UDP Socket to listen to VLP-16 (LiDAR) directly
    udpSock.on("error", function (err) {
        console.error("[websocket_interface] UDP Socket Error: \n".concat(err.stack));
        udpSock.close();
    });
    udpSock.on("message", function (msg, rinfo) {
        console.log("[websocket_interface] UDP Socket received msg: ".concat(msg, " from ").concat(rinfo.address, ":").concat(rinfo.port));
        if (ws.readyState === ws.OPEN)
            ws.send(msg); // Forward message (raw pointcloud data) to backend via websocket
    });
    udpSock.on("listening", function () {
        console.log("[websocket_interface] UDP Socket listening ".concat(udpSock.address().address, ":").concat(udpSock.address().port));
    });
    udpSock.on("close", function () {
        console.log("[websocket_interface] UDP Socket closed");
    });
    udpSock.bind(6000); // Listen to localhost port 6000 for VLP-16 data via velodyneSocketReader.cpp - Localhost is implicitly assigned by not providing an IP address
    rosnodejs_1.default.initNode("/ws_node").then(function (n) {
        // const sensorMsgs = rosnodejs.require("sensor_msgs").msg;
        // let pcSub = n.subscribe("arachnida/point_cloud/pcl", 'sensor_msgs/PointCloud2', (msg: JSON) => {
        // if frame number of msg is same as any of the messages in the frameQueue then add this point cloud to that frame
        // otherwise add a new Frame to the frameQeue
        // const msgStr: string = JSON.stringify(msg);
        // fs.writeFile('ws_log.json', msgStr, 'utf-8', (err) => {
        //     if(err) console.error(err);
        // })
        // console.log('Received point cloud msg: %j', msg);
        // let msgBuf: Buffer = Buffer.from(msgStr);
        // ws.send(msgBuf);
        // n.
        // });
        // let obstacleSub = n.subscribe("arachnida/obstacle_detection/obstacles", 'arachnida/ObstacleList', (msg) => {
        //     // if frame number of msg is same as any of the messages in the frameQueue then add these obstacles to that frame
        //     // otherwise add a new Frame to the frameQueue
        //     console.log('Received obstacle msg: %j', msg);
        // });
    });
});
var wssp = createWSServer(server);
