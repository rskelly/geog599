const ws = require("nodejs-websocket");

function handleRequest(obj, conn) {
	console.log(obj);
	conn.send(JSON.stringify({'message': obj.message}));
}

const server = ws.createServer(conn => {
    
	console.log("Listening...");
    
    conn.on('text', text => {
    	try {
    		const obj = JSON.parse(text);
    		handleRequest(obj, conn);
    	} catch(err) {
    		conn.sendText(JSON.stringify({'error': err.toString()}));
    	}
    });
    
    /*
    conn.on("binary", function (inStream) {
        // Empty buffer for collecting binary data
        let data = new Buffer(0)
        // Read chunks of binary data and add to the buffer
        inStream.on("readable", function () {
            const newData = inStream.read()
            if (newData)
                data = Buffer.concat([data, newData], data.length+newData.length)
        })
        inStream.on("end", function () {
            console.log("Received " + data.length + " bytes of binary data")
            process_my_data(data)
        })
    });
    */
    
    conn.on('close', (code, reason) => {
        console.log("Connection closed");
    });
    
}).listen(8001);

console.log('Started.');