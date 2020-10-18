var http = require('http');
var url = require('url');
var fs = require('fs');
var dt = require('./myfirstmodule.js');
var uc = require('upper-case');
var events = require('events');
var formidable = require('formidable');

http.createServer(function (req, res) {
  var path = url.parse(req.url, true).pathname;
  var q = url.parse(req.url, true);
  console.log(q.host);
  console.log(q.pathname);
  console.log(q.search);
  console.log(uc("Hello World!"));
  var qdata = q.query;
  var eventEmitter = new events.EventEmitter();
  var myEventHandler = function() {
    console.log('I hear a scream!');
  }
  eventEmitter.on('scream', myEventHandler);
  if (qdata.file == 'file1')
  {
    fs.readFile('demofile1.html', function(err, data){
      if (err) throw err;
      res.writeHead(200, {'Content-Type': 'text/html'});
      res.write(data);
      res.end();
    });
  }
  else if (qdata.file == 'write')
  {
    fs.writeFile('generated.txt', 'Hello content!',
    function (err) {
      if (err) throw err;
      console.log('Saved!');
    });
    res.end();
  }
  else if (qdata.file == 'delete')
  {
    fs.unlink('generated.txt', function (err) {
      if (err) throw err;
      console.log('File deleted!');
    });
    res.end();
  }
  else if (qdata.file == 'event')
  {
    eventEmitter.emit('scream');
    res.end();
  }
  else if (req.url == '/fileupload')
  {
    var form = new formidable.IncomingForm();
    form.parse(req, function (err, fields, files) {
      res.write('File accepted!');
      var oldpath = files.filetoupload.path;
      var newpath = './saved/' + files.filetoupload.name;
      console.log('oldpath: ' + oldpath);
      console.log('newpath: ' + newpath);
      fs.rename(oldpath, newpath, function (err) {
        if (err) throw err;
        res.write('File uploaded and saved!');
        res.end();
      });
      res.write('End of the case');
    });
  }
  else if (qdata.file == 'upload')
  {
    fs.readFile('upload.html', function(err, data){
      if (err) throw err;
      res.writeHead(200, {'Content-Type': 'text/html'});
      res.write(data);
      return res.end();
    });
  }
  else
  {
    res.writeHead(200, {'Content-Type': 'text/html'});
    res.write("The date and time are currently: " + dt.myDateTime() + "; ");
    res.write("URL: " + path + "; ");
    res.write("Queries: year = " + qdata.year + ", month = " + qdata.month);
    res.end();
  }
}).listen(8080);
