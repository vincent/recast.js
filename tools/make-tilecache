#!/usr/bin/env node
'use strict';

var fs       = require('fs');
var recast   = require('../lib/recast');
var argv     = require('minimist')(process.argv.slice(2));

var settings = argv.settings.split(',').map(function(s){
    return s.split('=');
});

console.log(settings);

argv.input  = argv.input  || argv.i || argv._[0];
argv.output = argv.output || argv.o || argv._[1];

if (! (argv.input && argv.output)) {
    console.error('Usage: make-tilecache --input <level.obj> --output <level.tilecache>');
    process.exit(1);
}

// settings(recast);

recast.OBJLoader(argv.input, function(){

    recast.buildTiled();
    recast.saveTileCache('./tilecache.bin', recast.cb(function (error, serialized) {

        var buffer = new Buffer(serialized.length);
        for (var i = 0; i < serialized.length; i++) {
            buffer.writeUInt8(serialized[i], i);
        }

        fs.writeFile(argv.output, buffer, function (err) {
            if (err) {
                console.error('cannot write tilecache file');
            }
            process.exit(err ? 1 : 0);
        });
    }));
});

